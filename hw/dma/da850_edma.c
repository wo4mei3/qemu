/*
 * TI DA850 / OMAP-L138 EDMA3 Channel Controller
 *
 * Minimal functional model for LEGO EV3 / DA850 SPI0.
 *
 * SPI0:
 *
 *     RX DMA channel = 14
 *     TX DMA channel = 15
 *
 * This is not a cycle-accurate EDMA3 model.
 *
 * The purpose of this model is to provide enough EDMA functionality
 * for the Linux davinci SPI driver used by the EV3 kernel.
 *
 * Important:
 *
 * Linux 2.6 ARM implements memcpy_toio() as:
 *
 *     writeb(...)
 *
 * repeatedly.
 *
 * Therefore PaRAM must accept byte-sized MMIO accesses.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qom/object.h"

#define TYPE_DA850_EDMA "da850-edma"

OBJECT_DECLARE_SIMPLE_TYPE(DA850EDMAState, DA850_EDMA)

/*
 * --------------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------------
 */

#define EDMA_NUM_CHANNELS 64
#define EDMA_NUM_SLOTS    128

#define SPI0_RX_CHAN      14
#define SPI0_TX_CHAN      15

/*
 * --------------------------------------------------------------------------
 * EDMA CC registers
 * --------------------------------------------------------------------------
 */

#define EDMA_ECR          0x1008
#define EDMA_ECRH         0x100c

#define EDMA_SHADOW0      0x2000

#define SH_ER             0x00
#define SH_ESR            0x10
#define SH_EER            0x20
#define SH_EESR           0x30
#define SH_SECR           0x40
#define SH_IER            0x50
#define SH_IESR           0x60
#define SH_IECR           0x58
#define SH_IPR            0x68
#define SH_ICR            0x70
#define SH_IEVAL          0x78

/*
 * --------------------------------------------------------------------------
 * PaRAM
 * --------------------------------------------------------------------------
 *
 * Each PaRAM entry is 0x20 bytes:
 *
 *   0x00 OPT
 *   0x04 SRC
 *   0x08 A_B_CNT
 *   0x0c DST
 *   0x10 SRC_DST_BIDX
 *   0x14 LINK_BCNTRLD
 *   0x18 SRC_DST_CIDX
 *   0x1c CCNT
 */

#define EDMA_PARM              0x4000

#define PARM_OPT               0x00
#define PARM_SRC               0x04
#define PARM_A_B_CNT           0x08
#define PARM_DST               0x0c
#define PARM_SRC_DST_BIDX      0x10
#define PARM_LINK_BCNTRLD      0x14
#define PARM_SRC_DST_CIDX      0x18
#define PARM_CCNT              0x1c

#define PARM_SIZE              0x20

/*
 * --------------------------------------------------------------------------
 * OPT
 * --------------------------------------------------------------------------
 */

#define OPT_SYNCDIM            (1u << 2)
#define OPT_STATIC              (1u << 3)

#define OPT_TCINTEN            (1u << 20)

#define OPT_TCC_SHIFT          12
#define OPT_TCC_MASK           (0x3fu << OPT_TCC_SHIFT)

/*
 * --------------------------------------------------------------------------
 * A/B/C count
 * --------------------------------------------------------------------------
 */

#define ACNT_MASK              0xffffu
#define BCNT_SHIFT             16
#define BCNT_MASK              0xffffu

/*
 * --------------------------------------------------------------------------
 * Raw PaRAM representation
 * --------------------------------------------------------------------------
 *
 * Do NOT represent PaRAM as uint32_t fields for MMIO storage.
 *
 * The Linux ARM kernel's memcpy_toio() writes one byte at a time.
 *
 * Keeping raw bytes lets us correctly handle:
 *
 *     writeb()
 *     writew()
 *     writel()
 *
 * to arbitrary byte offsets.
 */

typedef struct DA850EDMAParam {
    uint8_t bytes[PARM_SIZE];
} DA850EDMAParam;

/*
 * --------------------------------------------------------------------------
 * Runtime channel state
 * --------------------------------------------------------------------------
 */

typedef struct DA850EDMAChanState {
    bool armed;

    uint32_t cur_src;
    uint32_t cur_dst;

    uint32_t acnt;
    uint32_t bcnt;
    uint32_t ccnt;

    uint32_t b_index;
    uint32_t c_index;

    int32_t src_bidx;
    int32_t dst_bidx;

    int32_t src_cidx;
    int32_t dst_cidx;

    uint32_t opt;
} DA850EDMAChanState;

/*
 * --------------------------------------------------------------------------
 * Device state
 * --------------------------------------------------------------------------
 */

typedef struct DA850EDMAState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    /*
     * CCINT0
     */
    qemu_irq cc_irq;

    /*
     * PaRAM entries.
     */
    DA850EDMAParam parm[EDMA_NUM_SLOTS];

    /*
     * Runtime state.
     */
    DA850EDMAChanState chan[EDMA_NUM_CHANNELS];

    /*
     * Event registers.
     */
    uint64_t er;
    uint64_t eer;

    /*
     * Interrupt enable/pending.
     */
    uint64_t ier;
    uint64_t ipr;
} DA850EDMAState;


/*
 * ==========================================================================
 * PaRAM helpers
 * ==========================================================================
 */

/*
 * Read a little-endian 32-bit PaRAM field.
 */
static uint32_t da850_edma_param_ld32(const DA850EDMAParam *p,
                                      unsigned offset)
{
    uint32_t value;

    memcpy(&value, &p->bytes[offset], sizeof(value));

    return le32_to_cpu(value);
}

/*
 * Write a little-endian 32-bit PaRAM field.
 */
/*
static void da850_edma_param_st32(DA850EDMAParam *p,
                                  unsigned offset,
                                  uint32_t value)
{
    value = cpu_to_le32(value);

    memcpy(&p->bytes[offset], &value, sizeof(value));
}*/

/*
 * Read an arbitrary-size PaRAM MMIO access.
 *
 * QEMU's MemoryRegionOps passes the access size here.
 */
static uint64_t da850_edma_param_read(const DA850EDMAParam *p,
                                      unsigned field,
                                      unsigned size)
{
    uint64_t value = 0;

    if (field >= PARM_SIZE ||
        field + size > PARM_SIZE ||
        size > sizeof(value)) {
        return 0;
    }

    memcpy(&value, &p->bytes[field], size);

    /*
     * PaRAM is little endian on the DA850/ARM target.
     */
    if (size == 2) {
        return le16_to_cpu((uint16_t)value);
    }

    if (size == 4) {
        return le32_to_cpu((uint32_t)value);
    }

    return value;
}

/*
 * Write an arbitrary-size PaRAM MMIO access.
 */
static void da850_edma_param_write(DA850EDMAParam *p,
                                   unsigned field,
                                   uint64_t value,
                                   unsigned size)
{
    if (field >= PARM_SIZE ||
        field + size > PARM_SIZE ||
        size > sizeof(value)) {
        return;
    }

    if (size == 2) {
        uint16_t v = cpu_to_le16((uint16_t)value);

        memcpy(&p->bytes[field], &v, sizeof(v));
        return;
    }

    if (size == 4) {
        uint32_t v = cpu_to_le32((uint32_t)value);

        memcpy(&p->bytes[field], &v, sizeof(v));
        return;
    }

    /*
     * Byte access.
     */
    memcpy(&p->bytes[field], &value, size);
}


/*
 * ==========================================================================
 * Misc helpers
 * ==========================================================================
 */

static int32_t da850_edma_s16(uint32_t value)
{
    return (int16_t)(value & 0xffff);
}

static void da850_edma_update_irq(DA850EDMAState *s)
{
    bool level;

    level = (s->ipr & s->ier) != 0;
/*fprintf(stderr,
        "EDMA: IRQ update ipr=%016" PRIx64
        " ier=%016" PRIx64
        " pending=%016" PRIx64
        " level=%d\n",
        s->ipr,
        s->ier,
        s->ipr & s->ier,
        level);*/
    qemu_set_irq(s->cc_irq, level);
}


/*
 * ==========================================================================
 * Load PaRAM into runtime state
 * ==========================================================================
 */

static void da850_edma_load_from_parm(DA850EDMAState *s,
                                      unsigned chan)
{
    DA850EDMAParam *p;
    DA850EDMAChanState *c;

    uint32_t opt;
    uint32_t src;
    uint32_t a_b_cnt;
    uint32_t dst;
    uint32_t bidx;
    uint32_t cidx;
    uint32_t ccnt;

    if (chan >= EDMA_NUM_CHANNELS) {
        return;
    }

    /*
     * Channel number normally maps directly to its PaRAM slot
     * in the configuration used by the EV3 Linux driver.
     */
    p = &s->parm[chan % EDMA_NUM_SLOTS];
    c = &s->chan[chan];

    opt = da850_edma_param_ld32(p, PARM_OPT);
    src = da850_edma_param_ld32(p, PARM_SRC);
    a_b_cnt = da850_edma_param_ld32(p, PARM_A_B_CNT);
    dst = da850_edma_param_ld32(p, PARM_DST);
    bidx = da850_edma_param_ld32(p, PARM_SRC_DST_BIDX);
    cidx = da850_edma_param_ld32(p, PARM_SRC_DST_CIDX);
    ccnt = da850_edma_param_ld32(p, PARM_CCNT);

    c->opt = opt;

    c->cur_src = src;
    c->cur_dst = dst;

    c->acnt = a_b_cnt & ACNT_MASK;

    c->bcnt =
        (a_b_cnt >> BCNT_SHIFT) & BCNT_MASK;

    c->ccnt = ccnt & 0xffff;

    /*
     * A zero CCNT is treated as one transfer for this minimal model.
     */
    if (c->ccnt == 0) {
        c->ccnt = 1;
    }

    c->b_index = 0;
    c->c_index = 0;

    /*
     * SRCBIDX / DSTBIDX
     */
    c->src_bidx = da850_edma_s16(bidx);
    c->dst_bidx = da850_edma_s16(bidx >> 16);

    /*
     * SRCCIDX / DSTCIDX
     */
    c->src_cidx = da850_edma_s16(cidx);
    c->dst_cidx = da850_edma_s16(cidx >> 16);

    /*fprintf(stderr,
            "EDMA: load parm ch=%u "
            "opt=%08x src=%08x a_b=%08x dst=%08x "
            "bidx=%08x cidx=%08x ccnt=%08x\n",
            chan,
            opt,
            src,
            a_b_cnt,
            dst,
            bidx,
            cidx,
            ccnt);*/
}


/*
 * ==========================================================================
 * Completion
 * ==========================================================================
 */

static void da850_edma_complete(DA850EDMAState *s,
                                unsigned chan)
{
    DA850EDMAChanState *c;
    unsigned tcc;

    if (chan >= EDMA_NUM_CHANNELS) {
        return;
    }

    c = &s->chan[chan];

    c->armed = false;

    if (!(c->opt & OPT_TCINTEN)) {
        /*fprintf(stderr,
                "EDMA: complete ch=%u no TCINT\n",
                chan);*/
        return;
    }

    tcc = (c->opt & OPT_TCC_MASK) >> OPT_TCC_SHIFT;

    s->ipr |= 1ULL << (tcc & 63);

    /*fprintf(stderr,
            "EDMA: complete ch=%u tcc=%u ipr=%016" PRIx64 "\n",
            chan,
            tcc,
            s->ipr);
*/
    da850_edma_update_irq(s);
}


/*
 * ==========================================================================
 * One DMA A-element
 * ==========================================================================
 */

static bool da850_edma_step(DA850EDMAState *s,
                            unsigned chan)
{
    DA850EDMAChanState *c;
    uint8_t buf[64];

    c = &s->chan[chan];

    if (!c->armed) {
        return false;
    }

    if (c->bcnt == 0 ||
        c->ccnt == 0) {

        da850_edma_complete(s, chan);
        return false;
    }

    if (c->acnt == 0 ||
        c->acnt > sizeof(buf)) {

        /*fprintf(stderr,
                "EDMA: invalid ACNT ch=%u acnt=%u\n",
                chan,
                c->acnt);*/

        da850_edma_complete(s, chan);
        return false;
    }

    /*fprintf(stderr,
            "EDMA: step ch=%u src=%08x dst=%08x "
            "acnt=%u b=%u/%u c=%u/%u\n",
            chan,
            c->cur_src,
            c->cur_dst,
            c->acnt,
            c->b_index,
            c->bcnt,
            c->c_index,
            c->ccnt);*/

    /*
     * Memory -> peripheral / memory.
     */
    address_space_read(&address_space_memory,
                       c->cur_src,
                       MEMTXATTRS_UNSPECIFIED,
                       buf,
                       c->acnt);

    address_space_write(&address_space_memory,
                        c->cur_dst,
                        MEMTXATTRS_UNSPECIFIED,
                        buf,
                        c->acnt);

    /*
     * One A-element completed.
     */
    c->b_index++;

    /*
     * More B elements in this frame.
     */
    if (c->b_index < c->bcnt) {

        c->cur_src += c->src_bidx;
        c->cur_dst += c->dst_bidx;

        return true;
    }

    /*
     * End of B dimension.
     */
    c->b_index = 0;
    c->c_index++;

    /*
     * More frames.
     */
    if (c->c_index < c->ccnt) {

        /*
         * The final BIDX adjustment has already positioned us
         * at the end of the previous frame.
         *
         * CIDX is the frame-to-frame adjustment.
         */
        c->cur_src += c->src_cidx;
        c->cur_dst += c->dst_cidx;

        return true;
    }

    /*
     * Entire transfer complete.
     */
    da850_edma_complete(s, chan);

    return false;
}


/*
 * ==========================================================================
 * SPI0 paired execution
 * ==========================================================================
 *
 * The minimal SPI model performs an SSI transfer immediately when SPIDAT1
 * is written.
 *
 * Therefore:
 *
 *     TX DMA
 *         memory -> SPIDAT1
 *
 * causes:
 *
 *     SPI transfer
 *
 * and immediately produces:
 *
 *     SPIBUF
 *
 * The RX DMA can then read SPIBUF.
 *
 * This gives us a simple full-duplex SPI model.
 */

static void da850_edma_run_spi_pair(DA850EDMAState *s)
{
    DA850EDMAChanState *rx;
    DA850EDMAChanState *tx;

    rx = &s->chan[SPI0_RX_CHAN];
    tx = &s->chan[SPI0_TX_CHAN];

    /*fprintf(stderr,
            "EDMA: SPI pair "
            "RX armed=%d src=%08x dst=%08x "
            "TX armed=%d src=%08x dst=%08x\n",
            rx->armed,
            rx->cur_src,
            rx->cur_dst,
            tx->armed,
            tx->cur_src,
            tx->cur_dst);*/

    /*
     * Need both channels.
     */
    if (!rx->armed || !tx->armed) {
        return;
    }

    /*
     * Full duplex:
     *
     *     TX
     *     |
     *     v
     *    SPI
     *     |
     *     v
     *     RX
     */
    while (tx->armed && rx->armed) {

        if (!da850_edma_step(s, SPI0_TX_CHAN)) {
            break;
        }

        if (!da850_edma_step(s, SPI0_RX_CHAN)) {
            break;
        }
    }

    /*
     * If one side has a longer transfer, finish it.
     *
     * Normally EV3 SPI TX/RX have equal lengths.
     */
    while (tx->armed) {
        if (!da850_edma_step(s, SPI0_TX_CHAN)) {
            break;
        }
    }

    while (rx->armed) {
        if (!da850_edma_step(s, SPI0_RX_CHAN)) {
            break;
        }
    }
}


/*
 * ==========================================================================
 * Try to execute channel
 * ==========================================================================
 */

static void da850_edma_maybe_kick(DA850EDMAState *s,
                                  unsigned chan)
{
    DA850EDMAChanState *c;

    if (chan >= EDMA_NUM_CHANNELS) {
        return;
    }

    c = &s->chan[chan];

    if (!c->armed) {
        return;
    }

    /*
     * Do not execute merely because a single byte of a PaRAM was written.
     *
     * The channel is loaded only when it is explicitly armed.
     */
    if (c->acnt == 0 ||
        c->bcnt == 0 ||
        c->ccnt == 0) {
        return;
    }

    if (chan == SPI0_RX_CHAN ||
        chan == SPI0_TX_CHAN) {

        da850_edma_run_spi_pair(s);
        return;
    }

    /*
     * Generic fallback.
     */
    while (c->armed) {
        if (!da850_edma_step(s, chan)) {
            break;
        }
    }
}


/*
 * ==========================================================================
 * Arm channel
 * ==========================================================================
 */

static void da850_edma_arm(DA850EDMAState *s,
                           unsigned chan)
{
    DA850EDMAChanState *c;

    if (chan >= EDMA_NUM_CHANNELS) {
        return;
    }

    /*fprintf(stderr,
            "EDMA: arm ch=%u\n",
            chan);*/

    c = &s->chan[chan];

    /*
     * Load the complete PaRAM descriptor now.
     */
    da850_edma_load_from_parm(s, chan);

    c->armed = true;

    /*
     * Clear pending event.
     */
    s->er &= ~(1ULL << chan);

    /*
     * Try execution.
     *
     * For SPI this waits for both RX and TX.
     */
    da850_edma_maybe_kick(s, chan);
}


/*
 * ==========================================================================
 * PaRAM MMIO
 * ==========================================================================
 */

static uint64_t da850_edma_read_parm(DA850EDMAState *s,
                                     hwaddr addr,
                                     unsigned size)
{
    unsigned slot;
    unsigned field;

    slot = (addr - EDMA_PARM) / PARM_SIZE;
    field = (addr - EDMA_PARM) % PARM_SIZE;

    if (slot >= EDMA_NUM_SLOTS) {
        return 0;
    }

    return da850_edma_param_read(&s->parm[slot],
                                 field,
                                 size);
}


static void da850_edma_write_parm(DA850EDMAState *s,
                                  hwaddr addr,
                                  uint64_t val,
                                  unsigned size)
{
    unsigned slot;
    unsigned field;

    slot = (addr - EDMA_PARM) / PARM_SIZE;
    field = (addr - EDMA_PARM) % PARM_SIZE;

    if (slot >= EDMA_NUM_SLOTS) {
        return;
    }

    /*
     * Only print SPI0's active slots.
     *
     * Linux also initializes many other PaRAM slots during boot.
     */
    if (slot == SPI0_RX_CHAN ||
        slot == SPI0_TX_CHAN) {

        /*fprintf(stderr,
                "EDMA: PARM WRITE "
                "slot=%u field=%02x size=%u val=%08" PRIx64 "\n",
                slot,
                field,
                size,
                val);*/
    }

    /*
     * Update the raw PaRAM bytes.
     */
    da850_edma_param_write(&s->parm[slot],
                           field,
                           val,
                           size);

    /*
     * IMPORTANT:
     *
     * Do NOT call maybe_kick() here.
     *
     * Linux writes PaRAM using 32 individual byte accesses.
     *
     * Executing the channel after the first byte would observe
     * an incomplete descriptor.
     *
     * Execution begins from da850_edma_arm().
     */
}


/*
 * ==========================================================================
 * Register read
 * ==========================================================================
 */

static uint64_t da850_edma_read(void *opaque,
                                hwaddr addr,
                                unsigned size)
{
    DA850EDMAState *s = opaque;

    /*
     * PaRAM.
     */
    if (addr >= EDMA_PARM &&
        addr < EDMA_PARM +
               EDMA_NUM_SLOTS * PARM_SIZE) {

        return da850_edma_read_parm(s,
                                    addr,
                                    size);
    }

    /*
     * Shadow registers.
     */
    if (addr >= EDMA_SHADOW0 &&
        addr < EDMA_SHADOW0 + 0x200) {

        unsigned off;
        unsigned reg;
        bool high;
        uint64_t value;

        off = addr - EDMA_SHADOW0;

        /*
         * Registers are 32-bit.
         */
        reg = off & ~0x4u;
        high = (off & 0x4) != 0;
/*fprintf(stderr,
        "EDMA: READ shadow addr=%08" HWADDR_PRIx
        " off=%03x reg=%03x size=%u high=%d "
        "ipr=%016" PRIx64 " ier=%016" PRIx64 "\n",
        addr,
        off,
        reg,
        size,
        high,
        s->ipr,
        s->ier);*/
        switch (reg) {

        case SH_ER:
            value = s->er;
            break;

        case SH_EER:
            value = s->eer;
            break;

        case SH_IESR:
        case SH_IECR:
        case SH_IER:
            value = s->ier;
            break;

        case SH_IPR:
            value = s->ipr;
            break;

        default:
            value = 0;
            break;
        }
        /*fprintf(stderr,
                "EDMA: READ shadow addr=%08llx off=%03x reg=%03x "
                "size=%u high=%d value=%08x ipr=%016llx ier=%016llx\n",
                (unsigned long long)addr,
                off,
                reg,
                size,
                high,
                high ? (uint32_t)(value >> 32) : (uint32_t)value,
                (unsigned long long)s->ipr,
                (unsigned long long)s->ier);*/

        if (high) {
            return (uint32_t)(value >> 32);
        }

        return (uint32_t)value;
    }

    return 0;
}


/*
 * ==========================================================================
 * Register write
 * ==========================================================================
 */

static void da850_edma_write(void *opaque,
                             hwaddr addr,
                             uint64_t val,
                             unsigned size)
{
    DA850EDMAState *s = opaque;

    /*
     * PaRAM.
     */
    if (addr >= EDMA_PARM &&
        addr < EDMA_PARM +
               EDMA_NUM_SLOTS * PARM_SIZE) {

        da850_edma_write_parm(s,
                              addr,
                              val,
                              size);
        return;
    }

    /*
     * Shadow registers.
     */
    if (addr >= EDMA_SHADOW0 &&
        addr < EDMA_SHADOW0 + 0x200) {

        unsigned off;
        unsigned reg;
        bool high;
        uint64_t mask;

        off = addr - EDMA_SHADOW0;

        reg = off & ~0x4u;
        high = (off & 0x4) != 0;

        mask = val & 0xffffffffULL;

        if (high) {
            mask <<= 32;
        }

        switch (reg) {

        /*
         * Event set.
         *
         * The simplified model treats this as starting the channel.
         */
        case SH_ESR:

            for (unsigned i = 0;
                 i < EDMA_NUM_CHANNELS;
                 i++) {

                if (mask & (1ULL << i)) {
                    da850_edma_arm(s, i);
                }
            }

            break;

        /*
         * Event enable set.
         *
         * In the real EDMA this enables event association.
         *
         * For this functional model, we also arm the channel because
         * the EV3 SPI driver uses edma_start() to enable the event.
         */
        case SH_EESR:

            s->eer |= mask;

            for (unsigned i = 0;
                 i < EDMA_NUM_CHANNELS;
                 i++) {

                if (mask & (1ULL << i)) {
                    da850_edma_arm(s, i);
                }
            }

            break;

        /*
         * Event clear.
         */
        case SH_SECR:

            s->er &= ~mask;
            break;

        /*
         * Interrupt enable set.
         */
        case SH_IESR:

    /*fprintf(stderr,
            "EDMA: IESR mask=%016" PRIx64
            " old=%016" PRIx64
            " new=%016" PRIx64 "\n",
            mask,
            s->ier,
            s->ier | mask);*/

            s->ier |= mask;

            da850_edma_update_irq(s);

            break;

        /*
         * Interrupt enable clear.
         */
        case SH_IECR:

            s->ier &= ~mask;

            da850_edma_update_irq(s);

            break;

        /*
         * Interrupt pending clear.
         */
        case SH_ICR:

                /*fprintf(stderr,
            "EDMA: ICR mask=%016" PRIx64
            " old_ipr=%016" PRIx64,
            mask, s->ipr);*/

    s->ipr &= ~mask;

    /*fprintf(stderr,
            " new_ipr=%016" PRIx64 "\n",
            s->ipr);*/

            da850_edma_update_irq(s);

            break;

        /*
         * Interrupt evaluation.
         */
        case SH_IEVAL:

            da850_edma_update_irq(s);

            break;

        default:
            break;
        }

        return;
    }

    /*
     * Event clear registers outside the shadow region.
     */
    switch (addr) {

    case EDMA_ECR:

        s->er &= ~(uint64_t)(uint32_t)val;

        break;

    case EDMA_ECRH:

        s->er &= ~((uint64_t)(uint32_t)val << 32);

        break;

    default:
        break;
    }
}


/*
 * ==========================================================================
 * MemoryRegionOps
 * ==========================================================================
 *
 * IMPORTANT:
 *
 * memcpy_toio() on this old ARM kernel uses writeb().
 *
 * Therefore min_access_size MUST be 1.
 */

static const MemoryRegionOps da850_edma_ops = {
    .read = da850_edma_read,
    .write = da850_edma_write,

    .endianness = DEVICE_LITTLE_ENDIAN,

    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};


/*
 * ==========================================================================
 * Device initialization
 * ==========================================================================
 */

static void da850_edma_init(Object *obj)
{
    DA850EDMAState *s = DA850_EDMA(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio,
                          obj,
                          &da850_edma_ops,
                          s,
                          "da850-edma",
                          0x8000);

    sysbus_init_mmio(dev, &s->mmio);

    sysbus_init_irq(dev, &s->cc_irq);
}


/*
 * ==========================================================================
 * Realize
 * ==========================================================================
 */

static void da850_edma_realize(DeviceState *dev,
                               Error **errp)
{
    DA850EDMAState *s = DA850_EDMA(dev);

    memset(s->parm, 0, sizeof(s->parm));
    memset(s->chan, 0, sizeof(s->chan));

    s->er = 0;
    s->eer = 0;
    s->ier = 0;
    s->ipr = 0;
}


/*
 * ==========================================================================
 * Class initialization
 * ==========================================================================
 */

static void da850_edma_class_init(ObjectClass *klass,
                                   void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = da850_edma_realize;
}


/*
 * ==========================================================================
 * Type information
 * ==========================================================================
 */

static const TypeInfo da850_edma_info = {
    .name = TYPE_DA850_EDMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DA850EDMAState),

    .instance_init = da850_edma_init,
    .class_init = da850_edma_class_init,
};


/*
 * ==========================================================================
 * Register type
 * ==========================================================================
 */

static void da850_edma_register(void)
{
    type_register_static(&da850_edma_info);
}

type_init(da850_edma_register);