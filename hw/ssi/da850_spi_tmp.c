#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_DA850_SPI "da850-spi"
OBJECT_DECLARE_SIMPLE_TYPE(DA850SPIState, DA850_SPI)

#define SPI_MAX_CS 2

#define SPIGCR0  0x00
#define SPIGCR1  0x04
#define SPIINT   0x08
#define SPILVL   0x0c
#define SPIFLG   0x10
#define SPIPC0   0x14
#define SPIDAT1  0x3c
#define SPIBUF   0x40
#define SPIDELAY 0x48
#define SPIDEF   0x4c
#define SPIFMT0  0x50

#define SPIGCR1_MASTER_MASK    (1u << 0)
#define SPIGCR1_CLKMOD_MASK    (1u << 1)
#define SPIGCR1_POWERDOWN_MASK (1u << 8)
#define SPIGCR1_LOOPBACK_MASK  (1u << 16)
#define SPIGCR1_SPIENA_MASK    (1u << 24)

#define SPIDAT1_CSHOLD_MASK    (1u << 12)

#define SPIBUF_TXFULL_MASK     (1u << 29)
#define SPIBUF_RXEMPTY_MASK    (1u << 31)

/*
 * SPIINT
 *
 * davinci_spi uses this to enable DMA requests.
 */
#define SPIINT_DMA_REQ_EN      (1u << 16)


typedef struct DA850SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    SSIBus *ssi;

    qemu_irq irq;

    /*
     * Existing CS outputs.
     *
     * Keep these as sysbus IRQ outputs so the existing
     * machine-side:
     *
     *   sysbus_connect_irq(spi0, 1, cs_line)
     *
     * continues to work.
     */
    qemu_irq cs_lines[SPI_MAX_CS];

    /*
     * DMA request output.
     *
     * Named GPIO output:
     *
     *   "dma-tx"
     */
    qemu_irq dma_tx_req;

    uint32_t gcr0;
    uint32_t gcr1;
    uint32_t intreg;
    uint32_t lvl;
    uint32_t flg;
    uint32_t pc0;
    uint32_t dat1;
    uint32_t buf;
    uint32_t delay;
    uint32_t def;
    uint32_t fmt0;

    /*
     * True when SPIBUF contains a received value.
     */
    bool rx_ready;

    int cur_cs;

} DA850SPIState;


/*
 * Select CS0.
 *
 * SPI0 on the EV3 board has the SPI flash on CS0.
 */
static void da850_spi_select_cs0(
    DA850SPIState *s)
{
    qemu_set_irq(
        s->cs_lines[0],
        0
    );

    s->cur_cs = 0;
}


/*
 * Generate one TX DMA request pulse.
 *
 * This goes to EDMA event channel 15.
 */
static void da850_spi_dma_tx_request(
    DA850SPIState *s)
{
    fprintf(stderr,
            "SPI: DMA TX REQUEST\n");

    qemu_set_irq(
        s->dma_tx_req,
        1
    );

    qemu_set_irq(
        s->dma_tx_req,
        0
    );
}


/*
 * Perform one SPI transfer.
 */
static void da850_spi_do_transfer(
    DA850SPIState *s)
{
    uint8_t tx;
    uint8_t rx;

    tx = (uint8_t)(s->dat1 & 0xff);

    if (!(s->gcr1 & SPIGCR1_SPIENA_MASK)) {

        fprintf(stderr,
                "SPI: transfer ignored "
                "(SPIENA=0)\n");

        return;
    }

    fprintf(stderr,
            "SPI: CS LOW, TX=%02x\n",
            tx);

    da850_spi_select_cs0(s);

    /*
     * Perform one SSI frame.
     */
    rx = (uint8_t)ssi_transfer(
        s->ssi,
        tx
    );

    fprintf(stderr,
            "SPI: RX=%02x\n",
            rx);

    /*
     * Put received data in SPIBUF.
     */
    s->buf = rx & 0xff;

    /*
     * RX data is now available.
     */
    s->rx_ready = true;

    /*
     * RXEMPTY = 0 means data available.
     *
     * TXFULL = 0 means transmitter can accept
     * another value.
     */
    s->buf &= ~SPIBUF_RXEMPTY_MASK;
    s->buf &= ~SPIBUF_TXFULL_MASK;

    /*
     * Release CS unless CSHOLD is set.
     */
    if (!(s->dat1 & SPIDAT1_CSHOLD_MASK)) {

        fprintf(stderr,
                "SPI: CS HIGH\n");

        for (int i = 0;
             i < SPI_MAX_CS;
             i++) {

            qemu_set_irq(
                s->cs_lines[i],
                1
            );
        }

        s->cur_cs = -1;
    }
}


/*
 * SPI register read.
 */
static uint64_t da850_spi_read(
    void *opaque,
    hwaddr addr,
    unsigned size)
{
    DA850SPIState *s = opaque;

    switch (addr) {

    case SPIGCR0:
        return s->gcr0;

    case SPIGCR1:
        return s->gcr1;

    case SPIINT:
        return s->intreg;

    case SPILVL:
        return s->lvl;

    case SPIFLG:
        return s->flg;

    case SPIPC0:
        return s->pc0;

    case SPIDAT1:
        return s->dat1;

    case SPIBUF:
        /*
         * Important:
         *
         * The old implementation always returned
         *
         *     s->buf | SPIBUF_RXEMPTY_MASK
         *
         * which means EDMA could never read received data.
         */
        if (s->rx_ready) {

            uint32_t ret;

            ret = s->buf;

            /*
             * Reading SPIBUF consumes the received
             * value.
             */
            s->rx_ready = false;

            /*
             * FIFO/register is empty again.
             */
            s->buf |= SPIBUF_RXEMPTY_MASK;

            return ret;
        }

        return s->buf | SPIBUF_RXEMPTY_MASK;

    case SPIDELAY:
        return s->delay;

    case SPIDEF:
        return s->def;

    case SPIFMT0:
        return s->fmt0;

    default:
        return 0;
    }
}


/*
 * SPI register write.
 */
static void da850_spi_write(
    void *opaque,
    hwaddr addr,
    uint64_t val,
    unsigned size)
{
    DA850SPIState *s = opaque;

    fprintf(stderr,
            "SPI MMIO WRITE: "
            "addr=0x%" PRIx64
            " val=0x%" PRIx64
            "\n",
            addr,
            val);

    switch (addr) {

    case SPIGCR0:

        s->gcr0 = (uint32_t)val;

        break;

    case SPIGCR1:

        s->gcr1 = (uint32_t)val;

        break;

    case SPIINT:

        s->intreg = (uint32_t)val;

        /*
         * Linux does:
         *
         *     set_io_bits(SPIINT,
         *                  SPIINT_DMA_REQ_EN);
         *
         * This enables SPI DMA requests.
         *
         * Generate the first hardware event.
         */
        if (val & SPIINT_DMA_REQ_EN) {

            da850_spi_dma_tx_request(s);
        }

        break;

    case SPILVL:

        s->lvl = (uint32_t)val;

        break;

    case SPIFLG:

        /*
         * Write-1-to-clear.
         */
        s->flg &= ~(uint32_t)val;

        break;

    case SPIPC0:

        s->pc0 = (uint32_t)val;

        break;

    case SPIDAT1:

        fprintf(stderr,
                "SPI: SPIDAT1 WRITE = %02x\n",
                (unsigned)(val & 0xff));

        s->dat1 = (uint32_t)val;

        /*
         * CPU or EDMA writing SPIDAT1 causes
         * one SPI frame to be transferred.
         */
        da850_spi_do_transfer(s);

        break;

    case SPIDELAY:

        s->delay = (uint32_t)val;

        break;

    case SPIDEF:

        s->def = (uint32_t)val;

        break;

    case SPIFMT0:

        s->fmt0 = (uint32_t)val;

        break;

    default:

        break;
    }
}


static const MemoryRegionOps da850_spi_ops = {
    .read = da850_spi_read,
    .write = da850_spi_write,

    .endianness = DEVICE_NATIVE_ENDIAN,

    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};


/*
 * Initialize device.
 */
static void da850_spi_init(
    Object *obj)
{
    DA850SPIState *s =
        DA850_SPI(obj);

    SysBusDevice *dev =
        SYS_BUS_DEVICE(obj);

    /*
     * SSI bus.
     */
    s->ssi =
        ssi_create_bus(
            DEVICE(obj),
            "spi"
        );

    /*
     * MMIO.
     */
    memory_region_init_io(
        &s->mmio,
        obj,
        &da850_spi_ops,
        s,
        "da850-spi",
        0x1000
    );

    sysbus_init_mmio(
        dev,
        &s->mmio
    );

    /*
     * IRQ 0 = SPI interrupt.
     */
    sysbus_init_irq(
        dev,
        &s->irq
    );

    /*
     * IRQ 1.. = chip selects.
     *
     * Keep these exactly as before.
     */
    for (int i = 0;
         i < SPI_MAX_CS;
         i++) {

        sysbus_init_irq(
            dev,
            &s->cs_lines[i]
        );
    }

    /*
     * Named DMA output.
     *
     * This does NOT change the sysbus IRQ numbering.
     */
    qdev_init_gpio_out_named(
        DEVICE(obj),
        &s->dma_tx_req,
        "dma-tx",
        1
    );
}


/*
 * Realize.
 */
static void da850_spi_realize(
    DeviceState *dev,
    Error **errp)
{
    DA850SPIState *s =
        DA850_SPI(dev);

    s->buf =
        SPIBUF_RXEMPTY_MASK |
        SPIBUF_TXFULL_MASK;

    s->rx_ready = false;

    s->cur_cs = -1;

    for (int i = 0;
         i < SPI_MAX_CS;
         i++) {

        /*
         * Idle = deselected.
         */
        qemu_set_irq(
            s->cs_lines[i],
            1
        );
    }
}


static void da850_spi_class_init(
    ObjectClass *klass,
    void *data)
{
    DeviceClass *dc =
        DEVICE_CLASS(klass);

    dc->realize =
        da850_spi_realize;
}


static const TypeInfo da850_spi_info = {
    .name =
        TYPE_DA850_SPI,

    .parent =
        TYPE_SYS_BUS_DEVICE,

    .instance_size =
        sizeof(DA850SPIState),

    .instance_init =
        da850_spi_init,

    .class_init =
        da850_spi_class_init,
};


static void da850_spi_register(void)
{
    type_register_static(
        &da850_spi_info
    );
}


type_init(da850_spi_register);