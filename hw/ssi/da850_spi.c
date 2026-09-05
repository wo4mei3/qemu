/*
 * TI DA850/OMAP-L138 SPI controller (matches drivers/spi/davinci_spi.c,
 * "spi_davinci" platform device).
 *
 * This is a minimal functional model: it does not model prescale-based
 * clock timing, FIFO depth limits, or most error conditions. Each write
 * to SPIDAT1 performs one immediate, full-duplex byte transfer on the
 * QEMU SSI bus (whatever slave is selected via chip_select), and the
 * received byte is latched into SPIBUF. This works correctly with the
 * paired DA850 EDMA model (da850-edma.c hardcodes this controller's
 * SPI0 TX/RX DMA channel numbers, 15 and 14, and interleaves TX writes
 * to SPIDAT1 with RX reads of SPIBUF byte-by-byte), and also works for
 * direct CPU-driven (non-DMA) transfers.
 */

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

#define SPIDAT1_CSHOLD_MASK (1u << 12)
/* NOTE: davinci_spi encodes which CS line(s) to assert somewhere in
 * SPIDAT1 (a CSNR-style field), but the exact bit position was not
 * verified against the TRM. Since this board only ever has one device
 * on the SPI0 bus (the m25p80 flash, chip_select=0), that field is
 * intentionally NOT decoded here -- every transfer simply asserts CS0.
 * If a second real chip select is ever needed on this bus, the actual
 * bit position must be confirmed before relying on it. */

#define SPIBUF_TXFULL_MASK  (1u << 29)
#define SPIBUF_RXEMPTY_MASK (1u << 31)

typedef struct DA850SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    SSIBus *ssi;
    qemu_irq irq;
    qemu_irq cs_lines[SPI_MAX_CS];

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

    int cur_cs;
} DA850SPIState;

static void da850_spi_select_cs0(DA850SPIState *s)
{
    /* See the NOTE above SPIDAT1_CSHOLD_MASK: only CS0 (the m25p80
     * flash) is ever wired on this bus, so unconditionally assert it
     * for the duration of a transfer rather than decoding an unverified
     * CSNR field. */
    qemu_set_irq(s->cs_lines[0], 0);
    s->cur_cs = 0;
}

static void da850_spi_do_transfer(DA850SPIState *s)
{
    uint8_t tx = (uint8_t)(s->dat1 & 0xff);
    uint8_t rx;
/*fprintf(stderr,
        "SPI: TRANSFER TX=%02x\n",
        (unsigned)(s->dat1 & 0xff));*/
    if (!(s->gcr1 & SPIGCR1_SPIENA_MASK)) {
        /* Controller not enabled: don't touch the SSI bus. */
        return;
    }

    da850_spi_select_cs0(s);

    rx = ssi_transfer(s->ssi, tx);

    s->buf = rx & 0xff;
    s->buf &= ~SPIBUF_RXEMPTY_MASK; /* data available */
    s->buf &= ~SPIBUF_TXFULL_MASK;  /* transmitter ready again */

    if (!(s->dat1 & SPIDAT1_CSHOLD_MASK)) {
        /* Deselect all chip selects once the (last) word of a
         * transaction completes. */
        for (int i = 0; i < SPI_MAX_CS; i++) {
            qemu_set_irq(s->cs_lines[i], 1);
        }
    }
}

static uint64_t da850_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    DA850SPIState *s = opaque;

    switch (addr) {
    case SPIGCR0: return s->gcr0;
    case SPIGCR1: return s->gcr1;
    case SPIINT:  return s->intreg;
    case SPILVL:  return s->lvl;
    case SPIFLG:  return s->flg;
    case SPIPC0:  return s->pc0;
    case SPIDAT1: return s->dat1;
    case SPIBUF:  return s->buf | SPIBUF_RXEMPTY_MASK; /* read clears */
    case SPIDELAY: return s->delay;
    case SPIDEF:  return s->def;
    case SPIFMT0: return s->fmt0;
    default: return 0;
    }
}

static void da850_spi_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    DA850SPIState *s = opaque;
    /*fprintf(stderr,
        "SPI: WRITE addr=%08x value=%08x size=%u\n",
        (unsigned)addr,
        (unsigned)val,
        size);*/
    switch (addr) {
    case SPIGCR0:
        s->gcr0 = val;
        break;
    case SPIGCR1:
        s->gcr1 = val;
        /*fprintf(stderr, "SPI: SPIGCR1 WRITE=0x%08x (SPIENA=%d)\n",
                (unsigned)val, !!(val & SPIGCR1_SPIENA_MASK));*/
        break;
    case SPIINT:
        s->intreg = val;
        break;
    case SPILVL:
        s->lvl = val;
        break;
    case SPIFLG:
        /* Write-1-to-clear error flags. */
        s->flg &= ~val;
        break;
    case SPIPC0:
        s->pc0 = val;
        break;
    case SPIDAT1:
        s->dat1 = val;
        /*fprintf(stderr, "SPI: SPIDAT1 WRITE tx=0x%02x (gcr1=0x%08x)\n",
                (unsigned)(val & 0xff), s->gcr1);*/
        da850_spi_do_transfer(s);
        /*fprintf(stderr, "SPI: SPIBUF after transfer = 0x%08x\n", s->buf);*/   
        break;
    case SPIDELAY:
        s->delay = val;
        break;
    case SPIDEF:
        s->def = val;
        break;
    case SPIFMT0:
        s->fmt0 = val;
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

static void da850_spi_init(Object *obj)
{
    DA850SPIState *s = DA850_SPI(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    s->ssi = ssi_create_bus(DEVICE(obj), "spi");

    memory_region_init_io(&s->mmio, obj, &da850_spi_ops, s,
                           "da850-spi", 0x1000);
    sysbus_init_mmio(dev, &s->mmio);
    sysbus_init_irq(dev, &s->irq);

    for (int i = 0; i < SPI_MAX_CS; i++) {
        sysbus_init_irq(dev, &s->cs_lines[i]);
    }
}

static void da850_spi_realize(DeviceState *dev, Error **errp)
{
    DA850SPIState *s = DA850_SPI(dev);

    s->buf = SPIBUF_RXEMPTY_MASK | SPIBUF_TXFULL_MASK;
    s->cur_cs = -1;
    for (int i = 0; i < SPI_MAX_CS; i++) {
        qemu_set_irq(s->cs_lines[i], 1); /* idle = deselected */
    }
}

static void da850_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = da850_spi_realize;
}

static const TypeInfo da850_spi_info = {
    .name = TYPE_DA850_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DA850SPIState),
    .instance_init = da850_spi_init,
    .class_init = da850_spi_class_init,
};

static void da850_spi_register(void)
{
    type_register_static(&da850_spi_info);
}

type_init(da850_spi_register);