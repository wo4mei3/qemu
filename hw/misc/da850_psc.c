#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"

#define TYPE_DA850_PSC "da850-psc"
OBJECT_DECLARE_SIMPLE_TYPE(DA850PSCState, DA850_PSC)

#define MAX_LPSC 64

/* PSC register offsets */
#define EPCPR   0x070
#define PTCMD   0x120
#define PTSTAT  0x128
#define PDSTAT  0x200
#define PDCTL1  0x304
#define MDSTAT  0x800
#define MDCTL   0xA00

#define MDSTAT_STATE_MASK 0x1f

/* PSC states */
#define PSC_STATE_SYNCRESET    0x0
#define PSC_STATE_DISABLE      0x1
#define PSC_STATE_SWRSTDISABLE 0x2
#define PSC_STATE_ENABLE       0x3

typedef struct DA850PSCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t ptcmd;
    uint32_t ptstat;
    uint32_t pdstat;
    uint32_t pdctl1;

    uint32_t mdctl[MAX_LPSC];
    uint32_t mdstat[MAX_LPSC];
} DA850PSCState;

/* Commit pending MDCTL -> MDSTAT (simplified immediate transition) */
static void da850_psc_commit(DA850PSCState *s)
{
    int i;

    for (i = 0; i < MAX_LPSC; i++) {
        uint32_t next = s->mdctl[i] & MDSTAT_STATE_MASK;
        s->mdstat[i] = next;
    }

    /* transition complete */
    s->ptstat = 0;
}

static uint64_t da850_psc_read(void *opaque, hwaddr offset, unsigned size)
{
    DA850PSCState *s = opaque;

    switch (offset) {
    case PTSTAT:
        return s->ptstat;

    case PDSTAT:
        return s->pdstat;

    case PDCTL1:
        return s->pdctl1;

    case EPCPR:
        /* always "power domain ready" */
        return 0xffffffff;
    }

    if (offset >= MDSTAT && offset < MDSTAT + MAX_LPSC * 4) {
        unsigned id = (offset - MDSTAT) / 4;
        return s->mdstat[id];
    }

    return 0;
}

static void da850_psc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    DA850PSCState *s = opaque;

    switch (offset) {
    case PTCMD:
        s->ptcmd = value;

        /* mark transition active */
        s->ptstat = value;

        /* complete immediately */
        da850_psc_commit(s);
        return;

    case PDCTL1:
        s->pdctl1 = value;

        /* power domain on */
        s->pdstat = 1;
        return;
    }

    if (offset >= MDCTL && offset < MDCTL + MAX_LPSC * 4) {
        unsigned id = (offset - MDCTL) / 4;
        s->mdctl[id] = value;
        return;
    }
}

static const MemoryRegionOps da850_psc_ops = {
    .read = da850_psc_read,
    .write = da850_psc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void da850_psc_reset(DeviceState *dev)
{
    DA850PSCState *s = DA850_PSC(dev);
    int i;

    s->ptcmd = 0;
    s->ptstat = 0;

    /* domain active by default */
    s->pdstat = 1;
    s->pdctl1 = 0;

    for (i = 0; i < MAX_LPSC; i++) {
        /* default reset state */
        s->mdctl[i] = PSC_STATE_SWRSTDISABLE;
        s->mdstat[i] = PSC_STATE_SWRSTDISABLE;
    }
}

static void da850_psc_init(Object *obj)
{
    DA850PSCState *s = DA850_PSC(obj);

    memory_region_init_io(&s->iomem, obj,
                          &da850_psc_ops, s,
                          "da850-psc", 0x2000);
}

static void da850_psc_realize(DeviceState *dev, Error **errp)
{
    DA850PSCState *s = DA850_PSC(dev);

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void da850_psc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = da850_psc_realize;
    dc->reset = da850_psc_reset;
}

static const TypeInfo da850_psc_info = {
    .name          = TYPE_DA850_PSC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DA850PSCState),
    .instance_init = da850_psc_init,
    .class_init    = da850_psc_class_init,
};

static void da850_psc_register_types(void)
{
    type_register_static(&da850_psc_info);
}

type_init(da850_psc_register_types);