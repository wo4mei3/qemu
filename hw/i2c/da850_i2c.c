#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qom/object.h"

#define TYPE_DA850_I2C "da850-i2c"
#define TYPE_DA850_I2C_SLAVE "da850-i2c-slave"
OBJECT_DECLARE_SIMPLE_TYPE(DA850I2CState, DA850_I2C)
OBJECT_DECLARE_SIMPLE_TYPE(DA850I2CSlaveState, DA850_I2C_SLAVE)

#define I2C_OAR  0x00
#define I2C_IMR  0x04
#define I2C_STR  0x08
#define I2C_CLKL 0x0c
#define I2C_CLKH 0x10
#define I2C_CNT  0x14
#define I2C_DRR  0x18
#define I2C_SAR  0x1c
#define I2C_DXR  0x20
#define I2C_MDR  0x24
#define I2C_IVR  0x28
#define I2C_EMDR 0x2c
#define I2C_PSC  0x30

#define I2C_IVR_NACK 0x02
#define I2C_IVR_ARDY 0x03
#define I2C_IVR_RDR  0x04
#define I2C_IVR_XRDY 0x05
#define I2C_IVR_SCD  0x06
#define I2C_STR_NACK (1u << 1)
#define I2C_STR_ARDY (1u << 2)
#define I2C_STR_RRDY (1u << 3)
#define I2C_STR_XRDY (1u << 4)
#define I2C_STR_SCD  (1u << 5)
#define I2C_MDR_STP  (1u << 11)
#define I2C_MDR_STT  (1u << 13)
#define I2C_MDR_MST  (1u << 10)
#define I2C_MDR_TRX  (1u << 9)
#define I2C_MDR_IRS  (1u << 5)
#define I2C_IMR_NACK (1u << 1)

struct DA850I2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;
    uint32_t oar;
    uint32_t imr;
    uint32_t status;
    uint32_t clkl;
    uint32_t clkh;
    uint32_t count;
    uint32_t sar;
    uint32_t mdr;
    uint32_t ivr;
    uint32_t emdr;
    uint32_t psc;
    uint8_t data;
    uint32_t remaining;
    bool transfer_active;
    bool tx_ready;
};

struct DA850I2CSlaveState {
    I2CSlave parent_obj;
    uint8_t data[256];
    uint8_t offset;
    bool offset_valid;
};

static int da850_i2c_slave_send(I2CSlave *i2c, uint8_t value)
{
    DA850I2CSlaveState *s = DA850_I2C_SLAVE(i2c);

    if (!s->offset_valid) {
        s->offset = value;
        s->offset_valid = true;
    } else {
        s->data[s->offset++] = value;
    }
    return 0;
}

static uint8_t da850_i2c_slave_recv(I2CSlave *i2c)
{
    DA850I2CSlaveState *s = DA850_I2C_SLAVE(i2c);
    return s->data[s->offset++];
}

static int da850_i2c_slave_event(I2CSlave *i2c, enum i2c_event event)
{
    DA850I2CSlaveState *s = DA850_I2C_SLAVE(i2c);

    if (event == I2C_START_SEND) {
        s->offset_valid = false;
    }
    return 0;
}

static void da850_i2c_update_irq(DA850I2CState *s)
{
    qemu_set_irq(s->irq, (s->status & s->imr) != 0);
}

static uint64_t da850_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    DA850I2CState *s = opaque;

    switch (offset) {
    case I2C_OAR: return s->oar;
    case I2C_IMR: return s->imr;
    case I2C_STR: return s->status;
    case I2C_CLKL: return s->clkl;
    case I2C_CLKH: return s->clkh;
    case I2C_CNT: return s->count;
    case I2C_DRR:
        if (s->transfer_active && !(s->mdr & I2C_MDR_TRX) &&
            s->remaining) {
            if (--s->remaining == 0) {
                if (s->mdr & I2C_MDR_STP) {
                    s->status = I2C_STR_SCD;
                    s->ivr = I2C_IVR_SCD;
                }
            } else {
                s->status = I2C_STR_RRDY;
                s->ivr = I2C_IVR_RDR;
            }
            da850_i2c_update_irq(s);
        }
        return s->data;
    case I2C_SAR: return s->sar;
    case I2C_DXR: return s->data;
    case I2C_MDR: return s->mdr;
    case I2C_IVR: {
        uint32_t value = s->ivr;
        s->ivr = 0;
        s->status = 0;
        da850_i2c_update_irq(s);
        return value;
    }
    case I2C_EMDR: return s->emdr;
    case I2C_PSC: return s->psc;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "da850-i2c: unimplemented read at 0x%03" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void da850_i2c_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    DA850I2CState *s = opaque;

    switch (offset) {
    case I2C_OAR: s->oar = value; break;
    case I2C_IMR: s->imr = value; da850_i2c_update_irq(s); break;
    case I2C_STR:
        s->status &= ~(value & (I2C_STR_NACK | I2C_STR_ARDY |
                                I2C_STR_RRDY | I2C_STR_XRDY));
        da850_i2c_update_irq(s);
        break;
    case I2C_CLKL: s->clkl = value; break;
    case I2C_CLKH: s->clkh = value; break;
    case I2C_CNT: s->count = value; break;
    case I2C_SAR: s->sar = value; break;
    case I2C_DXR:
        s->data = value;
        if (s->transfer_active && (s->mdr & I2C_MDR_TRX) &&
            s->tx_ready) {
            s->tx_ready = false;
            if (i2c_send(s->bus, s->data)) {
                s->status = I2C_STR_NACK;
                s->ivr = I2C_IVR_NACK;
            } else if (--s->remaining == 0) {
                if (s->mdr & I2C_MDR_STP) {
                    s->status = I2C_STR_SCD;
                    s->ivr = I2C_IVR_SCD;
                } else {
                    s->status = I2C_STR_ARDY;
                    s->ivr = I2C_IVR_ARDY;
                }
            }
            da850_i2c_update_irq(s);
        }
        break;
    case I2C_MDR:
        s->mdr = value;
        if ((value & (I2C_MDR_STT | I2C_MDR_MST)) ==
            (I2C_MDR_STT | I2C_MDR_MST)) {
            bool read = !(value & I2C_MDR_TRX);
            int nack = i2c_start_transfer(s->bus, s->sar & 0x7f, read);

            if (nack) {
                s->status |= I2C_STR_NACK;
                s->ivr = I2C_IVR_NACK;
            } else if (read) {
                s->transfer_active = true;
                s->remaining = s->count;
                s->data = i2c_recv(s->bus);
                s->status |= I2C_STR_RRDY;
                s->ivr = I2C_IVR_RDR;
            } else {
                s->transfer_active = true;
                s->remaining = s->count;
                nack = i2c_send(s->bus, s->data);
                if (nack) {
                    s->status |= I2C_STR_NACK;
                    s->ivr = I2C_IVR_NACK;
                } else if (--s->remaining != 0) {
                    s->tx_ready = true;
                    s->tx_ready = true;
                    s->status |= I2C_STR_XRDY;
                    s->ivr = I2C_IVR_XRDY;
                } else {
                    if (s->mdr & I2C_MDR_STP) {
                        s->status |= I2C_STR_SCD;
                        s->ivr = I2C_IVR_SCD;
                    } else {
                        s->status |= I2C_STR_ARDY;
                        s->ivr = I2C_IVR_ARDY;
                    }
                }
            }
            da850_i2c_update_irq(s);
        }
        break;
    case I2C_EMDR: s->emdr = value; break;
    case I2C_PSC: s->psc = value; break;
    default:
        break;
    }
}

static const MemoryRegionOps da850_i2c_ops = {
    .read = da850_i2c_read,
    .write = da850_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void da850_i2c_reset(DeviceState *dev)
{
    DA850I2CState *s = DA850_I2C(dev);
    s->oar = 0;
    s->imr = 0;
    s->status = 0;
    s->count = 0;
    s->sar = 0;
    s->mdr = 0;
    s->ivr = 0;
    s->data = 0;
    s->remaining = 0;
    s->transfer_active = false;
    s->tx_ready = false;
}

static void da850_i2c_init(Object *obj)
{
    DA850I2CState *s = DA850_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &da850_i2c_ops, s,
                          "da850-i2c", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), NULL);
    i2c_slave_create_simple(s->bus, TYPE_DA850_I2C_SLAVE, 0x50);
    i2c_slave_create_simple(s->bus, TYPE_DA850_I2C_SLAVE, 0x54);
    i2c_slave_create_simple(s->bus, TYPE_DA850_I2C_SLAVE, 0x55);
    i2c_slave_create_simple(s->bus, TYPE_DA850_I2C_SLAVE, 0x56);
    i2c_slave_create_simple(s->bus, TYPE_DA850_I2C_SLAVE, 0x57);
}

static const VMStateDescription vmstate_da850_i2c = {
    .name = "da850-i2c",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(oar, DA850I2CState),
        VMSTATE_UINT32(imr, DA850I2CState),
        VMSTATE_UINT32(status, DA850I2CState),
        VMSTATE_UINT32(clkl, DA850I2CState),
        VMSTATE_UINT32(clkh, DA850I2CState),
        VMSTATE_UINT32(count, DA850I2CState),
        VMSTATE_UINT32(sar, DA850I2CState),
        VMSTATE_UINT32(mdr, DA850I2CState),
        VMSTATE_UINT32(ivr, DA850I2CState),
        VMSTATE_UINT32(emdr, DA850I2CState),
        VMSTATE_UINT32(psc, DA850I2CState),
        VMSTATE_UINT8(data, DA850I2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void da850_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = da850_i2c_reset;
    dc->vmsd = &vmstate_da850_i2c;
}

static void da850_i2c_slave_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    sc->send = da850_i2c_slave_send;
    sc->recv = da850_i2c_slave_recv;
    sc->event = da850_i2c_slave_event;
}

static const TypeInfo da850_i2c_info = {
    .name = TYPE_DA850_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DA850I2CState),
    .instance_init = da850_i2c_init,
    .class_init = da850_i2c_class_init,
};

static const TypeInfo da850_i2c_slave_info = {
    .name = TYPE_DA850_I2C_SLAVE,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(DA850I2CSlaveState),
    .class_init = da850_i2c_slave_class_init,
};

static void da850_i2c_register_types(void)
{
    type_register_static(&da850_i2c_info);
    type_register_static(&da850_i2c_slave_info);
}

type_init(da850_i2c_register_types)
