
#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/irq.h"

#define TYPE_DA850_TIMER "da850-timer"
OBJECT_DECLARE_SIMPLE_TYPE(DA850TimerState, DA850_TIMER)

typedef struct DA850TimerState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    QEMUTimer *timer;
    qemu_irq irq;

    uint32_t prd12;
    uint32_t enable;
    uint32_t intctlstat;

} DA850TimerState;

/* IRQ update */
static void da850_timer_irq(DA850TimerState *s)
{
    qemu_set_irq(s->irq, (s->enable && (s->intctlstat & 0x2)) ? 1 : 0);
}

/* forward prototype for GPIO input handler */
static void da850_timer_ack(void *opaque, int n, int level);

static void da850_timer_ack(void *opaque, int n, int level)
{
    DA850TimerState *s = opaque;

    if (level == 0) {
        s->intctlstat &= ~0x2;
        da850_timer_irq(s);
    }
}

/* timer fire */
static void da850_timer_cb(void *opaque)
{
    DA850TimerState *s = opaque;

    if (!s->enable || s->prd12 == 0)
        return;

    /* Interrupt is still pending; do not re-assert the line repeatedly. */
    if (s->intctlstat & 0x2) {
        uint64_t ns = ((uint64_t)s->prd12 * 1000ULL) / 24ULL;
        if (ns < 1000000ULL) {
            ns = 1000000ULL;
        }
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
        return;
    }

    s->intctlstat |= 0x2;
    da850_timer_irq(s);

    /* DA850 timer clock approximation: 24MHz, period in timer ticks. */
    uint64_t ns = ((uint64_t)s->prd12 * 1000ULL) / 24ULL;
    if (ns < 1000000ULL) {
        ns = 1000000ULL;
    }
    timer_mod(s->timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

/* read */
static uint64_t da850_timer_read(void *opaque, hwaddr addr, unsigned size)
{
    DA850TimerState *s = opaque;

    switch (addr) {
    case 0x18: /* PRD12 */
        return s->prd12;
    case 0x20: /* TCR/enable proxy */
        return s->enable;
    case 0x2c: /* INTCTLSTAT */
        return s->intctlstat;
    default:
        return 0;
    }
}

/* write */
static void da850_timer_write(void *opaque, hwaddr addr,
                              uint64_t val, unsigned size)
{
    DA850TimerState *s = opaque;

    switch (addr) {
    case 0x18: /* PRD12 */
        s->prd12 = val;
        break;

    case 0x20: /* TCR/enable proxy */
        s->enable = val ? 1 : 0;

        if (s->enable && s->prd12) {
            uint64_t ns = ((uint64_t)s->prd12 * 1000ULL) / 24ULL;
            if (ns < 1000000ULL) {
                ns = 1000000ULL;
            }
            timer_mod(s->timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
        }
        break;

    case 0x2c: /* INTCTLSTAT W1C */
        s->intctlstat &= ~(val & 0x2);
        break;
    }

    da850_timer_irq(s);
}

static const MemoryRegionOps da850_timer_ops = {
    .read = da850_timer_read,
    .write = da850_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void da850_timer_init(Object *obj)
{
    DA850TimerState *s = DA850_TIMER(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &da850_timer_ops,
                          s, "da850-timer", 0x100);

    sysbus_init_mmio(dev, &s->mmio);
    sysbus_init_irq(dev, &s->irq);
}

static void da850_timer_realize(DeviceState *dev, Error **errp)
{
    DA850TimerState *s = DA850_TIMER(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                            da850_timer_cb, s);
    qdev_init_gpio_in(dev, da850_timer_ack, 1);
}

static void da850_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = da850_timer_realize;
}

static const TypeInfo da850_timer_info = {
    .name = TYPE_DA850_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DA850TimerState),
    .instance_init = da850_timer_init,
    .class_init = da850_timer_class_init,
};

static void da850_timer_register(void)
{
    type_register_static(&da850_timer_info);
}

type_init(da850_timer_register);