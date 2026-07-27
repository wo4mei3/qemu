#if 0
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
#endif

/*
 * TI DA850/OMAP-L138 64-bit timer, modeled as two independent 32-bit
 * unchained up-counters (TIM12/PRD12 and TIM34/PRD34), matching how
 * arch/arm/mach-davinci/time.c drives this hardware.
 *
 * This model actually advances the counters using QEMU_CLOCK_VIRTUAL,
 * which is required because the kernel's clocksource (TIM34 on timer0)
 * must show real elapsed time, and the clockevent (TIM12 on timer0)
 * must fire an AINTC interrupt when the counter reaches PRD12.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/irq.h"

#define TYPE_DA850_TIMER "da850-timer"
OBJECT_DECLARE_SIMPLE_TYPE(DA850TimerState, DA850_TIMER)

/* DA850 timer runs off a 24MHz reference in this board model. */
#define DA850_TIMER_FREQ_HZ 24000000ULL

/* Register byte offsets (matches arch/arm/mach-davinci/time.c) */
#define REG_PID12    0x00
#define REG_TIM12    0x10
#define REG_TIM34    0x14
#define REG_PRD12    0x18
#define REG_PRD34    0x1c
#define REG_TCR      0x20
#define REG_TGCR     0x24
#define REG_WDTCR    0x28
#define REG_CMP12_0  0x60
#define REG_CMP12_7  0x7c

/* TCR ENAMODE fields */
#define TCR_ENAMODE_BOTTOM_SHIFT 6
#define TCR_ENAMODE_TOP_SHIFT    22
#define TCR_ENAMODE_MASK         0x3

enum {
    ENAMODE_DISABLE  = 0,
    ENAMODE_ONESHOT  = 1,
    ENAMODE_PERIODIC = 2,
};

/* TGCR reset fields */
#define TGCR_TIM12RS_BIT 0
#define TGCR_TIM34RS_BIT 1

typedef struct DA850SubTimer {
    uint32_t period;      /* PRDxx register: counter wraps at period+1 */
    uint32_t base_val;    /* counter value at reference time base_ns */
    int64_t  base_ns;     /* QEMU_CLOCK_VIRTUAL time base_val was true */
    uint8_t  enamode;     /* ENAMODE_* */
    bool     unreset;     /* TGCR RS bit: true = counting allowed */
    bool     running;     /* unreset && enamode != DISABLE */
    QEMUTimer *qtimer;
    qemu_irq irq;
} DA850SubTimer;

typedef struct DA850TimerState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    DA850SubTimer bottom; /* TIM12 / PRD12 */
    DA850SubTimer top;    /* TIM34 / PRD34 */

    uint32_t tgcr;
    uint32_t wdtcr;
    uint32_t cmp12[8];
} DA850TimerState;

static uint64_t sub_get_counter(DA850SubTimer *s, int64_t now_ns)
{
    uint64_t divisor;
    uint64_t elapsed_ticks;
    uint64_t val;

    if (!s->running) {
        return s->base_val;
    }

    elapsed_ticks = (uint64_t)(now_ns - s->base_ns) * DA850_TIMER_FREQ_HZ
                    / 1000000000ULL;
    val = (uint64_t)s->base_val + elapsed_ticks;

    divisor = (uint64_t)s->period + 1; /* 0xffffffff -> 2^32, natural wrap */
    val %= divisor;

    return val;
}

static void sub_schedule_next(DA850SubTimer *s)
{
    int64_t now;
    uint64_t cur, divisor, remaining_ticks, remaining_ns;

    if (!s->running) {
        timer_del(s->qtimer);
        return;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    cur = sub_get_counter(s, now);
    divisor = (uint64_t)s->period + 1;

    remaining_ticks = divisor - cur;
    if (remaining_ticks == 0) {
        remaining_ticks = divisor;
    }
    remaining_ns = remaining_ticks * 1000000000ULL / DA850_TIMER_FREQ_HZ;
    if (remaining_ns == 0) {
        remaining_ns = 1;
    }
  fprintf(stderr, "[TIMER SCHED] period=%u cur=%lu remaining_ns=%lu fire_at=%ld\n",
            s->period, (unsigned long)cur, (unsigned long)remaining_ns,
            (long)(now + (int64_t)remaining_ns));
    timer_mod(s->qtimer, now + (int64_t)remaining_ns);
}

/* Freeze the counter's current value into base_val/base_ns (used before
 * disabling, or before changing period/enamode) so the visible count
 * doesn't jump. */
static void sub_freeze(DA850SubTimer *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_val = (uint32_t)sub_get_counter(s, now);
    s->base_ns = now;
}

static void sub_update_running(DA850SubTimer *s)
{
    bool want_running = s->unreset && (s->enamode != ENAMODE_DISABLE);

    if (want_running == s->running) {
        if (s->running) {
            sub_schedule_next(s);
        }
        return;
    }

    if (want_running) {
        /* transitioning stopped -> running: resume counting from
         * whatever base_val currently holds (typically 0, since the
         * kernel always writes TIMx=0 right before enabling). */
        s->running = true;
        s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        sub_schedule_next(s);
    } else {
        /* transitioning running -> stopped: snapshot value, stop timer */
        sub_freeze(s);
        s->running = false;
        timer_del(s->qtimer);
    }
}

static void sub_timer_cb(void *opaque)
{
    DA850SubTimer *s = opaque;
fprintf(stderr, "[TIMER FIRE] enamode=%d\n", s->enamode);

    if (!s->running) {
        return;
    }

    /* Counter has reached period and wrapped to 0. */
    s->base_val = 0;
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    qemu_irq_pulse(s->irq);

    if (s->enamode == ENAMODE_PERIODIC) {
        sub_schedule_next(s);
    } else {
        /* ONESHOT: hardware keeps counting (no further match) until
         * software reprograms PRD/TCR again; do not re-arm qtimer. */
    }
}

static void sub_init(DA850SubTimer *s, qemu_irq irq)
{
    s->period = 0xffffffff;
    s->base_val = 0;
    s->base_ns = 0;
    s->enamode = ENAMODE_DISABLE;
    s->unreset = false;
    s->running = false;
    s->irq = irq;
    s->qtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sub_timer_cb, s);
}

static void sub_write_tim(DA850SubTimer *s, uint32_t val)
{
    timer_del(s->qtimer);
    s->base_val = val;
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->running) {
        sub_schedule_next(s);
    }
}

static void sub_write_prd(DA850SubTimer *s, uint32_t val)
{
    if (s->running) {
        sub_freeze(s);
    }
    s->period = val;
    if (s->running) {
        sub_schedule_next(s);
    }
}

static uint64_t da850_timer_read(void *opaque, hwaddr addr, unsigned size)
{
    DA850TimerState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    switch (addr) {
    case REG_PID12:
        return 0x4AF90000; /* arbitrary but nonzero revision id */

    case REG_TIM12:
        return sub_get_counter(&s->bottom, now);
    case REG_TIM34:
        return sub_get_counter(&s->top, now);

    case REG_PRD12:
        return s->bottom.period;
    case REG_PRD34:
        return s->top.period;

    case REG_TCR:
        return ((uint32_t)s->bottom.enamode << TCR_ENAMODE_BOTTOM_SHIFT) |
               ((uint32_t)s->top.enamode << TCR_ENAMODE_TOP_SHIFT);

    case REG_TGCR:
        return s->tgcr;

    case REG_WDTCR:
        return s->wdtcr;

    default:
        if (addr >= REG_CMP12_0 && addr <= REG_CMP12_7) {
            return s->cmp12[(addr - REG_CMP12_0) / 4];
        }
        return 0;
    }
}

static void da850_timer_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    DA850TimerState *s = opaque;

        fprintf(stderr, "[TIMER WRITE] addr=0x%02lx val=0x%08lx\n",
            (unsigned long)addr, (unsigned long)val);

    switch (addr) {
    case REG_TIM12:
        sub_write_tim(&s->bottom, (uint32_t)val);
        break;
    case REG_TIM34:
        sub_write_tim(&s->top, (uint32_t)val);
        break;

    case REG_PRD12:
        sub_write_prd(&s->bottom, (uint32_t)val);
        break;
    case REG_PRD34:
        sub_write_prd(&s->top, (uint32_t)val);
        break;

    case REG_TCR:
        s->bottom.enamode = (val >> TCR_ENAMODE_BOTTOM_SHIFT) & TCR_ENAMODE_MASK;
        s->top.enamode    = (val >> TCR_ENAMODE_TOP_SHIFT) & TCR_ENAMODE_MASK;
        sub_update_running(&s->bottom);
        sub_update_running(&s->top);
        break;

    case REG_TGCR:
        s->tgcr = (uint32_t)val;
        s->bottom.unreset = (val >> TGCR_TIM12RS_BIT) & 1;
        s->top.unreset    = (val >> TGCR_TIM34RS_BIT) & 1;
        sub_update_running(&s->bottom);
        sub_update_running(&s->top);
        break;

    case REG_WDTCR:
        s->wdtcr = (uint32_t)val;
        break;

    default:
        if (addr >= REG_CMP12_0 && addr <= REG_CMP12_7) {
            s->cmp12[(addr - REG_CMP12_0) / 4] = (uint32_t)val;
        }
        break;
    }
}

static const MemoryRegionOps da850_timer_ops = {
    .read = da850_timer_read,
    .write = da850_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void da850_timer_init(Object *obj)
{
    DA850TimerState *s = DA850_TIMER(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &da850_timer_ops,
                           s, "da850-timer", 0x100);
    sysbus_init_mmio(dev, &s->mmio);

    /* irq[0] = bottom half (TINT12), irq[1] = top half (TINT34) */
    sysbus_init_irq(dev, &s->bottom.irq);
    sysbus_init_irq(dev, &s->top.irq);
}

static void da850_timer_realize(DeviceState *dev, Error **errp)
{
    DA850TimerState *s = DA850_TIMER(dev);

    /* sub_init() re-takes the irq pointers set up in instance_init via
     * sysbus_init_irq; re-fetch them here since sub_init() also creates
     * the QEMUTimer. */
    qemu_irq bottom_irq = s->bottom.irq;
    qemu_irq top_irq = s->top.irq;

    sub_init(&s->bottom, bottom_irq);
    sub_init(&s->top, top_irq);

    s->tgcr = 0;
    s->wdtcr = 0;
    memset(s->cmp12, 0, sizeof(s->cmp12));
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