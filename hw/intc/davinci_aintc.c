#if 0
#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "exec/memory.h"
#include "qemu/log.h"

#define TYPE_DAVINCI_AINTC "davinci-aintc"
OBJECT_DECLARE_SIMPLE_TYPE(AINTCState, DAVINCI_AINTC)

#define NR_IRQS 101 /* DA850/AM1808 は 101個の割り込み */

/* AM1808 AINTC Register Offsets */
#define REG_REVID     0x000
#define REG_CR        0x004
#define REG_GER       0x010 /* Global Enable Register */
#define REG_SISR      0x020 /* System Interrupt Status Set Register (Index) */
#define REG_SICR      0x024 /* System Interrupt Status Clear Register (Index) */
#define REG_EISR      0x028 /* Enable Interrupt Set Register (Index) */
#define REG_EICR      0x02C /* Enable Interrupt Clear Register (Index) */
#define REG_HPIER     0x030
#define REG_GPIR      0x080 /* Global Prioritized Index Register */

struct AINTCState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq cpu_irq;
    
    bool pending[NR_IRQS];
    bool enable[NR_IRQS];
    bool global_enable;
    qemu_irq timer_ack_out; /* タイマーへの逆配線用出力ピン */
};

/* aintc_update: 割り込み状態を CPU に反映させる共通関数 
static void aintc_update(AINTCState *s)
{
    bool pending = false;
    
    // 全体の有効化フラグが立っていない場合は即座に 0
    if (!s->global_enable) {
        qemu_set_irq(s->cpu_irq, 0);
        return;
    }

    // いずれかの有効な IRQ がペンディングされているか確認
    for (int i = 0; i < NR_IRQS; i++) {
        if (s->pending[i] && s->enable[i]) {
            pending = true;
            break;
        }
    }
    qemu_set_irq(s->cpu_irq, pending ? 1 : 0);
}*/
static void aintc_update(AINTCState *s)
{
    bool pending = false;

    /* 有効かつ未処理の割り込みが1つでもあるかチェック */
    for (int i = 0; i < NR_IRQS; i++) {
        if (s->pending[i] && s->enable[i]) {
            pending = true;
            break;
        }
    }

    /* * ここが肝：pending が false なら、
     * QEMU の CPU オブジェクトに対して「物理線が 0 になったぞ」と即座に伝える。
     */
    qemu_set_irq(s->cpu_irq, pending ? 1 : 0);
}
static void aintc_set_irq(void *opaque, int irq, int level)
{
    if (level) {
        fprintf(stderr, "[AINTC DEBUG] IRQ %d raised by external device\n", irq);
    }
    AINTCState *s = opaque;
    if (irq < NR_IRQS) {
        s->pending[irq] = !!level;
        aintc_update(s);
    }
}

static uint64_t aintc_read(void *opaque, hwaddr addr, unsigned size)
{
    AINTCState *s = opaque;
    uint64_t ret = 0;

    switch (addr) {
    case 0x000: // REVID
        ret = 0x4E82A900;
        break;
    case 0x004: // CR (Control Register)
        ret = 0;
        break;
    case 0x010: // GER (Global Enable Register)
        ret = s->global_enable;
        break;
    case 0x080: // GPIR (IRQ 番号取得)
        ret = 0x800003FF; // 割り込みなしのデフォルト値
        for (int i = 0; i < NR_IRQS; i++) {
            if (s->pending[i] && s->enable[i]) {
                ret = i;
                break;
            }
        }
        break;

        
    /* 割り込みステータスレジスタ (SRSR1-4) */
    case 0x200: case 0x280: // SRSR1 (0-31)
        for (int i = 0; i < 32; i++) {
            if (s->pending[i]) ret |= (1U << i);
        }
        break;
    case 0x204: case 0x284: // SRSR2 (32-63)
        for (int i = 0; i < 32; i++) {
            if (s->pending[i + 32]) ret |= (1U << i);
        }
        break;
    case 0x208: case 0x288: // SRSR3 (64-95)
        for (int i = 0; i < 32; i++) {
            if (s->pending[i + 64]) ret |= (1U << i);
        }
        break;
    case 0x20c: case 0x28c: // SRSR4 (96-100+)
        for (int i = 0; i < 32; i++) {
            if ((i + 96) < NR_IRQS && s->pending[i + 96]) ret |= (1U << i);
        }
        break;
    default:
        break;
    }

    return ret;
}

static void aintc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AINTCState *s = opaque;
    // デバッグ用
    fprintf(stderr, "[AINTC WRITE] offset=0x%03lx, val=0x%lx\n", (unsigned long)addr, (unsigned long)val);

    switch (addr) {
    case 0x024: // SICR (System Interrupt Status Indexed Clear Register) ★これ！
        if (val < NR_IRQS) {
            s->pending[val] = false;
            //qemu_set_irq(qdev_get_gpio_in(DEVICE(s), val), 0);
            if (val == 21 || val == 22 || val == 24) {
                /* タイマーへの逆配線に 0 を送出 */
                s->pending[21] = false;
                s->pending[22] = false;
                s->pending[24] = false;
                qemu_set_irq(s->timer_ack_out, 0);
                qemu_set_irq(s->cpu_irq, 0);
            }
            aintc_update(s);
            fprintf(stderr, "[AINTC WRITE] SICR cleared IRQ %lu\n", (unsigned long)val);
        }
        break;

    case 0x028: // EISR (Enable Indexed Set Register)
        if (val < NR_IRQS) {
            s->enable[val] = true;
        }
        break;

    case 0x010: // GER (Global Enable Register)
        s->global_enable = (val & 1);
        break;

    case 0x280: case 0x300: // ECR (Event Clear Register)
        for (int i = 0; i < 32; i++) {
            if (val & (1U << i)) s->pending[i] = false;
        }
        break;

    /* 他のオフセットは既存の通り */
    }

    /* 状態が変わったので CPU への信号線を更新 */
    aintc_update(s);
}
static const MemoryRegionOps aintc_ops = {
    .read = aintc_read,
    .write = aintc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void aintc_init(Object *obj)
{
    AINTCState *s = DAVINCI_AINTC(obj);
    /* 初期状態ではすべて無効 */
    memset(s->pending, 0, sizeof(s->pending));
    memset(s->enable, 0, sizeof(s->enable));
    s->global_enable = false;

    memory_region_init_io(&s->mmio, obj, &aintc_ops, s, "aintc", 0x1000);
}

static void aintc_realize(DeviceState *dev, Error **errp)
{
    AINTCState *s = DAVINCI_AINTC(dev);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cpu_irq);
    /* 外部（タイマー等）からの入力ピンを作成 */
    qdev_init_gpio_in(dev, aintc_set_irq, NR_IRQS);
    qdev_init_gpio_out_named(dev, &s->timer_ack_out, "timer_ack", 1);
}

static void aintc_class_init(ObjectClass *klass, void *data)
{
    DEVICE_CLASS(klass)->realize = aintc_realize;
}

static const TypeInfo aintc_info = {
    .name          = TYPE_DAVINCI_AINTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AINTCState),
    .instance_init = aintc_init,
    .class_init    = aintc_class_init,
};

static void aintc_register(void)
{
    type_register_static(&aintc_info);
}

type_init(aintc_register);
#endif

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "exec/memory.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"

#define TYPE_DAVINCI_AINTC "davinci-aintc"
OBJECT_DECLARE_SIMPLE_TYPE(AINTCState, DAVINCI_AINTC)

#define NR_IRQS 101 

struct AINTCState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq cpu_irq;
    
    bool pending[NR_IRQS];
    bool enable[NR_IRQS];
    bool global_enable;
    qemu_irq timer_ack_out; 
};

static void aintc_update(AINTCState *s)
{
    bool pending = false;

    /* Global Enable が 0 なら割り込みを上げない */
    if (!s->global_enable) {
        qemu_set_irq(s->cpu_irq, 0);
        return;
    }

    for (int i = 0; i < NR_IRQS; i++) {
        if (s->pending[i] && s->enable[i]) {
            pending = true;
            break;
        }
    }

    qemu_set_irq(s->cpu_irq, pending ? 1 : 0);
}

static void aintc_set_irq(void *opaque, int irq, int level)
{
    AINTCState *s = opaque;
    if (irq < NR_IRQS) {
        /* AINTC はラッチ型として扱い、clear は SICR 側でのみ行う */
        if (level) {
            s->pending[irq] = true;
        }
        aintc_update(s);
    }
}



#if 0
static void aintc_set_irq(void *opaque, int irq, int level)
{
    AINTCState *s = opaque;
    if (irq >= NR_IRQS) return;

    if (level) {
        /* ★ここが防波堤：すでにPendingなら、タイマーが何度叩いても無視する */
        /*if (s->pending[irq]) {
            return; 
        }*/
        s->pending[irq] = true;
        // fprintf(stderr, "[AINTC DEBUG] IRQ %d raised\n", irq);
    } else {
        /* 明示的に0が来た時だけ落とす */
        s->pending[irq] = false;
    }
    aintc_update(s);
}
#endif 
static uint64_t aintc_read(void *opaque, hwaddr addr, unsigned size)
{
    AINTCState *s = opaque;
    uint64_t ret = 0;

    switch (addr) {
    case 0x000: ret = 0x4E82A900; break; // REVID
    case 0x010: ret = s->global_enable; break; // GER
    case 0x080: // GPIR
        ret = 0x800003FF;
        for (int i = 0; i < NR_IRQS; i++) {
            if (s->pending[i] && s->enable[i]) {
                ret = i;
                break;
            }
        }
        break;
    /* ステータス読み出し (SRSR) */
    case 0x200: case 0x280:
        for (int i = 0; i < 32; i++) if (s->pending[i]) ret |= (1U << i);
        break;
    case 0x204: case 0x284:
        for (int i = 0; i < 32; i++) if (s->pending[i + 32]) ret |= (1U << i);
        break;
    case 0x208: case 0x288:
        for (int i = 0; i < 32; i++) if (s->pending[i + 64]) ret |= (1U << i);
        break;
    case 0x20c: case 0x28c:
        for (int i = 0; i < 32; i++) if ((i + 96) < NR_IRQS && s->pending[i + 96]) ret |= (1U << i);
        break;
    }
    return ret;
}

#if 0
static void aintc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AINTCState *s = opaque;

    switch (addr) {
    case 0x024: // SICR (Clear)
        if (val < NR_IRQS) {
            s->pending[val] = false;
            
            /* ★重要：ACK通知を送り、CPU線を一旦強制的に下げる */
            qemu_set_irq(s->cpu_irq, 0); 
            if (val == 21 || val == 22 || val == 24) {
                qemu_set_irq(s->timer_ack_out, 1);
                qemu_set_irq(s->timer_ack_out, 0);
            }
        }
        break;

    case 0x028: // EISR (Enable Set)
        if (val < NR_IRQS) s->enable[val] = true;
        break;

    case 0x02C: // EICR (Enable Clear)
        if (val < NR_IRQS) s->enable[val] = false;
        break;

    case 0x010: // GER
        s->global_enable = (val & 1);
        break;
    }
    aintc_update(s);
}
#endif

static void aintc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AINTCState *s = opaque;

    //if (!(addr == 0x024 && val == 21)) {
        //fprintf(stderr, "[AINTC WRITE] addr=0x%03lx val=0x%08lx (GER=%d)\n",
         //       (unsigned long)addr, (unsigned long)val, s->global_enable);
    //}

    switch (addr) {
    case 0x024: // SICR (Clear)
        if (val < NR_IRQS) {
            s->pending[val] = false;
            
            /* ★重要：ACK通知を送り、CPU線を一旦強制的に下げる */
            qemu_set_irq(s->cpu_irq, 0); 
            if (val == 21 || val == 22 || val == 24) {
                qemu_set_irq(s->timer_ack_out, 1);
                qemu_set_irq(s->timer_ack_out, 0);
            }
        }
        break;

    case 0x028: // EISR (Enable Set)
        if (val < NR_IRQS) s->enable[val] = true;
        break;

    case 0x02C: // EICR (Enable Clear)
        if (val < NR_IRQS) s->enable[val] = false;
        break;

    case 0x010: // GER
    fprintf(stderr, "AINTC: GER WRITE = %d\n", (int)(val & 1));

        s->global_enable = (val & 1);
        break;
    }
    aintc_update(s);
}

static const MemoryRegionOps aintc_ops = {
    .read = aintc_read,
    .write = aintc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void aintc_init(Object *obj)
{
    AINTCState *s = DAVINCI_AINTC(obj);
    memset(s->pending, 0, sizeof(s->pending));
    memset(s->enable, 0, sizeof(s->enable));
    s->global_enable = false;

    memory_region_init_io(&s->mmio, obj, &aintc_ops, s, "aintc", 0x1000);
}

static void aintc_realize(DeviceState *dev, Error **errp)
{
    AINTCState *s = DAVINCI_AINTC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->cpu_irq);
    
    /* コンパイルエラー対策：第1引数は dev (DeviceState*) でOK */
    qdev_init_gpio_in(dev, aintc_set_irq, NR_IRQS);
    qdev_init_gpio_out_named(dev, &s->timer_ack_out, "timer_ack", 1);
}

static void aintc_class_init(ObjectClass *klass, void *data)
{
    DEVICE_CLASS(klass)->realize = aintc_realize;
}

static const TypeInfo aintc_info = {
    .name          = TYPE_DAVINCI_AINTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AINTCState),
    .instance_init = aintc_init,
    .class_init    = aintc_class_init,
};

static void aintc_register(void) {
    type_register_static(&aintc_info);
}
type_init(aintc_register);
