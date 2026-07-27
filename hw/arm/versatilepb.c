/*
 * ARM Versatile Platform/Application Baseboard System emulation.
 *
 * Copyright (c) 2005-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/arm/boot.h"
#include "hw/net/smc91c111.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/pci/pci.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/arm_sbcon_i2c.h"
#include "hw/irq.h"
#include "hw/boards.h"
#include "hw/block/flash.h"
#include "chardev/char.h"
#include "qemu/error-report.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/char/pl011.h"
#include "hw/sd/sd.h"
#include "qom/object.h"
#include "hw/usb/hcd-musb.h"

#define VERSATILE_FLASH_ADDR 0x34000000
#define VERSATILE_FLASH_SIZE (64 * 1024 * 1024)
#define VERSATILE_FLASH_SECT_SIZE (256 * 1024)

/* Primary interrupt controller.  */

#define TYPE_VERSATILE_PB_SIC "versatilepb_sic"
OBJECT_DECLARE_SIMPLE_TYPE(vpb_sic_state, VERSATILE_PB_SIC)

struct vpb_sic_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t level;
    uint32_t mask;
    uint32_t pic_enable;
    qemu_irq parent[32];
    int irq;
};

static const VMStateDescription vmstate_vpb_sic = {
    .name = "versatilepb_sic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(level, vpb_sic_state),
        VMSTATE_UINT32(mask, vpb_sic_state),
        VMSTATE_UINT32(pic_enable, vpb_sic_state),
        VMSTATE_END_OF_LIST()
    }
};

static void vpb_sic_update(vpb_sic_state *s)
{
    uint32_t flags;

    flags = s->level & s->mask;
    qemu_set_irq(s->parent[s->irq], flags != 0);
}

static void vpb_sic_update_pic(vpb_sic_state *s)
{
    int i;
    uint32_t mask;

    for (i = 21; i <= 30; i++) {
        mask = 1u << i;
        if (!(s->pic_enable & mask))
            continue;
        qemu_set_irq(s->parent[i], (s->level & mask) != 0);
    }
}

static void vpb_sic_set_irq(void *opaque, int irq, int level)
{
    vpb_sic_state *s = (vpb_sic_state *)opaque;
    if (level)
        s->level |= 1u << irq;
    else
        s->level &= ~(1u << irq);
    if (s->pic_enable & (1u << irq))
        qemu_set_irq(s->parent[irq], level);
    vpb_sic_update(s);
}

static uint64_t vpb_sic_read(void *opaque, hwaddr offset,
                             unsigned size)
{
    vpb_sic_state *s = (vpb_sic_state *)opaque;

    switch (offset >> 2) {
    case 0: /* STATUS */
        return s->level & s->mask;
    case 1: /* RAWSTAT */
        return s->level;
    case 2: /* ENABLE */
        return s->mask;
    case 4: /* SOFTINT */
        return s->level & 1;
    case 8: /* PICENABLE */
        return s->pic_enable;
    default:
        printf ("vpb_sic_read: Bad register offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void vpb_sic_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    vpb_sic_state *s = (vpb_sic_state *)opaque;

    switch (offset >> 2) {
    case 2: /* ENSET */
        s->mask |= value;
        break;
    case 3: /* ENCLR */
        s->mask &= ~value;
        break;
    case 4: /* SOFTINTSET */
        if (value)
            s->mask |= 1;
        break;
    case 5: /* SOFTINTCLR */
        if (value)
            s->mask &= ~1u;
        break;
    case 8: /* PICENSET */
        s->pic_enable |= (value & 0x7fe00000);
        vpb_sic_update_pic(s);
        break;
    case 9: /* PICENCLR */
        s->pic_enable &= ~value;
        vpb_sic_update_pic(s);
        break;
    default:
        printf ("vpb_sic_write: Bad register offset 0x%x\n", (int)offset);
        return;
    }
    vpb_sic_update(s);
}

static const MemoryRegionOps vpb_sic_ops = {
    .read = vpb_sic_read,
    .write = vpb_sic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void vpb_sic_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    vpb_sic_state *s = VERSATILE_PB_SIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    qdev_init_gpio_in(dev, vpb_sic_set_irq, 32);
    for (i = 0; i < 32; i++) {
        sysbus_init_irq(sbd, &s->parent[i]);
    }
    s->irq = 31;
    memory_region_init_io(&s->iomem, obj, &vpb_sic_ops, s,
                          "vpb-sic", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

/* ファイルの上部に追加 */
static uint64_t da850_syscfg_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case 0x00: /* JTAG_ID_REG */
        return 0x0B7D102F;
    case 0x18:
        return 0x1B7D102F;
    case 0x17c: /* CFGCHIP0 */
    case 0x188: /* CFGCHIP3 */
        return 0;
    default:
        return 0;
    }
}

/* 1. SYSCFG用の空のwriteを追加 */
static void da850_syscfg_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    /* 何もしない */
}

/* 2. opsに登録 */
static const MemoryRegionOps da850_syscfg_ops = {
    .read = da850_syscfg_read,
    .write = da850_syscfg_write, /* ←これを追加！ */
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* PSCのオフセット定義 */
#define PSC_PTSTAT      0x0128  /* Power Transition Status */
#define PSC_EPCPR       0x0070  /* External Power Control Status */
#define PSC_MDSTAT_BASE 0x0800  /* Module Status registers */

//static uint64_t da850_psc_read(void *opaque, hwaddr offset, unsigned size)
//{
    /* どんな ID (0x800以降) を聞かれても、とにかく「電源ON(3)」と答える */
//    if (offset >= 0x800) {
//        return 0x00000003; 
//    }

//    switch (offset) {
//        case 0x070: return 0xFFFFFFFF; /* EPCPR: Ready */
//        case 0x128: return 0x00000000; /* PTSTAT: No transition */
//        case 0x200: return 0x00000001; /* PDSTAT: Domain ON */
//        default: return 0;
//    }
//}

#if 0
static uint64_t da850_psc_read(void *opaque, hwaddr offset, unsigned size)
{
    /* DA850 の PSC は、遷移完了を待つために PTSTAT (0x128) が 0 になるのを見ます */
    if (offset == 0x128) {
        return 0; /* 常に 0 (遷移完了) */
    }

    /* 全ての LPSC (MDSTAT) を「Enabled」にする */
    if (offset >= 0x800 && offset <= 0x9FF) {
        return 0x3; /* Enabled */
    }

    /* その他、PDSTAT なども適当に「ON」を返しておく */
    return 1;
}

static uint64_t da850_psc_read(void *opaque, hwaddr offset, unsigned size)
{
    // 0x800以降は MDSTAT (各デバイスの現在の状態)
    if (offset >= 0x800 && offset <= 0x9FF) {
        /* * 期待される値: (STATE=3: Enable) | (STATUS=0: 完成)
         * 下位5ビットが状態。3なら「Enable」
         */
        return 0x00000003; 
    }

    switch (offset) {
        case 0x128: // PTSTAT (Power Transition Status)
            return 0; // 常に「遷移中ではない(完了)」と答える
        case 0x120: // PTCMD (Power Transition Command)
            return 0; // 書き込み専用に近いが、読み出しは0でOK
        default:
            return 0;
    }
}

/* write を NULL にせず、必ず空の関数を割り当てる */
static void da850_psc_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    /* 何もしない（セグフォ防止） */
}


/* PSCの内部状態を保持する（簡易版） */
static uint32_t psc_mdctl[64]; // 各モジュールの目標状態
static uint32_t psc_mdstat[64]; // 各モジュールの現在状態

/* QEMU側: da850_psc_read の修正案 */
static uint64_t da850_psc_read(void *opaque, hwaddr offset, unsigned size)
{
    // MDSTATn レジスタ (0x800 - 0x8FC)
    if (offset >= 0x800 && offset <= 0x8FC) {
        //uint32_t id = (offset - 0x800) >> 2;
        
        /* * カーネルはここを読んで、デバイスが「起きた」と判断します。
         * 0x3 = STATE: Enable
         */
        return 0x00000003; 
    }

    switch (offset) {
        case 0x128: // PTSTAT
            return 0; // 遷移完了
        case 0x070: // EPCPR
            return 1; // 電源ドメイン準備完了
        default:
            return 0;
    }
}

static void da850_psc_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    if (offset >= 0xA00 && offset <= 0xAF0) {
        int id = (offset - 0xA00) >> 2;
        psc_mdctl[id] = value & 0x1F; // NEXT状態を保存
    }
    else if (offset == 0x120) { // PTCMD: 遷移開始！
        if (value & 1) {
            /* * 本来はここで遅延処理を入れるが、QEMUでは即座に反映。
             * 全ての MDCTL の NEXT を MDSTAT の STATE にコピーする。
             */
            for (int i = 0; i < 64; i++) {
                psc_mdstat[i] = psc_mdctl[i]; 
            }
            // PTSTAT のビットを 0 (完了) のままにする、
            // もしくは一時的に 1 にして次の Read で 0 にする処理が「誠実」
        }
    }
}


static const MemoryRegionOps da850_psc_ops = {
    .read = da850_psc_read,
    .write = da850_psc_write, /* ここが重要！ */
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};
#endif
/* ================================================================= *
 * MUSB管理用のシンプルな構造体とMMIOオペレーション
 * ================================================================= */
typedef struct DA850USBState {
    MemoryRegion usb_mmio;
    MUSBState *musb;
} DA850USBState;



static uint64_t da850_usb_read(void *opaque, hwaddr addr, unsigned size)
{
    DA850USBState *s = opaque;
    switch (size) {
    case 1: return musb_read[0](s->musb, addr);
    case 2: return musb_read[1](s->musb, addr);
    case 4: return musb_read[2](s->musb, addr);
    default: return 0;
    }
}

static void da850_usb_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    DA850USBState *s = opaque;
    switch (size) {
    case 1: musb_write[0](s->musb, addr, val); break;
    case 2: musb_write[1](s->musb, addr, val); break;
    case 4: musb_write[2](s->musb, addr, val); break;
    }
}

static const MemoryRegionOps da850_usb_ops = {
    .read = da850_usb_read,
    .write = da850_usb_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* Board init.  */

/* The AB and PB boards both use the same core, just with different
   peripherals and expansion busses.  For now we emulate a subset of the
   PB peripherals and just change the board ID.  */

static struct arm_boot_info versatile_binfo;

static void versatile_init(MachineState *machine, int board_id)
{
    Object *cpuobj;
    ARMCPU *cpu;
    MemoryRegion *sysmem = get_system_memory();
  //  qemu_irq pic[32];
  //  qemu_irq sic[32];
  //  DeviceState *dev, *sysctl;
  //  SysBusDevice *busdev;
  //  DeviceState *pl041;
  //  PCIBus *pci_bus;
  //  NICInfo *nd;
  //  I2CBus *i2c;
  //  int n;
  //  int done_smc = 0;
    DriveInfo *dinfo;

    if (machine->ram_size > 0x10000000) {
        /* Device starting at address 0x10000000,
         * and memory cannot overlap with devices.
         * Refuse to run rather than behaving very confusingly.
         */
        error_report("versatilepb: memory size must not exceed 256MB");
        exit(1);
    }

    if (!machine->kernel_cmdline || !*machine->kernel_cmdline) {
        machine->kernel_cmdline = g_strdup("console=ttyS0,115200");
    }

    cpuobj = object_new(machine->cpu_type);

    /* By default ARM1176 CPUs have EL3 enabled.  This board does not
     * currently support EL3 so the CPU EL3 property is disabled before
     * realization.
     */
    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }

    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    cpu = ARM_CPU(cpuobj);

    /* ??? RAM should repeat to fill physical memory space.  */
    /* SDRAM at address zero.  */
    memory_region_add_subregion(sysmem, 0xc0000000, machine->ram);

#if 0
    sysctl = qdev_new("realview_sysctl");
    qdev_prop_set_uint32(sysctl, "sys_id", 0x41007004);
    qdev_prop_set_uint32(sysctl, "proc_id", 0x02000000);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sysctl), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sysctl), 0, 0x10000000);

    dev = sysbus_create_varargs("pl190", 0x10140000,
                                qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ),
                                qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ),
                                NULL);
    for (n = 0; n < 32; n++) {
        pic[n] = qdev_get_gpio_in(dev, n);
    }
    dev = sysbus_create_simple(TYPE_VERSATILE_PB_SIC, 0x10003000, NULL);
    for (n = 0; n < 32; n++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), n, pic[n]);
        sic[n] = qdev_get_gpio_in(dev, n);
    }

    sysbus_create_simple("pl050_keyboard", 0x10006000, sic[3]);
    sysbus_create_simple("pl050_mouse", 0x10007000, sic[4]);

    dev = qdev_new("versatile_pci");
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x10001000); /* PCI controller regs */
    sysbus_mmio_map(busdev, 1, 0x41000000); /* PCI self-config */
    sysbus_mmio_map(busdev, 2, 0x42000000); /* PCI config */
    sysbus_mmio_map(busdev, 3, 0x43000000); /* PCI I/O */
    sysbus_mmio_map(busdev, 4, 0x44000000); /* PCI memory window 1 */
    sysbus_mmio_map(busdev, 5, 0x50000000); /* PCI memory window 2 */
    sysbus_mmio_map(busdev, 6, 0x60000000); /* PCI memory window 3 */
    sysbus_connect_irq(busdev, 0, sic[27]);
    sysbus_connect_irq(busdev, 1, sic[28]);
    sysbus_connect_irq(busdev, 2, sic[29]);
    sysbus_connect_irq(busdev, 3, sic[30]);
    pci_bus = (PCIBus *)qdev_get_child_bus(dev, "pci");

    for(n = 0; n < nb_nics; n++) {
        nd = &nd_table[n];

        if (!done_smc && (!nd->model || strcmp(nd->model, "smc91c111") == 0)) {
            smc91c111_init(nd, 0x10010000, sic[25]);
            done_smc = 1;
        } else {
            pci_nic_init_nofail(nd, pci_bus, "rtl8139", NULL);
        }
    }
    if (machine_usb(machine)) {
        pci_create_simple(pci_bus, -1, "pci-ohci");
    }
    n = drive_get_max_bus(IF_SCSI);
    while (n >= 0) {
        dev = DEVICE(pci_create_simple(pci_bus, -1, "lsi53c895a"));
        lsi53c8xx_handle_legacy_cmdline(dev);
        n--;
    }

    /*
    pl011_create(0x101f1000, pic[12], serial_hd(0));
    pl011_create(0x0010c000, pic[13], serial_hd(1));
    pl011_create(0x0010d000, pic[14], serial_hd(2));
    pl011_create(0x10009000, sic[6], serial_hd(3));
    */
sysbus_realize_and_unref(aintc_bus, &error_fatal);
sysbus_mmio_map(aintc_bus, 0, 0xFFFEE000);
sysbus_connect_irq(aintc_bus, 0,

#endif

// --- DA850 AINTC 配線 ---
DeviceState *aintc = qdev_new("davinci-aintc");
SysBusDevice *aintc_bus = SYS_BUS_DEVICE(aintc);
sysbus_realize_and_unref(aintc_bus, &error_fatal);
sysbus_mmio_map(aintc_bus, 0, 0xFFFEE000);

sysbus_connect_irq(aintc_bus, 0, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));


    /* DA850 console is UART1, which Linux exposes as ttyS1 here. */
    serial_mm_init(get_system_memory(), 0x01c42000, 2, qdev_get_gpio_in(aintc, 25), 115200, serial_hd(0), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(get_system_memory(), 0x01d0c000, 2, qdev_get_gpio_in(aintc, 53), 115200, serial_hd(1), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(get_system_memory(), 0x01d0d000, 2, qdev_get_gpio_in(aintc, 61), 115200, serial_hd(2), DEVICE_LITTLE_ENDIAN);

    /* vpb_init の中に追加 */
    MemoryRegion *syscfg0 = g_new(MemoryRegion, 1);
    memory_region_init_io(syscfg0, NULL, &da850_syscfg_ops, NULL, "da850-syscfg0", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x01C14000, syscfg0);
/*
        MemoryRegion *psc0 = g_new(MemoryRegion, 1);
    memory_region_init_io(psc0, NULL, &da850_psc_ops, NULL, "da850.psc0", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x01c10000, psc0);
*/
    DeviceState *psc = qdev_new("da850-psc");
sysbus_realize_and_unref(SYS_BUS_DEVICE(psc), &error_fatal);
sysbus_mmio_map(SYS_BUS_DEVICE(psc), 0, 0x01c10000);
DeviceState *psc1 = qdev_new("da850-psc");
sysbus_realize_and_unref(SYS_BUS_DEVICE(psc1), &error_fatal);
sysbus_mmio_map(SYS_BUS_DEVICE(psc1), 0, 0x01e27000);

    /* PSC1 (UART2 等の電源管理用) 
    MemoryRegion *psc1 = g_new(MemoryRegion, 1);
    memory_region_init_io(psc1, NULL, &da850_psc_ops, NULL, "da850.psc1", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x01E27000, psc1);
    */
   /*
    DeviceState *timer0 = qdev_new("da850-timer");
    SysBusDevice *tbus0 = SYS_BUS_DEVICE(timer0);
    
    sysbus_realize_and_unref(tbus0, &error_fatal);
    sysbus_mmio_map(tbus0, 0, 0x01C20000);*/

/* ================================================================= *
     * 正しい MUSB (USBコア) の生成、MMIO転送、および割り込み配線 (決定版)
     * ================================================================= */
    /* ================================================================= *
     * 正しい MUSB (USBコア) の生成、MMIO転送、および割り込み配線 (完成版)
     * ================================================================= */
/* ================================================================= *
     * 正しい MUSB (USBコア) の生成、MMIO転送、および割り込み配線 (修正版)
     * ================================================================= */
   /* ================================================================= *
     * 正しい MUSB (USBコア) の生成、MMIO転送、および割り込み配線 (完全版)
     * ================================================================= */
    
/* ================================================================= *
     * 正しい MUSB (USBコア) の生成、MMIO転送、および割り込み配線 (修正版)
     * ================================================================= */
  /* ================================================================= *
     * 正しい MUSB (USBコア) の生成、MMIO転送、および割り込み配線 (完全決定版)
     * ================================================================= */
    {
        DA850USBState *usb_state = g_new0(DA850USBState, 1);

        /* 【アサーション＆コアダンプ完全回避の核心】
         * musb_init() は内部で以下のように動きます：
         * s->irqs[i] = qdev_get_gpio_in(parent, gpio_base + i);
         *
         * ここで、parent に「aintc」、gpio_base に「52」を渡します。
         * すると MUSB 内部で、
         * qdev_get_gpio_in(aintc, 52 + 0);
         * が呼び出されることになります。
         *
         * aintc はすでに多くの入力ピンを持って実体化している本物のデバイスなので、
         * num_in の範囲チェック（Assertion）を100%安全に一発でパスします！
         * かつ、MUSBの割り込み出力がダイレクトに AINTCの52番ピンに直接結びつきます。
         */
        usb_state->musb = musb_init(aintc, 58);

        /* MMIOの登録とアドレスマッピング */
        memory_region_init_io(&usb_state->usb_mmio, OBJECT(machine),
                              &da850_usb_ops, usb_state,
                              "da850-usb", 0x1000);
        memory_region_add_subregion(get_system_memory(), 0x01e00000, &usb_state->usb_mmio);
    }


// タイマーの出力を AINTC の 21, 22, 24 全てに突っ込む
//qemu_irq aintc_21 = qdev_get_gpio_in(aintc, 21);
//qemu_irq aintc_22 = qdev_get_gpio_in(aintc, 22);
//qemu_irq aintc_24 = qdev_get_gpio_in(aintc, 21);

// sysbus_connect_irq は 1対1 なので、手動で分配するか、
// とりあえずカーネルが今欲しがっている 24番に繋ぎ替える
//sysbus_connect_irq(tbus0, 0, aintc_24); 

DeviceState *timer0 = qdev_new("da850-timer");
SysBusDevice *tbus0 = SYS_BUS_DEVICE(timer0);

sysbus_realize_and_unref(tbus0, &error_fatal);
sysbus_mmio_map(tbus0, 0, 0x01C20000);

/* timer0: irq[0]=TINT12(bottom, clockevent) -> AINTC 21
 *         irq[1]=TINT34(top, clocksource)   -> AINTC 22 */
sysbus_connect_irq(tbus0, 0, qdev_get_gpio_in(aintc, 21));
sysbus_connect_irq(tbus0, 1, qdev_get_gpio_in(aintc, 22));

/* 旧: qemu_irq aintc_24 = ...  sysbus_connect_irq(tbus0, 0, aintc_24); ← 削除
 * 旧: qdev_connect_gpio_out_named(aintc, "timer_ack", ...);           ← 削除
 */

/* AINTC 側の SICR 処理も 24 に対応させる */
// aintc.c の SICR (0x24) の case をこうする：
// if (val == 21 || val == 22 || val == 24) { qemu_set_irq(s->timer_ack_out, 0); }
 //   qdev_connect_gpio_out_named(aintc, "timer_ack", 0,
   //                             qdev_get_gpio_in(timer0, 0));
/*
    DeviceState *timer1 = qdev_new("da850-timer");
    SysBusDevice *tbus1 = SYS_BUS_DEVICE(timer1);
    
    sysbus_realize_and_unref(tbus1, &error_fatal);
    sysbus_mmio_map(tbus1, 0, 0x01C21000);
    sysbus_connect_irq(tbus1, 0,
    qdev_get_gpio_in(aintc, 23));*/

#if 0
    /* 2. vpb_init の中に追加 */
    MemoryRegion *cp_intc = g_new(MemoryRegion, 1);
    /* アドレスは 0xFFFEE000。サイズは 0x2000 あれば十分 */
    memory_region_init_io(cp_intc, NULL, &da850_intc_ops, NULL, "da850-cp-intc", 0x2000);
    memory_region_add_subregion(get_system_memory(), 0xFFFEE000, cp_intc);

    dev = qdev_new("pl080");
    object_property_set_link(OBJECT(dev), "downstream", OBJECT(sysmem),
                             &error_fatal);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x10130000);
    sysbus_connect_irq(busdev, 0, pic[17]);

    sysbus_create_simple("sp804", 0x101e2000, pic[4]);
    sysbus_create_simple("sp804", 0x101e3000, pic[5]);

    sysbus_create_simple("pl061", 0x101e4000, pic[6]);
    sysbus_create_simple("pl061", 0x101e5000, pic[7]);
    sysbus_create_simple("pl061", 0x101e6000, pic[8]);
    sysbus_create_simple("pl061", 0x101e7000, pic[9]);

    /* The versatile/PB actually has a modified Color LCD controller
       that includes hardware cursor support from the PL111.  */
    dev = sysbus_create_simple("pl110_versatile", 0x10120000, pic[16]);
    /* Wire up the mux control signals from the SYS_CLCD register */
    qdev_connect_gpio_out(sysctl, 0, qdev_get_gpio_in(dev, 0));

    dev = sysbus_create_varargs("pl181", 0x10005000, sic[22], sic[1], NULL);
    dinfo = drive_get(IF_SD, 0, 0);
    if (dinfo) {
        DeviceState *card;

        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                               &error_fatal);
    }

    dev = sysbus_create_varargs("pl181", 0x1000b000, sic[23], sic[2], NULL);
    dinfo = drive_get(IF_SD, 0, 1);
    if (dinfo) {
        DeviceState *card;

        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                               &error_fatal);
    }

    /* Add PL031 Real Time Clock. */
    sysbus_create_simple("pl031", 0x101e8000, pic[10]);

    dev = sysbus_create_simple(TYPE_VERSATILE_I2C, 0x10002000, NULL);
    i2c = (I2CBus *)qdev_get_child_bus(dev, "i2c");
    i2c_slave_create_simple(i2c, "ds1338", 0x68);

    /* Add PL041 AACI Interface to the LM4549 codec */
    pl041 = qdev_new("pl041");
    qdev_prop_set_uint32(pl041, "nc_fifo_depth", 512);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pl041), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pl041), 0, 0x10004000);
    sysbus_connect_irq(SYS_BUS_DEVICE(pl041), 0, sic[24]);
#endif
    /* Memory map for Versatile/PB:  */
    /* 0x10000000 System registers.  */
    /* 0x10001000 PCI controller config registers.  */
    /* 0x10002000 Serial bus interface.  */
    /*  0x10003000 Secondary interrupt controller.  */
    /* 0x10004000 AACI (audio).  */
    /*  0x10005000 MMCI0.  */
    /*  0x10006000 KMI0 (keyboard).  */
    /*  0x10007000 KMI1 (mouse).  */
    /* 0x10008000 Character LCD Interface.  */
    /*  0x10009000 UART3.  */
    /* 0x1000a000 Smart card 1.  */
    /*  0x1000b000 MMCI1.  */
    /*  0x10010000 Ethernet.  */
    /* 0x10020000 USB.  */
    /* 0x10100000 SSMC.  */
    /* 0x10110000 MPMC.  */
    /*  0x10120000 CLCD Controller.  */
    /*  0x10130000 DMA Controller.  */
    /*  0x10140000 Vectored interrupt controller.  */
    /* 0x101d0000 AHB Monitor Interface.  */
    /* 0x101e0000 System Controller.  */
    /* 0x101e1000 Watchdog Interface.  */
    /* 0x101e2000 Timer 0/1.  */
    /* 0x101e3000 Timer 2/3.  */
    /* 0x101e4000 GPIO port 0.  */
    /* 0x101e5000 GPIO port 1.  */
    /* 0x101e6000 GPIO port 2.  */
    /* 0x101e7000 GPIO port 3.  */
    /* 0x101e8000 RTC.  */
    /* 0x101f0000 Smart card 0.  */
    /*  0x101f1000 UART0.  */
    /*  0x101f2000 UART1.  */
    /*  0x101f3000 UART2.  */
    /* 0x101f4000 SSPI.  */
    /* 0x34000000 NOR Flash */

    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (!pflash_cfi01_register(VERSATILE_FLASH_ADDR, "versatile.flash",
                          VERSATILE_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          VERSATILE_FLASH_SECT_SIZE,
                          4, 0x0089, 0x0018, 0x0000, 0x0, 0)) {
        fprintf(stderr, "qemu: Error registering flash memory.\n");
    }

    versatile_binfo.loader_start = 0xc0000000;
    versatile_binfo.ram_size = machine->ram_size;
    /* Ensure the kernel command line from -append is forwarded */
    versatile_binfo.kernel_cmdline = machine->kernel_cmdline;
    versatile_binfo.board_id = 0x86d;
    arm_load_kernel(cpu, machine, &versatile_binfo);
}

static void vpb_init(MachineState *machine)
{
    versatile_init(machine, 0x183);
}

static void vab_init(MachineState *machine)
{
    versatile_init(machine, 0x25e);
}

static void versatilepb_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "ARM Versatile/PB (ARM926EJ-S)";
    mc->init = vpb_init;
    mc->block_default_type = IF_SCSI;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_id = "versatile.ram";
}

static const TypeInfo versatilepb_type = {
    .name = MACHINE_TYPE_NAME("versatilepb"),
    .parent = TYPE_MACHINE,
    .class_init = versatilepb_class_init,
};

static void versatileab_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "ARM Versatile/AB (ARM926EJ-S)";
    mc->init = vab_init;
    mc->block_default_type = IF_SCSI;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_id = "versatile.ram";
}

static const TypeInfo versatileab_type = {
    .name = MACHINE_TYPE_NAME("versatileab"),
    .parent = TYPE_MACHINE,
    .class_init = versatileab_class_init,
};

static void versatile_machine_init(void)
{
    type_register_static(&versatilepb_type);
    type_register_static(&versatileab_type);
}

type_init(versatile_machine_init)

static void vpb_sic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_vpb_sic;
}

static const TypeInfo vpb_sic_info = {
    .name          = TYPE_VERSATILE_PB_SIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(vpb_sic_state),
    .instance_init = vpb_sic_init,
    .class_init    = vpb_sic_class_init,
};

static void versatilepb_register_types(void)
{
    type_register_static(&vpb_sic_info);
}

type_init(versatilepb_register_types)
