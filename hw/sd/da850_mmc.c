/* Minimal TI DaVinci/DA850 MMC controller for the EV3 machine. */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qom/object.h"

#define TYPE_DA850_MMC "da850-mmc"

OBJECT_DECLARE_SIMPLE_TYPE(DA850MMCState, DA850_MMC)

struct DA850MMCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;
    uint32_t ctl;
    uint32_t clk;
    uint32_t status;
    uint32_t int_mask;
    uint32_t response[4];
    uint32_t argument;
    uint32_t command;
    uint32_t block_len;
    uint32_t block_count;
};

#define MMCCTL       0x00
#define MMCCLK       0x04
#define MMCST0       0x08
#define MMCST1       0x0c
#define MMCIM        0x10
#define MMCBLEN      0x1c
#define MMCNBLK      0x20
#define MMCNBLC      0x24
#define MMCDRR       0x28
#define MMCDXR       0x2c
#define MMCCMD       0x30
#define MMCARGHL     0x34
#define MMCRSP01     0x38
#define MMCRSP23     0x3c
#define MMCRSP45     0x40
#define MMCRSP67     0x44

#define MMCST0_RSPDNE  (1u << 2)
#define MMCST0_TOUTRS  (1u << 4)
#define MMCCMD_INITCK  (1u << 14)
#define MMCCMD_RSPFMT_MASK (3u << 9)
#define MMCCMD_RSPFMT_NONE (0u << 9)

static void da850_mmc_update_irq(DA850MMCState *s)
{
    qemu_set_irq(s->irq, (s->status & s->int_mask) != 0);
}

static uint32_t da850_mmc_response_word(const uint8_t *response)
{
    return ((uint32_t)response[0] << 24) |
           ((uint32_t)response[1] << 16) |
           ((uint32_t)response[2] << 8) |
           response[3];
}

static void da850_mmc_command(DA850MMCState *s)
{
    if (s->command & MMCCMD_INITCK) {
        s->status |= MMCST0_RSPDNE;
        da850_mmc_update_irq(s);
        return;
    }

    SDRequest request = {
        .cmd = s->command & 0x3f,
        .arg = s->argument,
        .crc = 0,
    };
    uint8_t response[16] = { 0 };
    int response_len;

    response_len = sdbus_do_command(&s->sdbus, &request, response);
    memset(s->response, 0, sizeof(s->response));

    switch (s->command & MMCCMD_RSPFMT_MASK) {
    case MMCCMD_RSPFMT_NONE:
        s->status |= MMCST0_RSPDNE;
        break;
    case 2u << 9:
        if (response_len < 16) {
            s->status |= MMCST0_TOUTRS;
            break;
        }
        s->response[0] = da850_mmc_response_word(response + 0);
        s->response[1] = da850_mmc_response_word(response + 4);
        s->response[2] = da850_mmc_response_word(response + 8);
        s->response[3] = da850_mmc_response_word(response + 12);
        s->status |= MMCST0_RSPDNE;
        break;
    default:
        if (response_len < 4) {
            s->status |= MMCST0_TOUTRS;
            break;
        }
        s->response[3] = da850_mmc_response_word(response);
        s->status |= MMCST0_RSPDNE;
        break;
    }
    da850_mmc_update_irq(s);
}

static uint64_t da850_mmc_read(void *opaque, hwaddr offset, unsigned size)
{
    DA850MMCState *s = opaque;
    uint32_t value = 0;

    switch (offset) {
    case MMCCTL: value = s->ctl; break;
    case MMCCLK: value = s->clk; break;
    case MMCST0:
        value = s->status;
        s->status = 0;
        da850_mmc_update_irq(s);
        break;
    case MMCST1: break;
    case MMCIM: value = s->int_mask; break;
    case MMCBLEN: value = s->block_len; break;
    case MMCNBLK: value = s->block_count; break;
    case MMCNBLC: value = s->block_count; break;
    case MMCCMD: value = s->command; break;
    case MMCARGHL: value = s->argument; break;
    case MMCRSP01: value = s->response[3]; break;
    case MMCRSP23: value = s->response[2]; break;
    case MMCRSP45: value = s->response[1]; break;
    case MMCRSP67: value = s->response[0]; break;
    case MMCDRR: break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "da850-mmc: unimplemented read at 0x%03" HWADDR_PRIx "\n",
                      offset);
        break;
    }
    return value;
}

static void da850_mmc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    DA850MMCState *s = opaque;

    switch (offset) {
    case MMCCTL: s->ctl = value; break;
    case MMCCLK: s->clk = value; break;
    case MMCST0: s->status &= ~value; da850_mmc_update_irq(s); break;
    case MMCIM: s->int_mask = value; da850_mmc_update_irq(s); break;
    case MMCBLEN: s->block_len = value; break;
    case MMCNBLK: s->block_count = value; break;
    case MMCDXR: break;
    case MMCCMD:
        s->command = value;
        da850_mmc_command(s);
        break;
    case MMCARGHL: s->argument = value; break;
    default:
        break;
    }
}

static const MemoryRegionOps da850_mmc_ops = {
    .read = da850_mmc_read,
    .write = da850_mmc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void da850_mmc_reset(DeviceState *dev)
{
    DA850MMCState *s = DA850_MMC(dev);
    s->ctl = 0;
    s->clk = 0;
    s->status = 0;
    s->int_mask = 0;
    s->argument = 0;
    s->command = 0;
    s->block_len = 0;
    s->block_count = 0;
    memset(s->response, 0, sizeof(s->response));
}

static void da850_mmc_init(Object *obj)
{
    DA850MMCState *s = DA850_MMC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &da850_mmc_ops, s,
                          "da850-mmc", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(obj), "sd-bus");
}

static const VMStateDescription vmstate_da850_mmc = {
    .name = "da850-mmc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ctl, DA850MMCState),
        VMSTATE_UINT32(clk, DA850MMCState),
        VMSTATE_UINT32(status, DA850MMCState),
        VMSTATE_UINT32(int_mask, DA850MMCState),
        VMSTATE_UINT32_ARRAY(response, DA850MMCState, 4),
        VMSTATE_UINT32(argument, DA850MMCState),
        VMSTATE_UINT32(command, DA850MMCState),
        VMSTATE_UINT32(block_len, DA850MMCState),
        VMSTATE_UINT32(block_count, DA850MMCState),
        VMSTATE_END_OF_LIST()
    }
};

static void da850_mmc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = da850_mmc_reset;
    dc->vmsd = &vmstate_da850_mmc;
}

static const TypeInfo da850_mmc_info = {
    .name = TYPE_DA850_MMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DA850MMCState),
    .instance_init = da850_mmc_init,
    .class_init = da850_mmc_class_init,
};

static void da850_mmc_register_types(void)
{
    type_register_static(&da850_mmc_info);
}

type_init(da850_mmc_register_types)