/*
 * Sitronix ST7586 LCD controller emulation (178x128 for LEGO MINDSTORMS EV3).
 *
 * This code is licensed under the GPL v2 or later.
 */

#include "qemu/osdep.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "qom/object.h"

#define ST7586_WIDTH   178
#define ST7586_HEIGHT  128
#define ST7586_COLS    128
#define ST7586_ROWS    160
#define MAGNIFY        4

enum st7586_mode {
    ST7586_CMD  = 0,
    ST7586_DATA = 1,
};

/* ST7586 Commands */
#define ST7586_NOP      0x00
#define ST7586_SWRESET  0x01
#define ST7586_SLPIN    0x10
#define ST7586_SLPOUT   0x11
#define ST7586_PTLON    0x12
#define ST7586_NORON    0x13
#define ST7586_INVOFF   0x20
#define ST7586_INVON    0x21
#define ST7586_APOFF    0x22
#define ST7586_APON     0x23
#define ST7586_DISPOFF  0x28
#define ST7586_DISPON   0x29
#define ST7586_CASET    0x2a
#define ST7586_RASET    0x2b
#define ST7586_RAMWR    0x2c

struct st7586_state {
    SSIPeripheral ssidev;
    QemuConsole *con;

    uint32_t mode;
    uint8_t cur_cmd;
    uint32_t param_idx;

    int32_t col;
    int32_t col_start;
    int32_t col_end;
    int32_t row;
    int32_t row_start;
    int32_t row_end;

    int32_t invert;
    int32_t disp_on;
    int32_t all_on;
    int32_t redraw;

    uint8_t ddram[ST7586_ROWS * ST7586_COLS];
};

#define TYPE_ST7586 "st7586"
OBJECT_DECLARE_SIMPLE_TYPE(st7586_state, ST7586)

static uint32_t st7586_transfer(SSIPeripheral *dev, uint32_t data)
{
    st7586_state *s = ST7586(dev);
    uint8_t byte = data & 0xff;

    if (s->mode == ST7586_CMD) {
        s->cur_cmd = byte;
        s->param_idx = 0;

        switch (byte) {
        case ST7586_SWRESET:
            s->col = 0;
            s->col_start = 0;
            s->col_end = 59;
            s->row = 0;
            s->row_start = 0;
            s->row_end = 127;
            s->invert = 0;
            s->all_on = 0;
            s->disp_on = 1;
            memset(s->ddram, 0, sizeof(s->ddram));
            s->redraw = 1;
            break;
        case ST7586_INVOFF:
            s->invert = 0;
            s->redraw = 1;
            break;
        case ST7586_INVON:
            s->invert = 1;
            s->redraw = 1;
            break;
        case ST7586_NORON:
        case ST7586_APOFF:
            s->all_on = 0;
            s->redraw = 1;
            break;
        case ST7586_APON:
            s->all_on = 1;
            s->redraw = 1;
            break;
        case ST7586_DISPOFF:
            s->disp_on = 0;
            s->redraw = 1;
            break;
        case ST7586_DISPON:
            s->disp_on = 1;
            s->redraw = 1;
            break;
        case ST7586_RAMWR:
            s->col = s->col_start;
            s->row = s->row_start;
            break;
        default:
            break;
        }
    } else {
        /* ST7586_DATA */
        switch (s->cur_cmd) {
        case ST7586_CASET:
            /* 4 parameter bytes: 0x00, XS, 0x00, XE */
            if (s->param_idx == 1) {
                s->col_start = byte & 0x7f;
                s->col = s->col_start;
            } else if (s->param_idx == 3) {
                s->col_end = byte & 0x7f;
                if (s->col_end < s->col_start) {
                    s->col_end = s->col_start;
                }
            }
            s->param_idx++;
            break;

        case ST7586_RASET:
            /* 4 parameter bytes: 0x00, YS, 0x00, YE */
            if (s->param_idx == 1) {
                s->row_start = (byte < ST7586_ROWS) ? byte : (ST7586_ROWS - 1);
                s->row = s->row_start;
            } else if (s->param_idx == 3) {
                s->row_end = (byte < ST7586_ROWS) ? byte : (ST7586_ROWS - 1);
                if (s->row_end < s->row_start) {
                    s->row_end = s->row_start;
                }
            }
            s->param_idx++;
            break;

        case ST7586_RAMWR:
            if (s->row >= 0 && s->row < ST7586_ROWS &&
                s->col >= 0 && s->col < ST7586_COLS) {
                s->ddram[s->row * ST7586_COLS + s->col] = byte;
            }
            s->col++;
            if (s->col > s->col_end) {
                s->col = s->col_start;
                s->row++;
                if (s->row > s->row_end) {
                    s->row = s->row_start;
                }
            }
            s->redraw = 1;
            break;

        default:
            s->param_idx++;
            break;
        }
    }

    return 0;
}

static void st7586_put_pixel(uint8_t *dest, int bpp,
                             uint8_t r, uint8_t g, uint8_t b)
{
    switch (bpp) {
    case 15:
    {
        uint16_t v = ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3);
        dest[0] = v & 0xff;
        dest[1] = v >> 8;
        break;
    }
    case 16:
    {
        uint16_t v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        dest[0] = v & 0xff;
        dest[1] = v >> 8;
        break;
    }
    case 24:
        dest[0] = b;
        dest[1] = g;
        dest[2] = r;
        break;
    case 32:
        dest[0] = b;
        dest[1] = g;
        dest[2] = r;
        dest[3] = 0xff;
        break;
    default:
        break;
    }
}

static void st7586_update_display(void *opaque)
{
    st7586_state *s = ST7586(opaque);
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest;
    int bpp;
    int dest_width;
    int x, y, i;

    /*
     * 4-level grayscale palette matching the EV3 FSTN LCD:
     *   0 = LCD background (light greenish-gray)
     *   1 = light gray
     *   2 = dark gray
     *   3 = LCD foreground (black pixel)
     */
    static const uint8_t palette[4][3] = {
        { 0xb8, 0xc4, 0xac },
        { 0x80, 0x8a, 0x78 },
        { 0x45, 0x4b, 0x40 },
        { 0x10, 0x14, 0x0e },
    };

    if (!s->redraw) {
        return;
    }

    bpp = surface_bits_per_pixel(surface);
    switch (bpp) {
    case 0:
        return;
    case 15:
    case 16:
        dest_width = 2;
        break;
    case 24:
        dest_width = 3;
        break;
    case 32:
        dest_width = 4;
        break;
    default:
        return;
    }

    dest = surface_data(surface);
    for (y = 0; y < ST7586_HEIGHT; y++) {
        const uint8_t *row_src = &s->ddram[y * ST7586_COLS];
        uint8_t *row_start_ptr = dest;

        for (x = 0; x < ST7586_WIDTH; x++) {
            uint8_t byte = row_src[x / 3];
            int sub = x % 3;
            int val;

            if (sub == 0) {
                val = (byte >> 6) & 0x03;
            } else if (sub == 1) {
                val = (byte >> 3) & 0x03;
            } else {
                val = byte & 0x03;
            }

            if (s->all_on) {
                val = 3;
            }
            if (s->invert) {
                val = 3 - val;
            }

            for (i = 0; i < MAGNIFY; i++) {
                st7586_put_pixel(dest, bpp,
                                 palette[val][0],
                                 palette[val][1],
                                 palette[val][2]);
                dest += dest_width;
            }
        }

        for (i = 1; i < MAGNIFY; i++) {
            memcpy(dest, row_start_ptr, dest_width * ST7586_WIDTH * MAGNIFY);
            dest += dest_width * ST7586_WIDTH * MAGNIFY;
        }
    }

    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0,
                   ST7586_WIDTH * MAGNIFY, ST7586_HEIGHT * MAGNIFY);
}

static void st7586_invalidate_display(void *opaque)
{
    st7586_state *s = ST7586(opaque);
    s->redraw = 1;
}

/* GPIO input 0: A0 (D/C) pin: 0 = Command, 1 = Data */
static void st7586_cd(void *opaque, int n, int level)
{
    st7586_state *s = ST7586(opaque);
    s->mode = level ? ST7586_DATA : ST7586_CMD;
}

static const VMStateDescription vmstate_st7586 = {
    .name = "st7586_lcd",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(mode, st7586_state),
        VMSTATE_UINT8(cur_cmd, st7586_state),
        VMSTATE_UINT32(param_idx, st7586_state),
        VMSTATE_INT32(col, st7586_state),
        VMSTATE_INT32(col_start, st7586_state),
        VMSTATE_INT32(col_end, st7586_state),
        VMSTATE_INT32(row, st7586_state),
        VMSTATE_INT32(row_start, st7586_state),
        VMSTATE_INT32(row_end, st7586_state),
        VMSTATE_INT32(invert, st7586_state),
        VMSTATE_INT32(disp_on, st7586_state),
        VMSTATE_INT32(all_on, st7586_state),
        VMSTATE_INT32(redraw, st7586_state),
        VMSTATE_BUFFER(ddram, st7586_state),
        VMSTATE_SSI_PERIPHERAL(ssidev, st7586_state),
        VMSTATE_END_OF_LIST()
    }
};

static const GraphicHwOps st7586_ops = {
    .invalidate = st7586_invalidate_display,
    .gfx_update = st7586_update_display,
};

static void st7586_realize(SSIPeripheral *d, Error **errp)
{
    DeviceState *dev = DEVICE(d);
    st7586_state *s = ST7586(d);

    s->col_start = 0;
    s->col_end = 59;
    s->row_start = 0;
    s->row_end = 127;
    s->disp_on = 1;
    s->redraw = 1;

    s->con = graphic_console_init(dev, 0, &st7586_ops, s);
    qemu_console_resize(s->con,
                        ST7586_WIDTH * MAGNIFY,
                        ST7586_HEIGHT * MAGNIFY);

    qdev_init_gpio_in(dev, st7586_cd, 1);
}

static void st7586_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = st7586_realize;
    k->transfer = st7586_transfer;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_st7586;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo st7586_info = {
    .name          = TYPE_ST7586,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(st7586_state),
    .class_init    = st7586_class_init,
};

static void st7586_register_types(void)
{
    type_register_static(&st7586_info);
}

type_init(st7586_register_types)
