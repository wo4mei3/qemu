/*
 * Sitronix ST7586 LCD controller emulation (178x128 for LEGO MINDSTORMS EV3)
 * with authentic LEGO MINDSTORMS EV3 Intelligent Brick photo faceplate.
 *
 * This code is licensed under the GPL v2 or later.
 */

#include "qemu/osdep.h"
#include "hw/ssi/ssi.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "ui/input.h"
#include "qom/object.h"
#include <zlib.h>

#include "ev3_faceplate_data.h"
#include "ev3_faceplate_btnmap.h"

#define ST7586_WIDTH   178
#define ST7586_HEIGHT  128
#define ST7586_COLS    128
#define ST7586_ROWS    160

/*
 * Window dimensions match the official EV3 reference photo (1024 x 819).
 */
#define WIN_VIEW_W     EV3_IMG_W  /* 1024 */
#define WIN_VIEW_H     EV3_IMG_H  /* 819 */

/*
 * ST7586 LCD active screen window mapped into the brick's LCD bezel:
 * In cropped 536x819 space (brick cropped closely without white side margins):
 * x: [102 .. 434] (width = 333)
 * y: [120 .. 303] (height = 184)
 */
#define LCD_WIN_X      102
#define LCD_WIN_Y      120
#define LCD_WIN_W      333
#define LCD_WIN_H      184

enum st7586_mode {
    ST7586_CMD  = 0,
    ST7586_DATA = 1,
};

enum ev3_button_id {
    EV3_BTN_UP    = 0,
    EV3_BTN_ENTER = 1,
    EV3_BTN_DOWN  = 2,
    EV3_BTN_RIGHT = 3,
    EV3_BTN_LEFT  = 4,
    EV3_BTN_BACK  = 5,
    EV3_BTN_COUNT = 6,
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
    QemuInputHandlerState *input;
    qemu_irq button_out[EV3_BTN_COUNT];

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

    int32_t mouse_x;
    int32_t mouse_y;
    bool mouse_down;
    uint32_t mouse_buttons;
    uint32_t key_buttons;
    uint32_t active_buttons;

    uint8_t ddram[ST7586_ROWS * ST7586_COLS];

    /* Uncompressed RGB24 base faceplate and button hit-map */
    uint8_t bg_rgb[WIN_VIEW_H * WIN_VIEW_W * 3];
    uint8_t btn_map[WIN_VIEW_H * WIN_VIEW_W];
};

#define TYPE_ST7586 "st7586"
OBJECT_DECLARE_SIMPLE_TYPE(st7586_state, ST7586)

static void st7586_update_display(void *opaque);

static void st7586_init_faceplate(st7586_state *s)
{
    uLongf dest_len = sizeof(s->bg_rgb);
    int ret = uncompress(s->bg_rgb, &dest_len,
                         ev3_faceplate_zlib, EV3_IMG_COMPRESSED_SIZE);
    if (ret != Z_OK) {
        memset(s->bg_rgb, 200, sizeof(s->bg_rgb));
    }

    dest_len = sizeof(s->btn_map);
    ret = uncompress(s->btn_map, &dest_len,
                     ev3_faceplate_btnmap_zlib, EV3_BTNMAP_COMPRESSED_SIZE);
    if (ret != Z_OK) {
        memset(s->btn_map, 255, sizeof(s->btn_map));
    }
}

static void st7586_sync_buttons(st7586_state *s)
{
    uint32_t combined = s->key_buttons | s->mouse_buttons;
    if (combined == s->active_buttons) {
        return;
    }
    s->active_buttons = combined;
    for (int i = 0; i < EV3_BTN_COUNT; i++) {
        qemu_set_irq(s->button_out[i], (combined >> i) & 1);
    }
    s->redraw = 1;
    st7586_update_display(s);
}

static int st7586_hit_test_button(st7586_state *s, int x, int y)
{
    if (x < 0 || x >= WIN_VIEW_W || y < 0 || y >= WIN_VIEW_H) {
        return -1;
    }
    uint8_t val = s->btn_map[y * WIN_VIEW_W + x];
    return (val < EV3_BTN_COUNT) ? (int)val : -1;
}

static void st7586_input_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    st7586_state *s = ST7586(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_KEY:
    {
        InputKeyEvent *key = evt->u.key.data;
        int qcode = qemu_input_key_value_to_qcode(key->key);
        int btn = -1;

        switch (qcode) {
        case Q_KEY_CODE_UP:
        case Q_KEY_CODE_W:
            btn = EV3_BTN_UP;
            break;
        case Q_KEY_CODE_DOWN:
        case Q_KEY_CODE_S:
            btn = EV3_BTN_DOWN;
            break;
        case Q_KEY_CODE_RIGHT:
        case Q_KEY_CODE_D:
            btn = EV3_BTN_RIGHT;
            break;
        case Q_KEY_CODE_LEFT:
        case Q_KEY_CODE_A:
            btn = EV3_BTN_LEFT;
            break;
        case Q_KEY_CODE_RET:
        case Q_KEY_CODE_KP_ENTER:
        case Q_KEY_CODE_SPC:
            btn = EV3_BTN_ENTER;
            break;
        case Q_KEY_CODE_ESC:
        case Q_KEY_CODE_BACKSPACE:
            btn = EV3_BTN_BACK;
            break;
        default:
            return;
        }

        if (key->down) {
            s->key_buttons |= (1u << btn);
        } else {
            s->key_buttons &= ~(1u << btn);
        }
        st7586_sync_buttons(s);
        break;
    }

    case INPUT_EVENT_KIND_REL:
    {
        InputMoveEvent *move = evt->u.rel.data;
        if (move->axis == INPUT_AXIS_X) {
            s->mouse_x = CLAMP(s->mouse_x + move->value, 0, WIN_VIEW_W - 1);
        } else if (move->axis == INPUT_AXIS_Y) {
            s->mouse_y = CLAMP(s->mouse_y + move->value, 0, WIN_VIEW_H - 1);
        }
        if (s->mouse_down) {
            int hit = st7586_hit_test_button(s, s->mouse_x, s->mouse_y);
            s->mouse_buttons = (hit >= 0) ? (1u << hit) : 0;
            st7586_sync_buttons(s);
        }
        break;
    }

    case INPUT_EVENT_KIND_ABS:
    {
        InputMoveEvent *move = evt->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            s->mouse_x = qemu_input_scale_axis(move->value,
                                               INPUT_EVENT_ABS_MIN,
                                               INPUT_EVENT_ABS_MAX,
                                               0, WIN_VIEW_W - 1);
        } else if (move->axis == INPUT_AXIS_Y) {
            s->mouse_y = qemu_input_scale_axis(move->value,
                                               INPUT_EVENT_ABS_MIN,
                                               INPUT_EVENT_ABS_MAX,
                                               0, WIN_VIEW_H - 1);
        }
        if (s->mouse_down) {
            int hit = st7586_hit_test_button(s, s->mouse_x, s->mouse_y);
            s->mouse_buttons = (hit >= 0) ? (1u << hit) : 0;
            st7586_sync_buttons(s);
        }
        break;
    }

    case INPUT_EVENT_KIND_BTN:
    {
        InputBtnEvent *btn = evt->u.btn.data;
        if (btn->button == INPUT_BUTTON_LEFT) {
            s->mouse_down = btn->down;
            if (btn->down) {
                int hit = st7586_hit_test_button(s, s->mouse_x, s->mouse_y);
                s->mouse_buttons = (hit >= 0) ? (1u << hit) : 0;
            } else {
                s->mouse_buttons = 0;
            }
            st7586_sync_buttons(s);
        }
        break;
    }

    default:
        break;
    }
}

static QemuInputHandler st7586_input_handler = {
    .name  = "EV3 Brick Panel Buttons",
    .mask  = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_BTN |
             INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_REL,
    .event = st7586_input_event,
};

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
            if (s->param_idx == 1) {
                s->row_start = byte & 0xff;
                s->row = s->row_start;
            } else if (s->param_idx == 3) {
                s->row_end = byte & 0xff;
                if (s->row_end < s->row_start) {
                    s->row_end = s->row_start;
                }
            }
            s->param_idx++;
            break;
        case ST7586_RAMWR:
            if (s->row < ST7586_ROWS && s->col < ST7586_COLS) {
                s->ddram[s->row * ST7586_COLS + s->col] = byte;
                s->redraw = 1;
            }
            s->col++;
            if (s->col > s->col_end) {
                s->col = s->col_start;
                s->row++;
                if (s->row > s->row_end) {
                    s->row = s->row_start;
                }
            }
            break;
        default:
            break;
        }
    }

    return 0;
}

static inline void st7586_put_pixel(uint8_t *dest, int bpp,
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
    uint8_t *base;
    int bpp;
    int dest_width;
    int stride;
    int x, y;

    /*
     * Matte FSTN LCD 4-level grayscale palette (non-glossy matte reflection):
     *   0 = LCD background (matte light olive greenish-gray)
     *   1 = light gray
     *   2 = dark gray
     *   3 = LCD foreground (matte black dot)
     */
    static const uint8_t palette[4][3] = {
        { 0xb0, 0xbe, 0xa4 },
        { 0x7c, 0x86, 0x72 },
        { 0x44, 0x4b, 0x3e },
        { 0x18, 0x1c, 0x16 },
    };

    if (!s->redraw || !surface) {
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

    base = surface_data(surface);
    stride = surface_stride(surface);

    /* 1. Blit photo-real EV3 brick faceplate + button highlight */
    for (y = 0; y < WIN_VIEW_H; y++) {
        uint8_t *row_dst = base + y * stride;
        const uint8_t *row_bg = &s->bg_rgb[y * WIN_VIEW_W * 3];
        const uint8_t *row_btn = &s->btn_map[y * WIN_VIEW_W];

        for (x = 0; x < WIN_VIEW_W; x++) {
            uint8_t r = row_bg[x * 3 + 0];
            uint8_t g = row_bg[x * 3 + 1];
            uint8_t b = row_bg[x * 3 + 2];
            uint8_t btn = row_btn[x];

            if (btn < EV3_BTN_COUNT && ((s->active_buttons >> btn) & 1)) {
                /* Button press feedback: depressed shadow with warm amber tint */
                r = (uint8_t)CLAMP((r * 2 + 0xff) / 3, 0, 255);
                g = (uint8_t)CLAMP((g * 2 + 0x78) / 3, 0, 255);
                b = (uint8_t)CLAMP((b * 2 + 0x10) / 3, 0, 255);
            }
            st7586_put_pixel(row_dst + x * dest_width, bpp, r, g, b);
        }
    }

    /* 2. Render 178x128 ST7586 LCD into the LCD active window (333x184) */
    for (int ly = 0; ly < LCD_WIN_H; ly++) {
        int src_y = (ly * ST7586_HEIGHT) / LCD_WIN_H;
        const uint8_t *row_src = &s->ddram[src_y * ST7586_COLS];
        uint8_t *dest = base + (LCD_WIN_Y + ly) * stride +
                        LCD_WIN_X * dest_width;

        for (int lx = 0; lx < LCD_WIN_W; lx++) {
            int src_x = (lx * ST7586_WIDTH) / LCD_WIN_W;
            uint8_t byte = row_src[src_x / 3];
            int sub = src_x % 3;
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

            st7586_put_pixel(dest, bpp,
                             palette[val][0],
                             palette[val][1],
                             palette[val][2]);
            dest += dest_width;
        }
    }

    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, WIN_VIEW_W, WIN_VIEW_H);
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

    st7586_init_faceplate(s);

    s->con = graphic_console_init(dev, 0, &st7586_ops, s);
    qemu_console_resize(s->con, WIN_VIEW_W, WIN_VIEW_H);

    qdev_init_gpio_in(dev, st7586_cd, 1);
    qdev_init_gpio_out(dev, s->button_out, EV3_BTN_COUNT);

    s->input = qemu_input_handler_register(dev, &st7586_input_handler);
    qemu_input_handler_activate(s->input);
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
