/*
 * ARM Versatile Platform/Application Baseboard System emulation.
 *
 * Copyright (c) 2005-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
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
#include "hw/ssi/ssi.h"
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
#include "ui/input.h"
#include "audio/audio.h"
#include "qemu/thread.h"
#include <math.h>
#include <ifaddrs.h>
#include <net/if.h>

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

static uint32_t da850_syscfg_cfgchip2 = 0;

static uint64_t da850_syscfg_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case 0x00: /* JTAG_ID_REG */
        return 0x0B7D102F;
    case 0x18:
        return 0x1B7D102F;
    case 0x17c: /* CFGCHIP0 */
        return 0;
    case 0x184: /* CFGCHIP2 */
        /* USB PHY clock is always "good" in this model, and the PLL is
         * always reported as locked, so ohci_da8xx_clock()'s busy-wait
         * loop on CFGCHIP2_PHYCLKGD completes immediately. */
        return da850_syscfg_cfgchip2 | (1u << 17) /* CFGCHIP2_PHYCLKGD */
                                      | (1u << 6)  /* CFGCHIP2_PHY_PLLON echoed back */;
    case 0x188: /* CFGCHIP3 */
        return 0;
    default:
        return 0;
    }
}

static void da850_syscfg_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    switch (offset) {
    case 0x184: /* CFGCHIP2 */
        da850_syscfg_cfgchip2 = (uint32_t)value;
        break;
    default:
        /* other SYSCFG registers: not modeled, ignore writes */
        break;
    }
}

/* 2. opsに登録 */
static const MemoryRegionOps da850_syscfg_ops = {
    .read = da850_syscfg_read,
    .write = da850_syscfg_write, /* ←これを追加！ */
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define EV3_SOUND_SAMPLE_RATE 44100
#define EV3_SOUND_BUF_LEN     2048

typedef struct EV3SoundState {
    QEMUSoundCard card;
    SWVoiceOut *voice;
    int16_t sample_buf[EV3_SOUND_BUF_LEN];
    uint32_t freq_hz;
    uint32_t volume;
    uint32_t phase;
    bool enabled;
} EV3SoundState;

typedef struct DA850SimplePeripherals {
    MemoryRegion gpio_mmio;
    MemoryRegion timer_mmio;
    MemoryRegion ehrpwm0_mmio;
    MemoryRegion ehrpwm1_mmio;
    MemoryRegion ecap0_mmio;
    MemoryRegion ecap1_mmio;
    uint32_t gpio[0x1000 / 4];
    uint32_t gpio_buttons[10];
    uint8_t pwm0[0x1000];
    uint8_t pwm1[0x2854];
    uint64_t timer_start_ns;
    qemu_irq lcd_cd;
    qemu_irq lcd_led[4];
    EV3SoundState sound;
} DA850SimplePeripherals;

static void ev3_sound_callback(void *opaque, int free)
{
    EV3SoundState *s = opaque;
    if (!s->voice) {
        return;
    }

    while (free > 0) {
        int to_write = MIN(free / 2, EV3_SOUND_BUF_LEN);
        if (to_write <= 0) {
            break;
        }

        if (s->enabled && s->freq_hz >= 50 && s->freq_hz <= 15000 && s->volume > 0) {
            /* Generate sine wave tone scaled by volume */
            uint32_t phase_inc = (uint32_t)(((uint64_t)s->freq_hz << 32) / EV3_SOUND_SAMPLE_RATE);
            int32_t amp = (int32_t)((s->volume * 28000) / 100);
            for (int i = 0; i < to_write; i++) {
                /* Fast sine approximation */
                float angle = (float)(s->phase >> 16) * (2.0f * (float)M_PI / 65536.0f);
                s->sample_buf[i] = (int16_t)(sinf(angle) * amp);
                s->phase += phase_inc;
            }
        } else {
            memset(s->sample_buf, 0, to_write * sizeof(int16_t));
            s->phase = 0;
        }

        int written = AUD_write(s->voice, s->sample_buf, to_write * 2);
        if (written <= 0) {
            break;
        }
        free -= written;
    }
}

static void ev3_sound_init(EV3SoundState *s)
{
    struct audsettings as = {
        .freq = EV3_SOUND_SAMPLE_RATE,
        .nchannels = 1,
        .fmt = AUDIO_FORMAT_S16,
        .endianness = 0,
    };

    AUD_register_card("ev3-sound", &s->card);
    s->voice = AUD_open_out(&s->card, NULL, "ev3-speaker", s,
                            ev3_sound_callback, &as);
    if (s->voice) {
        AUD_set_active_out(s->voice, 1);
    }
}

/* ========================================================================= */
/* EV3 Native Communication & Wi-Fi Discovery Subsystem (No Python required) */
/* ========================================================================= */

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define EV3_WIFI_UDP_PORT 3015
#define EV3_TCP_PORT      5555
#define EV3_SERIAL_NUM    "001653123456"
#define EV3_BRICK_NAME    "EV3"

typedef struct {
    DA850SimplePeripherals *sp;
    QemuThread discovery_thread;
    QemuThread tcp_thread;
} EV3ComServer;

typedef struct {
    DA850SimplePeripherals *sp;
    uint32_t dur_ms;
} ToneStopArg;

static void *ev3_tone_stop_worker(void *opaque)
{
    ToneStopArg *arg = opaque;
    g_usleep((gulong)arg->dur_ms * 1000);
    arg->sp->sound.enabled = false;
    g_free(arg);
    return NULL;
}

static MemoryRegion *ev3_ram_mr = NULL;
static uint32_t ev3_usb_phys_bss = 0;

static uint32_t ev3_get_usb_bss(void)
{
    if (!ev3_ram_mr) return 0;

    uint8_t *ram = memory_region_get_ram_ptr(ev3_ram_mr);
    if (!ram) return 0;

    uint32_t ram_size = memory_region_size(ev3_ram_mr);

    if (ev3_usb_phys_bss && ev3_usb_phys_bss < ram_size) {
        return ev3_usb_phys_bss;
    }

    static const uint8_t pat[] = {
        0x0d, 0xc0, 0xa0, 0xe1, 0x30, 0xd8, 0x2d, 0xe9,
        0x04, 0xb0, 0x4c, 0xe2, 0x50, 0x40, 0x9f, 0xe5,
        0x01, 0x00, 0xa0, 0xe1, 0x08, 0x38, 0x94, 0xe5
    };

    for (uint32_t off = 0x00100000; off < ram_size - 0x1000; off += 4) {
        if (memcmp(ram + off, pat, sizeof(pat)) == 0) {
            uint32_t va_bss = *(uint32_t *)(ram + off + 0x64);
            if (va_bss >= 0xbf000000 && va_bss < 0xc0000000) {
                uint32_t pgd_idx = va_bss >> 20;
                uint32_t pgd_entry = *(uint32_t *)(ram + 0x4000 + pgd_idx * 4);
                if ((pgd_entry & 3) == 1) {
                    uint32_t pt_base = (pgd_entry & ~0x3ff) - 0xc0000000;
                    uint32_t pt_idx = (va_bss >> 12) & 0xff;
                    if (pt_base < ram_size) {
                        uint32_t pte = *(uint32_t *)(ram + pt_base + pt_idx * 4);
                        uint32_t phys_bss = (pte & ~0xfff) - 0xc0000000 + (va_bss & 0xfff);
                        if (phys_bss < ram_size) {
                            ev3_usb_phys_bss = phys_bss;
                            fprintf(stderr, "[EV3 USB BRIDGE] Found d_usbdev bss at physical 0x%08x (VA 0x%08x)!\n",
                                    phys_bss, va_bss);
                            return ev3_usb_phys_bss;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

static bool ev3_usb_send_packet(const uint8_t *pkt, size_t len,
                                uint8_t *rep, size_t *rep_len, int timeout_ms)
{
    uint32_t bss = ev3_get_usb_bss();
    if (!bss || !ev3_ram_mr) return false;

    uint8_t *ram = memory_region_get_ram_ptr(ev3_ram_mr);
    if (!ram) return false;

    volatile uint8_t *buf_out = ram + bss + 1;
    volatile uint8_t *buf_in = ram + bss + 0x401;
    volatile uint32_t *len_in = (volatile uint32_t *)(ram + bss + 0x804);
    volatile uint32_t *len_out = (volatile uint32_t *)(ram + bss + 0x808);
    volatile uint32_t *state_80c = (volatile uint32_t *)(ram + bss + 0x80c);

    /* Keep d_usbdev internal state at 0 so it never attempts to DMA to NULL USB endpoints */
    *state_80c = 0;

    int wait_cnt = 0;
    while (*len_out > 0 && wait_cnt < 25) {
        *state_80c = 0;
        g_usleep(1000);
        wait_cnt++;
    }
    if (*len_out > 0) {
        *len_out = 0;
    }

    *state_80c = 0;
    *len_in = 0;
    memcpy((void *)buf_out, pkt, len);
    *len_out = (uint32_t)len;

    if (rep && rep_len) {
        int waited = 0;
        int max_wait = timeout_ms * 1000;
        size_t total_received = 0;

        while (waited < max_wait) {
            while (*len_in == 0 && waited < max_wait) {
                *state_80c = 0;
                g_usleep(1000);
                waited += 1000;
            }
            if (*len_in == 0) break;

            uint32_t chunk_len = *len_in;
            if (chunk_len > 1024) chunk_len = 1024;
            memcpy(rep, (void *)buf_in, chunk_len);
            *len_in = 0;
            *state_80c = 0;

            /* Ignore empty dummy buffer */
            if (chunk_len < 2 || (rep[0] == 0 && rep[1] == 0)) {
                continue;
            }

            /* Filter out unsolicited daisy chain push packet (58 bytes, cmd_type 0x08) */
            if (chunk_len >= 5 && (rep[4] == 0x08 || (chunk_len == 58 && rep[0] == 0x3a && rep[1] == 0x00))) {
                continue;
            }

            total_received = chunk_len;
            if (total_received >= 2) {
                uint16_t expected_total = (rep[0] | (rep[1] << 8)) + 2;
                while (total_received < expected_total && total_received < 2048) {
                    int chunk_waited = 0;
                    while (*len_in == 0 && chunk_waited < 300000) {
                        *state_80c = 0;
                        g_usleep(1000);
                        chunk_waited += 1000;
                    }
                    if (*len_in == 0) break;
                    uint32_t next_chunk = *len_in;
                    volatile uint8_t *in_ptr = (volatile uint8_t *)buf_in;
                    if (next_chunk >= 5 && (in_ptr[4] == 0x08 ||
                        (next_chunk == 58 && in_ptr[0] == 0x3a && in_ptr[1] == 0x00))) {
                        *len_in = 0;
                        *state_80c = 0;
                        continue;
                    }
                    if (total_received + next_chunk > 2048) {
                        next_chunk = 2048 - total_received;
                    }
                    memcpy(rep + total_received, (void *)buf_in, next_chunk);
                    total_received += next_chunk;
                    *len_in = 0;
                    *state_80c = 0;
                }
                if (total_received > expected_total) {
                    total_received = expected_total;
                }
            }
            *rep_len = total_received;
            return true;
        }

        fprintf(stderr, "[EV3 USB BRIDGE] Reply timed out after %d ms (cmd=0x%02x, len_out=%u)!\n",
                timeout_ms, len > 5 ? pkt[5] : 0, *len_out);
        return false;
    }
    return true;
}

static void *ev3_discovery_worker(void *opaque)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        fprintf(stderr, "[EV3 COM] UDP socket failed: %s\n", strerror(errno));
        return NULL;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port = 0; /* Ephemeral port so replies return directly */
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        fprintf(stderr, "[EV3 COM] UDP bind failed: %s\n", strerror(errno));
        close(sock);
        return NULL;
    }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    fprintf(stderr, "[EV3 COM] Discovery worker active (broadcasting on UDP %d)...\n", EV3_WIFI_UDP_PORT);

    while (!ev3_get_usb_bss()) {
        g_usleep(500000);
    }
    fprintf(stderr, "[EV3 COM] LMS2012 ready, starting UDP broadcast...\n");

    const char *payload = 
        "Serial-Number: " EV3_SERIAL_NUM "\r\n"
        "Port: " STR(EV3_TCP_PORT) "\r\n"
        "Name: " EV3_BRICK_NAME "\r\n"
        "Protocol: EV3\r\n";
    int payload_len = strlen(payload);

    struct sockaddr_in bcast_addr;
    memset(&bcast_addr, 0, sizeof(bcast_addr));
    bcast_addr.sin_family = AF_INET;
    bcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    bcast_addr.sin_port = htons(EV3_WIFI_UDP_PORT);

    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    local_addr.sin_port = htons(EV3_WIFI_UDP_PORT);

    char host_ip[256] = { 0 };
    FILE *rf = fopen("/etc/resolv.conf", "r");
    if (rf) {
        char line[256];
        while (fgets(line, sizeof(line), rf)) {
            if (strncmp(line, "nameserver", 10) == 0) {
                char *ip = line + 10;
                while (*ip == ' ' || *ip == '\t') ip++;
                char *end = ip;
                while (*end && *end != ' ' && *end != '\t' && *end != '\r' && *end != '\n') end++;
                *end = '\0';
                if (*ip && strcmp(ip, "127.0.0.1") != 0 && strcmp(ip, "8.8.8.8") != 0) {
                    g_strlcpy(host_ip, ip, sizeof(host_ip));
                }
            }
        }
        fclose(rf);
    }

    struct sockaddr_in host_addr;
    memset(&host_addr, 0, sizeof(host_addr));
    if (host_ip[0] != '\0') {
        host_addr.sin_family = AF_INET;
        host_addr.sin_addr.s_addr = inet_addr(host_ip);
        host_addr.sin_port = htons(EV3_WIFI_UDP_PORT);
        fprintf(stderr, "[EV3 COM] Discovery targeting Windows host at %s:%d\n", host_ip, EV3_WIFI_UDP_PORT);
    }

    while (1) {
        sendto(sock, payload, payload_len, 0, (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));
        sendto(sock, payload, payload_len, 0, (struct sockaddr *)&local_addr, sizeof(local_addr));
        if (host_ip[0] != '\0') {
            sendto(sock, payload, payload_len, 0, (struct sockaddr *)&host_addr, sizeof(host_addr));
        }

        /* 1. Send to all interface broadcast addresses (WSL2 subnet broadcast) */
        struct ifaddrs *ifaddr, *ifa;
        if (getifaddrs(&ifaddr) != -1) {
            for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
                if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
                if (ifa->ifa_flags & IFF_LOOPBACK) continue;
                if ((ifa->ifa_flags & IFF_BROADCAST) && ifa->ifa_broadaddr) {
                    struct sockaddr_in baddr = *(struct sockaddr_in *)ifa->ifa_broadaddr;
                    baddr.sin_port = htons(EV3_WIFI_UDP_PORT);
                    sendto(sock, payload, payload_len, 0, (struct sockaddr *)&baddr, sizeof(baddr));
                }
            }
            freeifaddrs(ifaddr);
        }

        /* 2. Send directly to default gateway (Windows host in WSL2) */
        FILE *rf_gw = fopen("/proc/net/route", "r");
        if (rf_gw) {
            char rline[256];
            while (fgets(rline, sizeof(rline), rf_gw)) {
                char ifname[32];
                unsigned long dest = 0, gw = 0;
                if (sscanf(rline, "%31s %lx %lx", ifname, &dest, &gw) == 3) {
                    if (dest == 0 && gw != 0) {
                        struct sockaddr_in gw_addr;
                        memset(&gw_addr, 0, sizeof(gw_addr));
                        gw_addr.sin_family = AF_INET;
                        gw_addr.sin_addr.s_addr = gw;
                        gw_addr.sin_port = htons(EV3_WIFI_UDP_PORT);
                        sendto(sock, payload, payload_len, 0, (struct sockaddr *)&gw_addr, sizeof(gw_addr));
                    }
                }
            }
            fclose(rf_gw);
        }

        for (int i = 0; i < 5; i++) {
            char ack_buf[128];
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int rx = recvfrom(sock, ack_buf, sizeof(ack_buf) - 1, 0, (struct sockaddr *)&client_addr, &addr_len);
            if (rx > 0) {
                /* Reply directly to discovering client */
                sendto(sock, payload, payload_len, 0, (struct sockaddr *)&client_addr, addr_len);
            }
            g_usleep(500000);
        }
    }
    close(sock);
    return NULL;
}

static int ev3_decode_par(const unsigned char *p, int max_len, int *val, bool *is_global, int *consumed)
{
    if (max_len < 1) return -1;
    unsigned char b = p[0];
    if ((b & 0x80) == 0) {
        *consumed = 1;
        if ((b & 0x40) == 0) {
            *is_global = false;
            if ((b & 0x20) == 0) {
                *val = b & 0x1F;
            } else {
                *val = (int)(b & 0x1F) - 32;
            }
        } else {
            *val = b & 0x1F;
            *is_global = ((b & 0x20) != 0);
        }
        return 0;
    } else {
        int type = b & 0x07;
        if ((b & 0x40) == 0) {
            *is_global = false;
            if (type == 0 || type == 1) {
                if (max_len < 2) return -1;
                *val = (signed char)p[1];
                *consumed = 2;
            } else if (type == 2) {
                if (max_len < 3) return -1;
                *val = (int16_t)(p[1] | (p[2] << 8));
                *consumed = 3;
            } else if (type == 3) {
                if (max_len < 5) return -1;
                *val = (int32_t)(p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24));
                *consumed = 5;
            } else {
                *val = 0;
                *consumed = 1;
            }
            return 0;
        } else {
            *is_global = ((b & 0x20) != 0);
            if (type == 0 || type == 1) {
                if (max_len < 2) return -1;
                *val = p[1];
                *consumed = 2;
            } else if (type == 2) {
                if (max_len < 3) return -1;
                *val = p[1] | (p[2] << 8);
                *consumed = 3;
            } else if (type == 3) {
                if (max_len < 5) return -1;
                *val = (int32_t)(p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24));
                *consumed = 5;
            } else {
                *val = 0;
                *consumed = 1;
            }
            return 0;
        }
    }
}

static int ev3_decode_str(const unsigned char *p, int max_len, const char **str_out, int *consumed)
{
    if (max_len < 1) return -1;
    if (p[0] == 0x84 || p[0] == 0x80) {
        /* LCS null-terminated string */
        *str_out = (const char *)&p[1];
        int len = 0;
        while (1 + len < max_len && p[1 + len] != '\0') {
            len++;
        }
        *consumed = 1 + len + (1 + len < max_len ? 1 : 0);
        return 0;
    } else if (p[0] == 0x00) {
        *str_out = "";
        *consumed = 1;
        return 0;
    } else if (p[0] >= 0x20 && p[0] <= 0x7E) {
        /* Bare string without 0x84 prefix */
        *str_out = (const char *)p;
        int len = 0;
        while (len < max_len && p[len] != '\0') {
            len++;
        }
        *consumed = len + (len < max_len ? 1 : 0);
        return 0;
    } else {
        *str_out = "";
        *consumed = 1;
        return 0;
    }
}

static void ev3_mkdir_p(const char *path)
{
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", path);
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    mkdir(tmp, 0755);
}

static const char *ev3_clean_relpath(const char *path)
{
    const char *p = path ? path : "";
    while (*p == '.' || *p == '/') p++;
    if (strncmp(p, "home/root/lms2012/prjs", 22) == 0) {
        p += 22;
        while (*p == '/') p++;
    } else if (strncmp(p, "mnt/ramdisk/prjs", 16) == 0) {
        p += 16;
        while (*p == '/') p++;
    }
    if (strncmp(p, "prjs", 4) == 0) {
        p += 4;
        while (*p == '/') p++;
    } else if (strncasecmp(p, "projects", 8) == 0) {
        p += 8;
        while (*p == '/') p++;
    }
    while (*p == '/') p++;
    return p;
}

static void ev3_resolve_path(const char *ev3_path, char *out, size_t out_sz)
{
    mkdir("/tmp/ev3_prjs", 0755);
    mkdir("/tmp/ev3_prjs/BrkProg_SAVE", 0755);

    const char *p = ev3_clean_relpath(ev3_path);

    if (*p == '\0') {
        snprintf(out, out_sz, "/tmp/ev3_prjs");
    } else {
        snprintf(out, out_sz, "/tmp/ev3_prjs/%s", p);
        size_t len = strlen(out);
        if (len > 0 && out[len - 1] == '/') {
            out[len - 1] = '\0';
        }
    }
}

static void ev3_sync_to_guest(char cmd, const char *path, const void *data, size_t data_len)
{
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) return;

    struct sockaddr_un sun_addr;
    memset(&sun_addr, 0, sizeof(sun_addr));
    sun_addr.sun_family = AF_UNIX;
    snprintf(sun_addr.sun_path, sizeof(sun_addr.sun_path), "/tmp/ev3_guest_bridge.sock");

    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&sun_addr, sizeof(sun_addr)) < 0) {
        close(sock);
        return;
    }

    unsigned short plen = path ? strlen(path) : 0;
    if (cmd == 'D' || cmd == 'R') {
        send(sock, &cmd, 1, MSG_NOSIGNAL);
        send(sock, &plen, 2, MSG_NOSIGNAL);
        if (plen > 0) send(sock, path, plen, MSG_NOSIGNAL);
    } else if (cmd == 'F') {
        unsigned int flen = data_len;
        send(sock, &cmd, 1, MSG_NOSIGNAL);
        send(sock, &plen, 2, MSG_NOSIGNAL);
        send(sock, &flen, 4, MSG_NOSIGNAL);
        if (plen > 0) send(sock, path, plen, MSG_NOSIGNAL);
        if (flen > 0 && data) {
            send(sock, data, flen, MSG_NOSIGNAL);
        }
    }
    close(sock);
    fprintf(stderr, "[EV3 COM] ev3_sync_to_guest: cmd='%c' path='%s' (%zu bytes) synced to guest!\n", cmd, path ? path : "", data_len);
}

static void ev3_handle_client(DA850SimplePeripherals *sp, int client_fd)
{
    unsigned char buf[1024];
    int file_handles[8] = { 0 };
    char file_ev3_paths[8][1024] = { { 0 } };
    char file_safe_paths[8][1024] = { { 0 } };
    bool is_prg_running = false;

    fprintf(stderr, "[EV3 COM] Client connected on TCP 5555!\n");

    int wait_bss = 0;
    while (!ev3_get_usb_bss() && wait_bss < 60) {
        g_usleep(500000);
        wait_bss++;
    }

    while (1) {
        unsigned char hdr[2];
        int n = recv(client_fd, hdr, 2, MSG_WAITALL);
        if (n != 2) {
            fprintf(stderr, "[EV3 COM] recv header failed (n=%d)\n", n);
            break;
        }

        /* Check for official software VMTP1.0 unlock handshake */
        if (hdr[0] == 'G' && hdr[1] == 'E') {
            char http_buf[512] = { 'G', 'E' };
            int rn = 2;
            while (rn < (int)sizeof(http_buf) - 1) {
                char ch = 0;
                int r = recv(client_fd, &ch, 1, 0);
                if (r <= 0) break;
                http_buf[rn++] = ch;
                if ((rn >= 4 && memcmp(&http_buf[rn - 4], "\r\n\r\n", 4) == 0) ||
                    (rn >= 2 && memcmp(&http_buf[rn - 2], "\n\n", 2) == 0)) {
                    break;
                }
            }
            http_buf[rn] = '\0';
            fprintf(stderr, "[EV3 COM] Received VMTP unlock request (%d bytes): %s\n", rn, http_buf);
            send(client_fd, "Accept:EV340\r\n\r\n", 16, MSG_NOSIGNAL);
            fprintf(stderr, "[EV3 COM] Sent Accept:EV340 response\n");
            continue;
        }

        int pkt_len = hdr[0] | (hdr[1] << 8);
        if (pkt_len <= 0 || pkt_len > 1000) {
            fprintf(stderr, "[EV3 COM] Invalid pkt_len=%d (0x%02x 0x%02x)\n", pkt_len, hdr[0], hdr[1]);
            break;
        }

        buf[0] = hdr[0];
        buf[1] = hdr[1];
        n = recv(client_fd, &buf[2], pkt_len, MSG_WAITALL);
        if (n != pkt_len) {
            fprintf(stderr, "[EV3 COM] Incomplete body read (n=%d, expected=%d)\n", n, pkt_len);
            break;
        }
        buf[2 + pkt_len] = '\0';

        fprintf(stderr, "[EV3 COM] Pkt (%d bytes): ", 2 + pkt_len);
        for (int i = 0; i < 2 + pkt_len; i++) {
            fprintf(stderr, "%02x ", buf[i]);
        }
        fprintf(stderr, "\n");

        unsigned short counter = buf[2] | (buf[3] << 8);
        unsigned char cmd_type = buf[4];

        /* CLOSE_FILEHANDLE (0x98):
         * Guest lms2012 automatically closes the file and frees the handle at EOF
         * during CONTINUE_DOWNLOAD (0x93). Forwarding 0x98 to guest causes it to drop
         * the packet and ev3_usb_send_packet to time out after 2000ms.
         * Reply SUCCESS directly to PC software immediately.
         */
        if (cmd_type == 0x01 && buf[5] == 0x98) {
            unsigned char h = buf[6];
            fprintf(stderr, "[EV3 COM] Intercepted CLOSE_FILEHANDLE (handle=%d), replied SUCCESS (0ms)\n", h);
            unsigned char rep[8] = {
                0x06, 0x00,
                (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                0x03, 0x98, 0x00, h
            };
            send(client_fd, rep, 8, MSG_NOSIGNAL);

            /* Play download chime on brick */
            if (sp) {
                sp->sound.freq_hz = 1200;
                sp->sound.volume = 60;
                sp->sound.enabled = true;
                ToneStopArg *stop_arg = g_new0(ToneStopArg, 1);
                stop_arg->sp = sp;
                stop_arg->dur_ms = 150;
                QemuThread th;
                qemu_thread_create(&th, "ev3-tone-stop", ev3_tone_stop_worker, stop_arg, QEMU_THREAD_DETACHED);
            }
            continue;
        }

        /*
         * Intercept LIST_FILES (0x99):
         * Guest lms2012 leaves directory handles open, causing subsequent LIST_FILES
         * (e.g. BrkProg_SAVE) to hang. Reply natively in 0ms.
         */
        if (cmd_type == 0x01 && buf[5] == 0x99) {
            char req_path[256] = { 0 };
            if (pkt_len > 6) {
                snprintf(req_path, sizeof(req_path), "%.128s", (char *)&buf[8]);
            }
            char target_dir[512];
            ev3_resolve_path(req_path, target_dir, sizeof(target_dir));

            char list_str[2048] = { 0 };
            int list_len = 0;

            DIR *dir = opendir(target_dir);
            if (dir) {
                struct dirent *ent;
                while ((ent = readdir(dir)) != NULL) {
                    if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) continue;
                    char full_entry[1024];
                    snprintf(full_entry, sizeof(full_entry), "%s/%s", target_dir, ent->d_name);
                    struct stat st;
                    if (stat(full_entry, &st) == 0) {
                        if (S_ISDIR(st.st_mode)) {
                            list_len += snprintf(list_str + list_len, sizeof(list_str) - list_len, "%s/\n", ent->d_name);
                        } else {
                            list_len += snprintf(list_str + list_len, sizeof(list_str) - list_len,
                                                 "00000000000000000000000000000000 %08x %s\n",
                                                 (unsigned int)st.st_size, ent->d_name);
                        }
                    }
                }
                closedir(dir);
            }

            if (list_len == 0 && strcmp(target_dir, "/tmp/ev3_prjs") == 0) {
                list_len = snprintf(list_str, sizeof(list_str), "BrkProg_SAVE/\n");
            }

            int rep_payload_len = 10 + list_len;
            unsigned char rep[2048];
            rep[0] = rep_payload_len & 0xFF;
            rep[1] = (rep_payload_len >> 8) & 0xFF;
            rep[2] = (unsigned char)(counter & 0xFF);
            rep[3] = (unsigned char)((counter >> 8) & 0xFF);
            rep[4] = 0x03; /* SYSTEM_REPLY */
            rep[5] = 0x99; /* LIST_FILES */
            rep[6] = 0x00; /* SUCCESS */
            rep[7] = (unsigned char)(list_len & 0xFF);
            rep[8] = (unsigned char)((list_len >> 8) & 0xFF);
            rep[9] = (unsigned char)((list_len >> 16) & 0xFF);
            rep[10] = (unsigned char)((list_len >> 24) & 0xFF);
            rep[11] = 0x00; /* Handle: 0 = list completed in one chunk */
            memcpy(&rep[12], list_str, list_len);
            send(client_fd, rep, 2 + rep_payload_len, MSG_NOSIGNAL);
            fprintf(stderr, "[EV3 COM] LIST_FILES (%s): returned %d bytes (list_len=%d):\n%s", target_dir, 2 + rep_payload_len, list_len, list_str);
            continue;
        }

        /* Direct commands: forward short queries (<= 32 bytes) to guest LMS2012 with fast 40ms timeout */
        if ((cmd_type == 0x00 || cmd_type == 0x80) && pkt_len <= 32) {
            bool has_reply = (cmd_type == 0x00);
            uint8_t rep_buf[2048];
            size_t rep_sz = 0;
            if (ev3_usb_send_packet(buf, 2 + pkt_len, has_reply ? rep_buf : NULL, has_reply ? &rep_sz : NULL, 40)) {
                if (has_reply && rep_sz > 0) {
                    send(client_fd, rep_buf, rep_sz, MSG_NOSIGNAL);
                    fprintf(stderr, "[EV3 COM] Guest LMS2012 Replied (%zu bytes)\n", rep_sz);
                }
                continue;
            }
        }

        if (cmd_type == 0x01 || cmd_type == 0x81) {
            /* System Command */
            unsigned char sys_cmd = buf[5];
            if (sys_cmd == 0x92) {
                /* BEGIN_DOWNLOAD: buf[6..9] = filesize, buf[10...] = filename */
                char *filename = (char *)&buf[10];
                char safe_path[512];
                ev3_resolve_path(filename, safe_path, sizeof(safe_path));

                /* ensure parent directory exists */
                char parent[512];
                snprintf(parent, sizeof(parent), "%s", safe_path);
                char *slash = strrchr(parent, '/');
                if (slash && slash != parent) {
                    *slash = '\0';
                    ev3_mkdir_p(parent);
                }

                fprintf(stderr, "[EV3 COM] BEGIN_DOWNLOAD: %s (saving to %s)\n", filename, safe_path);

                int h = 1;
                for (int i = 1; i < 8; i++) {
                    if (file_handles[i] <= 0) { h = i; break; }
                }
                int fd = open(safe_path, O_CREAT | O_WRONLY | O_TRUNC, 0755);
                if (fd >= 0) {
                    file_handles[h] = fd;
                    snprintf(file_ev3_paths[h], sizeof(file_ev3_paths[h]), "%.1000s", filename);
                    snprintf(file_safe_paths[h], sizeof(file_safe_paths[h]), "%.1000s", safe_path);

                    /* Also forward BEGIN_DOWNLOAD to guest LMS2012 via d_usbdev so guest writes it to /mnt/ramdisk/prjs/ */
                    ev3_usb_send_packet(buf, 2 + pkt_len, NULL, NULL, 0);

                    if (cmd_type == 0x01) {
                        unsigned char rep[8] = {
                            0x06, 0x00,
                            (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                            0x03, 0x92, 0x00, (unsigned char)h
                        };
                        send(client_fd, rep, 8, MSG_NOSIGNAL);
                    }
                } else {
                    if (cmd_type == 0x01) {
                        unsigned char rep[8] = {
                            0x06, 0x00,
                            (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                            0x03, 0x92, 0x05, 0x00
                        };
                        send(client_fd, rep, 8, MSG_NOSIGNAL);
                    }
                }
            } else if (sys_cmd == 0x93) {
                /* CONTINUE_DOWNLOAD */
                unsigned char h = buf[6];
                int chunk_len = (2 + pkt_len) - 7;
                if (h < 8 && file_handles[h] > 0 && chunk_len > 0) {
                    if (write(file_handles[h], &buf[7], chunk_len) < 0) {
                        /* write error */
                    }
                }

                /* Also forward CONTINUE_DOWNLOAD to guest LMS2012 via d_usbdev */
                ev3_usb_send_packet(buf, 2 + pkt_len, NULL, NULL, 0);

                if (cmd_type == 0x01) {
                    unsigned char rep[8] = {
                        0x06, 0x00,
                        (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                        0x03, 0x93, 0x00, h
                    };
                    send(client_fd, rep, 8, MSG_NOSIGNAL);
                }
            } else if (sys_cmd == 0x98) {
                /* CLOSE_FILEHANDLE: official EV3 command is 0x98 */
                unsigned char h = buf[6];
                fprintf(stderr, "[EV3 COM] CLOSE_FILEHANDLE: handle %d (download finished!)\n", h);
                if (h < 8 && file_handles[h] > 0) {
                    close(file_handles[h]);
                    file_handles[h] = 0;

                    /* Also forward CLOSE_FILEHANDLE to guest LMS2012 via d_usbdev */
                    ev3_usb_send_packet(buf, 2 + pkt_len, NULL, NULL, 0);

                    /* Send immediate success reply to PC before syncing */
                    if (cmd_type == 0x01) {
                        unsigned char rep[8] = {
                            0x06, 0x00,
                            (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                            0x03, sys_cmd, 0x00, h
                        };
                        send(client_fd, rep, 8, MSG_NOSIGNAL);
                    }

                    /* Sync to EV3 guest at /mnt/ramdisk/prjs/<Project>/<Program> */
                    if (file_safe_paths[h][0] != '\0') {
                        FILE *fp = fopen(file_safe_paths[h], "rb");
                        if (fp) {
                            fseek(fp, 0, SEEK_END);
                            long fsz = ftell(fp);
                            fseek(fp, 0, SEEK_SET);
                            void *fdata = malloc(fsz);
                            if (fdata) {
                                if (fread(fdata, 1, fsz, fp) == (size_t)fsz) {
                                    ev3_sync_to_guest('F', ev3_clean_relpath(file_ev3_paths[h]), fdata, fsz);
                                }
                                free(fdata);
                            }
                            fclose(fp);
                        }
                    }

                    /* Play download chime on brick */
                    if (sp) {
                        sp->sound.freq_hz = 1200;
                        sp->sound.volume = 60;
                        sp->sound.enabled = true;
                        ToneStopArg *stop_arg = g_new0(ToneStopArg, 1);
                        stop_arg->sp = sp;
                        stop_arg->dur_ms = 150;
                        QemuThread th;
                        qemu_thread_create(&th, "ev3-tone-stop", ev3_tone_stop_worker, stop_arg, QEMU_THREAD_DETACHED);
                    }
                } else {
                    if (cmd_type == 0x01) {
                        unsigned char rep[8] = {
                            0x06, 0x00,
                            (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                            0x03, sys_cmd, 0x00, h
                        };
                        send(client_fd, rep, 8, MSG_NOSIGNAL);
                    }
                }
            } else if (sys_cmd == 0x94) {
                /* BEGIN_UPLOAD: buf[6..7] = bytes_to_read, buf[8...] = filename */
                if (pkt_len <= 5) {
                    /* ev3_tool mock close handle */
                    unsigned char h = buf[6];
                    if (h < 8 && file_handles[h] > 0) {
                        close(file_handles[h]);
                        file_handles[h] = 0;
                    }
                    if (cmd_type == 0x01) {
                        unsigned char rep[8] = {
                            0x06, 0x00,
                            (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                            0x03, 0x94, 0x00, h
                        };
                        send(client_fd, rep, 8, MSG_NOSIGNAL);
                    }
                } else {
                    uint16_t bytes_to_read = buf[6] | (buf[7] << 8);
                    const char *filename = (const char *)&buf[8];
                    char safe_path[512];
                    ev3_resolve_path(filename, safe_path, sizeof(safe_path));
                    fprintf(stderr, "[EV3 COM] BEGIN_UPLOAD: %s (safe_path=%s, bytes_to_read=%u)\n",
                            filename, safe_path, bytes_to_read);

                    FILE *fp = fopen(safe_path, "rb");
                    if (fp) {
                        fseek(fp, 0, SEEK_END);
                        uint32_t fsize = (uint32_t)ftell(fp);
                        fseek(fp, 0, SEEK_SET);

                        uint16_t chunk = bytes_to_read > 1024 ? 1024 : bytes_to_read;
                        if (chunk > fsize) chunk = (uint16_t)fsize;

                        uint8_t payload[1024];
                        size_t rd = fread(payload, 1, chunk, fp);

                        uint8_t upload_handle = 0;
                        if (rd < fsize) {
                            for (int i = 1; i < 8; i++) {
                                if (file_handles[i] <= 0) {
                                    file_handles[i] = fileno(fp);
                                    upload_handle = i;
                                    break;
                                }
                            }
                            if (upload_handle == 0) fclose(fp);
                        } else {
                            fclose(fp);
                        }

                        int rep_payload_len = 10 + rd;
                        uint8_t rep[2048];
                        rep[0] = rep_payload_len & 0xFF;
                        rep[1] = (rep_payload_len >> 8) & 0xFF;
                        rep[2] = (uint8_t)(counter & 0xFF);
                        rep[3] = (uint8_t)((counter >> 8) & 0xFF);
                        rep[4] = 0x03;
                        rep[5] = 0x94;
                        rep[6] = 0x00; /* SUCCESS */
                        rep[7] = fsize & 0xFF;
                        rep[8] = (fsize >> 8) & 0xFF;
                        rep[9] = (fsize >> 16) & 0xFF;
                        rep[10] = (fsize >> 24) & 0xFF;
                        rep[11] = upload_handle;
                        memcpy(&rep[12], payload, rd);

                        send(client_fd, rep, 2 + rep_payload_len, MSG_NOSIGNAL);
                        fprintf(stderr, "[EV3 COM] BEGIN_UPLOAD: Sent %zu/%u bytes (handle=%d)\n",
                                rd, fsize, upload_handle);
                    } else {
                        uint8_t rep[7] = {
                            0x05, 0x00,
                            (uint8_t)(counter & 0xFF), (uint8_t)((counter >> 8) & 0xFF),
                            0x05, 0x94, 0x05 /* HANDLE_NOT_READY / FILE_NOT_FOUND */
                        };
                        send(client_fd, rep, 7, MSG_NOSIGNAL);
                        fprintf(stderr, "[EV3 COM] BEGIN_UPLOAD: file not found: %s\n", safe_path);
                    }
                }
            } else if (sys_cmd == 0x95) {
                /* CONTINUE_UPLOAD */
                uint8_t h = buf[6];
                uint16_t bytes_to_read = buf[7] | (buf[8] << 8);
                if (h < 8 && file_handles[h] > 0) {
                    uint16_t chunk = bytes_to_read > 1024 ? 1024 : bytes_to_read;
                    uint8_t payload[1024];
                    ssize_t rd = read(file_handles[h], payload, chunk);
                    if (rd < 0) rd = 0;

                    uint8_t next_h = h;
                    if ((size_t)rd < chunk) {
                        close(file_handles[h]);
                        file_handles[h] = 0;
                        next_h = 0;
                    }

                    int rep_payload_len = 6 + rd;
                    uint8_t rep[2048];
                    rep[0] = rep_payload_len & 0xFF;
                    rep[1] = (rep_payload_len >> 8) & 0xFF;
                    rep[2] = (uint8_t)(counter & 0xFF);
                    rep[3] = (uint8_t)((counter >> 8) & 0xFF);
                    rep[4] = 0x03;
                    rep[5] = 0x95;
                    rep[6] = 0x00; /* SUCCESS */
                    rep[7] = next_h;
                    memcpy(&rep[8], payload, rd);

                    send(client_fd, rep, 2 + rep_payload_len, MSG_NOSIGNAL);
                    fprintf(stderr, "[EV3 COM] CONTINUE_UPLOAD: Sent %zd bytes (handle=%d)\n", rd, next_h);
                } else {
                    uint8_t rep[7] = {
                        0x05, 0x00,
                        (uint8_t)(counter & 0xFF), (uint8_t)((counter >> 8) & 0xFF),
                        0x05, 0x95, 0x05
                    };
                    send(client_fd, rep, 7, MSG_NOSIGNAL);
                }
            } else if (sys_cmd == 0x99) {
                /* LIST_FILES: buf[6..7] = max_len, buf[8...] = path */
                char req_path[256] = { 0 };
                if (pkt_len > 6) {
                    snprintf(req_path, sizeof(req_path), "%.128s", (char *)&buf[8]);
                }
                char target_dir[512];
                ev3_resolve_path(req_path, target_dir, sizeof(target_dir));

                char list_str[2048] = { 0 };
                int list_len = 0;

                DIR *dir = opendir(target_dir);
                if (dir) {
                    struct dirent *ent;
                    while ((ent = readdir(dir)) != NULL) {
                        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) continue;
                        char full_entry[1024];
                        snprintf(full_entry, sizeof(full_entry), "%s/%s", target_dir, ent->d_name);
                        struct stat st;
                        if (stat(full_entry, &st) == 0) {
                            if (S_ISDIR(st.st_mode)) {
                                list_len += snprintf(list_str + list_len, sizeof(list_str) - list_len, "%s/\n", ent->d_name);
                            } else {
                                list_len += snprintf(list_str + list_len, sizeof(list_str) - list_len,
                                                     "00000000000000000000000000000000 %08x %s\n",
                                                     (unsigned int)st.st_size, ent->d_name);
                            }
                        }
                    }
                    closedir(dir);
                }

                if (list_len == 0 && strcmp(target_dir, "/tmp/ev3_prjs") == 0) {
                    list_len = snprintf(list_str, sizeof(list_str), "BrkProg_SAVE/\n");
                }

                int rep_payload_len = 10 + list_len;
                unsigned char rep[2048];
                rep[0] = rep_payload_len & 0xFF;
                rep[1] = (rep_payload_len >> 8) & 0xFF;
                rep[2] = (unsigned char)(counter & 0xFF);
                rep[3] = (unsigned char)((counter >> 8) & 0xFF);
                rep[4] = 0x03; /* SYSTEM_REPLY */
                rep[5] = 0x99; /* LIST_FILES */
                rep[6] = 0x00; /* SUCCESS */
                rep[7] = (unsigned char)(list_len & 0xFF);
                rep[8] = (unsigned char)((list_len >> 8) & 0xFF);
                rep[9] = (unsigned char)((list_len >> 16) & 0xFF);
                rep[10] = (unsigned char)((list_len >> 24) & 0xFF);
                rep[11] = 0x00; /* Handle: 0 = list completed in one chunk */
                memcpy(&rep[12], list_str, list_len);
                send(client_fd, rep, 2 + rep_payload_len, MSG_NOSIGNAL);
                fprintf(stderr, "[EV3 COM] LIST_FILES (%s): returned %d bytes (list_len=%d):\n%s", target_dir, 2 + rep_payload_len, list_len, list_str);
            } else if (sys_cmd == 0x9B) {
                /* CREATE_DIR */
                char *dirpath = (char *)&buf[6];
                char safe_path[512];
                ev3_resolve_path(dirpath, safe_path, sizeof(safe_path));
                mkdir(safe_path, 0755);
                ev3_sync_to_guest('D', ev3_clean_relpath(dirpath), NULL, 0);
                fprintf(stderr, "[EV3 COM] CREATE_DIR: %s -> %s\n", dirpath, safe_path);
                unsigned char rep[7] = {
                    0x05, 0x00,
                    (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                    0x03, 0x9B, 0x00
                };
                send(client_fd, rep, 7, MSG_NOSIGNAL);
            } else if (sys_cmd == 0x9C) {
                /* DELETE_FILE */
                char *filename = (char *)&buf[6];
                char safe_path[512];
                ev3_resolve_path(filename, safe_path, sizeof(safe_path));
                unlink(safe_path);
                rmdir(safe_path);
                ev3_sync_to_guest('R', ev3_clean_relpath(filename), NULL, 0);
                fprintf(stderr, "[EV3 COM] DELETE_FILE: %s -> %s\n", filename, safe_path);
                unsigned char rep[7] = {
                    0x05, 0x00,
                    (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                    0x03, 0x9C, 0x00
                };
                send(client_fd, rep, 7, MSG_NOSIGNAL);
            } else if (cmd_type == 0x01) {
                /* Generic system reply */
                unsigned char rep[7] = {
                    0x05, 0x00,
                    (unsigned char)(counter & 0xFF), (unsigned char)((counter >> 8) & 0xFF),
                    0x03, sys_cmd, 0x00
                };
                send(client_fd, rep, 7, MSG_NOSIGNAL);
            }
        } else {
            /* Direct Command */
            if (pkt_len < 5) continue;
            int num_globals = buf[5] | ((buf[6] & 0x03) << 8);
            if (num_globals < 0) num_globals = 0;
            if (num_globals > 4096) num_globals = 4096;
            unsigned char globals[4096] = { 0 };

            unsigned char *bc = &buf[7];
            int bc_len = (2 + pkt_len) - 7;
            int pc = 0;

            while (pc < bc_len) {
                unsigned char op = bc[pc++];
                if (op == 0x0C) {
                    /* opPROGRAM_INFO: subcode, slot, dest */
                    int sub = 0, slot = 0, dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &sub, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &slot, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                        pc += cons;
                        if (is_g && dest >= 0 && dest < (int)sizeof(globals)) {
                            /* EV3 lms2012 program status: 0x00 = IDLE/NONE, 0x10 = RUNNING, 0x12 = STOPPED */
                            globals[dest] = is_prg_running ? 0x10 : 0x00;
                        }
                    }
                } else if (op == 0x02) {
                    /* opPROGRAM_STOP */
                    int slot = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &slot, &is_g, &cons) == 0) pc += cons;
                    if (slot > 0) {
                        is_prg_running = false;
                        fprintf(stderr, "[EV3 COM] opPROGRAM_STOP (slot %d)\n", slot);
                        if (sp && sp->lcd_led[0]) {
                            qemu_set_irq(sp->lcd_led[0], 0);
                            qemu_set_irq(sp->lcd_led[1], 0);
                            qemu_set_irq(sp->lcd_led[2], 0);
                            qemu_set_irq(sp->lcd_led[3], 0);
                        }
                    }
                } else if (op == 0x7F) {
                    /* opMEMORY_READ: prgid, objid, offset, size, dest */
                    int prgid = 0, objid = 0, offset = 0, size = 0, dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &prgid, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &objid, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &offset, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &size, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                        pc += cons;
                        if (is_g && dest >= 0 && dest < (int)sizeof(globals)) {
                            int clear_sz = (size > 0 && dest + size <= (int)sizeof(globals)) ? size : 1;
                            memset(&globals[dest], 0, clear_sz);
                        }
                    }
                } else if (op == 0x0A) {
                    /* opOBJECT_END */
                    /* No arguments */
                } else if (op == 0x81) {
                    /* opUI_READ */
                    int sub = 0, len = 0, dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &sub, &is_g, &cons) < 0) break;
                    pc += cons;

                    switch (sub) {
                    case 0x01: /* GET_VBATT: float voltage */
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                            pc += cons;
                            if (is_g && dest >= 0 && dest + 4 <= (int)sizeof(globals)) {
                                float v = 8.0f;
                                memcpy(&globals[dest], &v, 4);
                            }
                        }
                        break;
                    case 0x02: /* GET_IBATT: float current */
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                            pc += cons;
                            if (is_g && dest >= 0 && dest + 4 <= (int)sizeof(globals)) {
                                float i = 0.15f;
                                memcpy(&globals[dest], &i, 4);
                            }
                        }
                        break;
                    case 0x03: /* GET_OS_VERS */
                    case 0x09: /* GET_HW_VERS */
                    case 0x0A: /* GET_FW_VERS */
                    case 0x0B: /* GET_FW_BUILD */
                    case 0x0C: /* GET_OS_BUILD */
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &len, &is_g, &cons) == 0) {
                            pc += cons;
                            if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                                pc += cons;
                                if (is_g && dest >= 0 && dest < (int)sizeof(globals)) {
                                    const char *str = "V1.09E";
                                    if (sub == 0x09) str = "V0.60";
                                    else if (sub == 0x03 || sub == 0x0C) str = "2.6.33-rc4";
                                    int max_copy = (len > 0 && len < 64) ? len : 32;
                                    if (dest + max_copy > (int)sizeof(globals)) {
                                        max_copy = sizeof(globals) - dest;
                                    }
                                    snprintf((char *)&globals[dest], max_copy, "%s", str);
                                }
                            }
                        }
                        break;
                    case 0x12: /* GET_LBATT: Data8 battery percent */
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                            pc += cons;
                            if (is_g && dest >= 0 && dest < (int)sizeof(globals)) {
                                globals[dest] = 100;
                            }
                        }
                        break;
                    default:
                        break;
                    }
                } else if (op == 0xD3) {
                    /* opCOM_GET */
                    int sub = 0, len = 0, dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &sub, &is_g, &cons) < 0) break;
                    pc += cons;
                    if (sub == 0x0D) {
                        /* GET_BRICKNAME */
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &len, &is_g, &cons) == 0) {
                            pc += cons;
                            if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                                pc += cons;
                                if (is_g && dest >= 0 && dest < (int)sizeof(globals)) {
                                    int max_copy = (len > 0 && len < 32) ? len : 16;
                                    if (dest + max_copy > (int)sizeof(globals)) {
                                        max_copy = sizeof(globals) - dest;
                                    }
                                    snprintf((char *)&globals[dest], max_copy, "EV3");
                                }
                            }
                        }
                    }
                } else if (op == 0xC5) {
                    /* opMEMORY_USAGE: TOTAL, FREE */
                    int dest_total = 0, dest_free = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &dest_total, &is_g, &cons) == 0) {
                        pc += cons;
                        if (is_g && dest_total >= 0 && dest_total + 4 <= (int)sizeof(globals)) {
                            uint32_t total = 16384;
                            memcpy(&globals[dest_total], &total, 4);
                        }
                    }
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &dest_free, &is_g, &cons) == 0) {
                        pc += cons;
                        if (is_g && dest_free >= 0 && dest_free + 4 <= (int)sizeof(globals)) {
                            uint32_t free_mem = 12288;
                            memcpy(&globals[dest_free], &free_mem, 4);
                        }
                    }
                } else if (op == 0x99) {
                    /* opINPUT_DEVICE */
                    int sub = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &sub, &is_g, &cons) < 0) break;
                    pc += cons;
                    if (sub == 0x05) {
                        /* GET_TYPEMODE: LAYER, NO, TYPE_DEST, MODE_DEST */
                        int layer = 0, no = 0, type_dest = 0, mode_dest = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &type_dest, &is_g, &cons) == 0) {
                            pc += cons;
                            if (is_g && type_dest >= 0 && type_dest < (int)sizeof(globals)) globals[type_dest] = 0;
                        }
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &mode_dest, &is_g, &cons) == 0) {
                            pc += cons;
                            if (is_g && mode_dest >= 0 && mode_dest < (int)sizeof(globals)) globals[mode_dest] = 0;
                        }
                    } else if (sub == 0x15) {
                        /* GET_NAME: LAYER, NO, LENGTH, DEST */
                        int layer = 0, no = 0, len = 0, dest = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &len, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                            pc += cons;
                            if (is_g && dest >= 0 && dest < (int)sizeof(globals)) globals[dest] = 0;
                        }
                    } else if (sub == 0x1D || sub == 0x1E || sub == 0x1F) {
                        /* READY_SI / READY_RAW / READY_PCT: LAYER, NO, TYPE, MODE, VALUES, DEST */
                        int layer = 0, no = 0, type = 0, mode = 0, values = 0, dest = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &type, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &mode, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &values, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &dest, &is_g, &cons) == 0) {
                            pc += cons;
                            if (is_g && dest >= 0 && dest + 4 <= (int)sizeof(globals)) {
                                memset(&globals[dest], 0, 4);
                            }
                        }
                    } else if (sub == 0x1C || sub == 0x1A) {
                        /* GET_FIGURES / GET_MINMAX: LAYER, NO, DEST1, DEST2 */
                        int layer = 0, no = 0, d1 = 0, d2 = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &d1, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &d2, &is_g, &cons) == 0) pc += cons;
                    } else if (sub == 0x16 || sub == 0x17 || sub == 0x1B) {
                        /* GET_RAW / GET_CONNECTION / GET_CHANGES: LAYER, NO, DEST */
                        int layer = 0, no = 0, d = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &d, &is_g, &cons) == 0) pc += cons;
                    } else if (sub == 0x07) {
                        /* GET_SYMBOL: LAYER, NO, LEN, DEST */
                        int layer = 0, no = 0, len = 0, d = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &len, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &d, &is_g, &cons) == 0) pc += cons;
                    } else if (sub == 0x0E) {
                        /* CLR_ALL: LAYER */
                        int layer = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                    } else if (sub == 0x0F || sub == 0x11 || sub == 0x18) {
                        /* CLR_CHANGES / GET_BUMPMIN / STOP: LAYER, NO */
                        int layer = 0, no = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                    }
                } else if (op == 0xA0) {
                    /* opOUTPUT_GET_TYPE: LAYER, NO, TYPE_DEST */
                    int layer = 0, no = 0, type_dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &type_dest, &is_g, &cons) == 0) {
                        pc += cons;
                        if (is_g && type_dest >= 0 && type_dest < (int)sizeof(globals)) globals[type_dest] = 0;
                    }
                } else if (op == 0xA8) {
                    /* opOUTPUT_READ: LAYER, NO, SPEED_DEST, COUNT_DEST */
                    int layer = 0, no = 0, s_dest = 0, c_dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &s_dest, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &c_dest, &is_g, &cons) == 0) pc += cons;
                } else if (op == 0xA9) {
                    /* opOUTPUT_TEST: LAYER, NO, BUSY_DEST */
                    int layer = 0, no = 0, b_dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &b_dest, &is_g, &cons) == 0) pc += cons;
                } else if (op == 0xAA) {
                    /* opOUTPUT_READY: LAYER, NO */
                    int layer = 0, no = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                } else if (op == 0xB2) {
                    /* opOUTPUT_GET_COUNT: LAYER, NO, COUNT_DEST */
                    int layer = 0, no = 0, c_dest = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &no, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &c_dest, &is_g, &cons) == 0) pc += cons;
                } else if (op == 0xB3) {
                    /* opOUTPUT_PRG_STOP: LAYER */
                    int layer = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &layer, &is_g, &cons) == 0) pc += cons;
                } else if (op == 0x94 && pc < bc_len && bc[pc] == 0x01) {
                    /* opSOUND TONE: [0x94, 0x01, 0x81, vol, 0x82, freq_lo, freq_hi, 0x82, dur_lo, dur_hi] */
                    pc++;
                    int vol = 0, freq = 0, dur = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &vol, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &freq, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &dur, &is_g, &cons) == 0) pc += cons;

                    sp->sound.freq_hz = freq ? freq : 1000;
                    sp->sound.volume = vol ? vol : 50;
                    sp->sound.enabled = true;

                    ToneStopArg *stop_arg = g_new0(ToneStopArg, 1);
                    stop_arg->sp = sp;
                    stop_arg->dur_ms = dur ? dur : 300;
                    QemuThread th;
                    qemu_thread_create(&th, "ev3-tone-stop", ev3_tone_stop_worker, stop_arg, QEMU_THREAD_DETACHED);
                } else if (op == 0x82 && pc < bc_len && bc[pc] == 0x1B) {
                    /* opUI_WRITE LED: pattern */
                    pc++;
                    int pat = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &pat, &is_g, &cons) == 0) pc += cons;
                    bool rl = false, gl = false, rr = false, gr = false;
                    switch (pat) {
                    case 1: gl = gr = true; break; /* green */
                    case 2: rl = rr = true; break; /* red */
                    case 3: rl = gl = rr = gr = true; break; /* orange */
                    case 4: gl = gr = true; break; /* flash green */
                    case 5: rl = rr = true; break; /* flash red */
                    case 6: rl = gl = rr = gr = true; break; /* flash orange */
                    case 7: gl = gr = true; break; /* pulse green */
                    case 8: rl = rr = true; break; /* pulse red */
                    case 9: rl = gl = rr = gr = true; break; /* pulse orange */
                    default: break; /* off */
                    }
                    if (sp->lcd_led[0]) {
                        qemu_set_irq(sp->lcd_led[0], rl);
                        qemu_set_irq(sp->lcd_led[1], gl);
                        qemu_set_irq(sp->lcd_led[2], rr);
                        qemu_set_irq(sp->lcd_led[3], gr);
                    }
                } else if (op == 0xC0) {
                    /* opFILE */
                    int sub = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &sub, &is_g, &cons) == 0) pc += cons;

                    if (sub == 0x08) {
                        /* LOAD_IMAGE: slot, name, size_dest, ip_dest */
                        int slot = 0;
                        const char *path = "";
                        int size_dest = 0, ip_dest = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &slot, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_str(&bc[pc], bc_len - pc, &path, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &size_dest, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &ip_dest, &is_g, &cons) == 0) pc += cons;

                        fprintf(stderr, "[EV3 COM] opFILE LOAD_IMAGE: slot=%d path='%s' size_dest=%d ip_dest=%d\n",
                                slot, path ? path : "", size_dest, ip_dest);

                        uint32_t file_sz = 1024;
                        char safe_path[512];
                        ev3_resolve_path(path, safe_path, sizeof(safe_path));
                        struct stat st;
                        if (stat(safe_path, &st) == 0 && st.st_size > 0) {
                            file_sz = st.st_size;
                        }

                        if (size_dest >= 0 && size_dest + 4 <= (int)sizeof(globals)) {
                            memcpy(&globals[size_dest], &file_sz, 4);
                        }
                        if (ip_dest >= 0 && ip_dest + 4 <= (int)sizeof(globals)) {
                            uint32_t fake_ip = 0x00010000; /* non-zero valid pointer so deployment succeeds */
                            memcpy(&globals[ip_dest], &fake_ip, 4);
                        }

                        /* Ensure file is synced to guest */
                        FILE *fp = fopen(safe_path, "rb");
                        if (fp) {
                            fseek(fp, 0, SEEK_END);
                            long fsz = ftell(fp);
                            fseek(fp, 0, SEEK_SET);
                            void *fdata = malloc(fsz);
                            if (fdata) {
                                if (fread(fdata, 1, fsz, fp) == (size_t)fsz) {
                                    ev3_sync_to_guest('F', ev3_clean_relpath(path), fdata, fsz);
                                }
                                free(fdata);
                            }
                            fclose(fp);
                        }
                    } else if (sub == 0x09 || sub == 0x0C) {
                        /* GET_IMAGE: slot, ip_dest */
                        int slot = 0, ip_dest = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &slot, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &ip_dest, &is_g, &cons) == 0) pc += cons;
                        if (ip_dest >= 0 && ip_dest + 4 <= (int)sizeof(globals)) {
                            uint32_t fake_ip = 0x00010000;
                            memcpy(&globals[ip_dest], &fake_ip, 4);
                        }
                    } else if (sub == 0x0A) {
                        /* GET_ITEM: slot, item, val_dest */
                        int slot = 0, item = 0, val_dest = 0;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &slot, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &item, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &val_dest, &is_g, &cons) == 0) pc += cons;
                        if (val_dest >= 0 && val_dest + 4 <= (int)sizeof(globals)) {
                            uint32_t val = 0;
                            memcpy(&globals[val_dest], &val, 4);
                        }
                    } else if (sub == 0x0D) {
                        /* GET_CACHE_FILE: name, size_dest, ip_dest */
                        const char *path = "";
                        int size_dest = 0, ip_dest = 0;
                        if (ev3_decode_str(&bc[pc], bc_len - pc, &path, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &size_dest, &is_g, &cons) == 0) pc += cons;
                        if (ev3_decode_par(&bc[pc], bc_len - pc, &ip_dest, &is_g, &cons) == 0) pc += cons;
                        if (size_dest >= 0 && size_dest + 4 <= (int)sizeof(globals)) {
                            uint32_t z = 0;
                            memcpy(&globals[size_dest], &z, 4);
                        }
                        if (ip_dest >= 0 && ip_dest + 4 <= (int)sizeof(globals)) {
                            uint32_t z = 0;
                            memcpy(&globals[ip_dest], &z, 4);
                        }
                    } else if (sub == 0x1E) {
                        /* REMOVE_SUBFOLDERS: name */
                        const char *path = "";
                        if (ev3_decode_str(&bc[pc], bc_len - pc, &path, &cons) == 0) pc += cons;
                    } else {
                        fprintf(stderr, "[EV3 COM] opFILE: unhandled subcode 0x%02X\n", sub);
                    }
                } else if (op == 0x03) {
                    /* opPROGRAM_START: slot, size, ip, debug */
                    int slot = 0, size = 0, ip = 0, debug = 0, cons = 0;
                    bool is_g = false;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &slot, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &size, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &ip, &is_g, &cons) == 0) pc += cons;
                    if (ev3_decode_par(&bc[pc], bc_len - pc, &debug, &is_g, &cons) == 0) pc += cons;

                    if (slot == 1) {
                        is_prg_running = true;
                        fprintf(stderr, "[EV3 COM] opPROGRAM_START: user program slot=%d size=%d ip=%d debug=%d\n",
                                slot, size, ip, debug);

                        /* Play start sound & green LED on brick */
                        if (sp) {
                            sp->sound.freq_hz = 1500;
                            sp->sound.volume = 60;
                            sp->sound.enabled = true;
                            ToneStopArg *stop_arg = g_new0(ToneStopArg, 1);
                            stop_arg->sp = sp;
                            stop_arg->dur_ms = 250;
                            QemuThread th;
                            qemu_thread_create(&th, "ev3-tone-stop", ev3_tone_stop_worker, stop_arg, QEMU_THREAD_DETACHED);
                            if (sp->lcd_led[0]) {
                                qemu_set_irq(sp->lcd_led[1], 1);
                                qemu_set_irq(sp->lcd_led[3], 1);
                            }
                        }
                    }
                } else {
                    /* Skip unknown opcode: op was already consumed at loop top */
                }
            }

            /* Reply if requested (Type 0x00) */
            if (cmd_type == 0x00) {
                int rep_globals = num_globals;
                if (rep_globals == 0 && globals[4] != 0) {
                    rep_globals = 8;
                }
                int rep_len = 3 + rep_globals;
                unsigned char *rep = g_malloc0(2 + rep_len);
                rep[0] = rep_len & 0xFF;
                rep[1] = (rep_len >> 8) & 0xFF;
                rep[2] = (unsigned char)(counter & 0xFF);
                rep[3] = (unsigned char)((counter >> 8) & 0xFF);
                rep[4] = 0x02; /* DIRECT_REPLY */
                if (rep_globals > 0) {
                    memcpy(&rep[5], globals, rep_globals);
                }
                send(client_fd, rep, 2 + rep_len, MSG_NOSIGNAL);
                fprintf(stderr, "[EV3 COM] Sent Direct Reply (%d bytes, globals=%d)\n", 2 + rep_len, rep_globals);
                g_free(rep);
            }
        }
    }

    for (int i = 0; i < 8; i++) {
        if (file_handles[i] > 0) close(file_handles[i]);
    }
    close(client_fd);
}

static void *ev3_tcp_server_worker(void *opaque)
{
    signal(SIGPIPE, SIG_IGN);
    EV3ComServer *srv = opaque;
    fprintf(stderr, "[EV3 COM] TCP server worker thread started...\n");
    int server_fd = -1;

    while (1) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            g_usleep(250000);
            continue;
        }

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(EV3_TCP_PORT);

        if (bind(server_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == 0) {
            break;
        }
        fprintf(stderr, "[EV3 COM] TCP bind failed: %s (errno=%d)\n", strerror(errno), errno);
        close(server_fd);
        g_usleep(500000);
    }

    listen(server_fd, 5);
    fprintf(stderr, "[EV3 COM] TCP Server listening on port %d\n", EV3_TCP_PORT);

    while (1) {
        struct sockaddr_in cli_addr;
        socklen_t clilen = sizeof(cli_addr);
        int client_fd = accept(server_fd, (struct sockaddr *)&cli_addr, &clilen);
        if (client_fd < 0) continue;

        ev3_handle_client(srv->sp, client_fd);
    }
    close(server_fd);
    return NULL;
}

static void ev3_com_init(DA850SimplePeripherals *sp)
{
    EV3ComServer *srv = g_new0(EV3ComServer, 1);
    srv->sp = sp;

    qemu_thread_create(&srv->discovery_thread, "ev3-discovery",
                       ev3_discovery_worker, srv, QEMU_THREAD_DETACHED);
    qemu_thread_create(&srv->tcp_thread, "ev3-tcp-srv",
                       ev3_tcp_server_worker, srv, QEMU_THREAD_DETACHED);
}

/*
 * EV3 Brick 6-button mapping read by d_ui.ko (Device2TimerInterrupt1 -> IN_DATA):
 *   0: UP     -> Pin 0x7f (GP7_15) -> bank 3, bit 31 (1u << 31)
 *   1: ENTER  -> Pin 0x01 (GP0_1)  -> bank 0, bit  1 (1u << 1)  [EP2: Pin 0x1d -> bit 29]
 *   2: DOWN   -> Pin 0x7e (GP7_14) -> bank 3, bit 30 (1u << 30)
 *   3: RIGHT  -> Pin 0x7c (GP7_12) -> bank 3, bit 28 (1u << 28)
 *   4: LEFT   -> Pin 0x66 (GP6_6)  -> bank 3, bit  6 (1u << 6)
 *   5: BACK   -> Pin 0x6a (GP6_10) -> bank 3, bit 10 (1u << 10)
 */
static void ev3_gpio_button_irq(void *opaque, int n, int level)
{
    DA850SimplePeripherals *s = opaque;
    unsigned bank = 0;
    uint32_t mask = 0;

    switch (n) {
    case 0: /* UP */
        bank = 3;
        mask = (1u << 31);
        break;
    case 1: /* ENTER */
        bank = 0;
        mask = (1u << 1) | (1u << 29);
        break;
    case 2: /* DOWN */
        bank = 3;
        mask = (1u << 30);
        break;
    case 3: /* RIGHT */
        bank = 3;
        mask = (1u << 28);
        break;
    case 4: /* LEFT */
        bank = 3;
        mask = (1u << 6);
        break;
    case 5: /* BACK */
        bank = 3;
        mask = (1u << 10);
        break;
    default:
        return;
    }

    if (level) {
        s->gpio_buttons[bank] |= mask;
    } else {
        s->gpio_buttons[bank] &= ~mask;
    }
}

static uint64_t da850_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    DA850SimplePeripherals *s = opaque;
    unsigned bank = offset / 0x28;
    unsigned reg = offset % 0x28;

    if (bank < 10 && reg == 0x20) {
        return s->gpio[bank * 0x28 / 4 + 0x14 / 4] | s->gpio_buttons[bank];
    }
    if (offset < sizeof(s->gpio)) {
        return s->gpio[offset / 4];
    }
    return 0;
}

static void da850_gpio_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    DA850SimplePeripherals *s = opaque;
    unsigned bank = offset / 0x28;
    unsigned reg = offset % 0x28;
    unsigned index = bank * 0x28 / 4 + 0x14 / 4;

    if (bank < 10 && reg == 0x18) {
        s->gpio[index] |= value;
    } else if (bank < 10 && reg == 0x1c) {
        s->gpio[index] &= ~value;
    } else if (offset < sizeof(s->gpio)) {
        s->gpio[offset / 4] = value;
    }

    /* GPIO2_11 (pin 43: bank 1, bit 11) is ST7586 A0 (D/C) */
    if (bank == 1 && s->lcd_cd) {
        qemu_set_irq(s->lcd_cd, (s->gpio[1 * 0x28 / 4 + 0x14 / 4] >> 11) & 1);
    }

    /*
     * EV3 Status LEDs (d_ui.ko FINALB_UiLedPin):
     *   Pin 108 (bank 3, bit 12): Red Left   (Entry 0)
     *   Pin 33  (bank 1, bit 1):  Green Left (Entry 1)
     *   Pin 109 (bank 3, bit 13): Red Right  (Entry 2)
     *   Pin 103 (bank 3, bit 7):  Green Right(Entry 3)
     */
    if (s->lcd_led[0] && (bank == 1 || bank == 3)) {
        bool red_l   = (s->gpio[3 * 0x28 / 4 + 0x14 / 4] >> 12) & 1;
        bool green_l = (s->gpio[1 * 0x28 / 4 + 0x14 / 4] >> 1)  & 1;
        bool red_r   = (s->gpio[3 * 0x28 / 4 + 0x14 / 4] >> 13) & 1;
        bool green_r = (s->gpio[3 * 0x28 / 4 + 0x14 / 4] >> 7)  & 1;
        qemu_set_irq(s->lcd_led[0], red_l);
        qemu_set_irq(s->lcd_led[1], green_l);
        qemu_set_irq(s->lcd_led[2], red_r);
        qemu_set_irq(s->lcd_led[3], green_r);
    }

    /* Pin 111 (bank 3, bit 15) and Pin 161 (bank 5, bit 1): EV3 Audio Enable */
    if (bank == 3 || bank == 5) {
        bool amp111 = (s->gpio[3 * 0x28 / 4 + 0x14 / 4] >> 15) & 1;
        bool amp161 = (s->gpio[5 * 0x28 / 4 + 0x14 / 4] >> 1) & 1;
        if (!amp111 && !amp161) {
            s->sound.enabled = false;
        }
    }
}

static const MemoryRegionOps da850_gpio_ops = {
    .read = da850_gpio_read,
    .write = da850_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t da850_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    DA850SimplePeripherals *s = opaque;
    uint64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                       s->timer_start_ns;

    if (offset == 5 * 4) {
        return (elapsed * 24 / 1000) & 0xffffffff;
    }
    return 0;
}

static void da850_timer_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    /* Timer control writes are accepted; the counter is free-running. */
}

static const MemoryRegionOps da850_timer_ops = {
    .read = da850_timer_read,
    .write = da850_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t da850_pwm0_read(void *opaque, hwaddr offset, unsigned size)
{
    DA850SimplePeripherals *s = opaque;

    if (offset + size <= sizeof(s->pwm0)) {
        uint64_t val = 0;
        memcpy(&val, &s->pwm0[offset], size);
        return val;
    }
    return 0;
}

static void da850_pwm0_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    DA850SimplePeripherals *s = opaque;

    if (offset + size <= sizeof(s->pwm0)) {
        memcpy(&s->pwm0[offset], &value, size);
    }

    /*
     * eHRPWM0 Sound Controller:
     * Base: 0x01f00000
     * Offset 0x0a: TBPRD (Period). Tone frequency = 1,031,250 / (TBPRD + 1)
     * Offset 0x14: CMPA  (Duty). Volume = (CMPA * 200) / TBPRD (0..100%)
     * Offset 0x00: TBCTL (Control). CTRMODE == 3 (0x03) stops counter
     */
    uint16_t tbprd = *(uint16_t *)&s->pwm0[0x0a];
    uint16_t cmpa  = *(uint16_t *)&s->pwm0[0x14];
    uint16_t tbctl = *(uint16_t *)&s->pwm0[0x00];

    if (tbprd >= 10 && cmpa > 0 && (tbctl & 0x0003) != 0x0003) {
        uint32_t freq = 1031250u / (tbprd + 1);
        uint32_t vol  = (cmpa * 200u) / tbprd;
        if (vol > 100) vol = 100;

        s->sound.freq_hz = freq;
        s->sound.volume = vol;
        s->sound.enabled = true;
    } else if (tbprd == 0 || cmpa == 0 || (tbctl & 0x0003) == 0x0003) {
        s->sound.enabled = false;
    }
}

static const MemoryRegionOps da850_pwm0_ops = {
    .read = da850_pwm0_read,
    .write = da850_pwm0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t da850_pwm1_read(void *opaque, hwaddr offset, unsigned size)
{
    DA850SimplePeripherals *s = opaque;

    if (offset + size <= sizeof(s->pwm1)) {
        uint64_t val = 0;
        memcpy(&val, &s->pwm1[offset], size);
        return val;
    }
    return 0;
}

static void da850_pwm1_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    DA850SimplePeripherals *s = opaque;

    if (offset + size <= sizeof(s->pwm1)) {
        memcpy(&s->pwm1[offset], &value, size);
    }
}

static const MemoryRegionOps da850_pwm1_ops = {
    .read = da850_pwm1_read,
    .write = da850_pwm1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
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
    uint32_t ctrl;
    uint32_t intr_src;
    uint32_t intr_mask;
} DA850USBState;



static uint64_t da850_usb_read(void *opaque, hwaddr addr, unsigned size)
{
    DA850USBState *s = opaque;

    switch (addr) {
    case 0x00: return 0x00000100;
    case 0x04: return s->ctrl;
    case 0x08: return 1;
    case 0x20: return s->intr_src;
    case 0x2c: return s->intr_mask;
    case 0x38: return s->intr_src & s->intr_mask;
    default: break;
    }
    if (addr < 0x400) {
        return 0;
    }
    addr -= 0x400;
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

    switch (addr) {
    case 0x04:
        s->ctrl = val;
        if (val & 1) {
            musb_reset(s->musb);
        }
        return;
    case 0x24: s->intr_src |= val; return;
    case 0x28: s->intr_src &= ~val; return;
    case 0x30: s->intr_mask |= val; return;
    case 0x34: s->intr_mask &= ~val; return;
    case 0x3c: s->intr_src = 0; return;
    default: break;
    }
    if (addr < 0x400) {
        return;
    }
    addr -= 0x400;
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
    ev3_ram_mr = machine->ram;

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
    Chardev *chr2 = serial_hd(2);
    if (!chr2) {
        unlink("/tmp/ev3_guest_bridge.sock");
        chr2 = qemu_chr_new("ev3_guest_bridge", "unix:/tmp/ev3_guest_bridge.sock,server=on,wait=off", NULL);
    }
    serial_mm_init(get_system_memory(), 0x01d0d000, 2, qdev_get_gpio_in(aintc, 61), 115200, chr2, DEVICE_LITTLE_ENDIAN);

    /* vpb_init の中に追加 */
    MemoryRegion *syscfg0 = g_new(MemoryRegion, 1);
    memory_region_init_io(syscfg0, NULL, &da850_syscfg_ops, NULL, "da850-syscfg0", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x01C14000, syscfg0);

    DA850SimplePeripherals *simple_peripherals = g_new0(DA850SimplePeripherals, 1);
    simple_peripherals->timer_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    memory_region_init_io(&simple_peripherals->gpio_mmio, OBJECT(machine),
                          &da850_gpio_ops, simple_peripherals,
                          "da850-gpio", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x01e26000,
                                &simple_peripherals->gpio_mmio);
    memory_region_init_io(&simple_peripherals->timer_mmio, OBJECT(machine),
                          &da850_timer_ops, simple_peripherals,
                          "da850-timer64p3", 0x80);
    memory_region_add_subregion(get_system_memory(), 0x01f0d000,
                                &simple_peripherals->timer_mmio);
    memory_region_init_io(&simple_peripherals->ehrpwm0_mmio, OBJECT(machine),
                          &da850_pwm0_ops, simple_peripherals,
                          "da850-ehrpwm0", 0x2000);
    memory_region_add_subregion(get_system_memory(), 0x01f00000,
                                &simple_peripherals->ehrpwm0_mmio);
    memory_region_init_io(&simple_peripherals->ehrpwm1_mmio, OBJECT(machine),
                          &da850_pwm1_ops, simple_peripherals,
                          "da850-ehrpwm1", 0x2854);
    memory_region_add_subregion(get_system_memory(), 0x01f02000,
                                &simple_peripherals->ehrpwm1_mmio);
    memory_region_init_io(&simple_peripherals->ecap0_mmio, OBJECT(machine),
                          &da850_pwm1_ops, simple_peripherals,
                          "da850-ecap0", 0x60);
    memory_region_add_subregion(get_system_memory(), 0x01f06000,
                                &simple_peripherals->ecap0_mmio);
    memory_region_init_io(&simple_peripherals->ecap1_mmio, OBJECT(machine),
                          &da850_pwm1_ops, simple_peripherals,
                          "da850-ecap1", 0x60);
    memory_region_add_subregion(get_system_memory(), 0x01f07000,
                                &simple_peripherals->ecap1_mmio);

    ev3_sound_init(&simple_peripherals->sound);
    ev3_com_init(simple_peripherals);
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

DeviceState *i2c0 = qdev_new("da850-i2c");
sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c0), &error_fatal);
sysbus_mmio_map(SYS_BUS_DEVICE(i2c0), 0, 0x01c22000);
sysbus_connect_irq(SYS_BUS_DEVICE(i2c0), 0, qdev_get_gpio_in(aintc, 15));

    DeviceState *mmc = qdev_new("da850-mmc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mmc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mmc), 0, 0x01c40000);
    sysbus_connect_irq(SYS_BUS_DEVICE(mmc), 0, qdev_get_gpio_in(aintc, 16));

    DeviceState *sdcard = qdev_new(TYPE_SD_CARD);
    dinfo = drive_get(IF_SD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive_err(sdcard, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
    }
    qdev_realize_and_unref(sdcard, qdev_get_child_bus(mmc, "sd-bus"),
                           &error_fatal);

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

    /* USB 1.1 OHCI Host Controller (0x01e25000, IRQ 59) */
    sysbus_create_simple("sysbus-ohci", 0x01e25000, qdev_get_gpio_in(aintc, 59));


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
/* EDMA3 CC0 */
DeviceState *edma = qdev_new("da850-edma");
sysbus_realize_and_unref(SYS_BUS_DEVICE(edma), &error_fatal);
sysbus_mmio_map(SYS_BUS_DEVICE(edma), 0, 0x01c00000);
sysbus_connect_irq(SYS_BUS_DEVICE(edma), 0, qdev_get_gpio_in(aintc, 11)); /* IRQ_DA8XX_CCINT0 */

/* SPI0 + m25p80 flash */
DeviceState *spi0 = qdev_new("da850-spi");
sysbus_realize_and_unref(SYS_BUS_DEVICE(spi0), &error_fatal);
sysbus_mmio_map(SYS_BUS_DEVICE(spi0), 0, 0x01f0e000); /* SPI0の物理アドレス、要確認 */
sysbus_connect_irq(SYS_BUS_DEVICE(spi0), 0, qdev_get_gpio_in(aintc, 20)); /* IRQ_DA8XX_SPINT0 */
DeviceState *flash = qdev_new("s25sl12801");

SSIBus *spi0_bus =
    (SSIBus *)qdev_get_child_bus(spi0, "spi");

qdev_realize_and_unref(flash, BUS(spi0_bus), &error_fatal);

qemu_irq cs_line =
    qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0);

sysbus_connect_irq(SYS_BUS_DEVICE(spi0), 1, cs_line);
#endif

/*
 * EDMA
 */
DeviceState *edma =
    qdev_new("da850-edma");

sysbus_realize_and_unref(
    SYS_BUS_DEVICE(edma),
    &error_fatal
);

sysbus_mmio_map(
    SYS_BUS_DEVICE(edma),
    0,
    0x01c00000
);

sysbus_connect_irq(
    SYS_BUS_DEVICE(edma),
    0,
    qdev_get_gpio_in(aintc, 11)
); /* IRQ_DA8XX_CCINT0 */


/*
 * SPI0 (Flash m25p80 / s25sl12801)
 * Physical address: 0x01c41000, IRQ: 56 (SPINT0)
 */
DeviceState *spi0 = qdev_new("da850-spi");
sysbus_realize_and_unref(SYS_BUS_DEVICE(spi0), &error_fatal);
sysbus_mmio_map(SYS_BUS_DEVICE(spi0), 0, 0x01c41000);
sysbus_connect_irq(SYS_BUS_DEVICE(spi0), 0, qdev_get_gpio_in(aintc, 56)); /* IRQ_DA8XX_SPINT0 */

DeviceState *flash = qdev_new("s25sl12801");
SSIBus *spi0_bus = (SSIBus *)qdev_get_child_bus(spi0, "spi");
qdev_realize_and_unref(flash, BUS(spi0_bus), &error_fatal);

uint8_t *fstorage = m25p80_get_storage(flash);
if (fstorage) {
    const uint32_t storage_offset = 0x00cb0000;
    const uint32_t max_size = 0x00350000;
    FILE *fp = fopen("/tmp/ev3_data.tgz", "rb");
    if (!fp) {
        fp = fopen("/tmp/orig_fs/etc/def_sett.tgz", "rb");
    }
    if (fp) {
        fseek(fp, 0, SEEK_END);
        long sz = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        if (sz > 0 && sz <= max_size) {
            size_t rd = fread(fstorage + storage_offset, 1, sz, fp);
            fprintf(stderr, "[EV3 FLASH] Loaded %zu bytes into MTD4 storage (offset 0x%08x)\n", rd, storage_offset);
        }
        fclose(fp);
    }
}

qemu_irq cs_line0 = qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0);
sysbus_connect_irq(SYS_BUS_DEVICE(spi0), 1, cs_line0);

/*
 * SPI1 (Display ST7586 LCD)
 * Physical address: 0x01f0e000, IRQ: 20 (SPINT1)
 */
DeviceState *spi1 = qdev_new("da850-spi");
sysbus_realize_and_unref(SYS_BUS_DEVICE(spi1), &error_fatal);
sysbus_mmio_map(SYS_BUS_DEVICE(spi1), 0, 0x01f0e000);
sysbus_connect_irq(SYS_BUS_DEVICE(spi1), 0, qdev_get_gpio_in(aintc, 20)); /* IRQ_DA8XX_SPINT1 */

DeviceState *lcd = qdev_new("st7586");
SSIBus *spi1_bus = (SSIBus *)qdev_get_child_bus(spi1, "spi");
qdev_realize_and_unref(lcd, BUS(spi1_bus), &error_fatal);

qemu_irq cs_line1 = qdev_get_gpio_in_named(lcd, SSI_GPIO_CS, 0);
sysbus_connect_irq(SYS_BUS_DEVICE(spi1), 1, cs_line1);
simple_peripherals->lcd_cd = qdev_get_gpio_in(lcd, 0);
for (int i = 0; i < 4; i++) {
    simple_peripherals->lcd_led[i] = qdev_get_gpio_in_named(lcd, "ev3-led", i);
}

qemu_irq *ev3_btn_irqs = qemu_allocate_irqs(ev3_gpio_button_irq,
                                            simple_peripherals, 6);
for (int i = 0; i < 6; i++) {
    qdev_connect_gpio_out(lcd, i, ev3_btn_irqs[i]);
}


/*
 * ============================================================
 * SPI0 DMA TX request -> EDMA event 15
 * ============================================================
 *
 * SPI:
 *
 *     "dma-tx"
 *          |
 *          v
 *
 * EDMA:
 *
 *     event[15]
 *
 * Linux configured:
 *
 *     SPI0 TX = EDMA channel 15
 */
/*
qemu_irq edma_spi0_tx_event =
    qdev_get_gpio_in_named(
        edma,
        "event",
        15
    );

qdev_connect_gpio_out_named(
    spi0,
    "dma-tx",
    0,
    edma_spi0_tx_event
);*/

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
    versatile_binfo.board_id = board_id ? board_id : 0x86d;
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

static void ev3_init(MachineState *machine)
{
    versatile_init(machine, 0x86d);
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

static void ev3_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "LEGO MINDSTORMS EV3 (AM1808/DA850)";
    mc->init = ev3_init;
    mc->block_default_type = IF_SD;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_id = "ev3.ram";
    mc->default_ram_size = 64 * MiB;
}

static const TypeInfo ev3_type = {
    .name = MACHINE_TYPE_NAME("ev3"),
    .parent = TYPE_MACHINE,
    .class_init = ev3_class_init,
};

static const TypeInfo legoev3_type = {
    .name = MACHINE_TYPE_NAME("legoev3"),
    .parent = TYPE_MACHINE,
    .class_init = ev3_class_init,
};

static void versatile_machine_init(void)
{
    type_register_static(&versatilepb_type);
    type_register_static(&versatileab_type);
    type_register_static(&ev3_type);
    type_register_static(&legoev3_type);
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
