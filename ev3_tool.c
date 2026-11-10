/*
 * EV3 Native PC Tool (C Language - Zero Python Dependencies)
 * Usage:
 *   ./ev3_tool tone <freq_hz> <duration_ms> <volume%>
 *   ./ev3_tool led <off|green|red|orange|flash-green|flash-red|flash-orange>
 *   ./ev3_tool upload <local_file> <remote_path>
 *   ./ev3_tool run <remote_path>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define EV3_DEFAULT_IP   "127.0.0.1"
#define EV3_DEFAULT_PORT 5555

static int connect_ev3(const char *ip, int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);

    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void send_direct_cmd(int fd, const unsigned char *bc, int bc_len) {
    unsigned char pkt[1024];
    int payload_len = 5 + bc_len; /* counter(2) + type(1) + alloc(2) + bc */
    pkt[0] = payload_len & 0xFF;
    pkt[1] = (payload_len >> 8) & 0xFF;
    pkt[2] = 0x01; pkt[3] = 0x00; /* counter */
    pkt[4] = 0x80;                 /* DIRECT_COMMAND_NO_REPLY */
    pkt[5] = 0x00; pkt[6] = 0x00; /* 0 globals, 0 locals */
    memcpy(&pkt[7], bc, bc_len);
    send(fd, pkt, 2 + payload_len, 0);
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <tone|led|upload|run> [args...]\n", argv[0]);
        return 1;
    }

    const char *cmd = argv[1];
    int fd = connect_ev3(EV3_DEFAULT_IP, EV3_DEFAULT_PORT);
    if (fd < 0) {
        fprintf(stderr, "[!] Could not connect to EV3 at %s:%d\n", EV3_DEFAULT_IP, EV3_DEFAULT_PORT);
        return 1;
    }

    if (strcmp(cmd, "info") == 0 || strcmp(cmd, "version") == 0) {
        /* Direct Command with Reply: Query FW_VERS (at global 0) and HW_VERS (at global 16) */
        unsigned char pkt[64];
        unsigned char bc[] = {
            0x81, 0x0A, 0x10, 0x60,        /* opUI_READ GET_FW_VERS(len=16, dest=globals[0]) */
            0x81, 0x09, 0x10, 0x60 | 0x10, /* opUI_READ GET_HW_VERS(len=16, dest=globals[16]) */
            0x81, 0x01, 0x60 | 0x08        /* opUI_READ GET_VBATT(dest=globals[8]) */
        };
        int bc_len = sizeof(bc);
        int payload_len = 5 + bc_len;
        pkt[0] = payload_len & 0xFF;
        pkt[1] = (payload_len >> 8) & 0xFF;
        pkt[2] = 0x42; pkt[3] = 0x00; /* counter */
        pkt[4] = 0x00;                 /* DIRECT_COMMAND_REPLY */
        pkt[5] = 32;   pkt[6] = 0x00; /* 32 globals */
        memcpy(&pkt[7], bc, bc_len);

        send(fd, pkt, 2 + payload_len, 0);

        unsigned char rep[64] = { 0 };
        int n = recv(fd, rep, sizeof(rep), 0);
        if (n >= 5 && rep[4] == 0x02) {
            printf("[*] Connected to EV3 Brick!\n");
            printf("    Firmware Version: %s\n", (char *)&rep[5]);
            printf("    Hardware Version: %s\n", (char *)&rep[5 + 16]);
            float vbatt = 0;
            memcpy(&vbatt, &rep[5 + 8], 4);
            printf("    Battery Voltage : %.2f V\n", vbatt);
        } else {
            fprintf(stderr, "[!] Failed to get info reply (n=%d, rep[4]=0x%02x)\n", n, n >= 5 ? rep[4] : 0);
        }
    } else if (strcmp(cmd, "tone") == 0) {
        int freq = argc > 2 ? atoi(argv[2]) : 1000;
        int dur  = argc > 3 ? atoi(argv[3]) : 500;
        int vol  = argc > 4 ? atoi(argv[4]) : 50;
        unsigned char bc[10] = {
            0x94, 0x01, 0x81, (unsigned char)vol,
            0x82, (unsigned char)(freq & 0xFF), (unsigned char)((freq >> 8) & 0xFF),
            0x82, (unsigned char)(dur & 0xFF), (unsigned char)((dur >> 8) & 0xFF)
        };
        send_direct_cmd(fd, bc, sizeof(bc));
        printf("[*] Played tone: %d Hz, %d ms, volume %d%%\n", freq, dur, vol);
    } else if (strcmp(cmd, "led") == 0) {
        const char *col = argc > 2 ? argv[2] : "green";
        int pat = 1;
        if (strcmp(col, "off") == 0) pat = 0;
        else if (strcmp(col, "green") == 0) pat = 1;
        else if (strcmp(col, "red") == 0) pat = 2;
        else if (strcmp(col, "orange") == 0) pat = 3;
        else if (strstr(col, "flash") && strstr(col, "green")) pat = 4;
        else if (strstr(col, "flash") && strstr(col, "red")) pat = 5;
        else if (strstr(col, "flash")) pat = 6;
        unsigned char bc[4] = { 0x82, 0x1B, 0x81, (unsigned char)pat };
        send_direct_cmd(fd, bc, sizeof(bc));
        printf("[*] Set LED pattern: %s (%d)\n", col, pat);
    } else if (strcmp(cmd, "upload") == 0) {
        if (argc < 4) {
            fprintf(stderr, "Usage: %s upload <local_file> <remote_path>\n", argv[0]);
            close(fd);
            return 1;
        }
        const char *lpath = argv[2];
        const char *rpath = argv[3];
        FILE *fp = fopen(lpath, "rb");
        if (!fp) {
            fprintf(stderr, "Cannot open %s\n", lpath);
            close(fd);
            return 1;
        }
        fseek(fp, 0, SEEK_END);
        long size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        /* BEGIN_DOWNLOAD */
        int rlen = strlen(rpath) + 1;
        int psize = 8 + rlen;
        unsigned char pkt[512];
        pkt[0] = psize & 0xFF; pkt[1] = (psize >> 8) & 0xFF;
        pkt[2] = 0x0A; pkt[3] = 0x00;
        pkt[4] = 0x01; pkt[5] = 0x92;
        pkt[6] = size & 0xFF; pkt[7] = (size >> 8) & 0xFF;
        pkt[8] = (size >> 16) & 0xFF; pkt[9] = (size >> 24) & 0xFF;
        memcpy(&pkt[10], rpath, rlen);
        send(fd, pkt, 2 + psize, 0);

        unsigned char rep[64];
        recv(fd, rep, 64, 0);
        int handle = rep[7];

        /* CONTINUE_DOWNLOAD */
        unsigned char cbuf[800];
        long total_sent = 0;
        while (total_sent < size) {
            int chunk = fread(cbuf, 1, 800, fp);
            if (chunk <= 0) break;
            int c_psize = 4 + chunk;
            unsigned char cpkt[820];
            cpkt[0] = c_psize & 0xFF; cpkt[1] = (c_psize >> 8) & 0xFF;
            cpkt[2] = 0x0B; cpkt[3] = 0x00;
            cpkt[4] = 0x01; cpkt[5] = 0x93;
            cpkt[6] = (unsigned char)handle;
            memcpy(&cpkt[7], cbuf, chunk);
            send(fd, cpkt, 2 + c_psize, 0);
            recv(fd, rep, 64, 0);
            total_sent += chunk;
        }
        fclose(fp);

        /* CLOSE_FILEHANDLE */
        unsigned char clpkt[8] = { 0x04, 0x00, 0x0C, 0x00, 0x01, 0x94, (unsigned char)handle };
        send(fd, clpkt, 7, 0);
        recv(fd, rep, 64, 0);
        printf("[+] Uploaded '%s' (%ld bytes) -> '%s'\n", lpath, size, rpath);
    } else if (strcmp(cmd, "run") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Usage: %s run <remote_path>\n", argv[0]);
            close(fd);
            return 1;
        }
        const char *rpath = argv[2];
        int rlen = strlen(rpath) + 1;
        unsigned char bc[256];
        bc[0] = 0x03; bc[1] = 0x81; bc[2] = 0x01;
        memcpy(&bc[3], rpath, rlen);
        send_direct_cmd(fd, bc, 3 + rlen);
        printf("[*] Started execution of '%s'\n", rpath);
    }

    close(fd);
    return 0;
}
