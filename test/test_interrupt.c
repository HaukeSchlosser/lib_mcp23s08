#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <lib_mcp23s08.h>

#define CHIPDEV   "/dev/gpiochip0"
#define INT_LINE  17

int main() {
    int fd;
    uint8_t addr = 0;

    if ((fd = mcp23s08_init(0, 0, SPI_MODE_0)) < 0) {
        fprintf(stderr, "[main] Initialization failed\n");
        close(fd);
        return 1;
    }
    printf("[test_interrupt] MCP23S08 initialized successfully with fd: %d\n", fd);

    if (mcp23s08_enable_interrupt(fd, addr, INT_ENABLE, 0xFF, INT_ANY) < 0) {
        fprintf(stderr, "[main] enable_interrupt failed\n");
        close(fd);
        return 1;
    }

    mcp23s08_read(fd, addr, INTCAP);
    int fd_chip = open(CHIPDEV, O_RDONLY);
    if (fd_chip < 0) { 
        perror("open gpiochip"); 
        return 1; 
    }

    struct gpioevent_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffset   = INT_LINE;
    req.handleflags  = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags   = GPIOEVENT_REQUEST_FALLING_EDGE;
    strncpy(req.consumer_label, "mcp23s08-int", sizeof(req.consumer_label)-1);

    if (ioctl(fd_chip, GPIO_GET_LINEEVENT_IOCTL, &req) < 0) {
        perror("GPIO_GET_LINEEVENT_IOCTL");
        close(fd_chip);
        close(fd);
        return 1;
    }
    close(fd_chip);

    printf("[test_interrupt] Run test program...\n");
    printf("[test_interrupt] Press crtl + c to stop...\n");

    while(1) {
        struct gpioevent_data ev;
        ssize_t n = read(req.fd, &ev, sizeof(ev));
        if (n != (ssize_t)sizeof(ev)) {
            if (n < 0) perror("read event");
            else fprintf(stderr, "short read on event fd\n");
            break;
        }

        int16_t inf = mcp23s08_read(fd, addr, INTF);
        int16_t cap = mcp23s08_read(fd, addr, INTCAP);
        if (inf >= 0 && cap >= 0) {
            uint8_t f = (uint8_t)inf;
            uint8_t c = (uint8_t)cap;

            int pin = -1;
            for (int i = 0; i < 8; ++i) {
                if (f & (1u << i)) { pin = i; break; }
            }

            if (pin >= 0) {
                printf("INT pin: %d  state: %d\n", pin, (c >> pin) & 1);
            } else {
                printf("INT: multiple or no pins (INTF=0x%02X INTCAP=0x%02X)\n", f, c);
            }
        } else {
            fprintf(stderr, "INT read failed (cap=%d, inf=%d)\n", cap, inf);
        }
    }

    close(req.fd);
    close(fd);
    return 0;
}