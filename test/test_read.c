/*
* test_read.c
* @brief Test program for reading from an MCP23S08 GPIO expander via SPI.
*
* This program initializes the MCP23S08, configures its registers for input,
* and continuously reads the GPIO pin states every second. The program runs
* indefinitely until manually stopped (Ctrl + C).
*
* @author Hauke Schlosser
* @date 10/17/2025
*/

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <lib_mcp23s08.h>

int main() {
    int fd;

    if ((fd = mcp23s08_init(0, 0, SPI_MODE_0)) < 0) {
        fprintf(stderr, "[main] Initialization failed\n");
        close(fd);
        return 1;
    }
    
    printf("[test_read] MCP23S08 initialized successfully with fd: %d\n", fd);

    printf("[test_read] Run test program...\n");
    printf("[test_read] Press crtl + c to stop...\n");

    uint8_t addr = 0;

    uint8_t pin = 1;
    mcp23s08_write(fd, addr, IOCON, 0x08); // Enable HAEN
    while(1) {
        mcp23s08_write(fd, addr, IODIR, 0xFF);
        mcp23s08_write(fd, addr, GPPU, 0xFF);
        printf("Read all pins\n");
        mcp23s08_read(fd, addr, GPIO, NOT_SILENT);
    
        sleep(1);

        mcp23s08_write(fd, addr, IODIR, (1u << pin));
        mcp23s08_write(fd, addr, GPPU,  (1u << pin)); 
        printf("Read pin 1\n");
        mcp23s08_read_pin(fd, addr, GPIO, 1, NOT_SILENT);

        sleep(1);
    }

    return 0;
}