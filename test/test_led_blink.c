/**
 * @file test_led_blink.c
 * @brief Test program for controlling an MCP23S08 GPIO expander via SPI.
 *
 * This program initializes the MCP23S08, configures its registers, and continuously
 * toggles all GPIO pins on and off every second. The program runs indefinitely until
 * manually stopped (Ctrl + C).
 *
 * @author Hauke Schlosser
 * @date 03/10/2025
 */

 #include <stdint.h>
 #include <stdio.h>
 #include <unistd.h>
 #include <lib_mcp23s08.h>
 
 void testConfigure(int fd, uint8_t addr) {
     uint8_t iocon = 0;
     iocon |= (SEQOP_DISABLE     << 5);
     iocon |= (DISSLW_DISABLE    << 4);
     iocon |= (HAEN_ENABLE       << 3);
     iocon |= (ODR_ACTIVE_DRIV   << 2);
     iocon |= (INTPOL_LOW        << 1);
     mcp23s08_write(fd, addr, IOCON, iocon);
 }
 
 int main() {
     int fd;
 
     if ((fd = mcp23s08_init(0, 0)) < 0) {
         fprintf(stderr, "[main] Initialization failed\n");
         return 1;
     }
     
     printf("[main] Run test program...\n");
     printf("[main] Press crtl + c to stop...\n");
 
     int8_t addr = 0;
     testConfigure(fd, addr);
 
     while(1) {
         mcp23s08_write(fd, addr, GPIO, 0xFF);
         sleep(1);
         mcp23s08_write(fd, addr, GPIO, 0x00);
         sleep(1);
     }
 
     return 0;
 }