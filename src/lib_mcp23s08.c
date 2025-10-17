#include "lib_mcp23s08.h"
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

/**
 * @brief Generates the control byte for MCP23S08 SPI communication.
 *
 * @param cmd       The command bit (0 for write, 1 for read).
 * @param addr      The 3-bit hardware address of the MCP23S08 device.
 * 
 * @return uint8_t  The computed control byte for SPI communication.
 */
static uint8_t get_control_byte(uint8_t cmd, uint8_t addr) {
    uint8_t ctr_byte = DEV_TYPE | (addr << 1) | cmd;
    return ctr_byte;
}

/**
 * @brief Performs an SPI transaction using SPI ioctl interface.
 *
 * @param fd        File descriptor for the SPI device.
 * @param tx_buf    Pointer to the transmit buffer (data to send).
 * @param rx_buf    Pointer to the receive buffer (data received).
 * @param len       Length of the data to be transmitted/received (in bytes).
 *
 * @return int      Returns 0 on success, or -1 if the ioctl call fails.
 */
static int spi_transfer( int fd, uint8_t *tx_buf, uint8_t *rx_buf, unsigned int len) {
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (uintptr_t) tx_buf;
    spi.rx_buf = (uintptr_t) rx_buf;
    spi.len = len;
    spi.speed_hz = SPI_SPEED;
    spi.delay_usecs = SPI_DELAY;
    spi.bits_per_word = SPI_BPW;
    spi.word_delay_usecs = SPI_WDELAY;

    return ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
}

int mcp23s08_init(uint8_t bus, uint8_t cs, uint8_t spi_mode) {
    int fd;

    if (bus > 1) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid bus input: %d\n", bus);
        return -1;
    }

    if (cs > 1) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid chip select input: %d\n", cs);
        return -1;
    }

    if (spi_mode != SPI_MODE_0 && spi_mode != SPI_MODE_3) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid SPI mode input: %d\n", spi_mode);
        fprintf(stderr, "[mcp23s08_init] ERROR Expected values: SPI_MODE_0 (0x00) or SPI_MODE_3 (0x03)\n");
        return -1;
    }

    static const char * spidev[2][2] = {
        {"/dev/spidev0.0", "/dev/spidev0.1"},
        {"/dev/spidev1.0", "/dev/spidev1.1"},
    };

    if ((fd = open(spidev[bus][cs], O_RDWR)) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not open SPI device: %s\n", spidev[bus][cs]);
        perror("System Message");
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not set SPI_MODE: %u\n", spi_mode);
        perror("System Message");
        close(fd);
        return -1;
    }

    uint8_t spi_bpw = SPI_BPW;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not set SPI_BPW: %u\n", spi_bpw);
        perror("System Message");
	    close(fd);
        return -1;
    }

    uint32_t spi_speed = SPI_SPEED;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not set SPI_SPEED: %u\n", spi_speed);
        perror("System Message");
        close(fd);
        return -1;
    }

    return fd;
}

int8_t mcp23s08_write(int fd, uint8_t addr, uint8_t reg, uint8_t data) {

    if (fd < 0) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (addr > 0x07) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid address: %u\n", addr);
        fprintf(stderr, "[mcp23s08_write] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    if (reg > 0x0A) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_write] ERROR Expected range: 0x00 - 0x0A\n");
        return -1;
    }

    uint8_t ctr_byte = get_control_byte(WRITE_CMD, addr);
    uint8_t tx_buf[3] = {ctr_byte, reg, data};
    uint8_t rx_buf[sizeof tx_buf];

    if (spi_transfer(fd, tx_buf, rx_buf, sizeof tx_buf) < 0) {
        fprintf(stderr, "[mcp23s08_write] ERROR: SPI transaction failed\n");
        perror("System Message");
        return -1;
    }   

    return 0;
}

int16_t mcp23s08_read(int fd, uint8_t addr, uint8_t reg, uint8_t silent) {

    if (fd < 0) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (addr > 0x07) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid address: %u\n", addr);
        fprintf(stderr, "[mcp23s08_read] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    if (reg > 0x0A) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_read] ERROR Expected range: 0x00 - 0x0A\n");
        return -1;
    }

    if (silent > 1) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid silent parameter: %u\n", silent);
        fprintf(stderr, "[mcp23s08_read] ERROR Expected range: 0 or 1\n");
        return -1;
    }

    uint8_t ctr_byte = get_control_byte(READ_CMD, addr);
    uint8_t tx_buf[3] = {ctr_byte, reg, 0x00};
    uint8_t rx_buf[sizeof tx_buf];

    if (spi_transfer(fd, tx_buf, rx_buf, sizeof tx_buf) < 0) {
        fprintf(stderr, "[mcp23s08_read] ERROR: SPI transaction failed\n");
        perror("System Message");
        return -1;
    }

    if (!silent) {
        printf("[mcp23s08_read] rx_buf[0] %d ...\n", rx_buf[0]);
        printf("[mcp23s08_read] rx_buf[1] %d ...\n", rx_buf[1]);
        printf("[mcp23s08_read] rx_buf[2] %d ...\n", rx_buf[2]);
        printf("[mcp23s08_read] fd %d ...\n", fd);
        printf("[mcp23s08_read] addr %d ...\n", addr);
        printf("[mcp23s08_read] ctr_byte %d ...\n", ctr_byte);
    }

    return rx_buf[2];
}

int8_t mcp23s08_write_pin(int fd, uint8_t addr, uint8_t reg, uint8_t pin, uint8_t data) {

    if (fd < 0) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (addr > 0x07) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid address: %u\n", addr);
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    if (reg > 0x0A) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Expected range: 0x00 - 0x0A\n");
        return -1;
    }

    if (pin > 0x07) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    int16_t reg_data = mcp23s08_read(fd, addr, reg, SILENT);
    if (reg_data < 0) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR: Failed to read register: 0x%02X\n", reg);
        return -1;
    }

    if (data) {
        reg_data |= 1 << pin;
    } else {
        reg_data &= ~(1 << pin);
    }

    return mcp23s08_write(fd, addr, reg, reg_data);
}

int8_t mcp23s08_read_pin(int fd, uint8_t addr, uint8_t reg, uint8_t pin) {

    if (fd < 0) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (addr > 0x07) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid address: %u\n", addr);
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    if (reg > 0x0A) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Expected range: 0x00 - 0x0A\n");
        return -1;
    }

    if (pin > 0x07) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    return (mcp23s08_read(fd, addr, reg, NOT_SILENT) >> pin) & 1;
}

int8_t mcp23s08_set_dir(int fd, uint8_t addr, uint8_t pins) {

    if (fd < 0) {
        fprintf(stderr, "[mcp23s08_set_dir] ERROR Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (addr > 0x07) {
        fprintf(stderr, "[mcp23s08_set_dir] ERROR Invalid address: %u\n", addr);
        fprintf(stderr, "[mcp23s08_set_dir] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    if ((mcp23s08_write(fd, addr, IODIR, pins)) < 0) {
        fprintf(stderr, "[mcp23s08_set_dir] ERROR Failed to write to device\n");
        return -1;
    }

    return 0;
}