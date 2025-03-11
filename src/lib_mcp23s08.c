#include "lib_mcp23s08.h"
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

/**
 * @brief Opens and configures an SPI device file for MCP23S08 communication.
 *
 * @param bus   SPI bus number (0 or 1).
 * @param cs    Chip select line (0 or 1).
 * @return int  File descriptor for the opened SPI device, or -1 on failure.
 */
int mcp23s08_init(uint8_t bus, uint8_t cs) {
    int fd;

    if (bus > 1) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid bus input: %d\n", bus);
        return -1;
    }

    if (cs > 1) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid chip select input: %d\n", cs);
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

    uint8_t spi_mode = SPI_MODE;
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

/**
 * @brief Writes a byte of data to a specified register of the MCP23S08 via SPI.
 *
 * @param fd        File descriptor for the SPI device.
 * @param addr      MCP23S08 device address (3-bit hardware address).
 * @param reg       Register address to write to.
 * @param data      Byte of data to write.
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s08_write(int fd, uint8_t addr, uint8_t reg, uint8_t data) {

    if (fd <= 0) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (addr > 0x07) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid address: %u\n", addr);
        fprintf(stderr, "[mcp23s08_write] ERROR Expected range: 0x00 - 0x07\n");
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

/**
 * @brief Reads a byte from a specified register of the MCP23S08 via SPI.
 *
 * @param fd        File descriptor for the SPI device.
 * @param addr      MCP23S08 device address (3-bit hardware address).
 * @param reg       Register address to read from.
 * @return int8_t   The received byte from the SPI response, or -1 on failure.
 */
int16_t mcp23s08_read(int fd, uint8_t addr, uint8_t reg) {

    if (fd <= 0) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (addr > 0x07) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid address: %u\n", addr);
        fprintf(stderr, "[mcp23s08_read] ERROR Expected range: 0x00 - 0x07\n");
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

    return rx_buf[2];
}

/**
 * @brief Writes a value to a specific pin of the MCP23S08 I/O expander.
 *
 * @param fd    The file descriptor for the SPI device.
 * @param addr  The 3-bit hardware address of the MCP23S08 (0x00 - 0x07).
 * @param reg   The register address to write to (e.g., GPIO register).
 * @param pin   The pin number to modify (0-7).
 * @param data  The value to write (0 to clear, 1 to set).
 *
 * @return      0 on success, or -1 on error.
 */
int8_t mcp23s08_write_pin(int fd, uint8_t addr, uint8_t reg, uint8_t pin, uint8_t data) {

    if (pin > 0x07) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    if (data > 1) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid pin: %u\n", data);
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Expected range: 0x00 - 0x01\n");
        return -1;  
    }

    int8_t reg_data = mcp23s08_read(fd, addr, reg);
    if (reg_data < 0) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR: Failed to read register: 0x%02X\n", reg);
        return -1;
    }

    if (data) {
        reg_data |= 1 << pin;
    } else {
        reg_data &= 0xff ^ (1 << pin);
    }

    return mcp23s08_write(fd, addr, reg, reg_data);
}

/**
 * @brief Reads the state of a specific pin from the MCP23S08 I/O expander.
 *
 * @param fd    The file descriptor for the SPI device.
 * @param addr  The 3-bit hardware address of the MCP23S08 (0x00 - 0x07).
 * @param reg   The register address to read from (e.g., GPIO register).
 * @param pin   The pin number to read (0-7).
 *
 * @return      The state of the specified pin (0 or 1), or -1 on error.
 */
int8_t mcp23s08_read_pin(int fd, uint8_t addr, uint8_t reg, uint8_t pin) {

    if (pin > 0x07) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Expected range: 0x00 - 0x07\n");
        return -1;
    }

    return (mcp23s08_read(fd, addr, reg) >> pin) & 1;
}

/**
 * @brief Configures the direction of GPIO pins on the MCP23S08.
 *
 * @param fd        File descriptor for the SPI device.
 * @param addr      MCP23S08 device address (3-bit hardware address).
 * @param pins      Bitmask specifying the new state for the selected pins (o = OUTPUT, 1 = INPUT).
 * @return uint8_t  Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s08_set_dir(int fd, uint8_t addr, uint8_t pins) {

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

/**
 * @brief Generates the control byte for MCP23S08 SPI communication.
 *
 * @param cmd       The command bit (0 for write, 1 for read).
 * @param addr      The 3-bit hardware address of the MCP23S08 device.
 * @return uint8_t  The computed control byte for SPI communication.
 */
static uint8_t get_control_byte(uint8_t cmd, uint8_t addr) {
    uint8_t ctr_byte = 0x40 | (addr << 1) | cmd;
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