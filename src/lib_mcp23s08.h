#ifndef MCP23S08_H
#define MCP23S08_H

#include <stdint.h>

// MCP23S08 SPI Defines
#define SPI_MODE    0x00
#define SPI_BPW     0x08
#define SPI_DELAY   0x00
#define SPI_WDELAY  0x00
#define SPI_SPEED   0x186A0 // Supports up to 100000

// MCP23S08 Device Type
#define DEV_TYPE    0x40

// W/R Commands
#define WRITE_CMD   0x00
#define READ_CMD    0x01

// MCP23S08 Register Addresses
#define IODIR       0x00 // I/O Direction Register
#define IPOL        0x01 // Input Polarity Port Register
#define GPINTEN     0x02 // Interrupt-on-Change Control Register
#define DEFVAL      0x03 // Default Compare Register
#define INTCON      0x04 // Interrupt Control Register
#define IOCON       0x05 // Configuration Register
#define GPPU        0x06 // Pull-up Resistor Register
#define INTF        0x07 // Interrupt Flag Register
#define INTCAP      0x08 // Interrupt Captured Register
#define GPIO        0x09 // General Purpose I/O Register
#define OLAT        0x0A // Output Latch Register

// MCP23S08 IODIR Bits
#define IODIR_OUTPUT    0x00 // Pin is configured as output
#define IODIR_INPUT     0x01 // Pin is configured as input
// MCP23S08 IPOL Bits
#define IPOL_SAME       0x00 // GPIO bit will reflect same logic state
#define IPOL_OPST       0x01 // GPIO bit will reflect opposite logic state
// MCP23S08 GPINTEN Bits
#define GPINTEN_DISABLE 0x00 // Disable GPIO pin for interrupt-on-change
#define GPINTEN_ENABLE  0x01 // Enable GPIO pin for interrupt-on-change
// MCP23S08 DEFVAL Bits
#define DEFVAL_0        0x00 // Default comparison value
#define DEFVAL_1        0x01 // Default comparison value
// MCP23S08 INTCON Bits
#define INTCON_PREV     0x00 // Value is compared against previous value
#define INTCON_COMP     0x01 // Value is compared against DEFVAL bit
// MCP23S08 IOCON Bits
#define INTPOL_LOW      0x00 // Active Low
#define INTPOL_HIGH     0x01 // Active High
#define ODR_ACTIVE_DRIV 0x00 // Active Driver Output
#define ODR_OPEN_DRAIN  0x01 // Open-Drain Output
#define HAEN_DISABLE    0x00 // HW-Address disabled
#define HAEN_ENABLE     0x01 // HW-Address enabled
#define DISSLW_ENABLE   0x00 // Slew rate enabled
#define DISSLW_DISABLE  0x01 // Slew rate disabled
#define SEQOP_ENABLE    0x00 // Sequential operation enabled
#define SEQOP_DISABLE   0x01 // Sequential operation disabled
// MCP23S08 GPPU Bits
#define GPPU_DISABLE    0x00 // Pull-up disabled
#define GPPU_ENABLE     0x01 // Pull-up enabled
// MCP23S08 GPIO Bits
#define GPIO_LOW        0x00 // Logic Low
#define GPIO_HIGH       0x01 // Logic High
// MCP23S08 OLAT Bits
#define OLAT_LOW        0x00 // Logic Low
#define OLAT_HIGH       0x01 // Logic High

/**
 * @brief Opens and configures an SPI device file for MCP23S08 communication.
 *
 * @param bus   SPI bus number (0 or 1).
 * @param cs    Chip select line (0 or 1).
 * @return int  File descriptor for the opened SPI device, or -1 on failure.
 */
int mcp23s08_init(uint8_t bus, uint8_t cs);

/**
 * @brief Writes a byte of data to a specified register of the MCP23S08 via SPI.
 *
 * @param fd        File descriptor for the SPI device.
 * @param addr      MCP23S08 device address (3-bit hardware address).
 * @param reg       Register address to write to.
 * @param data      Byte of data to write.
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s08_write(int fd, uint8_t addr, uint8_t reg, uint8_t data);

/**
 * @brief Reads a byte from a specified register of the MCP23S08 via SPI.
 *
 * @param fd        File descriptor for the SPI device.
 * @param addr      MCP23S08 device address (3-bit hardware address).
 * @param reg       Register address to read from.
 * @return int8_t   The received byte from the SPI response, or -1 on failure.
 */
int16_t mcp23s08_read(int fd, uint8_t addr, uint8_t reg);

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
int8_t mcp23s08_write_pin(int fd, uint8_t addr, uint8_t reg, uint8_t pin, uint8_t data);

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
int8_t mcp23s08_read_pin(int fd, uint8_t addr, uint8_t reg, uint8_t pin);

/**
 * @brief Configures the direction of GPIO pins on the MCP23S08.
 *
 * @param fd        File descriptor for the SPI device.
 * @param addr      MCP23S08 device address (3-bit hardware address).
 * @param pins      Bitmask specifying the new state for the selected pins (o = OUTPUT, 1 = INPUT).
 * @return uint8_t  Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s08_set_dir(int fd, uint8_t addr, uint8_t pins);

/**
 * @brief Generates the control byte for MCP23S08 SPI communication.
 *
 * @param cmd       The command bit (0 for write, 1 for read).
 * @param addr      The 3-bit hardware address of the MCP23S08 device.
 * @return uint8_t  The computed control byte for SPI communication.
 */
static uint8_t get_control_byte(uint8_t cmd, uint8_t addr);

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
static int spi_transfer( int fd, uint8_t *tx_buf, uint8_t *rx_buf, unsigned int len);

#endif // MCP23S08_H