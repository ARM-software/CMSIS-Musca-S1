/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef MBED_FLASH_QSPI_H
#define MBED_FLASH_QSPI_H

#define BITMAP_SIZE    10        /* It has 80 bits for protection bitmap */

#define RBPR         0xE2        /* Read Block-Protection Register opcode */
#define WBPR         0xE3        /* Write Block-Protection Register opcde */
#define ULBPR        0xE4        /* Unlock Block Protection opcode */
#define SE           0x20        /* Sector Erase (4K) opcode */
#define BE           0xD8        /* Block Erase (8K, 32K, 64K) opcode */
#define CE           0xC7        /* Chip Erase opcode */
#define QUAD_OP      0x38        /* Enable 4-4-4 operation opcode */
#define RSTQIO       0xFF        /* Reset 4-4-4 operation opcode */
#define WREN         0x06        /* Write Enable opcode */
#define WRSR         0x01        /* Write Status register opcode */
#define PP           0x02        /* Program page opcode */
#define RD           0x03        /* Read data opcode */
#define WRCR         0x81        /* Write Volatile Configuration Register opcode */
#define WRNVCR       0xB1        /* Write Non Volatile Configuration Register opcode */
#define RDSR         0x05        /* Read Status Register opcode */
#define RDCR         0x85        /* Read Volatile Configuration Register opcode */
#define RDNVCR       0xB5        /* Read Non Volatile Configuration Register opcode */
#define RDID         0x9F        /* Read ID register */

#define SFDP         0x5a        /* Serial Flash Discoverable Parameters opcode */

/* QSPI Controller defines */
#define FLASH_QSPI_CTRL_BASE        0x4010A000

#define CTRL_CFG                    (FLASH_QSPI_CTRL_BASE)
#define CTRL_CFG_BAUD_OFFSET           19
#define CTRL_READ                   (FLASH_QSPI_CTRL_BASE + 0x04)
#define CTRL_READ_INSTR_TYPE_OFFSET     8
#define CTRL_READ_DTR_BIT_OFFSET       10
#define CTRL_READ_ADDR_TYPE_OFFSET     12
#define CTRL_READ_DATA_TYPE_OFFSET     16
#define CTRL_READ_MODE_BIT_OFFSET      20
#define CTRL_READ_DUMMY_CLK_OFFSET     24
#define CTRL_READ_DUMMY_CLK_MASK     0x1f
#define CTRL_WRITE                  (FLASH_QSPI_CTRL_BASE + 0x08)
#define CTRL_WRITE_DATA_TYPE_OFFSET    16
#define CTRL_WRITE_ADDR_TYPE_OFFSET    12
#define CTRL_RD_CAP                 (FLASH_QSPI_CTRL_BASE + 0x10)
#define CTRL_DEV_SIZE               (FLASH_QSPI_CTRL_BASE + 0x14)
#define CTRL_SRAM_PART              (FLASH_QSPI_CTRL_BASE + 0x18)
#define CTRL_INDIR_TRIG_ADDR        (FLASH_QSPI_CTRL_BASE + 0x1c)
#define CTRL_MODE_BIT               (FLASH_QSPI_CTRL_BASE + 0x28)
#define CTRL_SRAM_FILL_LEVEL        (FLASH_QSPI_CTRL_BASE + 0x2c)
#define CTRL_SRAM_LVL_READ_OFFSET        0
#define CTRL_SRAM_LVL_WRITE_OFFSET      16
#define CTRL_TX_THRESHOLD           (FLASH_QSPI_CTRL_BASE + 0x30)
#define CTRL_RX_THRESHOLD           (FLASH_QSPI_CTRL_BASE + 0x34)
#define CTRL_WRITE_COMPLETION       (FLASH_QSPI_CTRL_BASE + 0x38)
#define CTRL_WRITE_DELAY_OFFSET         24
#define CTRL_WRITE_POLL_COUNT_OFFSET    16
#define CTRL_WRITE_POLL_DISABLE_OFFSET  14
#define CTRL_INT_STATUS             (FLASH_QSPI_CTRL_BASE + 0x40)
#define CTRL_INT_MASK               (FLASH_QSPI_CTRL_BASE + 0x44)
#define CTRL_IREAD_TRANSFER         (FLASH_QSPI_CTRL_BASE + 0x60)
#define CTRL_IREAD_START_ADDR       (FLASH_QSPI_CTRL_BASE + 0x68)
#define CTRL_IREAD_COUNTER          (FLASH_QSPI_CTRL_BASE + 0x6c)
#define CTRL_IWRITE_TRANSFER        (FLASH_QSPI_CTRL_BASE + 0x70)
#define CTRL_IWRITE_START_ADDR      (FLASH_QSPI_CTRL_BASE + 0x78)
#define CTRL_IWRITE_COUNTER         (FLASH_QSPI_CTRL_BASE + 0x7c)
#define CTRL_INDIR_TRIG_RANGE       (FLASH_QSPI_CTRL_BASE + 0x80)
#define CTRL_CMD                    (FLASH_QSPI_CTRL_BASE + 0x90)
#define CTRL_CMD_OPCODE_OFFSET          24
#define CTRL_CMD_READ_OFFSET            20
#define CTRL_CMD_MODE_EN_OFFSET         18
#define CTRL_CMD_ADDR_OFFSET            16
#define CTRL_CMD_WRITE_OFFSET           12
#define CTRL_CMD_DUMMY_OFFSET            7
#define CTRL_CMD_EXEC_STATUS_MASK        2
#define CTRL_CMD_EXECUTE                 1
#define CTRL_CMD_ADDR               (FLASH_QSPI_CTRL_BASE + 0x94)
#define CTRL_READ_DATA_LOW          (FLASH_QSPI_CTRL_BASE + 0xa0)
#define CTRL_READ_DATA_HIGH         (FLASH_QSPI_CTRL_BASE + 0xa4)
#define CTRL_WRITE_DATA_LOW         (FLASH_QSPI_CTRL_BASE + 0xa8)
#define CTRL_WRITE_DATA_HIGH        (FLASH_QSPI_CTRL_BASE + 0xac)
#define CTRL_ID                     (FLASH_QSPI_CTRL_BASE + 0xfc)

#define FLASH_QSPI_INDIRECT_BASE    0x10000000
#define FLASH_QSPI_INDIRECT_SIZE    0xF            /* 8K reserved for indirect operations */

/* Driver defines */
#define ADDR_SIZE_UNKNOWN        (0)
#define ADDR_SIZE_3_BYTES        (1)
#define ADDR_SIZE_4_BYTES        (2)

#define BLOCK_ERASE_SIZE_4k      (1)
#define BLOCK_ERASE_SIZE_32k     (2)

#define FLASH_FAST_READ_1_1_1    (0)    /* standard SPI mode */
#define FLASH_FAST_READ_1_1_2    (1)    /* fast dual data output support (SPI command mode) */
#define FLASH_FAST_READ_1_2_2    (2)    /* fast dual address input, dual data output support (SPI command mode) */
#define FLASH_FAST_READ_2_2_2    (3)    /* dual command, dual address input, dual data output support (DSPI command mode) */
#define FLASH_FAST_READ_1_1_4    (4)    /* fast quad data output support (SPI command mode) */
#define FLASH_FAST_READ_1_4_4    (5)    /* fast quad address input, quad data output support (SPI command mode) */
#define FLASH_FAST_READ_4_4_4    (6)    /* quad command, quad address input, quad data output support (QSPI command mode) */
#define FLASH_DTR_CLOCK          (7)    /* double transfer rate support */

#define FLASH_FAST_READ_STATES   (7)    /* total number of read states supported */


#define FLASH_FAST_READ_DUAL    (FLASH_FAST_READ_1_1_2 | FLASH_FAST_READ_1_2_2 | FLASH_FAST_READ_2_2_2)
#define FLASH_FAST_READ_QUAD    (FLASH_FAST_READ_1_1_4 | FLASH_FAST_READ_1_4_4 | FLASH_FAST_READ_4_4_4)    

#define FLASH_QSPI_ERR_INVALID_SIGNATURE    (-5)

     /** Cadence QSPI Flash controller
     *
     * This controller has pre-assigned pins and does not require any specific setup.
     * It defaults to SPI mode 0 and a baud rate divisor of 32 for the reference clock
     * (SCLK = ref_clk / 32).
     *
     * Example use:
     * @code
     * // Read the JEDEC-ID and configure the controller using the
     * // Standard Flash Discoverable Parameters protocol. Display the content of the
     * // first block from the flash chip
     *
     * #include "mbed.h"
     *
     * Flash_QSPI controller(12500000, 0);         // frequency in hz, mode
     *
     * int main() {
     *     uint8_t response[8];
     *     uint8_t* buffer;
     *     unsigned int page_size = 256;
     *
     *     // command, address, byte count for address, bytes to write, byte count for write,
     *     // bytes to read, byte count for read, dummy cycles count
     *     controller.send_cmd(0x9f, 0, 0, NULL, 0, response, 3, 0);
     *     printf("Found flash ID 0x%02x%02x%02x\r\n", response[2], response[1], response[0]);
     *     int err = controller.probe();                       // configure the controller using SFDP
     *     if (err) {
     *         printf("Failed to configure flash controller\r\n");
     *         return err;
     *     }
     *
     *     buffer = malloc(sizeof(uint8_t) * page_size);
     *     if (!buffer) {
     *         printf("Failed to allocate buffer for reading flash page\r\n");
     *         return -1;
     *     }
     *
     *     err = controller.read(0, page_size, buffer, 0x03, 3, 0);
     *     if (err) {
     *         printf("Failed to read flash page\r\n");
     *         return err;
     *     }
     *
     *     for (int i = 0; i < page_size; i++)
     *         printf("%c", buffer[i]);
     *
     *     return 0;
     * }
     * @endcode
     */

    /** Initialise the Flash QSPI controller with the given baud rate divisor and SPI mode
     *
     * @param hz Frequency in Hz for the controller to operate at
     * @param mode SPI clock polarity and phase mode (0 - 3)
     *
     * @code
     * mode | POL PHA
     * -----+--------
     *   0  |  0   0
     *   1  |  0   1
     *   2  |  1   0
     *   3  |  1   1
     * @endcode
     */
    void Flash_QSPI(unsigned int hz, unsigned int mode);

    /** Configure the controller using the Serial Flash Discoverable Parameters protocol
     *
     * This function uses the SFDP protocol to discover the size and features supported
     * by the attached flash chip and configures the controller accordingly.
     *
     * @return 0 if successful, -1 if it has failed.
     */
    int probe(void);

    /** Test if SPI mode is supported
     *
     * This function will return true if the given mode is supported.
     *
     * @param mode the SPI mode queried for support
     * @return true if read mode supported, false otherwise.
     */
    unsigned int is_mode_supported(uint8_t mode);

    /** Enable mode
     *
     * This function tries to enable the given mode if supported.
     *
     * @param mode mode to be enabled.
     * @return 1 if successful, 0 otherwise.
     */
    int enable_mode(uint8_t mode);

    /** Disable the write completion status auto-poll
     *
     * This function will disable the controller's write completion status
     * auto-poll.
     */
    void disable_status_autopoll(void);

    /** Send a special flash command to the chip
     *
     * This function uses the Software Triggered Instruction Generator to create
     * chip specific commands that are not standard or are not generated automatically
     * by the controller.
     *
     * @param cmd The command byte to be sent
     * @param address Address to be sent after command.
     * @param address_bytes Number of address bytes to be sent after command. Zero disables
     * sending any address bytes
     * @param write_data Buffer with data to be written after command and address
     * @param write_bytes Number of bytes to write from the flash command write data registers,
     * zero disables sending any write bytes
     * @param read_data Buffer where response bytes will be stored
     * @param read_bytes Number of bytes to read into the flash command read data registers,
     * zero disables sending any read byte dummy values
     * @param dummy_cycles Number of dummy cycles to be sent, zero disables sending any dummy cycles
     */
    void send_cmd(uint8_t cmd, unsigned int address, uint8_t address_bytes,
              uint8_t* write_data, uint8_t write_bytes, uint8_t* read_data,
              uint8_t read_bytes, uint8_t dummy_cycles);

    /** Send a special flash command to the chip without any accompanying data
     *
     * This function uses the Software Triggered Instruction Generator to send
     * a chip specific command that cannot be generated otherwise by the
     * controller's internal state machine. The command sent does not require
     * any additional write data, dummy bytes and does not return any data.
     * This is a shorter version of send_cmd() when only the cmd parameter
     * needs to be specified.
     *
     * @param cmd The command byte to be sent
     */
    void send_simple_cmd(uint8_t cmd);

    /** Read flash content using indirect read method
     *
     * This function will perform an indirect read of the flash content by setting up the
     * controller to fetch data from the flash.
     *
     * @param address Address in the flash where to start reading from
     * @param buffer Buffer where to store the data that was read off the flash chip
     * @param size Number of bytes to read
     * @return 0 if successful, -1 if it has failed.
     */
    int read(unsigned int address, uint8_t* buffer, unsigned int size);

    /** Write content to flash using the indirect method
     *
     * This function will write the content of the given buffer into flash using the
     * indirect method.
     *
     * @param address Address in the flash where to start writing to
     * @param buffer Buffer that holds the data that needs to be written to the flash chip
     * @param size Number of bytes to write
     * @return 0 if successful, -1 if it has failed.
     */
    int write(unsigned int address, uint8_t* buffer, unsigned int size);

    void unlock_chip(void);
    void enable_active_mode(void);
    void erase_chip(uint8_t dummy_bytes);
    void erase_page(unsigned int address, uint8_t dummy_bytes);
    unsigned int read_id(void);
    void set_dummy_clocks(void);
    void busy_wait(void);

#endif /* FLASH_QSPI_H */
