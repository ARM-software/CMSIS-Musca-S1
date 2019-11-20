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

#include "Flash_QSPI.h"

#define min(a, b)   ((a < b) ? a : b)

uint8_t address_bytes;            /* number of bytes used for addresses */
uint8_t addr_byte_clocks = 8;     /* number of clock cycles needed to send one byte of address or dummy */

static unsigned int readl(volatile unsigned int reg)
{
    return *(unsigned int *)reg;
}

static void writel(volatile unsigned int reg, unsigned int val)
{
    *(unsigned int *)reg = val;
}

/* unlock the global block protection */
void unlock_chip(void) {
    send_simple_cmd(WREN);        /* enable write */
    send_simple_cmd(ULBPR);
}

/* Set Dummy Clocks in NV register */
void set_dummy_clocks(void) {
    uint8_t buffer[2];
    uint8_t response[3];
    
    // Read ID (20 BA XX) to make sure interface is working
    send_cmd(0x9F, 0, 0, NULL, 0, response, 3, 0);
    
    if ((response[1] == 0xBA) || (response[1] == 0xBB))
    {
        // Read Nonvolatile Configuration register
    send_cmd(RDNVCR, 0, 0, NULL, 0, response, 2, 0);

        if ((response[1] != 0x8F) || (response[0] != 0xFF))
        {
            // Write Nonvolatile Configuration register
            buffer[0] = 0x8F;
            buffer[1] = 0xFF;
            send_simple_cmd(WREN);
            send_cmd(WRNVCR, 0, 0, buffer, 2, NULL, 0, 0);
            busy_wait();
    
            // Reset memory to load new defaults
            send_simple_cmd(0x66);
            busy_wait();
            send_simple_cmd(0x99);
            busy_wait();
        }
    }
}


/* Read ID register */
unsigned int read_id(void) {
    unsigned int error = 1;
    uint8_t response[3];
    
    // Read ID register
    send_cmd(RDID, 0, 0, NULL, 0, response, 3, 0);
    if(((response[1] & 0xFF) == 0xBA) || ((response[1] & 0xFF) == 0xBB))
    {
        error = 0;
    }
    return error;
}

/* erase entire chip */
void erase_chip(uint8_t dummy_bytes) {
   
    send_simple_cmd(WREN);        /* enable write */
    send_simple_cmd(CE);        /* erase entire chip */
    busy_wait();
}

/* erase 64k byte page */
void erase_page(unsigned int address, uint8_t dummy_bytes) {
    uint8_t buffer[2];
    
    send_simple_cmd(WREN);        /* enable write */
    send_cmd(BE, address, 3, NULL, 0, buffer, 0, dummy_bytes);        /* erase 64k page */
    busy_wait();
}

void enable_configuration_mode(void)
{
    unsigned int val;

    while ((readl(CTRL_CFG) & 0x80000000) == 0)
    /* wait for IDLE flag */;
    val = readl(CTRL_CFG);    /* this read also serves as a delay to ensure that the low level is synchronised */
    val &= ~0x800080;        /* disable direct access mode */
    writel(CTRL_CFG, val);
}

void enable_active_mode(void)
{
    unsigned int val = readl(CTRL_CFG);
    val |= 0x000080;        /* enable direct access mode */
    writel(CTRL_CFG, val);
}

void send_cmd(uint8_t cmd, unsigned int address, uint8_t address_bytes,
              uint8_t* write_data, uint8_t write_bytes, uint8_t* read_data,
              uint8_t read_bytes, uint8_t dummy_bytes)
{
    unsigned int ab = address_bytes ? (8 | ((address_bytes - 1) & 0x3)) : 0;    /* build the enable bit and count for address */
    unsigned int rb = read_bytes ? (8 | ((read_bytes - 1) & 0x7)) : 0;        /* same for reading bytes */
    unsigned int wb = write_bytes ? (8 | ((write_bytes - 1) & 0x7)) : 0;        /* and writing bytes */
    unsigned int val = 0, i;
    uint8_t dummy_clocks = dummy_bytes * addr_byte_clocks;

    writel(CTRL_CMD_ADDR, address);

    if (write_bytes)
    {
        for (val = 0, i = 0; i < min(write_bytes, 4); i++)
        val = (val << 8) | write_data[i];
        writel(CTRL_WRITE_DATA_LOW, val);
        for (val = 0, i = 4; i < min(write_bytes, 8); i++)
        val = (val << 8) | write_data[i];
        writel(CTRL_WRITE_DATA_HIGH, val);
    }

    val = (cmd << CTRL_CMD_OPCODE_OFFSET) | (rb << CTRL_CMD_READ_OFFSET) | (ab << CTRL_CMD_ADDR_OFFSET) \
        | (wb << CTRL_CMD_WRITE_OFFSET) | ((dummy_clocks & 0x1f) << CTRL_CMD_DUMMY_OFFSET);
    writel(CTRL_CMD, val);

    // execute the command
    writel(CTRL_CMD, val | 1);
    // delay for command to start
    val = readl(CTRL_CMD);
    // wait until command is being executed
    while ((readl(CTRL_CMD) & 0x2) == 0x2)
        ;

    if (read_bytes)
    {
        val = readl(CTRL_READ_DATA_LOW);
        for (i = 0; i < min(read_bytes, 4); i++)
        {
            read_data[i] = (val & 0xff);
            val >>= 8;
        }
        val = readl(CTRL_READ_DATA_HIGH);
        for (i = 4; i < min(read_bytes, 8); i++)
        {
            read_data[i] = (val & 0xff);
            val >>= 8;
        }
    }
}

void send_simple_cmd(uint8_t cmd)
{
    unsigned int val;

    // Make command
    val = (cmd << CTRL_CMD_OPCODE_OFFSET);
    // Set the command
    writel(CTRL_CMD, val);
    // Execute the command
    writel(CTRL_CMD, val | 1);
    // Delay for command to start
    val = readl(CTRL_CMD);
    // Wait until command has been executed
    while ((readl(CTRL_CMD) & 0x2) == 0x2)
        continue;
}

void busy_wait(void)
{
    unsigned int val;

    do
    {
        // Make command
        val = (RDSR << CTRL_CMD_OPCODE_OFFSET) | (0x08 << CTRL_CMD_READ_OFFSET);
        // Set the command
        writel(CTRL_CMD, val);
        // Execute the command
        writel(CTRL_CMD, val | 1);
        // Delay for command to start
        val = readl(CTRL_CMD);
        // Wait until command has been executed
        while ((readl(CTRL_CMD) & 0x2) == 0x2)
            continue;
    } while (readl(CTRL_READ_DATA_LOW) & 0x01);
}

int write(unsigned int address, uint8_t* buffer, unsigned int size)
{
    unsigned char rx[8], tx[8];
    unsigned int  addr, remain = size;
    unsigned char * data = buffer; 

    // Program memory (max 16MB)
    for (addr = address; addr < (address + size); addr = addr + 4, data = data + 4)
    {
        // Write 4 bytes (max 8 bytes)
        if (remain >= 4)
        {
            tx[3] = (data[0]) & 0xFF;
            tx[2] = (data[1]) & 0xFF;
            tx[1] = (data[2]) & 0xFF;
            tx[0] = (data[3]) & 0xFF;
            remain = remain - 4;
        }
        else if(remain == 3)
        {
            tx[3] = (data[0]) & 0xFF;
            tx[2] = (data[1]) & 0xFF;
            tx[1] = (data[2]) & 0xFF;
            tx[0] = 0xFF;
            remain = 0;
        }
        else if(remain == 2)
        {
            tx[3] = (data[0]) & 0xFF;
            tx[2] = (data[1]) & 0xFF;
            tx[1] = 0xFF;
            tx[0] = 0xFF;
            remain = 0;
        }
        else if(remain == 1)
        {
            tx[3] = (data[0]) & 0xFF;
            tx[2] = 0xFF;
            tx[1] = 0xFF;
            tx[0] = 0xFF;
            remain = 0;
        }
       
        send_simple_cmd(WREN);
        send_cmd(PP, addr, 3, tx, 4, rx, 0, 0);
        busy_wait();
    }
    
    return (0);
}

int read(unsigned int address, uint8_t* buffer, unsigned int size)
{
    unsigned char rx[8], tx[8];
    
    // Read 4 bytes (max 8 bytes)
    send_cmd(RD, address, 3, tx, 4, rx, 4, 0);
    
    buffer[0] = (rx[0]);
    buffer[1] = (rx[1]);
    buffer[2] = (rx[2]);
    buffer[3] = (rx[3]);

    return (0);
}

