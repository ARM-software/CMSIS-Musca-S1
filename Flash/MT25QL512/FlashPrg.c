/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
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

#include "../FlashOS.H"        // FlashOS Structures
#include "Flash_QSPI.h"

/* 
   Mandatory Flash Programming Functions (Called by FlashOS):
                int Init        (unsigned long adr,   // Initialize Flash
                                 unsigned long clk,
                                 unsigned long fnc);
                int UnInit      (unsigned long fnc);  // De-initialize Flash
                int EraseSector (unsigned long adr);  // Erase Sector Function
                int ProgramPage (unsigned long adr,   // Program Page Function
                                 unsigned long sz,
                                 unsigned char *buf);

   Optional  Flash Programming Functions (Called by FlashOS):
                int BlankCheck  (unsigned long adr,   // Blank Check
                                 unsigned long sz,
                                 unsigned char pat);
                int EraseChip   (void);               // Erase complete Device
      unsigned long Verify      (unsigned long adr,   // Verify Function
                                 unsigned long sz,
                                 unsigned char *buf);

       - BlanckCheck  is necessary if Flash space is not mapped into CPU memory space
       - Verify       is necessary if Flash space is not mapped into CPU memory space
       - if EraseChip is not provided than EraseSector for all sectors is called
*/


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

    unsigned int error = 0;

#ifdef SET_DC
    set_dummy_clocks(); 
#endif
    error = read_id();
    if(error)
        return (1);
    
    return (0);  // Finished without Errors
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {

//  send_simple_cmd(RSTQIO);
  return (0);                                  // Finished without Errors
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {
  erase_chip(0);
  return (0);                                  // Finished without Errors
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {
  erase_page((adr & 0x00FFFFFF) - 0x00200000, 0);
  return (0);                                  // Finished without Errors
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
#ifdef DIRECT_MODE
    unsigned char* ptr = (unsigned char*)adr;
    unsigned int i;
    
    busy_wait();

    for(i = 0; i < sz; i++)
    {
        ptr[i] = buf[i];
    }
    
    busy_wait();
    
    return (0);                                  // Finished without Errors
#else
    return (write((unsigned int) (adr & 0x00FFFFFF) - 0x00200000, (uint8_t*) buf, (unsigned int) sz));
#endif
}

 /*
  *  Verify Flash Contents
  *    Parameter:      adr:  Start Address
  *                    sz:   Size (in bytes)
  *                    buf:  Data
  *    Return Value:   (adr+sz) - OK, Failed Address
  */
 unsigned long Verify (unsigned long adr, unsigned long sz, unsigned char *buf) {
#ifdef DIRECT_MODE
    unsigned char* ptr = (unsigned char*)adr;
    unsigned int i;
    for(i = 0; i < sz; i++)
    {
        if(ptr[i] != buf[i]) 
            return (unsigned long)(ptr + i);
    }
#else
    unsigned char* ptr = (unsigned char*)(adr & 0x00FFFFFF);
    unsigned int i;
    unsigned char data[4];
    ptr = ptr - 0x00200000;
    
    for (i = 0;  i < sz; i = i + 4)
    {
        read((unsigned int) ptr, (uint8_t*) data, 4);
        if( data[0] != buf[i + 0] ||
            data[1] != buf[i + 1] ||
            data[2] != buf[i + 2] ||
            data[3] != buf[i + 3] )
        {
            return (unsigned long)(0x00200000 + ptr + i);
        }
        
        ptr = ptr + 4;
    }
#endif
    return (adr + sz);
}

/*  Blank Check Block in Flash Memory
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

int BlankCheck (unsigned long adr, unsigned long sz, unsigned char pat)
{
#ifdef DIRECT_MODE
    unsigned char* ptr = (unsigned char*)adr;
    unsigned int i;
    
    for(i = 0; i < sz; i++)
    {
        if(ptr[i] != pat) return (1);
    }
#else
    unsigned char* ptr = (unsigned char*)(adr & 0x00FFFFFF);
    unsigned int i;
    unsigned char data[4];
    ptr = ptr - 0x00200000;
    
    for (i = 0;  i < sz; i = i + 4)
    {
        read((unsigned int) ptr, (uint8_t*) data, 4);
        if( data[0] != pat ||
            data[1] != pat ||
            data[2] != pat ||
            data[3] != pat )
        {
            return (1);
        }
        
        ptr = ptr + 4;
    }
#endif
    return (0);
}
