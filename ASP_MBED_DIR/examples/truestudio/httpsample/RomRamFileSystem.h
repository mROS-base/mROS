/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer*
* Copyright (C) 2015 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
#ifndef MBED_ROMRAMFILESYSTEM_H
#define MBED_ROMRAMFILESYSTEM_H

#include "FATFileSystem.h"
#include "FileSystemLike.h"
#include "FileHandle.h"
#include "ff.h"
#include <stdint.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"

#define NUM_OF_SECTORS     (1000)
#define SECTOR_SIZE        (512)

#if defined(TARGET_RZ_A1H)
#define ROM_START_ADDRESS  (0x18000000uL)  // for GR-PEACH
#define ROM_END_ADDRESS    (0x1FFFFFFFuL)  // for GR-PEACH
#else
#define ROM_START_ADDRESS  (0xFFFFFFFFuL)
#define ROM_END_ADDRESS    (0xFFFFFFFFuL)
#endif

using namespace mbed;

class RomRamFileSystem : public FATFileSystem {
 public:
    // NUM_OF_SECTORS sectors, each 512 bytes
    char *sectors[NUM_OF_SECTORS];

    RomRamFileSystem(const char* name) : FATFileSystem(name) { 
		memset(sectors, 0, sizeof(sectors));
	}
	
    virtual ~RomRamFileSystem() {
        for (int i = 0; i < NUM_OF_SECTORS; i++) {
            if ((sectors[i] != NULL) && (isRomAddress(sectors[i]) == false)) {
                free(sectors[i]);
            }
        }
    }

    // read a sector in to the buffer, return 0 if ok
    virtual int disk_read(uint8_t *buffer, uint32_t sector, uint32_t count) {
        for (uint64_t sec_no = sector; sec_no < (sector + count); sec_no++) {
            if (sectors[sec_no] == NULL) {
                // nothing allocated means sector is empty
                memset(buffer, 0, SECTOR_SIZE);
            } else {
                memcpy(buffer, sectors[sec_no], SECTOR_SIZE);
            }
            buffer += SECTOR_SIZE;
        }
        return 0;
    }

    // write a sector from the buffer, return 0 if ok
    virtual int disk_write(const uint8_t *buffer, uint32_t sector, uint32_t count) {
        for (uint64_t sec_no = sector; sec_no < (sector + count); sec_no++) {
            bool all_zero = true;
            for (int i = 0; i < SECTOR_SIZE; i++) {
                if (buffer[i] != NULL) {
                    all_zero = false;
                    break;
                }
            }
            if (all_zero != false) {
                if (sectors[sec_no] != NULL) {
                    if (isRomAddress(sectors[sec_no]) == false) {
                        free(sectors[sec_no]);
                    }
                    sectors[sec_no] = NULL;
                }
                return 0;
            }
            // allocate a sector if needed, and write
            if (isRomAddress((char *)buffer) == false) {
                if ((sectors[sec_no] == NULL) || (isRomAddress(sectors[sec_no]) != false)) {
                    char *sec = (char*)malloc(SECTOR_SIZE);
                    if (sec == NULL) {
                        return 1; // out of memory
                    }
                    sectors[sec_no] = sec;
                }
                memcpy(sectors[sec_no], buffer, SECTOR_SIZE);
            } else {
                if (isRomAddress(sectors[sec_no]) == false) {
                    free(sectors[sec_no]);
                }
                sectors[sec_no] = (char *)buffer;
            }
            buffer += SECTOR_SIZE;
        }
        return 0;
    }

    // return the number of sectors
    virtual uint32_t disk_sectors() {
        return NUM_OF_SECTORS;
    }

    void dump(FILE *fp) {
        for (int i = 0; i < NUM_OF_SECTORS; i++) {
            fwrite(&sectors[i], sizeof(int), 1, fp);
            if (sectors[i] != NULL) {
                fwrite(sectors[i], sizeof(char), SECTOR_SIZE, fp);
            }
        }
    }

    void load(FILE *fp) {
        int sec_info = 0;
        for (int i = 0; i < NUM_OF_SECTORS; i++) {
            fread(&sec_info, sizeof(int), 1, fp);
            if (sec_info != 0) {
                char *sec = (char *)malloc(SECTOR_SIZE);
                fread(sec, sizeof(char), SECTOR_SIZE, fp);
                sectors[i] = sec;
            }
        }
    }

private:
    bool isRomAddress(char * address) {
        if (((uint32_t)address >= ROM_START_ADDRESS)
         && ((uint32_t)address <= (ROM_END_ADDRESS - SECTOR_SIZE + 1))) {
            return true;
        }
        return false;
    }
};
#endif
