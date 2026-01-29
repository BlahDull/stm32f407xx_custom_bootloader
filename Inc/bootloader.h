/*
 * bootloader.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Blah
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#define BL_GET_VER 0x51
#define BL_GET_HELP 0x52
#define BL_GET_CID 0x53
#define BL_GET_RDP_STATUS 0x54
#define BL_GO_TO_ADDR 0x55
#define BL_FLASH_ERASE 0x56
#define BL_MEM_WRITE 0x57
#define BL_EN_RW_PROTECT 0x58
#define BL_MEM_READ 0x59
#define BL_READ_SECTOR_STATUS 0x5A
#define BL_OTP_READ 0x5B
#define BL_DIS_RW_PROTECT 0x5C

void BL_GetVer(uint8_t*);
void BL_GetHelp(uint8_t*);
void BL_GetCID(uint8_t*);
void BL_GetRDP(uint8_t*);
void BL_GotoAddr(uint8_t*);
void BL_FlashErase(uint8_t*);
void BL_MemWrite(uint8_t*);
void BL_MemRead(uint8_t*);
void BL_ReadSectorStatus(uint8_t*);
void BL_OTPRead(uint8_t*);
void BL_DisRWProtect(uint8_t*);
void BL_EnRWPProtect(uint8_t*);

#define BL_ACK 0xA5
#define BL_NACK 0x7F

#define CRC_OK 0x1
#define CRC_ERR 0x0

#define BL_VER 0x1A

#define DBGMCU_ID 0xE0042000

#endif /* INC_BOOTLOADER_H_ */
