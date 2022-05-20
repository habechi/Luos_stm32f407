#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
void Delay_TIME(__IO uint32_t nCount);
void SystemClock_HSE();
void config_Gpio_USART3(void);
void config_USART3(void);
void USART3__Send1_Char(uint8_t c);
void USART3_send_String(uint8_t *pt);
char  USART3__Recive1_char();
char USART3__Recive_String(void);
char receive_data();
void Cofiguration_Led();
void SPI1_Init();
uint8_t send_Recive_Byte(uint8_t data);
void NSS_Hardware_Reset(void);
void NSS_Hardware_Set(void);
void Write_Reg(uint8_t adr , uint8_t value);
uint8_t    Read_Reg(uint8_t adr);
void Write_Reg_RC522(uint8_t adr ,uint8_t value);
uint8_t    Read_Reg_RC522(uint8_t adr);
void antenna_on(void);
void reset();
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
void RC522_Init(void);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t * sendData, uint8_t sendLen, uint8_t * backData, uint16_t * backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t * TagType);
void rfid_calcul(void);


