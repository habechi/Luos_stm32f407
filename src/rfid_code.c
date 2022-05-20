#include "rfid_code.h"

uint32_t 	TimingDelay;
char character[30],T,x[10];
uint8_t		str[16];												// MFRC522_MAX_LEN = 16
uint8_t		lastID[4],h=0,o;
uint8_t 	TagID[4]={0x63, 0x3c, 0x63, 0x5a};
uint8_t 	CardID[4]={0x47, 0x89, 0xe6, 0x2c};
int i;

/*Cablage
MOSI(RC522)=>>>>>>PA7(STM32F4)
MISO(RC522)=>>>>>>PA6(STM32F4)
SCK(RC522)=>>>>>>PA5(STM32F4)
SDA(RC522)=>>>>>>PC1(STM32F4)
RX(HC_05)=>>>>>>PB10(STM32F4)
TX(HC_05)=>>>>>>PB11(STM32F4)

/*****************************************TEMPO*********************************/
void Delay_TIME(__IO uint32_t nCount)
{
while(nCount--);
}

/*******************************Configuration clock system ********************/
void SystemClock_HSE()
{
	RCC->CFGR = 0x00000001; //select HSE as system clock a partirdes SW1 SW2
		RCC->CR |= 1<<16; //make HSE ON
		while (!(RCC->CR & (1<<17)));//While HSE is Ready Flag
}

/*******************************Configuration USART ********************/
void config_Gpio_USART3(void){

	RCC->AHB1ENR |= 0x00000002; // Enable clock for GPIOB
	RCC->APB1ENR|=0x00040000; // Enable clock for USART3
	GPIOB->AFR[1]=0x00007700; // //enable USART3_TX to PB10 and USART3_RX to PB11
	GPIOB->MODER|=0x00A00000; // configuring the USART3 ALTERNATE function to PB10 and PB11
	}
void config_USART3(void)
{
USART3->BRR=0X016C;//115200 Baud BRR=8Mhz/115200  115200 bits per second
//USART3->BRR=0x00D0;//115200 Baud BRR=8Mhz/115200  115200 bits per second

USART3->CR1|=0x0000202C; // USART3 enable

}
/*******************************LES Fonction USART ********************/

void USART3__Send1_Char(uint8_t c)
{
while((USART3->SR & (0x0080))==0); //wait for TXE to be 1 [1:Data tranfered to dhift_registre 0:NOT]

USART3->DR=c;
}
void USART3_send_String(uint8_t *pt)
{
while(*pt)
{
	USART3__Send1_Char(*pt);
pt++;
}
}
char  USART3__Recive1_char(){
	while(((USART3->SR) & (0x0020))!=0)//when Data is Reeady to read RXNE=1 else RXN3=0 'when  data is not recived'
		T =USART3->DR;
		return T ;
}
char USART3__Recive_String(void){
	char c,TAB[100];
	while(!(USART3->SR) & (0x20==0)){////when Data is Reeady to read RXNE=1 else RXN3=0 'when  data is not recived'
		c=USART3->DR;
		TAB[i]=c;
		i++;
	}
	return(TAB);
}
char receive_data()
{
char RXCH = USART3->DR;
return RXCH;
}
void Cofiguration_Led()
{
RCC->AHB1ENR |= 1<<3; //Clocks for GPIOD
GPIOD->MODER = 0x55<<24; //Bits 12..15 are output
GPIOD->PUPDR = 0x55555555; //ACTIVATE PULL_UP for PORTD
}

/********************************Configuration SPI ********************/
void SPI1_Init(){
RCC->AHB1ENR |= 0x00000004;/// Enable clock for GPIOC
GPIOC->MODER |=0x00000004;/// pc1 (pin de selection) output
RCC->AHB1ENR |= 0x00000001; // Enable clock for GPIOA(PA5:SPI_SCK  PA6:SPI_MOSI  PA7:SPI_MISO) 	GPIOC(chip Select)
GPIOA->MODER |=0x0000AA00; // configuring the( SPI_SCK SPI_MOSI SPI_MISO :Alternate Function)
GPIOA->AFR[0]=0x55550000; // enable SPI_MOSI SPI_MISO SPI_SCK SPI_NSS Pin MOde AF
RCC->APB2ENR |= 0x00001000; // Enable clock for SPI1 (activation SPI1)
SPI1->CR1 |=1<<9;  //Use Software NSS Management
SPI1->CR1 |=0x00000003;  //Clock phase =1 Clock Polarity=1 :les donnes sont valide en 2 eme front montant
SPI1->CR1 |=1<<2;  //Master Configuration
SPI1->CR1 |=1<<8;//slect the slave by putting 1 in SSI
//SPI1->CR1 =0<<7;   //MSB Transmitted First
//SPI1->CR1 =0<<11;  //Les donnes sont codée sur 8 Bits
//SPI1->CR1 =0<<10;  //Full-duplex
//SPI1->CR1 =0<<15;  //2 lines unidirectional Data Mode (mode SPI)*/
// MSB is Transmitted First / Les donnes sont codée sur 8 Bits /Full-duplex/ 2 lines unidirectional Data Mode (mode SPI)
SPI1->CR1 |=0x00000008;  //Baudrate fPCLK/32 = 0.25Mhz
//SPI1->CR2 |=0<<2;  //NSS output Pin(PA4) Disable
//SPI1->CR2 |=0<<4;    // SPI MotorOla Mode
//SPI1->CR2 |=0xC0;  //Enable  flags (TXE RXNE ERRTXE)
SPI1->CR1 |=1<<6;  //SPI1 Enable
//SPI1->I2SPR = 0x230;
}
/*******************************LES Fonctions SPI ********************/
uint8_t send_Recive_Byte(uint8_t data){
	uint8_t data1;
	int i=0;
	while( !(SPI1->SR & 0x0002));  //SPI_SR_TXE   Verification flag TXE
	SPI1->DR = data;

	while(!(SPI1->SR & 0x0001)); //  SPI_SR_RXNE   Verification flag RXNE
	data1=SPI1->DR;
	return(data1);
}
void NSS_Hardware_Reset(void){// USE PC1 for the selection of RC522
GPIOC->BSRR=0x00020000;// reset (PC1)
	}
void NSS_Hardware_Set(void){
GPIOC->BSRR=0x00000002;// set (PC1)
}

void Write_Reg(uint8_t adr , uint8_t value){
	NSS_Hardware_Reset();//pc1=0
	send_Recive_Byte(adr);
	send_Recive_Byte(value);
	NSS_Hardware_Set();//pc1=1
}
uint8_t    Read_Reg(uint8_t adr){
	uint8_t value;
	NSS_Hardware_Reset();//pc1=0
	send_Recive_Byte(adr);
	value=send_Recive_Byte(0x00); //
	NSS_Hardware_Set();//pc1=1
	return(value);
}
 /*The MSB of the first byte defines the mode used. To read data from the MFRC522 the
 MSB is set to logic 1. To write data to the MFRC522 the MSB must be set to logic 0. Bits 6
 to 1 define the address and the LSB is set to logic 0.*/
void Write_Reg_RC522(uint8_t adr ,uint8_t value){
	adr= (adr<<1 & 0x7E) ;
	Write_Reg(adr ,  value);
}
uint8_t    Read_Reg_RC522(uint8_t adr){
	uint8_t value;
	adr=( (adr<<1) &(0x7E) ) | 0x80;
	value=Read_Reg( adr);
	return(value);
}
void antenna_on(void){
	//Controls the logical behavior of the antenna driver pins TX1 and TX2.
	uint8_t val= Read_Reg_RC522(0x14);

	if (!(val & 0x03)) {//for making sure that output signal on pin TX1 et TX2 delivers the 13.56 MHz energy carrier
		val=val|0x03;
		Write_Reg_RC522(0x14,val);
	}
}
void reset(){
	Write_Reg_RC522(0x01,0x0F); //Reset the RC522
}

void MFRC522_SetBitMask(uint8_t reg, uint8_t mask) {
	Write_Reg_RC522(reg, Read_Reg_RC522(reg) | mask);
}

void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask){
	Write_Reg_RC522(reg, Read_Reg_RC522(reg) & (~mask));
}
void RC522_Init(void){
	reset();
	Write_Reg_RC522(0x2A,0x8D);// // enable timer and select 4 higher bits of psk value(1101) a timer can be used to mesure the time  interval between to event to indicate that there  is an error occurred after a specific time
	Write_Reg_RC522(0x2B,0x3E);// select 8 lower bits of psk value (111110) f(timer)=13.56Mhz/((Psk*2+1)      *(TReloadVal + 1))
	Write_Reg_RC522(0x2D,30);//Defines the lower 8 bits of 16-bits timer reload value
	Write_Reg_RC522(0x2C,0);//Defines the higher 8 bits of 16-bits timer reload value
	Write_Reg_RC522(0x26,0x70);//Defines the receiver signal voltage gain factor 48db
	Write_Reg_RC522(0x15,0x40);//forces a 100 % ASK modulation independent of the ModGsPReg register setting taux de modulation
	Write_Reg_RC522(0x11,0x3D);
	antenna_on();
}
uint8_t MFRC522_ToCard(uint8_t command, uint8_t * sendData, uint8_t sendLen, uint8_t * backData, uint16_t * backLen) {
	uint8_t status = 2;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) { // suivant la commande
		case 0x0E  : { 	//0x0E    Authentication Key
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case 0x0C: { 	//0x0C    Transmit and receive data,
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
		break;
	}
	Write_Reg_RC522(0x02, irqEn | 0x80);//allow the interruptions(reciver interrupt,transmission interrupt,error,timer,etc..)
	MFRC522_ClearBitMask(0x04, 0x80);
	MFRC522_SetBitMask(0x0A, 0x80); //Clear the internal FIFO buffer
	Write_Reg_RC522(0x01, 0x00);// cancel current commande excution

	// Writing data to the FIFO
	for (i = 0; i < sendLen; i++) Write_Reg_RC522(	0x09, sendData[i]); // 0x09:adrress fifo data // Transfert 0x26

	// Execute the command
	Write_Reg_RC522(0x01, command);//transmits data from FIFO buffer to antenna and automatically activates the receiver after  transmission
	if (command == 0x0C) MFRC522_SetBitMask(0x0D, 0x80);			// StartSend=1,transmission of data starts

    // Waiting to receive data to complete//
	i = 2000;	// i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms
	do {
		// CommIrqReg[7..0]
		// Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = Read_Reg_RC522(0x04);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq)); //waitIRq : 1 if command terminates

	MFRC522_ClearBitMask(0x0D, 0x80);//stop the transmission of data			// StartSend=0

	if (i != 0)  {
		if (!(Read_Reg_RC522(0x06) & 0x1B)) {// show the error status of the last command executed.=there is no error
			status = 0;
			if (n & irqEn & 0x01) status = 1;
			if (command == 0x0C) {
				n = Read_Reg_RC522(0x0A);//indicates the number of bytes stored in the FIFO buffer
				lastBits = Read_Reg_RC522(0x0C) & 0x07;
				if (lastBits) *backLen = (n - 1) * 8 + lastBits; else *backLen = n * 8;
				if (n == 0) n = 1;
				if (n > 16) n = 16;
				for (i = 0; i < n; i++) backData[i] = Read_Reg_RC522(0x09);		// Reading the received data in FIFO
			}
		} else status =2;
	}
	return status;
}
uint8_t MFRC522_Anticoll(uint8_t* serNum) {
	uint8_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	Write_Reg_RC522(0x0D, 0x00);												// TxLastBists = BitFramingReg[2..0]
	serNum[0] = 	0x93;
	serNum[1] = 0x20;
	status = MFRC522_ToCard(0x0C, serNum, 2, serNum, &unLen);
	if (status == 0) {
		// Check card serial number
		for (i = 0; i < 4; i++) serNumCheck ^= serNum[i];
		if (serNumCheck != serNum[i]) status = 2;
	}
	return status;
}

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t * TagType) { // 0x26  // tagtype chaine de char
	uint8_t status;
	uint16_t backBits;					// The received data bits

	Write_Reg_RC522(0x0D, 0x07);// find the antenna area
	TagType[0] = reqMode;
	status = MFRC522_ToCard(0x0C, TagType, 1, TagType, &backBits);	//commande 0x0C Transmit and receive data,
	if ((status != 0) || (backBits != 0x10)) status = 2;
	return status;
}




void rfid_calcul(void) {
uint8_t i, j, q;
if(h==0){
USART3_send_String("\n Please Scan your Card ^______^");
Delay_TIME(0xFF);
h=1;}
	if (!MFRC522_Request(0x26, str)) {


	if (!MFRC522_Anticoll(str)) {

		j = 0;
		q = 0;
		for (i=0; i<4; i++){


		for (i=0; i<4; i++) if (lastID[i] != str[i]) j = 1;
		if (j) {h=0;
			for (i=0; i<4; i++) lastID[i] = str[i];
			USART3_send_String("\nCard ID: ");
			for (i=0; i<3; i++){
				sprintf(x,"0x %02x   ",str[i]); //The %02x is used to convert one character to a hexadecimal string.
				USART3_send_String(x);
			}

			sprintf(x,"0x %02x  ",str[3]);
			USART3_send_String(x);
			if (str[3]=='Z') //USART3_send_String(" \n hello");
			GPIOD->ODR = 0x8000; 
			if(str[3]==',')  //USART3_send_String(" \n hello  ");
			GPIOD->ODR = 0xF000; 
			USART3_send_String(" \n Welcome");
		j=0;}
	}
}
}
}