/*
 * main.c
 */
/*
 * Name RC522DumpMifare.ino
 *
 * 2013-07-xx started
 * 2013-08-03 first commit
 * 2013-08-08 anticol/select 7byte UID, RATS
 *
 * Based on code by www.electrodragon.com
 * and modified/fixed by http://twitter.com/regnerischerTag
 *
 * TODO:
 *  - ISO/IEC 14443-4
 *  - Auth 7byte
 *
 *


 *
 * ported to Keil for Stellaris/Tiva Launchpads
 * Stephan Goldenberg
 * 2014-06-01 - started
 * 2014-06-08 - 1st successful build
 *
 */


#include "tm4c123gh6pm.h"								// TM4C123G Launchpad
#include "MFRC522.h"									// symbols for RC522 by regnerischerTag
#include <stdbool.h>									// true AND false
#include <string.h>										// memcpy


// using SSI0 [PA_5 - MOSI, PA_4 - MISO, PA_3 - CS, PA_2 - SCK], PA_6 - CS, PA_7 - NRSTPD
// using UART0 PA_0 & PA_1 for serial communication

#define chipSelectPin 0x40
#define NRSTPD 0x80
#define HIGH 0x01
#define LOW 0x00

#define	uchar	unsigned char
#define	uint	unsigned int
//data array maxium length
#define MAX_LEN 16

unsigned char serNum[];
unsigned char serNum7[];

void Clock_Init(void);
void SPI_Init(void);
void UART_Init(void);
// void digitalWrite(int, int);
unsigned char SPI_transfer(unsigned char);
unsigned char sendAfterWaiting(unsigned char);
void Serial_print(unsigned char[]);
void Serial_println(unsigned char[]);
void Serial_print_hex(unsigned char);
void Serial_println_hex(unsigned char);
void UART_OutChar(unsigned char);
void SysTick_Init(void);
void SysTick_Wait(unsigned int);
void delay(unsigned int);

void MFRC522_Init(void);
bool selectCard(bool);
void Write_MFRC522(unsigned char, unsigned char);
unsigned char Read_MFRC522(unsigned char);
unsigned char MFRC522_Read(unsigned char, unsigned char *);
unsigned char MFRC522_Auth(unsigned char, unsigned char, unsigned char *, unsigned char *);
void dumpHex(char* buffer, int len);
unsigned char MFRC522_Request(unsigned char, unsigned char *);
unsigned char MFRC522_Anticoll(unsigned char *);
unsigned char MFRC522_Anticoll2(unsigned char *);
unsigned char MFRC522_SelectTag(unsigned char *, unsigned char *);
unsigned char MFRC522_SelectTag2(unsigned char *, unsigned char *);
unsigned char MFRC522_RATS(unsigned char *, unsigned int *);
unsigned char MFRC522_ToCard(unsigned char, unsigned char *, unsigned char, unsigned char *, unsigned int *);
unsigned char MFRC522_RATS(unsigned char *, unsigned int *);
void CalulateCRC(unsigned char *, unsigned char, unsigned char *);


void Clock_Init(){
	SYSCTL_RCC2_R |= 0x80000000;										// use RCC2
	SYSCTL_RCC2_R |= 0x00000800;										// bypass PLL during setup
	SYSCTL_RCC_R = (SYSCTL_RCC_R& ~0x000007C0)+0x00000540;				// 16 MHz XTAL
	SYSCTL_RCC2_R &= ~0x00000070;										// config for main oscillator source
	SYSCTL_RCC2_R &= ~0x00002000;										// activatte PLL by clearing PWRDN
	SYSCTL_RCC2_R |= 0x40000000;										// use 400 MHz PLL
	SYSCTL_RCC2_R = (SYSCTL_RCC2_R& ~0x1FC00000)+(4<<22);				// 80 MHz
	while((SYSCTL_RIS_R&0x00000040) == 0){};							// wait for PLL lock
	SYSCTL_RCC2_R &= ~0x00000800;										// enable PLL by clearing BYPASS
	// volatile unsigned int tictac;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;								// activate clock for port A
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_SSI0;								// activate clock for SSI 0
	SYSCTL_RCGC1_R |= SYSCTL_SCGC1_UART0;								// activate clock for UART 0
	//tictac = SYSCTL_RCGC2_R;											// waste some time for the oscillator to settle
}

void SPI_Init(){
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0F00FF) + 0x00202200;
	GPIO_PORTA_AMSEL_R = 0;												// disable analog modules
	GPIO_PORTA_AFSEL_R |= 0x34;											// enable alt function on PA2,4,5
	GPIO_PORTA_DIR_R |= 0x08;											// disable PA3 (CS) - done using regular GPIO
	GPIO_PORTA_DEN_R |= 0x3C;											// enable digital IO on PA2,3,4,5
	GPIO_PORTA_DATA_R |= 0x08;											// pull PA3 high

	SSI0_CR1_R &= ~SSI_CR1_SSE;											// disable SSI0
	SSI0_CR1_R &= ~SSI_CR1_MS;											// master mode
	SSI0_CPSR_R = (SSI0_CPSR_R&~SSI_CPSR_CPSDVSR_M)+8;					// clock prescaler for 4 MHz SSIClk
	SSI0_CR0_R &= ~(SSI_CR0_SCR_M | SSI_CR0_SPH | SSI_CR0_SPO);
	// SSI0_CR0_R |= SSI_CR0_SPO;											// set SPO for a try -> tried all 4 combinations, doesn't work SPI sucks !
	SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_FRF_M)+SSI_CR0_FRF_MOTO;			// Freescale mode
	SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_DSS_M)+SSI_CR0_DSS_8;				// 8-bit data
	SSI0_CR1_R |= SSI_CR1_SSE;											// enable SSI
}

void UART_Init(){
	// SYSCTL_RCGC1_R |= 0x0001;										// activate UART0
	// SYSCTL_RCGC2_R |= 0x0001;										// activate port A
	UART0_CTL_R &= ~0001;												// disable UART0
	UART0_IBRD_R = 43;													// IBRD = (80M / (16 * 115k2)) = int(43.40278)
	UART0_FBRD_R = 26;													// FBRD = round(0.40278 * 64) = 26
	UART0_LCRH_R = 0x0070;												// 8-bit word length, enable FIFO
	UART0_CTL_R = 0x0301;												// enable RXE, TXE and UART
	GPIO_PORTA_PCTL_R |= (GPIO_PORTA_PCTL_R&0xFFFFFF00) + 0x00000011;
	GPIO_PORTA_AMSEL_R &= ~0x03;										// disable analog function
	GPIO_PORTA_AFSEL_R |= 0x03;											// enable alt functon on PA0-1
	GPIO_PORTA_DEN_R |= 0x03;											// enable digital IO on PA0-1
}

void SysTick_Init(){
	NVIC_ST_CTRL_R = 0;													// disable SysTick during setup
	NVIC_ST_RELOAD_R = 0x00FFFFFF;										// max reload value
	NVIC_ST_CURRENT_R = 0;												// any write to CURRENT clears it
	NVIC_ST_CTRL_R = 0x05;												// enable SysTick with core clock
}

void SysTick_Wait(unsigned int reload){									// wait for (reload-1) * 12.5ns
	NVIC_ST_RELOAD_R = reload-1;										// # of counts to wait
	NVIC_ST_CURRENT_R = 0;												// any write to CURRENT clears it
	while((NVIC_ST_CTRL_R&0x000010000) == 0){};							// wait for count flag
}

void delay(unsigned int milis){											// delay milis ms
	unsigned int i;
	for(i=0; i<milis; i++){
		SysTick_Wait(80000);											// 80,000 * 12.5ns = 1ms
	}
}


/*void digitalWrite(int pin, int state){
		if(state == HIGH){
		GPIO_PORTA_DATA_R |= pin;
	} else{
		GPIO_PORTA_DATA_R &= ~pin;
	}
} */

unsigned char SPI_transfer(unsigned char data){
	unsigned short rec_data;

	GPIO_PORTA_DATA_R &= ~chipSelectPin;								// !CS = 0
	sendAfterWaiting(data);												// send command
	rec_data = sendAfterWaiting(0);										// send nothing, receive byte
	GPIO_PORTA_DATA_R |= chipSelectPin;									// !CS = 1

	return (unsigned char)rec_data;
}

unsigned char sendAfterWaiting(unsigned char code){
	while((SSI0_SR_R&SSI_SR_TFE) == 0){};								// wait until FIFO empty
	SSI0_DR_R = code;													// push data out -> This has no effect, SSI0_DR keeps 0x00 - what's up with this TI shit ?
	while((SSI0_SR_R&SSI_SR_RNE) == 0){};								// wait for response

	return (unsigned char)SSI0_DR_R;
}

void UART_OutChar(unsigned char data){
	while((UART0_FR_R&0x0020) != 0);									// wait until TXFF is 0
	UART0_DR_R = data;
}

void Serial_print(unsigned char text[]){
	unsigned int i = 0;
	while(text[i] != 0){
		UART_OutChar(text[i]);
		i++;
	}
}

void Serial_println(unsigned char text[]){
	Serial_print(text);
	UART_OutChar('\r');
	UART_OutChar('\n');
}

void Serial_print_hex(unsigned char hexnum){
	unsigned char outchar;

	outchar = hexnum / 16;
	if (outchar > 9){
		outchar += 0x30;
	} else{
		outchar += 0x40;
	}
	UART_OutChar(outchar);
	outchar = hexnum % 16;
	if (outchar > 9){
		outchar += 0x30;
	} else{
		outchar += 0x40;
	}
	UART_OutChar(outchar);
}

void Serial_println_hex(unsigned char hexnum){
	Serial_print_hex(hexnum);
	UART_OutChar('\r');
	UART_OutChar('\n');
}


int main(void){
	unsigned char serNum[5];											// 4 bytes Serial number of card, the 5th byte is crc
	unsigned char serNum7[8];											// 7 bytes Serial number of card, the 8th byte is crc
	//buffer
	//uchar str[MAX_LEN];

	uchar defaultKeyA[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uchar madKeyA[16] =     { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5 };
	uchar NDEFKeyA[16] =    { 0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7 };

	Clock_Init();
	SysTick_Init();
	UART_Init();
	SPI_Init();

	while(1){
	// pinMode(chipSelectPin,OUTPUT);         // Set digital pin 10 as OUTPUT to connect it to the RFID /ENABLE pin
	GPIO_PORTA_DATA_R &= ~chipSelectPin;      								// Activate the RFID reader
	// pinMode(NRSTPD,OUTPUT);                // Set digital pin 10 , Not Reset and Power-down
	GPIO_PORTA_DATA_R |= NRSTPD;

	MFRC522_Init();

	//display version info
	//9.3.4.8 VersionReg register : 0x91 / 0x92
	uchar version = Read_MFRC522(VersionReg);
	Serial_print("MFRC522 Version: 0x");
	Serial_println_hex(version);

	// while(1){

    uchar status;
    uchar buffer[MAX_LEN];
    if (selectCard(true)){
			int block;
         for(block=0; block < 64; block++)
         {
              status = MFRC522_Auth(PICC_AUTHENT1A, block, defaultKeyA, serNum); //auth with default key
              if (status != MI_OK)
              {
                   selectCard(false);
                   status = MFRC522_Auth(PICC_AUTHENT1A, block, madKeyA, serNum); //auth with MAD key
              }
              if (status != MI_OK)
              {
                   selectCard(false);
                   status = MFRC522_Auth(PICC_AUTHENT1A, block, NDEFKeyA, serNum); //auth NDEF data key
              }
              if (status == MI_OK)
              {
                   status = MFRC522_Read(block, buffer);
                   if (status == MI_OK)
                   {
                        if (block % 4 == 0)
                        {
                            Serial_print("Sector ");
                            Serial_print_hex(block / 4);
                            Serial_println(": ");

                        }
                        dumpHex((char*)buffer, MAX_LEN);
                    }
                    else
                    {
                        Serial_println("Read failed");
                        break;
                    }
              }
              else
              {
                  Serial_println("Auth failed");
                  //TODO Mifare Ultra-Light
                  //MIFARE Ultralight C http://www.nxp.com/documents/short_data_sheet/MF0ICU2_SDS.pdf
                  break;
              }
         }//for
         delay(1000);
    }
    else
    {
        Serial_println("no card select");
    }
    //reset/init for next loop
    MFRC522_Init();
    delay(500);
	}
}


bool selectCard(bool dumpInfo){

  uchar status;
  uchar buffer[MAX_LEN];
  //Search card, return card types
  status = MFRC522_Request(PICC_REQIDL, buffer);//ShortFrame: 0x26 REQA (Request Type A)
  //status = MFRC522_Request(PICC_REQALL, buffer);//0x52 WUPA (Wake-Up)
  if (status == MI_OK)
  {
     if (dumpInfo)
     {
         Serial_print("Card detected.\r\n ATQA:");
         dumpHex((char*)buffer, 2);
         Serial_println(" ");
     }
     //Prevent conflict, return the 4 bytes Serial number of the card
     status = MFRC522_Anticoll(buffer);
     if (status == MI_OK){
			 unsigned char sak = 0;
         memcpy(serNum, buffer, 5);
         status = MFRC522_SelectTag(serNum, &sak);
         if (status == MI_OK && ((sak & 0x04) == 0x00))
         {
             if (dumpInfo)
             {
                 Serial_print(" UID: ");
                 dumpHex((char*)serNum, 4);
                 Serial_println("");
             }
             if ((sak & 0x20) == 0x20)
             {
                 //ISO/IEC FCD 14443-3: Table 9 — Coding of SAK
                 //if (dumpInfo)
                 //    Serial_println(" UID complete, PICC compliant with ISO/IEC 14443-4");
                 //send RATS (Request for Answer To Select)
                 uchar ats[MAX_LEN];
                 uint unLen = 0;
                 status = MFRC522_RATS(ats, &unLen);
                 if (status == MI_OK && dumpInfo)
                 {
                      Serial_println(" ATS: ");
                      dumpHex((char*)ats, ats[0]);
                      Serial_println("");
                 }
             }
             if (dumpInfo)
             {
                 Serial_print(" SAK: ");
                 Serial_print_hex(sak);
                 Serial_println("");
             }
             return true;
         }
         else
         {
             //cascading level 2
             memcpy(serNum7, &serNum[1], 3);//cascading L1
             status = MFRC522_Anticoll2(buffer);
             if (status == MI_OK)
             {
                 memcpy(&serNum7[3], buffer, 4);
                 status = MFRC522_SelectTag2(&serNum7[3], &sak);
                 if (dumpInfo)
                 {
                    Serial_print(" UID: ");
                    dumpHex((char*)serNum7, 7);
                    Serial_println("");
                    Serial_print(" SAK: ");
                    Serial_print_hex(sak);
                    Serial_println("");
                 }
                 return true;
             }
             else
             {
                 Serial_println("ANTICOLL error: cascading level 2");
             }
         }
   }//Anticoll
   else
   {
      Serial_print("ANTICOLL failed");
   }
 }
 else
 {
     //Serial_print("-");
 }
 return false;
}//selectCard

/*
 * Function:Write_MFRC5200
 * Description:write a byte data into one register of MR RC522
 * Input parameter:addr--register address;val--the value that need to write in
 * Return:Null
 */
void Write_MFRC522(uchar addr, uchar val)
{
	GPIO_PORTA_DATA_R &= ~chipSelectPin;

	//address format:0XXXXXX0
	SPI_transfer((addr<<1)&0x7E);
	SPI_transfer(val);

	GPIO_PORTA_DATA_R |= chipSelectPin;
}


/*
 * Function:Read_MFRC522
 * Description:read a byte data into one register of MR RC522
 * Input parameter:addr--register address
 * Return:return the read value
 */
uchar Read_MFRC522(uchar addr)
{
	uchar val;

	GPIO_PORTA_DATA_R &= ~chipSelectPin;

	//address format:1XXXXXX0
	SPI_transfer(((addr<<1)&0x7E) | 0x80);
	val =SPI_transfer(0x00);

	GPIO_PORTA_DATA_R |= chipSelectPin;

	return val;
}

/*
 * Function:SetBitMask
 * Description:set RC522 register bit
 * Input parameter:reg--register address;mask--value
 * Return:null
 */
void SetBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}


/*
 * Function:ClearBitMask
 * Description:clear RC522 register bit
 * Input parameter:reg--register address;mask--value
 * Return:null
 */
void ClearBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}


/*
 * Function:AntennaOn
 * Description:Turn on antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter:null
 * Return:null
 */
void AntennaOn(void)
{
	uchar temp;

	temp = Read_MFRC522(TxControlReg);
	if (!(temp & 0x03))
	{
		SetBitMask(TxControlReg, 0x03);
	}
}


/*
 * Function:AntennaOff
 * Description:Turn off antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter:null
 * Return:null
 */
void AntennaOff(void)
{
	ClearBitMask(TxControlReg, 0x03);
}


/*
 * Function:ResetMFRC522
 * Description: reset RC522
 * Input parameter:null
 * Return:null
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}


/*
 * Function:InitMFRC522
 * Description:initilize RC522
 * Input parameter:null
 * Return:null
 */
void MFRC522_Init(void)
{
   GPIO_PORTA_DATA_R |= NRSTPD;

   MFRC522_Reset();

    //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_MFRC522(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_MFRC522(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
    Write_MFRC522(TReloadRegL, 30);
    Write_MFRC522(TReloadRegH, 0);

    Write_MFRC522(TxAutoReg, 0x40);		//100%ASK
    Write_MFRC522(ModeReg, 0x3D);		//CRC initilizate value 0x6363	???

    //ClearBitMask(Status2Reg, 0x08);		//MFCrypto1On=0
    //Write_MFRC522(RxSelReg, 0x86);		//RxWait = RxSelReg[5..0]
    //Write_MFRC522(RFCfgReg, 0x7F);   		//RxGain = 48dB

    AntennaOn();		//turn on antenna
}


/*
 * Function:MFRC522_Request
 * Description:Searching card, read card type
 * Input parameter:reqMode--search methods,
 *			 TagType--return card types
 *			 	0x4400 = Mifare_UltraLight
 *				0x0400 = Mifare_One(S50)
 *				0x0200 = Mifare_One(S70)
 *				0x0800 = Mifare_Pro(X)
 *				0x4403 = Mifare_DESFire
 * return:return MI_OK if successed
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
	uchar status;
	uint backBits;			//the data bits that received

	Write_MFRC522(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10))
	{
	     status = MI_ERR;
/*
             Serial_print("status: ");
             Serial_print_hex(status);
             Serial_print(" backBits: ");
             Serial_print_hex(backBits);
             Serial_println("");
*/
	}
	return status;
}


/*
 * Function:MFRC522_ToCard
 * Description:communicate between RC522 and ISO14443
 * Input parameter:command--MF522 command bits
 *			 sendData--send data to card via rc522
 *			 sendLen--send data length
 *			 backData--the return data from card
 *			 backLen--the length of return data
 * return:return MI_OK if successed
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    uint i;

    switch (command)
    {
        case PCD_AUTHENT:		//verify card password
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
	case PCD_TRANSCEIVE:	//send data in the FIFO
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
	default:
			break;
    }

    Write_MFRC522(CommIEnReg, irqEn|0x80);	//Allow interruption
    ClearBitMask(CommIrqReg, 0x80);			//Clear all the interrupt bits
    SetBitMask(FIFOLevelReg, 0x80);			//FlushBuffer=1, FIFO initilizate

    Write_MFRC522(CommandReg, PCD_IDLE);	//NO action;cancel current command	???

    //write data into FIFO
    for (i=0; i<sendLen; i++)
    {
        Write_MFRC522(FIFODataReg, sendData[i]);
    }

    //procceed it
    Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80);		//StartSend=1,transmission of data starts
    }

    //waite receive data is finished
    i = 2000;	//i should adjust according the clock, the maxium the waiting time should be 25 ms???
    do
    {
	//CommIrqReg[7..0]
	//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);			//StartSend=0

    if (i != 0)
    {
        if(!(Read_MFRC522(ErrorReg) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
               status = MI_NOTAGERR;			//??
            }

            if (command == PCD_TRANSCEIVE)
            {
               	n = Read_MFRC522(FIFOLevelReg);
              	lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {
		   *backLen = (n-1)*8 + lastBits;
		}
                else
                {
		   *backLen = n*8;
		}

                if (n == 0)
                {
		    n = 1;
		}
                if (n > MAX_LEN)
                {
		    n = MAX_LEN;
		}

		//read the data from FIFO
                for (i=0; i<n; i++)
                {
		    backData[i] = Read_MFRC522(FIFODataReg);
		}
            }
        }
        else
        {
	    status = MI_ERR;
	}

    }
    else
    {
         //Serial_print("i=0");
    }
	
    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE);

    return status;
}


/*
 * Function:MFRC522_Anticoll
 * Description:Prevent conflict, read the card serial number
 * Input parameter:serNum--return the 4 bytes card serial number, the 5th byte is recheck byte
 * return:return MI_OK if successed
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
    uchar serNumCheck=0;
    uint unLen;


    //ClearBitMask(Status2Reg, 0x08);		//TempSensclear
    //ClearBitMask(CollReg,0x80);		//ValuesAfterColl
    Write_MFRC522(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
    {
	 //Verify card serial number
         for (i=0; i<4; i++)
	 {
	     serNumCheck ^= serNum[i];
	 }
	 if (serNumCheck != serNum[i])
	 {
	     status = MI_ERR;
	 }
    }
    //SetBitMask(CollReg, 0x80);		//ValuesAfterColl=1
    return status;
}

//ANTICOLL cascading level 2
uchar MFRC522_Anticoll2(uchar *serNum)
{
    uchar status;
    uchar i;
    uchar serNumCheck=0;
    uint unLen;


    //ClearBitMask(Status2Reg, 0x08);		//TempSensclear
    //ClearBitMask(CollReg,0x80);		//ValuesAfterColl
    Write_MFRC522(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL2;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
    {
	 //Verify card serial number
         for (i=0; i<4; i++)
	 {
	     serNumCheck ^= serNum[i];
	 }
	 if (serNumCheck != serNum[i])
	 {
	     status = MI_ERR;
	 }
    }
    //SetBitMask(CollReg, 0x80);		//ValuesAfterColl=1
    return status;
}

//send RATS and returns ATS
uchar MFRC522_RATS(uchar *recvData, uint *pLen)
{
    uchar status;
    uint unLen = 0;

    recvData[0] = 0xE0; // RATS
    recvData[1] = 0x50; // FSD=128, CID=0
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
    /*
    Serial_print("unLen: ");
    Serial_print_hex(unLen);
    Serial_print(" status: ");
    Serial_print_hex(status);
    Serial_println("");
    */
    //TODO
    //if ((status != MI_OK) || (unLen != 0x90))
    //{
    //    status = MI_ERR;
    //}
    return status;
}


/*
 * Function:CalulateCRC
 * Description:Use MF522 to caculate CRC
 * Input parameter:pIndata--the CRC data need to be read,len--data length,pOutData-- the caculated result of CRC
 * return:Null
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);			//CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);			//Clear FIFO pointer
    //Write_MFRC522(CommandReg, PCD_IDLE);

	//Write data into FIFO
    for (i=0; i<len; i++)
    {
        Write_MFRC522(FIFODataReg, *(pIndata+i));
    }
    Write_MFRC522(CommandReg, PCD_CALCCRC);

	//waite CRC caculation to finish
    i = 0xFF;
    do
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//read CRC caculation result
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}


/*
 * Function:MFRC522_SelectTag
 * Description:Select card, read card storage volume
 * Input parameter:serNum--Send card serial number
 * sak see ISO14443-3 Table 9 — Coding of SAK
 * return return MI_OK if successed
 */
uchar MFRC522_SelectTag(uchar *serNum, uchar *sak)
{
    uchar i;
    uchar status;
    //uchar size;
    uint recvBits;
    uchar buffer[9];
    //uchar buffCheck=0;

    //ClearBitMask(Status2Reg, 0x08);			//MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
    	buffer[i+2] = *(serNum+i);
    }
    CalulateCRC(buffer, 7, &buffer[7]);		//??
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    //TODO: the above call returns 2 instead of MI_OK -> why?
    status = MI_OK;
    //Serial_print("recvBits: ");
    //Serial_print(recvBits, DEC);
    /*
    for (i=0; i<recBits / 8; i++)
    {
    	buff[i] = *(buffer+i);
    }*/
    //dumpHex((char*)buffer, recvBits / 8);
    *sak = buffer[0];
    //Verify received buffer
    /* TODO
    for (i=0; i< recvBits/8; i++)
    {
       buffCheck ^= buffer[i];
    }
    if (buffCheck != buffer[i])
    {
       status = MI_ERR;
    }*/
    return status;
}

uchar MFRC522_SelectTag2(uchar *serNum, uchar *sak)
{
    uchar i;
    uchar status;
    //uchar size;
    uint recvBits;
    uchar buffer[9];
    //uchar buffCheck=0;

    //ClearBitMask(Status2Reg, 0x08);			//MFCrypto1On=0

    buffer[0] = PICC_ANTICOLL2;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
    	buffer[i+2] = *(serNum+i);
    }
    CalulateCRC(buffer, 7, &buffer[7]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    //TODO: the above call returns 2 instead of MI_OK -> why?
    status = MI_OK;
    //Serial_print("recvBits: ");
    //Serial_print(recvBits, DEC);
    /*
    for (i=0; i<recBits / 8; i++)
    {
    	buff[i] = *(buffer+i);
    }*/
    //dumpHex((char*)buffer, recvBits / 8);
    *sak = buffer[0];
    //Verify received buffer
    /* TODO
    for (i=0; i< recvBits/8; i++)
    {
       buffCheck ^= buffer[i];
    }
    if (buffCheck != buffer[i])
    {
       status = MI_ERR;
    }*/
    return status;
}


/*
 * Function:MFRC522_Auth
 * Description:verify card password
 * Input parameters:authMode--password verify mode
                 0x60 = verify A passowrd key
                 0x61 = verify B passowrd key
             BlockAddr--Block address
             Sectorkey--Block password
             serNum--Card serial number ,4 bytes
 * return:return MI_OK if successed
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
{
    uchar status;
    uint recvBits;
    uchar i;
    uchar buff[12];

    //Verify command + block address + buffer password + card SN
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {
        buff[i+2] = *(Sectorkey+i);
    }
    for (i=0; i<4; i++)
    {
        buff[i+8] = *(serNum+i);
    }
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {
        status = MI_ERR;
    }

    return status;
}


/*
 * Function:MFRC522_Read
 * Description:Read data
 * Input parameters:blockAddr--block address;recvData--the block data which are read
 * return:return MI_OK if successed
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
{
    uchar status;
    uint unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}


/*
 * Function:MFRC522_Write
 * Description:write block data
 * Input parameters:blockAddr--block address;writeData--Write 16 bytes data into block
 * return:return MI_OK if successed
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint recvBits;
    uchar i;
    uchar buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        for (i=0; i<16; i++)		//Write 16 bytes data into FIFO
        {
        	buff[i] = *(writeData+i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
			status = MI_ERR;
		}
    }

    return status;
}

/*
 * Function:MFRC522_Halt
 * Description:Command the cards into sleep mode
 * Input parameters:null
 * return:null
 */
void MFRC522_Halt(void)
{
    uchar status;
    uint unLen;
    uchar buff[4];

    //ISO14443-3: 6.4.3 HLTA command
    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

void dumpHex(char* buffer, int len){
	unsigned char i;
  for(i=0; i < len; i++) {
     char text[4];
     if (i % 16 == 0) {
        Serial_print(" ");
     }
     sprintf(text, "%02X \x00", (unsigned char)(*(buffer + i)));
     Serial_print(text);

     if (i % 16 == 15) {
        Serial_println("");
     }
  }
  //Serial_println(" ");
}

