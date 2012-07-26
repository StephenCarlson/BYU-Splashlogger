// BYU Splash Lab Logger, by Stephen Carlson, Janurary 2012
// Use Notepad++ with Tab=4

// Pins Configuration:
//    -- Package --
// PDIP-28		TQFP-32		Pro		Name	Function Used	Assignment			Code Label
//    14		   12		8		PB0		DIO				SPI CS for ADXL		CS_ADXL
//    15		   13		9		PB1		DIO				Blue LED			LED
//    16		   14		10		PB2		/SS				SPI CS for Flash	CS_FLASH
//    17		   15		11		PB3		MOSI			SPI Tx
//    18		   16		12		PB4		MISO			SPI Rx
//    19		   17		13		PB5		SCK				SPI, Green LED
//     9		   --		-		PB6		XTAL1
//    10		   --		-		PB7		XTAL2
//    23		   23		A0		PC0		ADC0			
//    24		   24		A1		PC1		ADC1			
//    25		   25		A2		PC2		ADC2			
//    26		   26		A3		PC3		ADC3			
//    27		   27		A4		PC4		SDA				I2C SDA
//    28		   28		A5		PC5		SCL				I2C SCL
//    1			   29		RST		PC6		/Reset			Reset
//    2			   30		RXI		PD0		RXD				UART Rx
//    3			   31		TXO		PD1		TXD				UART Tx
//    4			   32		2		PD2		INT0			ADXL Int 2			ADXLINT2
//    5			   1		3		PD3		INT1			ADXL Int 1			ADXLINT1
//    6			   2		4		PD4				
//    11		   9		5		PD5					
//    12		   10		6		PD6					
//    13		   11		7		PD7						
//    --		   19		-		ADC6
//    --		   22		-		ADC7

//           76543210	7		6		5		4		3		2		1		0
// DDRB = 0b --SsSSLS	XTAL2	XTAL1	SCK		MISO	MOSI	CS Fl	LED		CS Adxl
// DDRC = 0b -0CC____	--		Reset	SCL		SDA
// DDRD = 0b ____ii10									Int1	Int2	TXD		RXD

// Legend: - N/A		_ Avaliable		S/s SPI		C I2C		i Interrupt		1/0 Output/Input

// =======================================================================

// Behavioral Switches
#define ITG3200

// Debug Switches
//#define DEBUG_MASTER // Wiped out with my 6 April Trimming
//#define DEBUG_ADC
#define DEBUG_VOLTS
//#devine DEBUG_I2C

// System Parameters
#define F_CPU			16000000UL
//#define BAUD			115200 //19200
#define UART_UBRR		8 //(((((F_CPU * 10) / (16L * BAUD)) + 5) / 10) - 1)
#define I2C_FREQ		400000L
#define LOOP_PERIOD		2500	// (16000000 Hz / 64) / 100 Hz
#define BUFFER_SIZE 	512
#define ADXL_FIFO		25 // 28, Critical for proper bandwidth
#define ADXL_RATE		0b00001111 // 0110 Lowest
#define TEST_BLOCKS		16 // (84 Sa/page)(8 pages/block)(16 blocks) = 10752 Sa, / 3200 Sa/sec = 3.36 sec, 
#define TEST_OFFSET		1  // Erase by page: Skip Block 0 as it holds test run number
#define TEST_MAX		30 // (512-1) blocks * (1 2.56sec test)/(16 blocks) >= 30 ("31")

// System Constants
#define SLEEP 			0
#define ARMED 			1
#define ACTIVE 			2
#define SAMPLING		3
#define FAULT 			-1
#define HIGH			1
#define LOW				0
#define READ			1 // ADXL SPI Flags
#define WRITE			0 // Also I2C Direction Flags
#define SINGLE			0
#define MULTI			1
#define ITG3200ADDR		0x68 // was 0x68 for Kyle's
#define ACK				1
#define NACK			0

// Port Definitions and Macros
typedef struct{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 
#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt

#define LED			REGISTER_BIT(PORTB,1)
#define CS_FLASH	REGISTER_BIT(PORTB,2)
#define CS_ADXL		REGISTER_BIT(PORTB,0)
#define ADXLINT1	(PIND &(1<<3))
#define ADXLINT2	(PIND &(1<<2))

// Included Headers
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/twi.h>

// Function Prototypes
void setup(void);
void loop(void);
void testSampleSequence(void);

void dumpSamples(uint8_t);
uint16_t getBatt(void);
char deviceIdCheck(void);

uint16_t getAccelFIFO(uint16_t);
uint16_t getGyroSample(uint16_t);
void ADXL345Init(void);
void ADXL345Mode(int8_t);

void ITG3200Mode(int8_t);

void dataFlashReadBuffer(uint8_t, uint8_t *);
void dataFlashReadPage(uint16_t, uint8_t *);
uint8_t dataFlashReadByte(uint16_t, uint16_t);
void dataFlashWritePage(uint16_t, uint8_t, uint8_t*);
void dataFlashWritePointer(uint8_t);
void dataFlashCleanTestBlocks(uint8_t);
void dataFlashEraseBlock(uint16_t);
uint8_t dataFlashStatus(void);
void dataFlashMode(int8_t);

uint16_t readADC(uint8_t);
uint8_t transferSPI(uint8_t);
void startI2C(uint8_t, uint8_t);
void stopI2C(void);
void writeI2C(uint8_t);
uint8_t readI2C(uint8_t);
static int putUARTchar(char c, FILE *stream);
uint8_t getUARTchar(void);
void flashLED(uint8_t, uint8_t, uint8_t);

// Global Variables
static FILE uart_io = FDEV_SETUP_STREAM(putUARTchar, NULL, _FDEV_SETUP_WRITE);
uint8_t dataBufferA[BUFFER_SIZE]; //volatile
uint8_t dataBufferG[54];
volatile uint8_t adxlInitFlag = 0;
uint8_t testNumber = 0;


// Interrupt Vectors
ISR(USART_RX_vect){
	uint8_t command = UDR0;
	
	switch(command){
		case 'D':
			dumpSamples(testNumber); //(testNumber==0)? 0 : testNumber-1);
			break;
		case '+':
			testNumber = (testNumber >= TEST_MAX)? TEST_MAX : testNumber + 1;
			printf("Test#: %u\n", (testNumber+1));
			break;
		case '-':
			testNumber = (testNumber==0)? 0 : testNumber - 1;
			printf("Test#: %u\n", (testNumber+1));
			break;
		case 'R':
			testNumber = 0;
			dataFlashCleanTestBlocks(testNumber);
			dataFlashWritePointer(testNumber);
			while(dataFlashStatus());
			printf("Test# Reset to 1\n");
			break;
		case 'B':
			printf("Battery: %u\n", getBatt());
			break;
		case 'T':
			printf("Forced Trigger, Sampling...\n");
			testSampleSequence();
		case 'N':
			printf("Test#: %u\n", (testNumber+1));
			break;
		case '?':
			printf("\nConsole Useage:\n+/-\tInc/Dec#\nD\tDump\nR\tReset Test#=1\nB\tBattery mV\nN\tCurrent Test#\nT\tForce Trigger\n?\tConsole Useage\n\n");
			printf("Impending Test# (Stored in Flash): %u\n\n", (dataFlashReadByte(0,0) +1));
			break;
			
			// +/-		Inc/Dec Test #
			// N		Display Test Number
			// R		Reset Test Number
			// B		Battery Voltage (estimated)
			// T		Force Test Trigger
			// D		Dump Selected Test
			// ?		Display Usage Hints
	}	
}

// Main Program
int main(void){
	setup();

	while(1){		
		loop();
	}
	return(0);
}

void setup(void){
	// System
	MCUCR |= (1<<PUD);		// Pull-up Disable

	// Timers
	TCCR1A = 0;
	TCCR1B = (1<<CS12); //(1<<CS11)|(1<<CS10); //
	
	// IO Ports
	// 0: Input (Hi-Z) 1: Output
	//        76543210		7		6		5		4		3		2		1		0
	DDRB |= 0b00001111; //	XTAL2	XTAL1	SCK		MISO	MOSI	CS Fl	LED		CS Adxl
    DDRC |= 0b00000000; //	--		Reset	SCL		SDA
    DDRD |= 0b00000010; //									Int1	Int2	TXD		RXD
	PORTB |=0b00100111; // CS_FLASH = 1 and CS_ADXL = 1 possible alternate
	
	// Serial Port
	UBRR0H = UART_UBRR >> 8;
    UBRR0L = UART_UBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    stdout = &uart_io; //= stdin 
	
	//SPI
	SPCR	= (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);
	
	//I2C
	TWCR = (1<<TWEN) | (1<<TWEA);
	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
	TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
	
	// ADC
	ADMUX 	= (1<<REFS0);	// AVcc Connected
	ADCSRA 	= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
	DIDR0 	= (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);

	// Tasks and Routines
	printf("\n\nBYU Splash Logger dev July2012\n\n");
	
	flashLED(20,10,40);
	
	printf("Device ID Check: ");
	if(deviceIdCheck()){
		printf("OK\n");
		
		ADXL345Init();
		ADXL345Mode(ACTIVE);

#if defined(ITG3200)
		ITG3200Mode(ACTIVE);
#else
		ITG3200Mode(SLEEP);
#endif
		
		for(uint16_t i = 0; i<(BUFFER_SIZE-1); i++){
			dataBufferA[i] = 0x2A; //0x20 + (i & 0x5F); 
		}
		dataBufferA[BUFFER_SIZE-1] = '\n'; //0x2A;
	} else{
		printf("FAILED!\n");
		flashLED(10,30,30);
		return;
	}	
	
	testNumber = dataFlashReadByte(0,0);
	printf("Impending Test #: %u\n", (testNumber+1));
	if(testNumber >= TEST_MAX){		
		printf("Flash Memory Full, Resetting Test # to 1\n");
		testNumber = 0;
		dataFlashWritePointer(testNumber);
		while(dataFlashStatus());
	}
	
	// Critical for Flash to write samples correctly and as fast as possible
	dataFlashCleanTestBlocks(testNumber);
	
	// Console Usage Hints
	printf("\nConsole Useage:\n+/-\tInc/Dec#\nD\tDump\nR\tReset Test#=1\nB\tBattery mV\nN\tCurrent Test#\nT\tForce Trigger\n?\tConsole Useage\n\n");
	
	sei();
}

void loop(void){
	//static int count = 0;
	static char state = 0;
	uint8_t status;
	
	if(ADXLINT1){
		LED = HIGH;
		
		CS_ADXL = LOW;
			transferSPI((READ<<7) | (SINGLE<<6) | 0x30);
			status = transferSPI(0x00);
		CS_ADXL = HIGH;
		
		/*
		printf("%u\t",++count);
		if(status&(1<<6)){
			printf("[1 Tap]");			
			//if(state == 0) testSampleSequence();
			//state = 1;
		}
		if(status&(1<<5)){
			printf("[2 Tap]");
			//if(state == 0) testSampleSequence();
			//state = 1;
		}
		if(status&(1<<4)) printf("[Activity]");
		*/
		if(status&(1<<3)){
			//printf("[Inactive]");
			state = 0;
		}
		if(status&(1<<2)){
			//printf("[Freefall]");
			if(state == 0) testSampleSequence();
			state = 1;
		}
		//printf("\n");
		
		_delay_ms(5);
	}
	LED = LOW;
}

void testSampleSequence(void){
	CS_ADXL = LOW;
		transferSPI((READ<<7) | (SINGLE<<6) | 0x39);
		uint8_t fifoSamples = transferSPI(0x00);
	CS_ADXL = HIGH;
	for(;fifoSamples>ADXL_FIFO;fifoSamples--){
		CS_ADXL = LOW;
			transferSPI((READ<<7) | (SINGLE<<6) | 0x32);
			transferSPI(0x00);
		CS_ADXL = HIGH;
	}
	// CS_ADXL = LOW;
		// transferSPI((READ<<7) | (SINGLE<<6) | 0x39);
		// transferSPI(0x00);
	// CS_ADXL = HIGH;


	uint16_t page = ((TEST_BLOCKS*testNumber)+TEST_OFFSET)<<3;
	TCNT1 = 0;
	
	uint16_t gyroTimeStamp = 0;
	uint8_t gyroCount = 0;
	
	for(uint8_t i=0; i<(8*TEST_BLOCKS); i++){
		LED = HIGH;
		uint16_t time = TCNT1; // If CS12, 62.5 kHz Timer 
		uint16_t bufferIndex = 0;
		uint8_t gyroIndex = (i==0)? 12 : 0;
		
		// 3*(ADXL_FIFO(27)*6+6) + 2 = 
		for(uint8_t j=0; j<3; j++){ // Enable the FIFO and Watermark Interrupt for this
			while(!ADXLINT2){ // if(!GyroReadFlag){ Reading(); Set GyroReadFlag; }
				if((TCNT1-gyroTimeStamp)>167 && (gyroCount<2)){ //about 2.88ms
					gyroTimeStamp = TCNT1;
					gyroCount++;
					gyroIndex = getGyroSample(gyroIndex);
				}
			}
			CS_ADXL = LOW;
				transferSPI((READ<<7) | (SINGLE<<6) | 0x39);
				transferSPI(0x00);
			CS_ADXL = HIGH;
			bufferIndex = getAccelFIFO(bufferIndex); //6 bytes * ADXL_FIFO
			//if(bufferIndex > 504){
			//	printf("OVERFLOW!");
			//	break;
			//}
			
			while(ADXLINT2); // Clear GyroReadFlag
			
			CS_ADXL = LOW;
				transferSPI((READ<<7) | (SINGLE<<6) | 0x39);
				transferSPI(0x00);
			CS_ADXL = HIGH;
			
			gyroTimeStamp = TCNT1;
			gyroCount=0;
			gyroIndex = getGyroSample(gyroIndex);
		}
		
		for(gyroIndex=0; gyroIndex<54; gyroIndex++){
			dataBufferA[bufferIndex++] = dataBufferG[gyroIndex];
		}
		
		dataBufferA[504] = time>>8;
		dataBufferA[505] = time;
		
		uint16_t timeEnd = TCNT1;
		dataBufferA[506] = timeEnd>>8;
		dataBufferA[507] = timeEnd;
		dataBufferA[508] = bufferIndex>>8;
		dataBufferA[509] = bufferIndex;
		LED = LOW;
		dataFlashWritePage(page+i, i&0x01, dataBufferA);
	}
	while(dataFlashStatus());
	
	testNumber = (testNumber >= TEST_MAX)? 0 : testNumber + 1;
	dataFlashCleanTestBlocks(testNumber);
	dataFlashWritePointer(testNumber);
	while(dataFlashStatus());
}




void dumpSamples(uint8_t test){
	if(test>TEST_MAX) return;
	uint16_t page = ((TEST_BLOCKS*test)+TEST_OFFSET)<<3;
	for(uint8_t i=0; i<(8*TEST_BLOCKS); i++){
		dataFlashReadPage(page+i,dataBufferA);
		uint16_t index = 0;
		for(uint8_t j=0; j<(3); j++){
			for(uint8_t k=0; k<ADXL_FIFO; k++){
				printf("_A\t");
				for(uint8_t l=0; l<6; l+=2){
					int16_t value = dataBufferA[index+l]|(dataBufferA[index+l+1]<<8);
					printf("%d\t",value);
				}
				index+=6;
				printf("\n");
			}
			
			
			
			
			
			
			// uint16_t k=(j*6*28);
			// for(; k<(j*6*28+6*ADXL_FIFO); k+=6){
				// int16_t value1 = dataBufferA[k]|(dataBufferA[k+1]<<8);
				// int16_t value2 = dataBufferA[k+2]|(dataBufferA[k+3]<<8);
				// int16_t value3 = dataBufferA[k+4]|(dataBufferA[k+5]<<8);
				// printf("_A%u\t%d\t%d\t%d\n",k,value1,value2,value3);
			// }
			
			// for(k=(j*6*28+6*ADXL_FIFO); k<((j+1)*6*28); k+=6){
				// int16_t value1 = (dataBufferA[k]<<8)|dataBufferA[k+1];
				// int16_t value2 = (dataBufferA[k+2]<<8)|dataBufferA[k+3];
				// int16_t value3 = (dataBufferA[k+4]<<8)|dataBufferA[k+5];
				// printf("_G%u\t%d\t%d\t%d\n",k,value1,value2,value3);
			// }
		}
		
		
		for(uint8_t k=0; k<9; k++){
			printf("_G\t");
			for(uint8_t l=0; l<6; l+=2){
				int16_t value = (dataBufferA[index+l]<<8)|dataBufferA[index+l+1];
				printf("%d\t",value);
			}
			index+=6;
			printf("\n");
		}
		
		uint16_t time1 = (dataBufferA[504]<<8)+dataBufferA[505];
		uint16_t time2 = (dataBufferA[506]<<8)+dataBufferA[507];
		printf("_T\t%u\t%u\n", time1, time2);
		//printf("\n");
		//}
	}
}

uint16_t getBatt(void){
	uint16_t voltSample = readADC(14);
	//voltSample = 11000/voltSample;
	voltSample = 5353 - ((voltSample<<2)+(voltSample<<1)+(voltSample>>2));
	return voltSample;
}

char deviceIdCheck(void){
	CS_ADXL = LOW;
		transferSPI((READ<<7) | (SINGLE<<6) | 0x00);
		uint8_t accel = transferSPI(0x00);
	CS_ADXL = HIGH;
	
	CS_FLASH = LOW;
		transferSPI(0xAB);
	CS_FLASH = HIGH;
	_delay_ms(1);
	CS_FLASH = LOW;
		transferSPI(0x9F);
		transferSPI(0x00);
		uint8_t flash = transferSPI(0x00);
	CS_FLASH = HIGH;
	
	ITG3200Mode(ACTIVE); // 3 April 2012: Had to place this here to fix 0x4E Init Problem
	startI2C(ITG3200ADDR,WRITE);
		writeI2C(0x00);
	stopI2C();
	startI2C(ITG3200ADDR,READ);
		uint8_t gyro = readI2C(NACK);
	stopI2C();
	
	printf("Accel: %X\tFlash: %X\tGyro: %X\n", accel,flash,gyro);
	
	
	if((accel ^ 0b11100101) == 0 && (flash ^ 0x26) == 0 && ((gyro & 0b01111110) ^ 0x68) == 0) return 1;
	return 0;
}

uint16_t getAccelFIFO(uint16_t index){	
	for(uint8_t j=0; j<ADXL_FIFO; j++){
		CS_ADXL = LOW;
			transferSPI((READ<<7) | (MULTI<<6) | 0x32);
			for(uint8_t i=0; i<6; i++){
				if(index > BUFFER_SIZE-1) break; // printf("break");
				dataBufferA[index++] = transferSPI(0x00);
			}
			//transferSPI(0x00);
			//transferSPI(0x00);
		CS_ADXL = HIGH;
	}
	return index;
}
uint16_t getGyroSample(uint16_t index){
	startI2C(ITG3200ADDR, WRITE);
		writeI2C(0x1D);
	stopI2C();
	startI2C(ITG3200ADDR, READ);
		for(uint8_t k=0; k<6; k++){
			if(index > 54-1) break;
			uint8_t ackType = (k < (6-1))? ACK : NACK ;
			dataBufferG[index++] = readI2C(ackType);
		}
	stopI2C();
	return index;
}

void ADXL345Init(void){
	char configArray[] = {					// 21 Values
		0xB0,		// 	1D	THRESH_TAP	6g		62.5 mg/LSB unsigned 0xFF = +16 g
		0x00,		// 	1E	OFSX					15.6 mg/LSB signed 0x7F = +2 g
		0x00,		// 	1F	OFSY
		0x00,		// 	20	OFSZ
		0x30,		// 	21	DUR			30ms	625 uS/LSB		Max time/width of tap peak
		0x50,		// 	22	Latent		100ms	1.25 ms/LSB		No other peak until after this
		0xF0,		// 	23	Window		300ms	1.25 ms/LSB 	Period after latent to make a second peak
		0x08,		// 	24	THRESH_ACT	.5g		62.5 mg/LSB unsigned	Exceed value to flag activity
		0x04,		// 	25	THRESH_INACT	.25g	62.5 mg/LSB unsigned	Stay below for TIME_INACT for inactivity
		0x05,		// 	26	TIME_INACT	5sec	1 sec/LSB
		0b11111111,	// 	27	ACT_INACT_CTL 		ACT[dc/AC][X|Y|Z] INACT[dc/AC][X|Y|Z]
		0x08,		// 	28	THRESH_FF	500mg	62.5 mg/LSB unsigned sqrt(x^2+y^2+z^2)
		0x14,		// 	29	TIME_FF		100ms	5 ms/LSB
		0b00001111,	// 	2A	TAP_AXES				0[7:4], [Suppress] Enable[X|Y|Z]
		0x00,		// 	2B	ACT_TAP_STATUS READ-ONLY [0] Activity[X|Y|Z] [Asleep] Tap[X|Y|Z]
		ADXL_RATE,	// 	2C	BW_RATE				0[7:5], [Low Power] RateCode[3:0]
		0b00100000,	// 	2D	POWER_CTL			0[7:6], [Link][AutoSleep][Measure][Sleep] WakeRate[1:0]
		0b00000000,	// 	2E	INT_ENABLE			[DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
		0b10000011,	// 	2F	INT_MAP				[DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
		0x00,		// 	30	INT_SOURCE READ-ONLY
		0b00001011	// 	31	DATA_FORMAT  FULL_RES bit set
	};
	
	CS_ADXL = LOW;
		transferSPI((WRITE<<7) | (MULTI<<6) | 0x1D);
		for(int i=0; i < 21; i++){
			transferSPI(configArray[i]);
		}
	CS_ADXL = HIGH;
	
	_delay_ms(1);
	
	// FIFO
	CS_ADXL = LOW;
		transferSPI((WRITE<<7) | (SINGLE<<6) | 0x38); // FIFO_CTL
		transferSPI(0b10000000 | (ADXL_FIFO-2)); // Stream Mode
	CS_ADXL = HIGH;
	
	adxlInitFlag = 1;
}

void ADXL345Mode(int8_t mode){
	if(!adxlInitFlag) ADXL345Init();
	
	// if(mode == SLEEP){
		// CS_ADXL = LOW;
			// transferSPI((WRITE<<7) | (MULTI<<6) | 0x2D); // POWER_CTL
			// transferSPI(0b00000111); // Measure off, temp
			// transferSPI(0b00010000);
		// CS_ADXL = HIGH;
	// }
	// if(mode == ARMED){
		// CS_ADXL = LOW;
			// transferSPI((WRITE<<7) | (MULTI<<6) | 0x2D); // POWER_CTL
			// transferSPI(0b00001100);
			// transferSPI(0b01110100);
		// CS_ADXL = HIGH;
	// }	
	if((mode == ACTIVE)||(mode == SAMPLING)){
		CS_ADXL = LOW;
			transferSPI((WRITE<<7) | (MULTI<<6) | 0x2D); // BW_RATE was 0x2C
			//transferSPI(0b00001111); // Rate code on Table 6 on pg. 6 0110 for slow
			transferSPI(0b00101000); // MEASURE bit set, pg. 16
			transferSPI(0b00011110); // [DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
			//transferSPI(0b01111110); // [DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
		CS_ADXL = HIGH;
	}
}

void ITG3200Mode(int8_t mode){
	if(mode == SLEEP || mode == ARMED){
		startI2C(ITG3200ADDR, WRITE);
			writeI2C(0x3E); // Power Management
			writeI2C(0x40); // SLEEP bit set, pg. 27
		stopI2C();
	}
	else{ //if(mode == ACTIVE){
		startI2C(ITG3200ADDR, WRITE);
			writeI2C(0x3E); // Power Management
			writeI2C(0x01); // SLEEP bit cleared, pg. 27
		stopI2C();
		_delay_ms(1);
		startI2C(ITG3200ADDR, WRITE);
			writeI2C(0x15); // Sample Rate Divider
			writeI2C(0x00); // SMPLRT_DIV set to 0x00, pg. 23
			writeI2C(0x18); // DLPF: FS_SEL set to 0x03, pg. 24
			
			// Interrupt on New Data
			//writeI2C(0b00110001);
		stopI2C();
	}
}

void dataFlashReadBuffer(uint8_t bufferSelect, uint8_t *bufferAddr){
	CS_FLASH = LOW;
		transferSPI((bufferSelect == 0) ? 0xD1 : 0xD3);
		transferSPI(0x00);
		transferSPI(0x00);
		transferSPI(0x00);
		for(uint16_t i=0; i<BUFFER_SIZE; i++){
			bufferAddr[i] = transferSPI(0x00);
		}
	CS_FLASH = HIGH;
}

void dataFlashReadPage(uint16_t page, uint8_t *bufferAddr){
	// xxPPPPPP PPPPPPBB BBBBBBBB
	CS_FLASH = LOW;
		transferSPI(0x03);
		transferSPI(page>>6);
		transferSPI(page<<2);
		transferSPI(0x00);
		for(uint16_t i=0; i<BUFFER_SIZE; i++){
			bufferAddr[i] = transferSPI(0x00);
		}
	CS_FLASH = HIGH;
}

uint8_t dataFlashReadByte(uint16_t page, uint16_t byte){
	// xxPPPPPP PPPPPPBB BBBBBBBB
	CS_FLASH = LOW;
		transferSPI(0x03);
		transferSPI(page>>6);
		transferSPI((page<<2)|(byte>>8));
		transferSPI(byte);
		uint8_t value = transferSPI(0x00);
	CS_FLASH = HIGH;
	return value;
}

void dataFlashWritePage(uint16_t page, uint8_t bufferSelect, uint8_t *bufferAddr){
	// xxxxxxxx xxxxxxBB BBBBBBBB
	CS_FLASH = LOW;
		transferSPI((bufferSelect == 0) ? 0x84 : 0x87);
		transferSPI(0x00);
		transferSPI(0x00);
		transferSPI(0x00);
		for(uint16_t i=0; i<BUFFER_SIZE; i++){
			transferSPI(bufferAddr[i]);
		}
	CS_FLASH = HIGH;
	CS_FLASH = HIGH;
	CS_FLASH = HIGH;
	// xxPPPPPP PPPPPPxx xxxxxxxx
	CS_FLASH = LOW;
		transferSPI((bufferSelect == 0) ? 0x88 : 0x89);
		transferSPI(page>>6);
		transferSPI(page<<2);
		transferSPI(0x00);
	CS_FLASH = HIGH;
}

void dataFlashWritePointer(uint8_t value){
	CS_FLASH = LOW;
		transferSPI(0x82);
		transferSPI(0x00);
		transferSPI(0x00);
		transferSPI(0x00);
		for(uint16_t i=0; i<BUFFER_SIZE; i++){
			transferSPI(value);
		}
	CS_FLASH = HIGH;
}

void dataFlashCleanTestBlocks(uint8_t testNumber){
	uint16_t block = (TEST_BLOCKS*testNumber)+TEST_OFFSET;
	for(int i=0; i<TEST_BLOCKS; i++){
		dataFlashEraseBlock(block+i);
		while(dataFlashStatus());
	}
}

void dataFlashEraseBlock(uint16_t block){
	// 0000000BBBBBBBBB     
	CS_FLASH = LOW;
		transferSPI(0x50);
		transferSPI(block>>3);
		transferSPI(block<<5);
		transferSPI(0x00);
	CS_FLASH = HIGH;
}

uint8_t dataFlashStatus(void){
	// SC1011PZ
	CS_FLASH = LOW;
		transferSPI(0xD7);
		int8_t status = transferSPI(0x00);
	CS_FLASH = HIGH;
	return ~(status>>7);
}

void dataFlashMode(int8_t mode){
	if(mode == SLEEP){
		CS_FLASH = LOW;
			transferSPI(0xB9);
		CS_FLASH = HIGH;
	} else{
		CS_FLASH = LOW;
			transferSPI(0xAB);
		CS_FLASH = HIGH;
	}
}

uint16_t readADC(uint8_t adcChannel){
	ADMUX 	= (1<<REFS0) | adcChannel;
	ADCSRA 	|= (1<<ADSC);
	while (ADCSRA & (1 << ADSC));
	return (ADCL + ((uint16_t) ADCH << 8));
}

uint8_t transferSPI(uint8_t data){
	SPDR = data;
	while(!(SPSR & _BV(SPIF)));
	return SPDR;
}

void startI2C(uint8_t address, uint8_t intent){ // i.e. ITG3200ADDR, WRITE
	//while( !(TWCR &(1<<TWINT)));			// Avoid Crashing
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA);	// Send Start
	while( !(TWCR &(1<<TWINT))); //printf("ST1: %X\n",TWSR);			// Wait
#if defined(DEBUG_I2C)
	if(TW_STATUS != TW_START) printf("BadStart\n");
#endif
	TWDR = ((address<<1) | (intent & 0x01));			// Hail Slave Device
	TWCR = (1<<TWINT)|(1<<TWEN);			// Engage
	while( !(TWCR &(1<<TWINT))); //printf("ST2: %X\n",TWSR);			// Wait
#if defined(DEBUG_I2C)
	if(TW_STATUS == TW_NO_INFO || TW_STATUS == TW_BUS_ERROR) printf("BadHold\n");
#endif
}

void stopI2C(void){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	// Send Stop
	while(TWCR &(1<<TWSTO)); //printf("SP: %X\n",TWSR);
}

void writeI2C(uint8_t data){
	TWDR = data;							// Data
	TWCR = (1<<TWINT)|(1<<TWEN);			// Enable
	while( !(TWCR &(1<<TWINT))); //printf("WT: %X\n",TWSR);			// Wait
#if defined(DEBUG_I2C)
	if(TW_STATUS != TW_MT_DATA_ACK) printf("BadFrame\n");
#endif
}

uint8_t readI2C(uint8_t ackType){
	//printf("TWCR: %X", (1<<TWINT)|(1<<TWEN)|(ackType<<TWEA));
	TWCR = (1<<TWINT)|(1<<TWEN)|(ackType<<TWEA);
	while( !(TWCR &(1<<TWINT))); //printf("RV: %X\n",TWSR);
	return TWDR;
}

static int putUARTchar(char c, FILE *stream){
    if (c == '\n') putUARTchar('\r', stream);
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

uint8_t getUARTchar(void){
    while( !(UCSR0A & (1<<RXC0)));
    return(UDR0);
}

void flashLED(uint8_t count, uint8_t high, uint8_t low){
	for(;count>0; count--){
		LED = HIGH;
		_delay_ms(high);
		LED = LOW;
		_delay_ms(low);
	}
}




//	Function						OpCode		Time		Field Description
//
//	Read		Page Directly		D2			66 MHz		3 bytes: 12-bits page, 10 or 9-bits byte, 4 padding bytes
//				Continuous Array	03			33 MHz		3 bytes: 12-bits page, 10-bits byte (for 528 byte version)
//				Buffer 1			D1			-
//				Buffer 2			D3			-
//	Page to		Buffer 1			53			400 us
//				Buffer 2			55			400 us			
//	Write to	Buffer 1			84			-
//				Buffer 2			87			-
//	Commit		Buffer 1 w/ Erase	83			40 ms
//				Buffer 2 w/ Erase	86			40 ms
//				Buffer 1 w/o Erase	88			6 ms
//				Buffer 2 w/o Erase	89			6 ms
//	Erase		Page				81			35 ms
//				Block				50			100 ms
//				Sector				7C			5 seconds
//				Chip				7C 94 80 9A	15+ seconds
//
//	Power Down						B9			Down to 15 uA
//	Resume							AB			Resume to normal
//	Status							D7 			[Ready / !Busy][Compare Result][1011][Protected][Page Size (0)]
//	Device ID						9F			Yields 0x1F 0x26

// Direct Page Read (0xD2)
// xxPPPPPP PPPPPPBB BBBBBBBB (0x00)x4

// Continuous Array Read (0x03 Low Freq Mode)
// xxPPPPPP PPPPPPBB BBBBBBBB

// Extract a Page to Buffer 1/2 (0xD1 / 0xD3)
// xxPPPPPP PPPPPPxx xxxxxxxx

// Write Data to Buffer 1/2 (0x84 / 0x87)
// xxxxxxxx xxxxxxBB BBBBBBBB

// Commit Buffer 1/2 to Flash w/o Erase (0x88 / 0x89)
// xxPPPPPP PPPPPPxx xxxxxxxx

// Page Erase (0x81) Page 0 to 4095
// xxPPPPPP PPPPPPxx xxxxxxxx

// Block Erase (0x50) Block 0 to 511
// xxPPPPPP PPPxxxxx xxxxxxxx

// Sector Erase (0x7C)
// xxPPPPPP PPPxxxxx xxxxxxxx Sector 0a or 0b
// xxPPPPxx xxxxxxxx xxxxxxxx Sector 1 to 15