// BYU Splash Lab microLogger, by Stephen Carlson, August 2012
// Use Notepad++ with Tab=4

// Pins Configuration:
//    -- Package --
// PDIP-28		TQFP-32		Pro		Name	Function Used	Assignment			Code Label
//    14		   12		8		PB0		DIO				SPI CS for MPU		CS_MPU MPU_VLOGIC
//    15		   13		9		PB1		DIO				SPI CS for ADXL		CS_ADXL
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
//    4			   32		2		PD2		INT0			MPU Int Line		MPUINT
//    5			   1		3		PD3		INT1			ADXL Int 1			ADXLINT1
//    6			   2		4		PD4		DIO				ADXL Int 2			ADXLINT2		
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
//#define ITG3200
//#define MPU9150
//#define TRIGGER_SELECT

// Debug Switches
//#define DEBUG_MASTER // Wiped out with my 6 April Trimming
//#define DEBUG_ADC
#define DEBUG_VOLTS
//#devine DEBUG_I2C

// System Parameters
#define F_CPU			16000000UL
//#define BAUD			115200 //19200
#define UART_UBRR		8 //(((((F_CPU * 10) / (16L * BAUD)) + 5) / 10) - 1)
#define I2C_FREQ		100000L
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
#define ITG3200ADDR		0x69 // was 0x68 for Kyle's
#define ACK				1
#define NACK			0
#define EEPROM_START	10

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

#define MPU_VLOGIC	REGISTER_BIT(PORTB,0)
#define CS_ADXL		REGISTER_BIT(PORTB,1)
#define CS_FLASH	REGISTER_BIT(PORTB,2)
#define LED			REGISTER_BIT(PORTD,5)
#define MPUINT		(PIND &(1<<2))
#define ADXLINT1	(PIND &(1<<3))
#define ADXLINT2	(PIND &(1<<4))

// Included Headers
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/eeprom.h>

#include "i2c.c"
#include "spi.c"
#include "adxl345.c"
#include "dataflash.c"

#if defined(MPU9150)
	#include "mpu9150.c"
#elif defined(ITG3200)
	#include "itg3200.c"
#else
	uint16_t getGyroSample(uint16_t, uint8_t *);
	uint16_t getGyroSample(uint16_t index, uint8_t *array){
		return index++;
	}
#endif

// Function Prototypes
void setup(void);
void loop(void);
void testSampleSequence(void);
void systemSleep(uint8_t);
void atMegaInit(void);

void dumpSamples(uint8_t);
uint16_t getBatt(void);
char deviceIdCheck(void);
void printHelpInfo(void);
void printTriggerSources(void);

static int putUARTchar(char c, FILE *stream);
uint8_t getUARTchar(void);
uint16_t readADC(uint8_t);
void flashLED(uint8_t, uint8_t, uint8_t);

// Global Variables
static FILE uart_io = FDEV_SETUP_STREAM(putUARTchar, NULL, _FDEV_SETUP_WRITE);
static uint8_t dataBufferA[BUFFER_SIZE]; //volatile
static uint8_t dataBufferG[54];
static uint8_t testNumber = 0;
static struct{
	uint8_t onOneTap:1;
	uint8_t onTwoTap:1;
	//uint8_t onActivity:1;
	uint8_t onFreeFall:1;
} configFlags;
static uint8_t bootloaderFlag = 0;


// Interrupt Vectors
ISR(WDT_vect){
	flashLED(2,10,40);
}

// ISR(USART_RX_vect){
	// printf("%u",UDR0);
// }

ISR(USART_RX_vect){
	uint8_t command = UDR0;
	
	if(command == ' ' && bootloaderFlag==1){
		WDTCSR |= _BV(WDCE) | _BV(WDE);
		WDTCSR = _BV(WDE);
		while(1);
	}
	else bootloaderFlag = 0;
	
	switch(command){
		case '0':
			bootloaderFlag = 1;
			break;
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
			printHelpInfo();
			printTriggerSources();
			break;
/*
#if defined(TRIGGER_SELECT)
		case '1':
			configFlags.onOneTap ^= 1;
			printf("Trigger on 1 Tap: ");
			if(configFlags.onOneTap) printf("Enabled\n");
			else printf("Disabled\n");
			break;
		case '2':
			configFlags.onTwoTap ^= 1;
			printf("Trigger on 2 Taps: ");
			if(configFlags.onTwoTap) printf("Enabled\n");
			else printf("Disabled\n");
			break;
		// case '3':
			// configFlags.onActivity ^= 1;
			// printf("Trigger on Activity: ");
			// if(configFlags.onActivity) printf("Enabled\n");
			// else printf("Disabled\n");
			// break;
		case '3':
			configFlags.onFreeFall ^= 1;
			printf("Trigger on Freefall: ");
			if(configFlags.onFreeFall) printf("Enabled\n");
			else printf("Disabled\n");
			break;
		case '0':
			eeprom_update_byte((uint8_t*)EEPROM_START,(*(uint8_t*) &configFlags)); // *((uint8_t*) &configFlags)
			printTriggerSources();
			break;
#endif
*/
	}	
}

// Main Program
int main(void){
	setup();

	while(1){		
		loop();
		wdt_reset();
	}
	return(0);
}

void setup(void){
	atMegaInit();
	
	// Tasks and Routines
	printf("\n\nBYU Splash Logger dev Aug2012\n\n");
	
	flashLED(20,10,40);
	MPU_VLOGIC = HIGH;

	printf("Device ID Check: ");
	if(deviceIdCheck()){
		printf("OK\n");
		
		ADXL345Init();
		ADXL345Mode(ACTIVE);

		#if defined(MPU9150)
			MPU9150Mode(ACTIVE);
		#endif
		
		for(uint16_t i=0; i<(BUFFER_SIZE-1); i++){
			dataBufferA[i] = 0x2A; //0x20 + (i & 0x5F); 
		}
		dataBufferA[BUFFER_SIZE-1] = '\n'; //0x2A;
	} else{
		printf("FAILED!\n");
		flashLED(10,30,30);
		//return;
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
	
	*((uint8_t*) &configFlags) = eeprom_read_byte((const uint8_t*) EEPROM_START);
	
	// Console Usage Hints
	printHelpInfo();
	printTriggerSources();
}

void loop(void){
	static int count = 0;
	static char state = 0;
	uint8_t status;
	
	// Test for how SPI bus effects AD0 line on MPU-9150
	// transferSPI('W');
	// DDRB |= (1<<4);
	// PORTB ^= (1<<4);
	// // PINB = (1<<4);
	
	// I2C Register Sniffer
	// for(uint8_t i=0; i<255; i++){
		// // transferSPI(i);
		// //uint8_t test = startI2C(i, READ);
		// uint8_t test = startI2C(0x69, WRITE);
			// writeI2C(i);
		// stopI2C();
		// startI2C(0x69, READ);
			// uint8_t value = readI2C(NACK);
		// stopI2C();
		// if(test != 0x38){
			// transferSPI(i);
			// transferSPI(value);
		// }
	// }
	/*
	if(ADXLINT1){
		LED = HIGH;
		
		CS_ADXL = LOW;
			transferSPI((READ<<7) | (SINGLE<<6) | 0x30);
			status = transferSPI(0x00);
		CS_ADXL = HIGH;
		
		LED = LOW;
		
#if defined(TRIGGER_SELECT)
		//printf("%u\t",++count);
		if((status&(1<<6)) && (configFlags.onOneTap)){
			//printf("[1 Tap]");
			// if(state == 0) testSampleSequence();
			// state = 1;
			testSampleSequence();
		}
		if((status&(1<<5)) && (configFlags.onTwoTap)){
			//printf("[2 Tap]");
			// if(state == 0) testSampleSequence();
			// state = 1;
			testSampleSequence();
		}
		//if((status&(1<<4))) //printf("[Activity]"); // && (configFlags.onActivity)) 
		
		if(status&(1<<3)){
			//printf("[Inactive]");
			state = 0;
		}
		if((status&(1<<2)) && (configFlags.onFreeFall)){
			//printf("[Freefall]");
			// if(state == 0) testSampleSequence();
			// state = 1;
			testSampleSequence();
		}
		//printf("\n");
#else
		if(status&(1<<2)){
			testSampleSequence();
		}
#endif
		
		_delay_ms(50);
		CS_ADXL = LOW;
			transferSPI((READ<<7) | (SINGLE<<6) | 0x30);
			status = transferSPI(0x00);
		CS_ADXL = HIGH;
		_delay_ms(50);
	}
	LED = LOW;
	*/
	
	//systemSleep(7);
	count++;
	//printf("%u\n",count);
	
	
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
		getBatt(); // Pre-heat the ADC for correct battery measurement in test cycle.
	}

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
					gyroIndex = getGyroSample(gyroIndex, dataBufferG);
				}
			}
			LED = LOW;
			
			bufferIndex = getAccelFIFO(bufferIndex, dataBufferA); //6 bytes * ADXL_FIFO
			//if(bufferIndex > 504){
			//	printf("OVERFLOW!");
			//	break;
			//}
			
			while(ADXLINT2); // Clear GyroReadFlag
			
			gyroTimeStamp = TCNT1;
			gyroCount=0;
			gyroIndex = getGyroSample(gyroIndex, dataBufferG);
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
		if(i==(8*TEST_BLOCKS-1)){
			uint16_t battVolt = getBatt();
			dataBufferA[510] = battVolt>>8;
			dataBufferA[511] = battVolt;
		}
		LED = LOW;
		dataFlashWritePage(page+i, i&0x01, dataBufferA);
	}
	while(dataFlashStatus());
	
	testNumber = (testNumber >= TEST_MAX)? testNumber : testNumber + 1;
	dataFlashCleanTestBlocks(testNumber);
	dataFlashWritePointer(testNumber);
	while(dataFlashStatus());
}

void systemSleep(uint8_t interval){
	//	Interval	0	1	2	3	4	5	6	7	8	9
	//	Time in ms	16	32	64	128	256	512	1k	2k	4k	8k
	
	//ADXL345Mode(SLEEP);
	//dataFlashMode(SLEEP);
	
	TWCR = 0;
	TWSR = 0;
	SPCR = 0;
	ADMUX = 0;
	ADCSRA = 0;
	DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);
	DIDR1 = (1<<AIN1D)|(1<<AIN0D);
	UCSR0B =0;
	TCCR1B = 0;
	PORTB = PORTC = PORTD = 0;
	DDRB = DDRC = DDRD = 0;
	
	//MPU_VLOGIC = LOW;
	power_all_disable();
	
	wdt_reset();
	MCUSR = 0;
	WDTCSR |= (1<<WDCE)|(1<<WDE); //(1<<WDE)|
	WDTCSR = (1<<WDIE)|((interval&0x08)<<WDP3)|((interval&0x04)<<WDP2)|
		((interval&0x02)<<WDP1)|((interval&0x01)<<WDP0); // Pretty sure this is just rebooting the device forever?
	//wdt_enable(9);
	
	cli();
	PCICR = (1<<PCIE2);
	PCMSK2 = (1<<PCINT16);
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); //  SLEEP_MODE_IDLE
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	sleep_disable();
	
	wdt_reset();
	atMegaInit();

}

void atMegaInit(void){
	// System
	MCUCR |= (1<<PUD);		// Pull-up Disable
	MCUSR = 0;
	//uint8_t startupStatus = MCUSR;
	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = _BV(WDE) | _BV(WDP3) | _BV(WDP0);

	// Timers
	TCCR1A = 0;
	TCCR1B = (1<<CS12); //(1<<CS11)|(1<<CS10); //
	
	// IO Ports
	// 0: Input (Hi-Z) 1: Output
	//        76543210		7		6		5		4			3			2			1		0
	DDRB |= 0b00101111; //	XTAL2	XTAL1	SCK		MISO		MOSI		CS_FLASH	CS_ADXL	MPU_VLOGIC
    DDRC |= 0b00000000; //	--		Reset	SCL		SDA
    DDRD |= 0b00100010; //					LED		ADXLINT2	ADXLINT1	MPUINT		TXD		RXD
	PORTB |=0b00000111; // MPU_VLOGIC is HIGH
	
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

	PRR = 0;
	
	PCICR = 0; //(1<<PCIE2);
	PCMSK2 = 0; //(1<<PCINT16);
	
	sei();
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
		if(i==(8*TEST_BLOCKS-1)){
			uint16_t battVolt = (dataBufferA[510]<<8)+dataBufferA[511];
			printf("_B\t%u\n",battVolt);
		}
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
	
	//ITG3200Mode(ACTIVE); // 3 April 2012: Had to place this here to fix 0x4E Init Problem
	// startI2C(ITG3200ADDR,WRITE);
		// writeI2C(0x75); // Hope the 9150 matches the 6050/6000
	// stopI2C();
	// startI2C(ITG3200ADDR,READ);
		// uint8_t gyro = readI2C(NACK);
	// stopI2C();
	
	//printf("Accel: %X\tFlash: %X\tGyro: %X\n", accel,flash,gyro);
	
	
	if((accel ^ 0b11100101) == 0 && (flash ^ 0x26) == 0 ) return 1; //&& ((gyro & 0b01111110) ^ 0x68) == 0
	return 0;
}

void printHelpInfo(void){
	printf("\nConsole Useage:\n"
		"+/-\tInc/Dec#\n"
		"D\tDump\n"
		"R\tReset Test#=1\n"
		"B\tBattery mV\n"
		"N\tCurrent Test#\n"
		"T\tForce Trigger\n"
		"1 to 3\tToggle Trigger Sources\n"
		"0\tWrite Toggles to EEPROM and Review\n"
		"?\tConsole Useage\n\n");
	printf("Impending Test# (Stored in Flash): %u\n\n", (dataFlashReadByte(0,0) +1));
	printf("Important Info: _T units are 62.5 kHz clock ticks, roll over on 2^16\n"
		"Test Duration: 3.00 Sec; 9600 Sa from ADXL345 @ 3200 Sa/sec +/-0.5%%\n\n");

}

void printTriggerSources(void){
	printf("\nTest Triggers: \n");
#if defined(TRIGGER_SELECT)
	printf("OneTap: %u\n",configFlags.onOneTap);
	printf("TwoTap: %u\n",configFlags.onTwoTap);
	//printf("Activity: %u\n",configFlags.onActivity);
	printf("Freefall: %u\n\n",configFlags.onFreeFall);
#endif
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

uint16_t readADC(uint8_t adcChannel){
	ADMUX 	= (1<<REFS0) | adcChannel;
	ADCSRA 	|= (1<<ADSC);
	while (ADCSRA & (1 << ADSC));
	return (ADCL + ((uint16_t) ADCH << 8));
}

void flashLED(uint8_t count, uint8_t high, uint8_t low){
	for(;count>0; count--){
		LED = HIGH;
		_delay_ms(high);
		LED = LOW;
		_delay_ms(low);
	}
}

