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
//    11		   9		5		PD5		DIO				Blue LED			LED			
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

// Behavioral Switches (Evoked in Makefile, shown here for reference)
//#define ITG3200
//#define MPU6000
//#define MPU9150
//#define RN42_BLUETOOTH

// Debug Switches
//#define DEBUG_MAIN_LOOP

// Behavioral Parameters
#define TIMEOUT_SERIAL	10 // Multiply by Watchdog Period
#define TIMEOUT_STDBY	5
#define WAKE_ACT_CNT	2

// System Parameters
//#define F_CPU			16000000UL
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
#define EEPROM_START	10 // The offest in the ATmega's EEPROM for keeping settings.

// System Constants
#define DOWN 			0
#define SLEEP			1
#define STANDBY 		1 //SLEEP
#define ACTIVE 			2
#define FORCE_TRIGGER	3
#define INT_SRC_CLEAR	0
#define INT_SRC_INTx	1
#define INT_SRC_WDT		2
#define INT_SRC_UART	3
#define HIGH			1
#define LOW				0
#define READ			1 // ADXL SPI Flags
#define WRITE			0 // Also I2C Direction Flags
#define SINGLE			0
#define MULTI			1


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

#if defined(MPU9150)
	#define MPU_VLOGIC	REGISTER_BIT(PORTB,0)
#elif defined(MPU6000)
	#define CS_MPU	REGISTER_BIT(PORTB,0)
#endif
#define CS_ADXL		REGISTER_BIT(PORTB,1)
#define CS_FLASH	REGISTER_BIT(PORTB,2)
#define LED			REGISTER_BIT(PORTD,5)
#define MPUINT		(PIND &(1<<2))
#define ADXLINT1	(!(PIND &(1<<3)))
#define ADXLINT2	(!(PIND &(1<<4)))

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

#if defined(ITG3200)
	#include "itg3200.c"
#elif defined(MPU6000)
	#include "mpu6000.c"
#elif defined(MPU9150)
	#include "mpu9150.c"
#else
	uint16_t getGyroSample(uint16_t, uint8_t *);
	uint16_t getGyroSample(uint16_t index, uint8_t *array){
		return index++;
	}
#endif

// Function Prototypes
void setup(void);
void loop(void);
uint8_t triggerStatus(void);
void testSampleSequence(void);
uint8_t systemSleep(uint8_t);
uint8_t atMegaInit(void);
void bluetoothMode(uint8_t);
void gyroMode(uint8_t);

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
static uint8_t testNumber; // = 0; //http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_varinit
static struct{
	uint8_t overWriteEn:1;
	uint8_t sleepInterval:3;
	uint8_t wdtSlpEn:1;
	uint8_t onOneTap:1;
	uint8_t onTwoTap:1;
	uint8_t onFreeFall:1;
} configFlags;
static volatile struct{
	uint8_t systemState:2;
	uint8_t bootloaderFlag:1;
	uint8_t uartSuppress:1;
	uint8_t ledBlink:1;
	uint8_t intSource:2;
} stateFlags;



// Interrupt Vectors
ISR(WDT_vect){
	stateFlags.intSource = INT_SRC_WDT;
}

ISR(PCINT2_vect){
	stateFlags.intSource = INT_SRC_UART;
	PCICR = 0;
	PCMSK2 = 0;
}

ISR(INT0_vect){
	stateFlags.intSource = INT_SRC_INTx;
	EIMSK = 0;
}

ISR(INT1_vect){
	stateFlags.intSource = INT_SRC_INTx;
	EIMSK = 0;
}

ISR(USART_RX_vect){
	stateFlags.intSource = INT_SRC_UART;
	
	if(stateFlags.uartSuppress) return;
	
	uint8_t command = UDR0;
	if(command == ' ' && stateFlags.bootloaderFlag==1){
		WDTCSR |= _BV(WDCE) | _BV(WDE);
		WDTCSR = _BV(WDE);
		while(1);
	}
	else stateFlags.bootloaderFlag = 0;
	
	switch(command){
		case '0':
			stateFlags.bootloaderFlag = 1;
			break;
		case 'D':
			dumpSamples(testNumber);
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
			stateFlags.systemState = FORCE_TRIGGER;
			break;
		case 'N':
			printf("Test#: %u\n", (testNumber+1));
			break;
		
		case 'Z':
			stateFlags.systemState = DOWN;
			break;
		case '?':
			printHelpInfo();
			printTriggerSources();
			break;
		case '7':
			configFlags.wdtSlpEn ^= 1;
			printf("Sleep LED Heartbeat: ");
			if(configFlags.wdtSlpEn) printf("Enabled\n");
			else printf("Disabled\n");
			break;
		case '8':
			configFlags.sleepInterval = (configFlags.sleepInterval >=0b111)? 0: 
										configFlags.sleepInterval+1;
			printf("Heartbeat Period: %u\n", configFlags.sleepInterval);
			break;
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
		case '3':
			configFlags.onFreeFall ^= 1;
			printf("Trigger on Freefall: ");
			if(configFlags.onFreeFall) printf("Enabled\n");
			else printf("Disabled\n");
			break;
		case '`':
			eeprom_update_byte((uint8_t*)EEPROM_START,(*(uint8_t*) &configFlags)); // *((uint8_t*) &configFlags)
			printTriggerSources();
			break;
	}	
}

// Main Program
int main(void){
	setup();

	while(1){		
		loop();
		//wdt_reset();
	}
}

void setup(void){
	uint8_t startStatus = atMegaInit();
	stateFlags.systemState = ACTIVE;
	
	bluetoothMode(ACTIVE);
	
	// Tasks and Routines
	printf("\n\nBYU Splash Logger 19Nov2012\n\n");
	printf("Reset Status: %X\n", startStatus);
	
	flashLED(10,10,40);
	
	#if defined(MPU9150)
		MPU_VLOGIC = HIGH;
	#elif defined(MPU6000)
		CS_MPU = HIGH;
	#endif
	
	
	ADXL345Init();
	ADXL345Mode(ACTIVE);
	gyroMode(ACTIVE);
	dataFlashMode(ACTIVE);
	
	printf("Device ID Check: ");
	if(deviceIdCheck()){
		printf("OK\n");
		
		ADXL345Init();
		ADXL345Mode(ACTIVE);
		gyroMode(ACTIVE);
		
		for(uint16_t i=0; i<(BUFFER_SIZE-1); i++){
			dataBufferA[i] = 0x2A; //0x20 + (i & 0x5F); 
		}
		dataBufferA[BUFFER_SIZE-1] = '\n'; //0x2A;
	} else{
		printf("FAILED!\n");
		//stateFlags.systemState = FAULT; // BAD! -1 unrepresentable
		//return;
	}	
	
	*((uint8_t*) &configFlags) = eeprom_read_byte((const uint8_t*) EEPROM_START);
	
	testNumber = dataFlashReadByte(0,0);
	printf("Impending Test #: %u\n", (testNumber+1));
	if(testNumber >= TEST_MAX){		
		printf("Flash Memory Full\n");
		if(configFlags.overWriteEn){
			testNumber = 0;
			printf("Resetting Test # to 1\n");
		} else{
			testNumber = TEST_MAX-1;
			printf("Overwriting Previous Test\n");
		}
		dataFlashWritePointer(testNumber);
		while(dataFlashStatus());
	}
	
	// Critical for Flash to write samples correctly and as fast as possible
	dataFlashCleanTestBlocks(testNumber);
	
	// Console Usage Hints
	printHelpInfo();
	printTriggerSources();
}

void loop(void){
// Notes on State Machine: 
// Term				Description
// wdtIntCntDwn		Used to keep processor awake during serial comm w/ User via terminal
// activityCount	For responding to the user picking up or rattling the device to wake it
// intSource		Wakeup Source: UART Serial, WDT 2/4/8sec count, or device interrupt.
// systemState		Defines the current system state: Down, Sleep/Standby, Active and Sampling
// goSleep			Commands sleep at the end of the main loop.
// ledBlink			You may set whether the LED blinks when in the main loop.
	static uint8_t activityCount = 0;
	static uint8_t wdtIntCntDwn = 0;
	
	//stateFlags.ledBlink = 1;
	//if(stateFlags.ledBlink) LED = HIGH;
	LED = HIGH;
	_delay_us(5);
	LED = LOW; // Protect the microLogger LED
	
	if(stateFlags.intSource==INT_SRC_CLEAR && stateFlags.systemState != STANDBY) _delay_ms(100); // 
	switch(stateFlags.intSource){
		// case INT_SRC_INTx:
			
			// break;
		case INT_SRC_WDT:
			wdtIntCntDwn = (wdtIntCntDwn>0)? wdtIntCntDwn-1: 0;
			WDTCSR |= _BV(WDIE);
			//wdt_reset();
			break;
		case INT_SRC_UART:
			wdtIntCntDwn = TIMEOUT_SERIAL;
			
			break;
	}
	#ifdef DEBUG_MAIN_LOOP
	printf("T: %u, S: %u, A: %u, St: %u\n",wdtIntCntDwn, stateFlags.intSource, activityCount, stateFlags.systemState);
	#endif
	stateFlags.intSource = INT_SRC_CLEAR;

	uint8_t adxlStatus = triggerStatus();
	if(stateFlags.systemState == DOWN){
		activityCount = (adxlStatus&(1<<ACTIVITY))?	activityCount+1 : activityCount;
		if(activityCount >= WAKE_ACT_CNT){
			// stateFlags.ledBlink = 1;
			activityCount = 0;
			wdtIntCntDwn = (wdtIntCntDwn+TIMEOUT_STDBY > 250)? 250 : wdtIntCntDwn+TIMEOUT_STDBY;
		}
	} else{
		uint8_t testTrigger = adxlStatus &(
			 ((configFlags.onOneTap)<<ONETAP)
			&((configFlags.onTwoTap)<<TWOTAP)
			&((configFlags.onFreeFall)<<FREEFALL));
		if(testTrigger || stateFlags.systemState == FORCE_TRIGGER){
			stateFlags.systemState = ACTIVE;
			dataFlashMode(ACTIVE);
			gyroMode(ACTIVE);
			ADXL345Mode(ACTIVE);
			testSampleSequence();
			
		}
	}
	
	// Moore and Mealy Transitions
	if(wdtIntCntDwn==0){
		// if(stateFlags.systemState != DOWN){
			
		// }
		stateFlags.systemState = DOWN;
	} else{
		if(stateFlags.systemState != STANDBY){
			ADXL345Mode(ACTIVE);
			dataFlashMode(ACTIVE);
		}
		stateFlags.systemState = STANDBY;
	}
	LED = LOW;
	
	//if((wdtIntCntDwn==0) && (stateFlags.systemState==DOWN)) stateFlags.goSleep = 1;
	//stateFlags.goSleep = ((wdtIntCntDwn==0) && (stateFlags.systemState==DOWN))? 1 : 0;
	if((stateFlags.systemState==DOWN) ){ //&& (wdtIntCntDwn==0)
		dataFlashMode(SLEEP);
		gyroMode(SLEEP);
		//bluetoothMode(SLEEP);
		ADXL345Init(); // 29 Oct Critical Note! Seems that the ADXL does I2C address matching when the CS
						// Pin is high. The datasheet has an OR-Gate solution, sort of annoying. Anyhow,
						// I suppose that the part needs to be the last thing spoken to before sleep, and that 
						// I should be re-initiated completely before sending to sleep.
						// Also, how this has never blown away Kyle's Logger the past year, I don't know.
		ADXL345Mode(SLEEP);
		uint8_t sleepInterval = configFlags.sleepInterval;; //(stateFlags.systemState = DOWN)? 8: 5; //configFlags.sleepInterval;
		systemSleep(sleepInterval+2);
	}
}

uint8_t triggerStatus(void){
	CS_ADXL = LOW;
		transferSPI((READ<<7) | (SINGLE<<6) | 0x30);
		uint8_t status = transferSPI(0x00);
	CS_ADXL = HIGH;

	return(status & 0b01111100);
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
		//wdt_reset();
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
			// if(bufferIndex > 504){
				// printf("OVERFLOW!");
				// break;
			// }
			
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
	printf("...Test# %u Complete\n", (testNumber));
}

uint8_t systemSleep(uint8_t interval){
	//	Interval	0	1	2	3	4	5	6	7	8	9
	//	Time in ms	16	32	64	128	256	512	1k	2k	4k	8k
	
	//LED = LOW;
	
	cli();
	
	TWCR = 0;
	TWSR = 0;
	SPCR = 0;
	ADMUX = 0;
	ADCSRA = 0;
	DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);
	DIDR1 = (1<<AIN1D)|(1<<AIN0D);
	UCSR0B =0;
	TCCR1B = 0;
	PORTC = PORTD = 0; // PORTB = 
	DDRB = DDRC = DDRD = 0;
	PORTB |=0b00111111;
	
	//MPU_VLOGIC = LOW;
	power_all_disable();
	
	wdt_reset();
	uint8_t value = (uint8_t)( ((configFlags.wdtSlpEn)<<WDIE) | (interval & 0x08? (1<<WDP3): 0x00) | (interval & 0x07) );
	MCUSR = 0;
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	WDTCSR = value;
	
	PCMSK2 = (1<<PCINT16);
	PCICR = (1<<PCIE2);

	EICRA = 0;
	EIMSK = (1<<INT1); //|(1<<INT0);
	
	
	// if(stateFlags.systemState == DOWN)			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	// else if(stateFlags.systemState == SLEEP) 	set_sleep_mode(SLEEP_MODE_STANDBY);
	// else										set_sleep_mode(SLEEP_MODE_IDLE);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	
	sleep_disable();
	wdt_reset();
	uint8_t systemReturnState = atMegaInit();
	
	//LED = HIGH;
	//ADXL345Init();
	//ADXL345Mode(ACTIVE);
	//dataFlashMode(ACTIVE);
	//gyroMode(ACTIVE);
	//bluetoothMode(ACTIVE);
	
	return systemReturnState;
}

uint8_t atMegaInit(void){
	uint8_t startupStatus = MCUSR; //wdt_init(); //
	MCUSR = 0;
	WDTCSR |= _BV(WDCE) | _BV(WDE); // Three Options Below:
	WDTCSR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1); // Hardwire the WDT for 1 Sec
	//WDTCSR = _BV(WDE) | _BV(WDP3) | _BV(WDP0);
	//WDTCSR = 0;
	wdt_reset();
	
	// System
	//MCUCR |= (1<<PUD);		// Pull-up Disable
	MCUCR = 0;
	PRR = 0;

	// Timers
	TCCR1A = 0;
	TCCR1B = (1<<CS12); //(1<<CS11)|(1<<CS10); //
	
	// IO Ports
	// 0: Input (Hi-Z) 1: Output
	//        76543210		7		6		5		4			3			2			1		0
	DDRB |= 0b00101111; //	XTAL2	XTAL1	SCK		MISO		MOSI		CS_FLASH	CS_ADXL	MPU_VLOGIC
    DDRC |= 0b00000000; //	--		Reset	SCL		SDA
    DDRD |= 0b00100010; //					LED		ADXLINT2	ADXLINT1	MPUINT		TXD		RXD
	//PORTB |=0b00000111; // MPU_VLOGIC is HIGH
	PORTB |=0b00111111;
	
	// Serial Port
	UBRR0H = UART_UBRR >> 8;
    UBRR0L = UART_UBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    stdout = &uart_io; //= stdin 
	
	//SPI
	SPCR	= (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR0); // 1MHz for MPU-6000
	
	//I2C
	TWCR = (1<<TWEN) | (1<<TWEA);
	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
	TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
	
	// ADC
	ADMUX 	= (1<<REFS0);	// AVcc Connected
	ADCSRA 	= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
	DIDR0 	= (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);

	//PCICR = 0; //(1<<PCIE2);
	//PCMSK2 = 0; //(1<<PCINT16);
	
	EICRA = 0;
	EIMSK = 0; //(1<<INT1)|(1<<INT0);
	
	sei();
	
	return startupStatus;
}

void bluetoothMode(uint8_t mode){
	#if defined(RN42_BLUETOOTH)
	//cli();
	stateFlags.uartSuppress = 1;
	
	if(mode == SLEEP){
		//putUARTchar(
		printf("$$$");
		_delay_ms(5);
		// printf("+\n");
		// _delay_ms(5);
		printf("ST,255\n");
		_delay_ms(5);
		//printf("F,1\n");
		printf("---\n");
		_delay_ms(5);
		
		/*
		"S|, " //Sleep Mode
		"SJ,0020" // Short
		"SI,0020" // Short  
		"SW,1900" // 4 Sec Window
		"SW,8000" // Sleep Bit
		*/
	
	} else{
	
	
	}
	
	stateFlags.uartSuppress = 0;
	//sei();
	#endif
}

void gyroMode(uint8_t mode){
	#if defined(ITG3200)
		ITG3200Mode(mode);
	#elif defined(MPU6000)
		MPU6000Mode(mode);
	#elif defined(MPU9150)
		MPU9150Mode(mode);
	#endif
}

void dumpSamples(uint8_t test){
	if(test>TEST_MAX) return;
	uint16_t page = ((TEST_BLOCKS*test)+TEST_OFFSET)<<3;
	for(uint8_t i=0; i<(8*TEST_BLOCKS); i++){
		//wdt_reset();
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
	//voltSample = 1100*1023/voltSample;
	voltSample = 5353 - ((voltSample<<2)+(voltSample<<1)+(voltSample>>2));
	return voltSample;
}

char deviceIdCheck(void){
	CS_ADXL = LOW;
		transferSPI((READ<<7) | (SINGLE<<6) | 0x00);
		uint8_t accel = transferSPI(0x00);
	CS_ADXL = HIGH;
	accel ^= 0b11100101;
	
	CS_FLASH = LOW;
		transferSPI(0xAB);
	CS_FLASH = HIGH;
	_delay_ms(1);
	CS_FLASH = LOW;
		transferSPI(0x9F);
		transferSPI(0x00);
		uint8_t flash = transferSPI(0x00);
	CS_FLASH = HIGH;
	flash ^= 0x26;
	
	#if defined(ITG3200)
		ITG3200Mode(ACTIVE); // 3 April 2012: Had to place this here to fix 0x4E Init Problem
		startI2C(ITG3200ADDR,WRITE);
			writeI2C(0x00);
		stopI2C();
		startI2C(ITG3200ADDR,READ);
			uint8_t gyro = readI2C(NACK);
		stopI2C();
		gyro = (gyro & 0b01111110) ^ 0x68;
	#elif defined(MPU6000)
		CS_MPU = LOW;
			transferSPI((READ<<7)|0x75); // Hope the 9150 matches the 6050/6000
			uint8_t gyro = transferSPI(0x00);
		CS_MPU = HIGH;
		gyro = (gyro & 0b01111110) ^ 0x68;
	#elif defined(ITG3200)
		MPU9150Mode(ACTIVE); // 3 April 2012: Had to place this here to fix 0x4E Init Problem
		startI2C(MPU9150ADDR,WRITE);
			writeI2C(0x75);
		stopI2C();
		startI2C(MPU9150ADDR,READ);
			uint8_t gyro = readI2C(NACK);
		stopI2C();
		gyro = (gyro & 0b01111110) ^ 0x68;
	#else
		uint8_t gyro = 0;
	#endif
	
	printf("Flash: %X\tAccel: %X\tGyro: %X\n", flash,accel,gyro);
	
	if(flash == 0 && accel == 0 && gyro == 0) return 1;
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
	printf("Sleep Heartbeat: ");
			if(configFlags.wdtSlpEn) printf("Enabled\n");
			else printf("Disabled\n");
	printf("Heartbeat Period: %u\n", configFlags.sleepInterval);
}

void printTriggerSources(void){
	printf("\nTest Triggers: \n");
	printf("OneTap: %u\n",configFlags.onOneTap);
	printf("TwoTap: %u\n",configFlags.onTwoTap);
	//printf("Activity: %u\n",configFlags.onActivity);
	printf("Freefall: %u\n\n",configFlags.onFreeFall);
}

static int putUARTchar(char c, FILE *stream){
    if (c == '\n') putUARTchar('\r', stream);
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
	//transferSPI(c);
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


// COM20 for AD03

// ACCIPITER 001F81000830

/*

$$$ (no \r\n needed)
--- \r\n

*** SET COMMANDS ***
SA,<1,0>   - Authentication
SB,<num>   - Send Break
SC,<hex>   - Service Class
SD,<hex>   - Device Class
SE,<1,0>   - Encryption
SF,1       - Factory Defaults
SI,<hex>   - Inquiry Scan Window
SJ,<hex>   - Page Scan Window
SL,<E,O,N> - Parity
SM,<0-5>   - Mode (0=slav,1=mstr,2=trig,3=auto,4=DTR,5=Any)
SN,<name>  - Name
SO,<text>  - conn/discon Status
SP,<text>  - Pin Code
SR,<adr>   - Remote Address
				,Z Erases; ,I uses most recent Inquery
SS,<text>  - Service Name
ST,<num>   - Config Timer
SU,<rate>  - Baudrate
SW,<hex>   - Sniff Rate
SX,<1,0>   - Bonding
SY,<hex>   - TX power
SZ,<num>   - Raw Baudrate
S7,<0-1>   - 7bit data
S~,<0-3>   - Profile (0=SPP,1=DCE,2=DTE,3=MDM,4=D&S
S?,<0-1>   - role switch
S$,<char>  - CMD mode char
S@,<hex>   - io port dir
S&,<hex>   - io port val
S%,<hex>   - io boot dir
S^,<hex>   - io boot val
S*,<hex>   - pio(8-11) set
S|,<hex>   - low power timers
*** DISPLAY      ***
D     - Basic Settings
E     - Extended Settings
G<X>  - Stored setting
GR \r\n NOT SET or actual address
G| \r\n 0000
GW 0
GJ 0100
GI 0100




GB    - BT Address
		\r\n needed, 00066646AD03
GK    - Connect Status 
		\r\n needed, 0 or 1 returned
G&    - I/O Ports
V     - Firmare version
*** OTHER        ***
C,<adr>    - Connect
F,1        - Fast Mode
I,<time>,<cod> - Device Scan Inquiry
K,         - Kill (disconnect)
L,         - toggle local echo
P,<text>   - Pass Thru
Q          - Quiet (no discovery)
R,1        - Reboot
T,<0,1>    - send data in CMD mode
U,<rate>,<E,O,N> - Temp Uart Change
Z          - low power sleep
&          - Read switches

W			Wake the device, opposite of Q

*/



// D4206DEA48DD,HTC Inspire 4G,5A020C
/*
***Settings***
BTA=00066646AD03
BTName=RN42-AD03
Baudrt(SW4)=115K
Parity=None
Mode  =Slav
Authen=0
Encryp=0
PinCod=1234
Bonded=0
Rem=NONE SET
*/



















// 29 Oct Notes:
// The UART is busted now: Test and Sleep and likely other functions are broken
// To Do:
// Make a macro that allows 4MHz to the ADXL, 1MHz to MPU



	// // Test for MPU working
	// CS_MPU = LOW;
		// transferSPI((READ<<7)|0x75); // Hope the 9150 matches the 6050/6000
		// uint8_t gyro = transferSPI(0x00);
	// CS_MPU = HIGH;
	// transferSPI(gyro);
	// ledBlink = 0;
	// if(gyro == 0x68) ledBlink = 1;
	// _delay_ms(200);
	
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