#ifndef ADXL345_H
#define ADXL345_H

#define ONETAP		6
#define TWOTAP		5
#define FREEFALL	4
#define ACTIVITY	3
#define INACTIVE	2

uint16_t getAccelFIFO(uint16_t, uint8_t *);
void ADXL345Init(void);
void ADXL345Mode(int8_t);
uint8_t ADXL345Status(void);

volatile uint8_t adxlInitFlag = 0;

uint16_t getAccelFIFO(uint16_t index, uint8_t *array){	
	for(uint8_t j=0; j<ADXL_FIFO; j++){
		CS_ADXL = LOW;
			transferSPI((READ<<7) | (MULTI<<6) | 0x32);
			for(uint8_t i=0; i<6; i++){
				if(index > BUFFER_SIZE-1) break; // printf("break");
				array[index++] = transferSPI(0x00);
			}
			//transferSPI(0x00);
			//transferSPI(0x00);
		CS_ADXL = HIGH;
	}
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
		0b00101011	// 	31	DATA_FORMAT  FULL_RES bit set
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
	
	if(mode == SLEEP){
		CS_ADXL = LOW;
			transferSPI((WRITE<<7) | (MULTI<<6) | 0x2D); // POWER_CTL
			transferSPI(0b00001111); // 0[7:6], [Link][AutoSleep][Measure][Sleep] WakeRate[1:0]
			transferSPI(0b00010000); // [DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
		CS_ADXL = HIGH;
	}
	if((mode == ACTIVE)||(mode == SAMPLING)){
		CS_ADXL = LOW;
			transferSPI((WRITE<<7) | (MULTI<<6) | 0x2D); // POWER_CTL
			transferSPI(0b00101000); // 0[7:6], [Link][AutoSleep][Measure][Sleep] WakeRate[1:0]
			transferSPI(0b01101110); // [DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
		CS_ADXL = HIGH;
	}
}

uint8_t ADXL345Status(void){
	CS_ADXL = LOW;
		transferSPI((READ<<7) | (SINGLE<<6) | 0x2B);
		uint8_t status = transferSPI(0x00);
	CS_ADXL = HIGH;
	return status;
}

#endif