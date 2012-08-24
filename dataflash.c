#ifndef DATAFLASH_H
#define DATAFLASH_H


void dataFlashReadBuffer(uint8_t, uint8_t *);
void dataFlashReadPage(uint16_t, uint8_t *);
uint8_t dataFlashReadByte(uint16_t, uint16_t);
void dataFlashWritePage(uint16_t, uint8_t, uint8_t*);
void dataFlashWritePointer(uint8_t);
void dataFlashCleanTestBlocks(uint8_t);
void dataFlashEraseBlock(uint16_t);
uint8_t dataFlashStatus(void);
void dataFlashMode(int8_t);

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


#endif