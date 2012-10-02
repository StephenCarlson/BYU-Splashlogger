#ifndef MPU6000_H
#define MPU6000_H

#define MPU6000ADDR		0x69


uint16_t getGyroSample(uint16_t, uint8_t *);
void MPU6000Mode(int8_t);

uint16_t getGyroSample(uint16_t index, uint8_t *array){
	CS_MPU = LOW;
		writeI2C((READ<<7)|0x43);
		for(uint8_t k=0; k<6; k++){
			if(index > 54-1) break;
			array[index++] = transferSPI(0x00);
		}
	CS_MPU = HIGH;
	return index;
}

void MPU6000Mode(int8_t mode){
	if(mode == SLEEP){
		CS_MPU = LOW;
			transferSPI((WRITE<<7)|0x3E); // Power Management
			transferSPI(0x40); // SLEEP bit set, pg. 27
		CS_MPU = HIGH;
	}
	else{ //if(mode == ACTIVE){
		CS_MPU = LOW;
			transferSPI((WRITE<<7)|0x3E); // Power Management
			transferSPI(0x01); // SLEEP bit cleared, pg. 27
		CS_MPU = HIGH;
		_delay_ms(1);
		CS_MPU = LOW;
			transferSPI((WRITE<<7)|0x15); // Sample Rate Divider
			transferSPI(0x00); // SMPLRT_DIV set to 0x00, pg. 23
			transferSPI(0x18); // DLPF: FS_SEL set to 0x03, pg. 24
			
			// Interrupt on New Data
			//transferSPI(0b00110001);
		CS_MPU = HIGH;
	}
}


#endif
