#ifndef MPU9150_H
#define MPU9150_H

#define MPU9150ADDR		0x69


uint16_t getGyroSample(uint16_t, uint8_t *);
void MPU9150Mode(int8_t);

uint16_t getGyroSample(uint16_t index, uint8_t *array){
	startI2C(MPU9150ADDR, WRITE);
		writeI2C(0x43);
	stopI2C();
	startI2C(MPU9150ADDR, READ);
		for(uint8_t k=0; k<6; k++){
			if(index > 54-1) break;
			uint8_t ackType = (k < (6-1))? ACK : NACK ;
			array[index++] = readI2C(ackType);
		}
	stopI2C();
	return index;
}

void MPU9150Mode(int8_t mode){
	if(mode == SLEEP){ // || mode == ARMED){
		startI2C(MPU9150ADDR, WRITE);
			writeI2C(0x6B); // PWR_MGMT_1
			writeI2C(1<<6); // SLEEP bit set
		stopI2C();
	}
	else{ //if(mode == ACTIVE){
		startI2C(MPU9150ADDR, WRITE);
			writeI2C(0x6B); // PWR_MGMT_1
			writeI2C(0x03); // Z-Axis Gyro Reference
		stopI2C();
		_delay_ms(1);
		startI2C(MPU9150ADDR, WRITE);
			writeI2C(0x19); // SMPRT_DIV
			writeI2C(0x00); // SMPRT_DIV
			writeI2C(0x00); // CONFIG
			writeI2C(3<<3); // GYRO_CONFIG
			writeI2C((3<<3)|(0)); // ACCEL_CONFIG
		stopI2C();
	}
}


#endif
