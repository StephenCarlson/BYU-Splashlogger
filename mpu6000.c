#ifndef MPU6000_H
#define MPU6000_H

// Needs work
void ITG3200Mode(int8_t);

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


#endif
