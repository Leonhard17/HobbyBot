/*
 * PCA9685.c
 *
 *  Created on: Sep 30, 2024
 *      Author: leonh
 */

#include "PCA9685.h"
#include "math.h"


// Function to write one bite to the PCA9685
PCA9685_STATUS PCA9685_Write8(PCA9685_HandleTypeDef* hpca9685, uint8_t addres, uint8_t* data){
	if(HAL_I2C_Mem_Write(hpca9685->i2c_handle, hpca9685->device_address, addres, 1, data, 1, PCA9685_I2C_TIMEOUT) == HAL_OK){
		return PCA9685_OK;
	}else{
		return PCA9685_ERROR;
	}
}

// Function to read one byte from the PCA9685
PCA9685_STATUS PCA9685_Read8(PCA9685_HandleTypeDef* hpca9685, uint8_t addres, uint8_t* dest){
	if(HAL_I2C_Mem_Read(hpca9685->i2c_handle, hpca9685->device_address, addres, 1, dest, 1, PCA9685_I2C_TIMEOUT) == HAL_OK){
		return PCA9685_OK;
	}else{
		return PCA9685_ERROR;
	}
}

PCA9685_STATUS PCA9685_SetBit(PCA9685_HandleTypeDef* hpca9685, uint8_t addres, uint8_t Bit, uint8_t Value){
	// create variable to store the new value
	uint8_t tempByte;
	// check if Value is a binary, else set to 0
	Value = (Value != 0) ? 1 : 0;

	if(HAL_I2C_Mem_Read(hpca9685->i2c_handle, hpca9685->device_address, addres, 1, &tempByte, 1, PCA9685_I2C_TIMEOUT) != HAL_OK){
		// error while receiving
		return PCA9685_ERROR;
	}
	// prepare data
	if(addres == PCA9685_MODE1){
		// set the bit and restart in one operation
		tempByte &= ~(MODE1_RESTART|(1<<Bit)); // clear the restart bit and our target bit
		tempByte |= (Value&1)<<Bit; // set the bit to the desired value
	}else{
		// set the byte
		tempByte &= ~1<<Bit;
		tempByte |= (Value&1)<<Bit;
	}
	// transmit
	if(HAL_I2C_Mem_Write(hpca9685->i2c_handle, hpca9685->device_address, addres, 1, &tempByte, 1, PCA9685_I2C_TIMEOUT) == HAL_OK){
		return PCA9685_OK;
	}else{
		return PCA9685_ERROR;
	}
}

// Function to initialise the PCA9685
PCA9685_STATUS PCA9685_Init(PCA9685_HandleTypeDef* hpca9685) {
	// set frequency to 50Hz
	uint8_t Prescale = PCA9685_PRE_SCALE_50;
	if(HAL_I2C_Mem_Write(hpca9685->i2c_handle, hpca9685->device_address, PCA9685_PRE_SCALE, 1, &Prescale, 1, PCA9685_I2C_TIMEOUT) != HAL_OK){
		return PCA9685_ERROR;
	}
	// create variable to store the new value
	uint8_t tempByte = 0;
	// Prepare Byte
	tempByte |=  (MODE1_AI | MODE1_ALLCALL);
	// send to mode1 register mode2 can be left in default state
	if(HAL_I2C_Mem_Write(hpca9685->i2c_handle, hpca9685->device_address, PCA9685_MODE1, 1, &tempByte, 1, PCA9685_I2C_TIMEOUT) == HAL_OK){
		return PCA9685_OK;
	}else{
		return PCA9685_ERROR;
	}
}

// Function to reset the PCA9685
PCA9685_STATUS PCA9685_Reset(PCA9685_HandleTypeDef* hpca9685) {
    // Implementation of reset
	// Isn't send to device address but address 0x00
	uint8_t Command = PCA9685_SWRST_CMD;
	if(HAL_I2C_Master_Transmit(hpca9685->i2c_handle, 0x00, &Command, 1, PCA9685_I2C_TIMEOUT) == HAL_OK){
		return PCA9685_OK;
	}else{
		return PCA9685_ERROR;
	}
}

// Function to put the PCA9685 to sleep
PCA9685_STATUS PCA9685_Sleep(PCA9685_HandleTypeDef* hpca9685) {
	return PCA9685_SetBit(hpca9685, PCA9685_MODE1, MODE1_SLEEP, 1);
}

// Function to wake up the PCA9685
PCA9685_STATUS PCA9685_WakeUp(PCA9685_HandleTypeDef* hpca9685) {
	if(PCA9685_SetBit(hpca9685, PCA9685_MODE1, MODE1_SLEEP, 0) == PCA9685_OK){
		HAL_Delay(1); // Delay 1ms
		return PCA9685_OK;
	}else{
		return PCA9685_ERROR;
	}
}

// Function to wake up the PCA9685 with prescale
PCA9685_STATUS PCA9685_SetExtClk(PCA9685_HandleTypeDef* hpca9685, uint8_t prescale) {
    // Implementation of wake up with prescale
	return PCA9685_ERROR; // Not implemented
}

// Function to set the PWM frequency
PCA9685_STATUS PCA9685_SetPWMFreq(PCA9685_HandleTypeDef* hpca9685, uint16_t freq) {
    // This sets the frequency of the output waveform
	float appPrescaler;
	uint8_t Prescaler;

	// adjust to limits
	if(freq >= 1526){
		Prescaler = PCA9685_PRE_SCALE_MIN;
	}else if(freq <= 24){
		Prescaler = PCA9685_PRE_SCALE_MAX;
	}else{
		appPrescaler = ((hpca9685->oscillator_frequency)/(4096 * (float)freq)) - 1;
		Prescaler = floor(appPrescaler);
	}

	// set prescale Value
	if(PCA9685_Sleep(hpca9685) == PCA9685_ERROR){
		return PCA9685_ERROR;
	}
	if(HAL_I2C_Mem_Write(hpca9685->i2c_handle, hpca9685->device_address, PCA9685_PRE_SCALE, 1, &Prescaler, 1, PCA9685_I2C_TIMEOUT) != HAL_OK){
		return PCA9685_ERROR;
	}
	if(PCA9685_WakeUp(hpca9685) == PCA9685_ERROR){
		return PCA9685_ERROR;
	}
	if(PCA9685_SetBit(hpca9685, PCA9685_MODE1, MODE1_RESTART, 1) == PCA9685_ERROR){
		return PCA9685_ERROR;
	}
	// transmission successful
	return PCA9685_OK;
}

// Function to get the prescale value
PCA9685_STATUS PCA9685_GetPrescale(PCA9685_HandleTypeDef* hpca9685, uint8_t* dest) {
	if(HAL_I2C_Mem_Read(hpca9685->i2c_handle, hpca9685->device_address, PCA9685_PRE_SCALE, 1, dest, 1, PCA9685_I2C_TIMEOUT) != HAL_OK){
		return PCA9685_ERROR;
	}else{
		return PCA9685_OK;
	}
}

// Function to set the output mode
PCA9685_STATUS PCA9685_OutputMode(PCA9685_HandleTypeDef* hpca9685, bool totempole) {
    // Implementation of setting output mode
	return PCA9685_ERROR; // not implemented
}

// Function to get the PWM value of a channel
PCA9685_STATUS PCA9685_GetPWM(PCA9685_HandleTypeDef* hpca9685, uint8_t channel, uint8_t* dest) {
    uint8_t RegAddress;

	// Calculate Register Address
	RegAddress = PCA9685_LED0_ON_L + (4*channel);

	// Get the Data
	if(HAL_I2C_Mem_Read(hpca9685->i2c_handle, hpca9685->device_address, RegAddress, 1, dest, 4, PCA9685_I2C_TIMEOUT) != HAL_OK){
		return PCA9685_ERROR;
	}else{
		return PCA9685_OK;
	}
}

// Function to set the PWM value of a channel
PCA9685_STATUS PCA9685_SetPWM(PCA9685_HandleTypeDef* hpca9685, uint8_t channel, uint16_t onTime) {
	uint8_t RegAddress;
	uint8_t ByteVals[4];

	// Off time isn't available since it is not required for Servo Control
	ByteVals[0] = 0;
	ByteVals[1] = 0;
	// Calculate Register Address and Values
	RegAddress = PCA9685_LED0_ON_L + (4*channel);
	ByteVals[2] = onTime & 0xFF; // lower 8 bits
	ByteVals[3] = onTime >> 8;


	// transmit
	if(HAL_I2C_Mem_Write(hpca9685->i2c_handle, hpca9685->device_address, RegAddress, 1, ByteVals, 4, PCA9685_I2C_TIMEOUT) != HAL_OK){
		return PCA9685_ERROR;
	}else{
		return PCA9685_OK;
	}

}

// Function to set the oscillator frequency
PCA9685_STATUS PCA9685_SetOscillatorFreq(PCA9685_HandleTypeDef* hpca9685, uint32_t freq) {
    hpca9685->oscillator_frequency = freq;
    return PCA9685_OK;
}

uint32_t PCA9685_GetOscillatorFreq(PCA9685_HandleTypeDef* hpca9685) {
	return hpca9685->oscillator_frequency;
}

uint16_t Map_AngleToPWM(uint16_t angle) {
    // Map angle to PWM code
	if(angle >= PCA9685_SERVO_MAX_ANGLE){
		return(PCA9685_SERVO_MAX);
	}else if(angle <= PCA9685_SERVO_MIN_ANGLE){
		return(PCA9685_SERVO_MIN);
	}

	return (angle * (PCA9685_SERVO_MAX - PCA9685_SERVO_MIN) / (PCA9685_SERVO_MAX_ANGLE - PCA9685_SERVO_MIN_ANGLE)) + PCA9685_SERVO_MIN;
}

PCA9685_STATUS PCA9685_SetAngle(PCA9685_HandleTypeDef* hpca9685, uint8_t channel, uint16_t angle) {
    uint16_t pwm = Map_AngleToPWM(angle);
    if (PCA9685_SetPWM(hpca9685, channel, pwm) == PCA9685_OK){
    	return PCA9685_OK;
    }else{
    	return PCA9685_ERROR;
    }
}
