/*
 * PCA9685.h
 *
 *  Created on: Sep 30, 2024
 *      Author: Leonhard17
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

#include "stm32wbxx_hal.h"
#include <stdbool.h>

// defines
#define PCA9685_DEFAULT_ADDRESS 0x80 // default I2C address of the IC, can be changed with pin connections, 0x40 shifted left
#define PCA9685_OSC_FREQ 26361816 // default is 25000000
#define PCA9685_I2C_TIMEOUT 1
// register addresses
#define  PCA9685_MODE1 0x00 // mode 1 register
#define  PCA9685_MODE2 0x01 // mode 2 register
#define  PCA9685_SUBADR1 0x02 // i2c sub-address 1
#define  PCA9685_SUBADR2 0x03 // i2c sub-address 2
#define  PCA9685_SUBADR3 0x04 // i2c sub-address 3
#define  PCA9685_ALLCALLADR 0x05 // LED ALL Call I2C address
// first LED
#define  PCA9685_LED0_ON_L 0x06 // output control byte 0, on low-byte
#define  PCA9685_LED0_ON_H 0x07 // output control byte 1, on high-byte
#define  PCA9685_LED0_OFF_L 0x08 // output control byte 2, off low-byte
#define  PCA9685_LED0_OFF_H 0x09 // output control byte 3, off high-byte
// same pattern for next 15 LEDs, Last PCA9685_LDE15_OFF_H 0x45
#define  PCA9685_ALL_LED_ON_L 0xFA //load all LEDn_ON_L registers
#define  PCA9685_ALL_LED_ON_H 0xFB //load all LEDn_ON_H registers
#define  PCA9685_ALL_LED_OFF_L 0xFC //load all LEDn_OFF_L registers
#define  PCA9685_ALL_LED_OFF_H 0xFD //load all LEDn_OFF_H registers

#define  PCA9685_PRE_SCALE 0xFE // prescaler for PWM output frequency, blocked when SLEEP is logic 0 (Mode 1)
#define  PCA9685_TestMode 0xFF // Test mode to be entered, Reserved

// Mode 1 bits
//sets response to requests: 1 responds, 0 does not respond
#define  MODE1_ALLCALL 0x01
#define  MODE1_SUB3 0x02
#define  MODE1_SUB2 0x04
#define  MODE1_SUB1 0x08
// sets mode
#define  MODE1_SLEEP 0x10
#define  MODE1_AI 0x20 // Auto-increment, works for LED settings rolls over at the end, also works for all Led registers
#define  MODE1_EXTCLK 0x40 // Use of external clock, to activate set to sleep and activate SLEEP and EXTCLK at the same time, sticky bit only cleared power cycle or reset
#define  MODE1_RESTART 0x80 // disables or enables restarts, write 1 to clear to 0

// Mode 2 bits
#define  MODE2_OUTNE_0 0x01 // Output drivers not enabled
#define  MODE2_OUTNE_1 0x02 // 00 OE=1, LEDn =0, 01 OE=1 LEDn=1 when OUTFRV=1 or LEDn=high impedance OUTDRV=0 (Same as OUTNE 10), 1X OE=1 LEDn=high impedance
#define  MODE2_OUTDRV 0x04 // output configuration, 0 open drain, 1 totem pole (used for this driver)
#define  MODE2_OCH 0x08 // Output change event, 0 Stop, 1 ACK
#define  MODE2_INVRT 0x10 // Invert output, 0 off, 1 on

// software reset command
#define PCA9685_SWRST_CMD 0x06

// Prescaler, only adjustable in sleep mode
#define  PCA9685_PRE_SCALE_MIN 3 // minimum value, 1526Hz
#define  PCA9685_PRE_SCALE_MAX 255 // maximum value, 24Hz
#define  PCA9685_PRE_SCALE_50 128 // base value for 50Hz, default is 121

// range of PWM to control the servo-motors, adjust the values as needed
#define PCA9685_SERVO_MIN 103 // usually 110
#define PCA9685_SERVO_MAX 515 // usually 500
// range of PWM to control the servo-motors, adjust the values as needed
#define PCA9685_SERVO_MIN_ANGLE 0
#define PCA9685_SERVO_MAX_ANGLE 270

// variables for transmission
//uint8_t servo_buf[40]; to be used in the future

// type definition of PCA9685
typedef struct{
	I2C_HandleTypeDef *i2c_handle;
	uint16_t device_address;
	uint32_t oscillator_frequency;
}PCA9685_HandleTypeDef;

// Error Handling
typedef enum{
	PCA9685_OK = 0,
	PCA9685_ERROR = 1
}PCA9685_STATUS;

// functions
PCA9685_STATUS PCA9685_Write8(PCA9685_HandleTypeDef* hpca9685, uint8_t addres, uint8_t* data);
PCA9685_STATUS PCA9685_Read8(PCA9685_HandleTypeDef* hpca9685, uint8_t addres, uint8_t* dest);
PCA9685_STATUS PCA9685_SetBit(PCA9685_HandleTypeDef* hpca9685, uint8_t addres, uint8_t Bit, uint8_t Value); // use with mode 1 or 2 register only
PCA9685_STATUS PCA9685_Init(PCA9685_HandleTypeDef* hpca9685);
PCA9685_STATUS PCA9685_Reset(PCA9685_HandleTypeDef* hpca9685);
PCA9685_STATUS PCA9685_Sleep(PCA9685_HandleTypeDef* hpca9685);
PCA9685_STATUS PCA9685_WakeUp(PCA9685_HandleTypeDef* hpca9685);
PCA9685_STATUS PCA9685_SetExtClk(PCA9685_HandleTypeDef* hpca9685, uint8_t prescale);
PCA9685_STATUS PCA9685_SetPWMFreq(PCA9685_HandleTypeDef* hpca9685, uint16_t freq); // IC output frequency, 50Hz for servo-motors
PCA9685_STATUS PCA9685_GetPrescale(PCA9685_HandleTypeDef* hpca9685, uint8_t* dest);
PCA9685_STATUS PCA9685_OutputMode(PCA9685_HandleTypeDef* hpca9685, bool totempole);
PCA9685_STATUS PCA9685_GetPWM(PCA9685_HandleTypeDef* hpca9685, uint8_t channel, uint8_t* dest);
PCA9685_STATUS PCA9685_SetPWM(PCA9685_HandleTypeDef* hpca9685, uint8_t channel, uint16_t onTime); // Sets PWM of a channel, max 4095 on
PCA9685_STATUS PCA9685_SetOscillatorFreq(PCA9685_HandleTypeDef* hpca9685, uint32_t freq);
uint32_t PCA9685_GetOscillatorFreq(PCA9685_HandleTypeDef* hpca9685);
//Special Servo-control functions
uint16_t Map_AngleToPWM(uint16_t angle); // maps an angle from 0 to 270, to a PWM for servo-control
PCA9685_STATUS PCA9685_SetAngle(PCA9685_HandleTypeDef* hpca9685, uint8_t channel, uint16_t angle);

#endif /* INC_PCA9685_H_ */
