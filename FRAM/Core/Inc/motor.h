#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"


/* TMC2226 Register Addresses */
#define TMC2226_GCONF       0x00
#define TMC2226_GSTAT       0x01
#define TMC2226_IFCNT       0x02
#define TMC2226_NODECONF   0x03



/*OTPMAGIC
Set to 0xbd to enable programming. A programming
time of minimum 10ms per bit is recommended (check
by reading OTP_READ).*/
#define TMC2226_OTP_MAGIC 0xBD00  // Magic value shifted to bits 15-8
#define TMC2226_OTP_PROG    0x04
#define TMC2226_OTP_READ    0x05

// OTP bit positions for internal sense resistor
#define OTP_INTERNAL_RSENSE_BIT 6
#define OTP_INTERNAL_RSENSE_BYTE 0



#define TMC2226_IOIN        0x06
#define TMC2226_FACTORY_CONF 0x07
#define TMC2226_IHOLD_IRUN  0x10
#define TMC2226_TPOWERDOWN  0x11
#define TMC2226_TSTEP       0x12
#define TMC2226_TPWMTHRS    0x13
#define TMC2226_VACTUAL     0x22
#define TMC2226_MSCNT       0x6A
#define TMC2226_MSCURACT    0x6B
#define TMC2226_CHOPCONF    0x6C
#define TMC2226_DRVSTATUS   0x6F
#define TMC2226_PWMCONF     0x70
#define TMC2226_PWM_SCALE   0x71
#define TMC2226_PWM_AUTO    0x72


/* TMC2226 UART Settings */
#define TMC2226_SYNC_BYTE   0x05
#define TMC2226_SLAVE_ADDR  0x00  // Default slave address
#define TMC2226_WRITE_FLAG  0x80
#define TMC2226_READ_FLAG   0x00



// Motor pin definitions
#define DIR_PORT GPIOC
#define DIR_PIN GPIO_PIN_10
#define STEP_PORT GPIOC
#define STEP_PIN GPIO_PIN_11
#define ENABLE_PORT GPIOC
#define ENABLE_PIN GPIO_PIN_9

// TMC2226 Microstepping pins
#define MS1_PORT GPIOC
#define MS1_PIN GPIO_PIN_7
#define MS2_PORT GPIOC
#define MS2_PIN GPIO_PIN_8

// TMC2226 UART pins
#define STEPPER_TX_PORT GPIOC
#define STEPPER_TX_PIN GPIO_PIN_12
#define STEPPER_RX_PORT GPIOD
#define STEPPER_RX_PIN GPIO_PIN_2

// TMC2226 Control pins
#define SPREAD_PORT GPIOA
#define SPREAD_PIN GPIO_PIN_8
#define DIAG_PORT GPIOA
#define DIAG_PIN GPIO_PIN_9
#define INDEX_PORT GPIOA
#define INDEX_PIN GPIO_PIN_10

// Proximity sensor pin definition
#define CASE_PORT GPIOB
#define CASE_PIN GPIO_PIN_15

// LAMP pin definitions
#define LAMP_PWM_PORT GPIOA
#define LAMP_PWM_PIN GPIO_PIN_7
#define LAMP_ISENSE_PORT GPIOA
#define LAMP_ISENSE_PIN GPIO_PIN_5

// PDN_UART pins
#define PDN_UART_RX_PORT GPIOC
#define PDN_UART_RX_PIN GPIO_PIN_2
#define PDN_UART_TX_PORT GPIOD
#define PDN_UART_TX_PIN GPIO_PIN_12

extern uint8_t data[8];
void enablespreadcyclemode(void);
void setMicrostepping(uint8_t ms1_state, uint8_t ms2_state);
void enableStealthMode(void);
void setupMotor(void);
void rotateMotor(int step, uint8_t clockwise);
void homePosition(void);
void protection(void);

void lampOn(void);
void lampOff(void);

uint16_t readLampCurrent(void);
void enableStealthMode(void);
void microsecond_delay(uint16_t microseconds);



// UART communication functions
uint8_t calculateCRC(uint8_t *data, uint8_t length);
void TMC2226_WriteRegister(uint8_t reg, uint32_t value);
uint32_t TMC2226_ReadRegister(uint8_t reg);
void TMC2226_ProgramOTP_InternalRsense(void);
void TMC2226_SetCurrent(uint16_t run_current, uint16_t hold_current);





#endif
