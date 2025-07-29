/* spectrum_sensor.h */
#ifndef SPECTRUM_SENSOR_H
#define SPECTRUM_SENSOR_H

#include "main.h"
#include "ADS1015_ADS1115.h"

// Spectrum sensor constants
#define CHANNELS 8
#define PACKET_SIZE 8
#define ADC_I2C_ADDRESS 0x48 // External ADC I2C address

// Spectrum sensor pin definitions - Updated to match actual hardware
#define ST_PORT GPIOC          // Start/Strobe signal
#define ST_PIN GPIO_PIN_1      // PC1_SENSOR_ST

#define CLK_PORT GPIOB         // Clock signal
#define CLK_PIN GPIO_PIN_10    // PB10_SENSOR_CLK

#define EOS_PORT GPIOC         // End of Scan signal (input)
#define EOS_PIN GPIO_PIN_0     // PC0_SENSOR_EOS

// Buffer enable signals
#define I2C_BUF_EN_PORT GPIOA  // I2C buffer enable
#define I2C_BUF_EN_PIN GPIO_PIN_6  // PA6_I2C_BUF_EN

#define SIG_BUF_EN_PORT GPIOA  // Signal buffer enable
#define SIG_BUF_EN_PIN GPIO_PIN_4  // PA4_SIG_BUF_EN

// ADC configuration for video signal
#define VIDEO_PIN ADC_CHANNEL_0  // Assuming video goes to ADC channel 0
// Note: If video signal goes through external ADC via I2C (PB8/PB9),
// then internal ADC channel may not be used

// I2C pins for external ADC communication
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_8     // PB8_SENSOR_SCL

#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN GPIO_PIN_9     // PB9_SENSOR_SDA


// Global variables
extern uint16_t spectra[CHANNELS];
extern volatile uint8_t dataAvailable;
extern volatile uint16_t packetIndex;
extern volatile uint8_t i2c_rx_buffer[10];
extern volatile uint8_t i2c_tx_buffer[10];
extern volatile uint8_t command_received;
extern volatile uint8_t current_command;
extern I2C_HandleTypeDef hi2c1;

// Function prototypes
void setupSensor(void);
void acquireSpectra(void);
uint16_t readExternalADC(void);
HAL_StatusTypeDef enableBuffers(void);
uint8_t waitForEOS(void);  // Function to wait for End of Scan signal
void initializeADC1115(void);

#endif
