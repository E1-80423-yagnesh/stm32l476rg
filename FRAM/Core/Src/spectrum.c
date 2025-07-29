#include "spectrum.h"

#include "motor.h"
#include <stdio.h>

// Global variables
uint16_t spectra[CHANNELS];
volatile uint8_t dataAvailable = 0;
volatile uint16_t packetIndex = 0;

ADS1xx5_I2C i2c;
int16_t adc1;
int millivolt;
char snum[7];


HAL_StatusTypeDef enableBuffers(void)
{
    // Enable signal buffer
    HAL_GPIO_WritePin(SIG_BUF_EN_PORT, SIG_BUF_EN_PIN, GPIO_PIN_SET);

    // Enable I2C buffer
    HAL_GPIO_WritePin(I2C_BUF_EN_PORT, I2C_BUF_EN_PIN, GPIO_PIN_SET);

    // Small delay to allow buffers to stabilize
    HAL_Delay(1);

    return HAL_OK;
}

HAL_StatusTypeDef checkEOS(void)
{
    // Check End of Scan signal
    GPIO_PinState eosState = HAL_GPIO_ReadPin(EOS_PORT, EOS_PIN);
    return (eosState == GPIO_PIN_SET) ? HAL_OK : HAL_ERROR;
}



void setupSensor(void)
{

    static uint8_t initialized = 0;  // Static flag to track initialization

    if (initialized) {
        return;  // Already initialized, don't print again
    }
    // Enable buffers first
    enableBuffers();

    //initialize adc
    initializeADC1115();

    // ST pin as OUTPUT, initially HIGH
    HAL_GPIO_WritePin(ST_PORT, ST_PIN, GPIO_PIN_SET);

    // CLK pin as OUTPUT, initially LOW
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);

    // Initialize data available flag
    dataAvailable = 0;
    packetIndex = 0;
    initialized = 1;

}

void initializeADC1115(void)
{

    // Initialize ADS1115 with I2C parameters
    ADS1115(&i2c, &hi2c1, ADS_ADDR_GND);

    // Set gain
    ADSsetGain(&i2c, GAIN_ONE);

}

uint16_t readADC1115(void)
{


    uint16_t adcValue = 0;

    // Read 2 bytes from ADC
    adcValue = ADSreadADC_SingleEnded(&i2c, 1);



    return adcValue;
}

void acquireSpectra(void)
{
    // delayTime (dt) = 52microseconds, pixelread = 0
    uint32_t dt = 52;
    int pixelread = 0;



    // Main acquisition loop - Discard first loop to avoid noisy acquisition, acquire from second loop
	for (int loop = 0; loop < 2; loop++) {
		// 3 clock cycles
		for (int clk = 0; clk < 3; clk++) {
			microsecond_delay(dt);
		}

		// CLK, LOW → dt
		HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
		microsecond_delay(dt);

		// ST, LOW
		HAL_GPIO_WritePin(ST_PORT, ST_PIN, GPIO_PIN_RESET);
		// 1 clock cycle
		microsecond_delay(dt);

		// ST, HIGH
		HAL_GPIO_WritePin(ST_PORT, ST_PIN, GPIO_PIN_SET);
		// 1 clock cycle
		microsecond_delay(dt);

		// 1 clock cycle
		microsecond_delay(dt);

		for (int channel = 0; channel < CHANNELS; channel++) {
			// CLK, HIGH
			HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);

			// Read Video, store pixel[Channel] (only in second loop to discard first loop)
			if (loop == 1) {
				spectra[channel] = readADC1115();
				pixelread++;
			}

			// CLK, LOW → dt
			HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
			microsecond_delay(dt);

			// 2 clock cycles
			for (int clk = 0; clk < 2; clk++) {
				microsecond_delay(dt);
			}
		}

		// 5 clock cycles between loops
		for (int clk = 0; clk < 5; clk++) {
			microsecond_delay(dt);
		}

		// Check EOS signal after first loop
		if (loop == 0)
		{
			if (checkEOS() != HAL_OK)
			{
				printf("Warning: EOS signal not detected\r\n");
			}
		}
	}

    // Set data available flag
    dataAvailable = 1;
    packetIndex = 0;

    printf("Spectra acquisition complete - %d pixels read\r\n", pixelread);
}


void triggerAndReceiveSpectra(void)
{
    //1. Send trigger command 'S' to slave device
    char command = 'S';
    HAL_StatusTypeDef status;

    printf("Sending trigger command 'S'\r\n");

    // Send command via I2C to slave device
    status = HAL_I2C_Master_Transmit(&hi2c1, ADS_ADDR_GND << 1, (uint8_t*)&command, 1, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        printf("Error: Failed to send trigger command\r\n");
        return;
    }

    //2.wait for 0.5 second
    HAL_Delay(500); // 0.5 sec delay

    //3.
    acquireSpectra();

    //4. Read and print spectra data
    for(int channel = 0; channel < CHANNELS; channel++)
    {
	   printf("Channel %d: %d\r\n", channel, spectra[channel]);
    }

   //5. Waits 1 second before allowing the next acquisition

    HAL_Delay(1000); // 1 sec delay


}
