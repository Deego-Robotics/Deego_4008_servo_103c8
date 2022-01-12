#include "TLE5012b.h"

// SPI init structure
SPI_HandleTypeDef spiConfig;

// Main initialization structure
GPIO_InitTypeDef GPIO_InitStructure;

TLE5012B::TLE5012B(){}

// Function to setup the encoder
void TLE5012B::init() {

    pinMode(ENCODER_SS, OUTPUT);

    // Setup pin A5, A6, and A7
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Enable the clock for the SPI bus
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Set the peripheral to be used
    spiConfig.Instance = SPI1;

    // Configure the settings for transactions
    spiConfig.Init.Direction = SPI_DIRECTION_2LINES;
    spiConfig.Init.Mode = SPI_MODE_MASTER;
    spiConfig.Init.DataSize = SPI_DATASIZE_8BIT;
    spiConfig.Init.CLKPolarity = SPI_POLARITY_LOW;
    spiConfig.Init.CLKPhase = SPI_PHASE_2EDGE;
    spiConfig.Init.NSS = SPI_NSS_SOFT;
    spiConfig.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spiConfig.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiConfig.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiConfig.Init.CRCPolynomial = 10;

    // Initialize the SPI bus with the parameters we set
    if (HAL_SPI_Init(&spiConfig) != HAL_OK) {
        //Serial.println(F("SPI not initialized!"));
    }

    digitalWrite(ENCODER_SS, LOW);

    uint8_t rxbuf[2];
    uint8_t txbuf[2] = { 0x50, 0x80 };

    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    txbuf[0] = 0x08, txbuf[1] = 0x00;
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);
    
    digitalWrite(ENCODER_SS, HIGH);

    // velocity calculation init
	angle_prev = 0;
	velocity_calc_timestamp = _micros(); 

	// full rotations tracking number
	full_rotation_offset = 0;
	angle_data_prev = getRawCount();  
}

// Read the value of a register
uint16_t readEncoderRegister(uint16_t registerAddress) {

    // Pull CS low to select encoder
    digitalWrite(ENCODER_SS, LOW);

    // Add read bit to address
    registerAddress |= ENCODER_READ_COMMAND + 1;

    // Setup RX and TX buffers
    uint8_t rxbuf[2];
    uint8_t txbuf[2] = { uint8_t(registerAddress >> 8), uint8_t(registerAddress) };

    // Send address we want to read, response seems to be equal to request
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);

    txbuf[0] = 0xFF, txbuf[1] = 0xFF;
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);

    // Deselect encoder
    digitalWrite(ENCODER_SS, HIGH);

    // Return value as uint16
    return rxbuf[0] << 8 | rxbuf[1];
}

// Read multiple registers
void readMultipleEncoderRegisters(uint16_t registerAddress, uint16_t* data, uint16_t dataLength) {

    // Pull CS low to select encoder
    digitalWrite(ENCODER_SS, LOW);

    // Setup TX and RX buffers
    registerAddress |= ENCODER_READ_COMMAND + dataLength;
    uint8_t txbuf[dataLength * 2] = { uint8_t(registerAddress >> 8), uint8_t(registerAddress) };
    uint8_t rxbuf[dataLength * 2];

    // Send address we want to read, response seems to be equal to request
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    // Set the MOSI pin to open drain
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Send 0xFFFF (like BTT code), this returns the wanted value
    // Array length is doubled as we're using 8 bit values instead of 16
    for (uint8_t i = 0; i < dataLength * 2; i++) {
        txbuf[i] = 0xFF;
    }
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, dataLength * 2, 100);
    
    // Write the received data into the array
    for (uint8_t i = 0; i < dataLength; i++) {
        data[i] = rxbuf[i * 2] << 8 | rxbuf[i * 2 + 1];
    }

    // Set MOSI back to Push/Pull
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Deselect encoder
    digitalWrite(ENCODER_SS, HIGH);
}

int TLE5012B::getRawCount() {
    uint16_t rawData = (readEncoderRegister(ENCODER_ANGLE_REG) & (DELETE_BIT_15));
    return rawData >> 1;
}

// Reads the value for the angle of the encoder in Radians
float TLE5012B::getAngle() {
  // raw data from the sensor
  float angle_data = getRawCount(); 

  // tracking the number of rotations 
  // in order to expand angle range form [0,2PI] 
  // to basically infinity
  float d_angle = angle_data - angle_data_prev;
  // if overflow happened track it as full rotation
  if(abs(d_angle) > ENCODER_80_RANGE ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
  // save the current angle value for the next steps
  // in order to know if overflow happened
  angle_data_prev = angle_data;

  // return the full angle 
  // (number of full rotations)*2PI + current sensor angle 
  return full_rotation_offset + ( angle_data / (float)ENCODER_FULL_RANGE) * _2PI;
}

float TLE5012B::getVelocity(){
  // calculate sample time
  unsigned long now_us = _micros();
  float Ts = (now_us - velocity_calc_timestamp)*1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // current angle
  float angle_c = getAngle();
  // velocity calculation
  float vel = (angle_c - angle_prev)/Ts;

  // save variables for future pass
  angle_prev = angle_c;
  velocity_calc_timestamp = now_us;
  return vel;
}

