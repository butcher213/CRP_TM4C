#ifndef _CRP_H_
#define _CRP_H_

// Arducam read/write address
#define ARDUCAM_ADDR 0x10
// Important registers
#define AWAKE_REG 0x0000
#define COARSE_INTEGRATION_TIME_REG 0x3012
#define FINE_INTEGRATION_TIME_REG 0x3014

// Camera IDs
#define TOP_ARDUCAM_0 I2C0_BASE
#define TOP_ARDUCAM_1 I2C1_BASE
#define TOP_ARDUCAM_2 I2C2_BASE
#define TOP_ARDUCAM_3 I2C3_BASE
#define BOTTOM_ARDUCAM_4 I2C4_BASE
#define BOTTOM_ARDUCAM_5 I2C5_BASE
#define BOTTOM_ARDUCAM_6 I2C6_BASE
#define BOTTOM_ARDUCAM_7 I2C7_BASE


// UART commands
#define FPGA_STATUS 0x20
#define FPGA_READ   0x40
/*
#define TOP_FPGA_ID UART5_BASE
#define BOTTOM_FPDA_ID UART7_BASE
// Camera functions
void initCamera(uint32_t cameraId);
void takePicture(uint32_t cameraId);
void softResetCamera(uint32_t cameraId);
void firmResetCamera(uint32_t cameraId);

// Peripheral-specific functions
void initMiscPeripherals(void);

// I2C generic functions
void I2CSend(uint32_t I2Cbase, uint16_t reg, uint16_t data);
void I2CSendSingle(uint32_t I2Cbase, uint16_t reg, uint8_t data);
uint16_t I2CReceive(uint32_t I2Cbase, uint16_t reg);
void initI2C(uint32_t I2Cbase);

// FPGA UART Functions
void initUART(uint32_t UARTbase);
void UARTSendByte(uint32_t fpgaId, uint8_t data);
void UART7IntHandler(void);
void UART5IntHandler(void);
uint64_t readFPGAData(uint32_t fpgaId);
void resetFPGA(uint32_t fpgaId);*/
#endif