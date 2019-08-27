//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "driverlib/rom_map.h"
#include <stdarg.h>
#include "inc/hw_i2c.h"
#include "driverlib/uart.h"
#include "crp.h"
//#include "drivers/buttons.h"
//*****************************************************************************
//

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
uint32_t qwerty = -1;
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
  while(1);
}
#endif
void
UARTSend(const uint8_t *pui8Buffer)
{
  uint32_t ui32Count =  (uint32_t)strlen((char *)pui8Buffer);
  //
  // Loop while there are more characters to send.
  //
  while(ui32Count--)
  {
    //
    // Write the next character to the UART.
    //
    UARTCharPut(UART0_BASE, *pui8Buffer++);
  }
}
uint32_t g_ui32SysClock;
void InitI2C0(void)
{
  //enable I2C module 0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  
  //reset module
  SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
  
  //enable GPIO peripheral that contains I2C 0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  
  // Configure the pin muxing for I2C0 functions on port B2 and B3.
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  
  // Select the I2C function for these pins.
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
  
  // Enable and initialize the I2C0 master module.  Use the system clock for
  // the I2C0 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
  
  //clear I2C FIFOs
  HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}
//sends an I2C command to the specified slave

/*void I2CSend(uint8_t slave_addr,uint8_t reg1, uint8_t reg2, uint8_t data1, uint8_t data2)
{
  // Tell the master module what address it will place on the bus when
  // communicating with the slave.
  I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
  I2CMasterDataPut(I2C0_BASE,reg1);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,reg2);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,data1);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,data2);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  while(I2CMasterBusy(I2C0_BASE));
  
}*/
void I2CSend(uint8_t slave_addr,uint16_t reg, uint16_t data)
{
  // Tell the master module what address it will place on the bus when
  // communicating with the slave.
  uint8_t reg2 = reg & 0xFF;
  uint8_t reg1 = reg>>8;
  uint8_t data2 = data & 0xFF;
  uint8_t data1 = data>>8;
  I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
  I2CMasterDataPut(I2C0_BASE,reg1);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,reg2);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,data1);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,data2);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  while(I2CMasterBusy(I2C0_BASE));
  
}
uint8_t receivedChar = 0x0;
char buf[30];
void
UART7IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART7_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART7_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
  //  UARTSend("Got stuff\r");
    while(ROM_UARTCharsAvail(UART7_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        
        receivedChar = ROM_UARTCharGetNonBlocking(UART7_BASE);
      
        sprintf(buf, "Got: %c\r", receivedChar);
        UARTSend(buf);
        //
        // Blink the LED to show a character transfer is occuring.
        //
        
    }
  //  UARTSend("Got stuff\r");
}
void I2CSendSingle(uint8_t slave_addr,uint16_t reg, uint8_t data)
{
  uint8_t reg2 = reg & 0xFF;
  uint8_t reg1 = reg>>8;
  // Tell the master module what address it will place on the bus when
  // communicating with the slave.
  I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
  I2CMasterDataPut(I2C0_BASE,reg1);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,reg2);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE,data);

  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  while(I2CMasterBusy(I2C0_BASE));
}
//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint16_t reg)
{
  uint8_t reg2 = reg & 0xFF;
  uint8_t reg1 = reg>>8;
  I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
  I2CMasterDataPut(I2C0_BASE, reg1);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);  
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterDataPut(I2C0_BASE, reg2);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  while(I2CMasterBusy(I2C0_BASE));
  I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
   uint32_t output1 = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));
     uint32_t output2 = I2CMasterDataGet(I2C0_BASE);
   I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  
   uint32_t output = (output2<<8) + output1;
   while(I2CMasterBusy(I2C0_BASE));
  return output;
}
void initCamera(void) {
  char buf[30];
  volatile uint32_t ui32Loop;
  uint32_t recievedValue = 0x0000;
  while (recievedValue != 0x2E00) {
    
    recievedValue = I2CReceive(ARDUCAM_ADDR, AWAKE_REG);
    sprintf(buf, "%d\r\n", recievedValue);
    UARTSend(buf);
    for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
// UARTSend(recievedValue);
  }
  // initial intitialization
  I2CSend(ARDUCAM_ADDR, 0x301A, 0x0018);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSendSingle(ARDUCAM_ADDR, 0x0103, 1);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x31AE, 0x0301);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0112, 0x0C0C);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x3064, 0x0805);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x301E, 0x00A8);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSendSingle(ARDUCAM_ADDR, 0x0104, 1);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSendSingle(ARDUCAM_ADDR, 0x0100, 0);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  
  // Configure pixel clock
  I2CSend(ARDUCAM_ADDR, 0x0300, 0x0004);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0302, 0x0001);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0304, 0x0008);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0306, 0x006E);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0308, 0x000C);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x030A, 0x0001);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  
  // configure pixel output
  I2CSend(ARDUCAM_ADDR, 0x3016, 0x0111);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0344, 0x0090);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0348, 0x11AF);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0346, 0x0020);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x034A, 0x0CF7);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x3040, 0x0041);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0400, 0x0000);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0404, 0x0010);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x034C, 0x1120);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x034E, 0x0CD8);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0342, 0xFFFF);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0340, 0x0D67);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x3010, 0x00CF);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, COARSE_INTEGRATION_TIME_REG, 0xFFFF);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, FINE_INTEGRATION_TIME_REG, 0xFFFF);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x3018, 0x0000);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  
  I2CSend(ARDUCAM_ADDR, 0x30E8, 0x8001);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSendSingle(ARDUCAM_ADDR, 0x1004, 0x00);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  
//  I2CSendSingle(ARDUCAM_ADDR, 0x01, 0x00, 0x01);
 // for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  
  // reset register?
  I2CSend(ARDUCAM_ADDR, 0x301A, 0x5CCC);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x3012, 0x03E8); // dec 1000
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0206, 0x0021); // dec 33
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0208, 0x0032); // dec 50
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x020A, 0x0032); // dec 50
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x020C, 0x0021); // dec 33
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x0304, 0x0008); // dec 8
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  
  // set analog gains
  I2CSend(ARDUCAM_ADDR, 0x305E, 0x1430);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x3028, 0x0016);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x302A, 0x0012);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x302C, 0x001A);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x302E, 0x001A);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_ADDR, 0x3030, 0x0012);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
}
//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int
main(void)
{
  volatile uint32_t ui32Loop;
  char buf[30];
  volatile uint32_t recievedValue;
  
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                           SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                               SYSCTL_CFG_VCO_480), 120000000);
  //
  // Enable the GPIO port that is used for the on-board LED.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  //
  // Check if the peripheral access is enabled.
  //
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
  {
  }
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
  
  //
  // Enable the peripherals used by this example.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  
  //
  // Enable processor interrupts.
  //
 
  
  //
  // Set GPIO A0 and A1 as UART pins.
  //
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  
  GPIOPinConfigure(GPIO_PC4_U7RX);
  GPIOPinConfigure(GPIO_PC5_U7TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
  
  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
  ROM_UARTConfigSetExpClk(UART7_BASE, g_ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
  //
  // Prompt for text to be entered.
  //
  ROM_IntEnable(INT_UART7);
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
  UARTSend((uint8_t *)"\033[2JEnter text: ");
  //
  // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
  IntMasterDisable();
  InitI2C0();
  
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
  
  //
  // Loop forever.
  // 
 /* uint32_t loopCount = 0;
  recievedValue = 0;
 // I2CSend(ARDUCAM_READ_ADDR, 0x06, 0x00,0,1);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
//  I2CSend(ARDUCAM_READ_ADDR, 0x03, 0x0A,0,0x10);
 // I2CSendSingle(ARDUCAM_READ_ADDR, 0x01, 0x00,1);
  //recievedValue = I2CReceive(ARDUCAM_READ_ADDR, 0x30, 0x1A );
  
  I2CSend(ARDUCAM_READ_ADDR, 0x31, 0x5E, 0x00, 0x01);
  for(ui32Loop = 0; ui32Loop < 5000000; ui32Loop++);
  I2CSend(ARDUCAM_READ_ADDR, 0x31, 0x1c, 0x00, 0x01);
  while (recievedValue != 0x2F) {
   // recievedValue = I2CReceive(ARDUCAM_READ_ADDR, 0x00, 0x00 );
    recievedValue = I2CReceive(ARDUCAM_READ_ADDR, 0x31, 0x5E);
    sprintf(buf, "%d\r\n", recievedValue);
    UARTSend(buf);
    for(ui32Loop = 0; ui32Loop < 10000000; ui32Loop++);
   // if(loopCount % 2 != 0) {
   // I2CSendSingle(ARDUCAM_READ_ADDR, 0x01, 0x00,1);
  //}
   // I2CSendSingle(ARDUCAM_READ_ADDR, 0x30, 0x1C,1);
   // } else {
  //  I2CSendSingle(ARDUCAM_READ_ADDR, 0x30, 0x1C,0);  
   // }
  //  I2CSend(ARDUCAM_READ_ADDR, 0x30, 0x1A, 0x08, 0x9C);
  // for(ui32Loop = 0; ui32Loop < 10000000; ui32Loop++);
    loopCount++;
  }
  UARTSend("Exiting Hardware Standby");
  uint32_t globalSeqValue = 1;
  for(ui32Loop = 0; ui32Loop < 2000000; ui32Loop++)
  {
  }
  I2CSend(ARDUCAM_READ_ADDR, 0x31, 0x5e,0,1);
  //I2CSend(ARDUCAM_READ_ADDR, 0x30,0x1C, 0x00);
  for(ui32Loop = 0; ui32Loop < 2000000; ui32Loop++)
  {
  }
//  I2CSend(ARDUCAM_READ_ADDR, 0x31, 0x
  while(1) {
   // I2CSend(ARDUCAM_READ_ADDR, 0x30,0x1C, 0x00);
    for(ui32Loop = 0; ui32Loop < 50000; ui32Loop++);
    UARTSend("Reading picture");
  }
  UARTSend("trying to take another");
  //I2CSend(ARDUCAM_READ_ADDR, ARDUCAM_GLOBAL_SEQ_TRIGGER_ADDR, globalSeqValue);
  UARTSend("\n\rPicture Taken (hopefully)");*/
 // initCamera();
  IntMasterEnable();
  while(1)
  {
   
    
 
    //
    // Turn on the LED.
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
    
    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 2000000; ui32Loop++)
    {
    }
    
    //
    // Turn off the LED.
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);
    
    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 2000000; ui32Loop++)
    {
    }
    //loopCount++;
  }
}
