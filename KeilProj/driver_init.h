/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>
#include <hal_flash.h>
#include <hal_i2c_m_sync.h>
#include <hal_usart_async.h>
#include <hal_usart_async.h>
#include <hal_usart_async.h>
#include <hal_usart_async.h>
#include <hal_usart_async.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>
#include <hal_can_async.h>

extern struct i2c_m_sync_desc        I2C_0;
extern struct usart_async_descriptor USART_1;
extern struct usart_async_descriptor USART_2;
extern struct usart_async_descriptor USART_3;
extern struct usart_async_descriptor USART_4;
extern struct usart_async_descriptor USART_5;
extern struct timer_descriptor       TIMER_0;
extern struct can_async_descriptor   CAN_0;

void I2C_0_CLOCK_init(void);
void I2C_0_init(void);
void I2C_0_PORT_init(void);

void USART_1_PORT_init(void);
void USART_1_CLOCK_init(void);
void USART_1_init(void);

void USART_2_PORT_init(void);
void USART_2_CLOCK_init(void);
void USART_2_init(void);

void USART_3_PORT_init(void);
void USART_3_CLOCK_init(void);
void USART_3_init(void);

void USART_4_PORT_init(void);
void USART_4_CLOCK_init(void);
void USART_4_init(void);

void USART_5_PORT_init(void);
void USART_5_CLOCK_init(void);
void USART_5_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
