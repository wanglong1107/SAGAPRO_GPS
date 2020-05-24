/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

/*! The buffer size for USART */
#define USART_1_BUFFER_SIZE 16

/*! The buffer size for USART */
#define USART_2_BUFFER_SIZE 16

/*! The buffer size for USART */
#define USART_3_BUFFER_SIZE 16

/*! The buffer size for USART */
#define USART_4_BUFFER_SIZE 16

/*! The buffer size for USART */
#define USART_5_BUFFER_SIZE 16

struct usart_async_descriptor USART_1;
struct usart_async_descriptor USART_2;
struct usart_async_descriptor USART_3;
struct usart_async_descriptor USART_4;
struct usart_async_descriptor USART_5;
struct timer_descriptor       TIMER_0;
struct can_async_descriptor   CAN_0;

static uint8_t USART_1_buffer[USART_1_BUFFER_SIZE];
static uint8_t USART_2_buffer[USART_2_BUFFER_SIZE];
static uint8_t USART_3_buffer[USART_3_BUFFER_SIZE];
static uint8_t USART_4_buffer[USART_4_BUFFER_SIZE];
static uint8_t USART_5_buffer[USART_5_BUFFER_SIZE];

struct i2c_m_sync_desc I2C_0;

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(PA08,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA08, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_pull_mode(PA09,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA09, PINMUX_PA09C_SERCOM0_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM0_bit(MCLK);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM0);
	I2C_0_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_1_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM1_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_1_PORT_init()
{

	gpio_set_pin_function(PA16, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_function(PA17, PINMUX_PA17C_SERCOM1_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_1_init(void)
{
	USART_1_CLOCK_init();
	usart_async_init(&USART_1, SERCOM1, USART_1_buffer, USART_1_BUFFER_SIZE, (void *)NULL);
	USART_1_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_2_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM2_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_2_PORT_init()
{

	gpio_set_pin_function(PA12, PINMUX_PA12C_SERCOM2_PAD0);

	gpio_set_pin_function(PA13, PINMUX_PA13C_SERCOM2_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_2_init(void)
{
	USART_2_CLOCK_init();
	usart_async_init(&USART_2, SERCOM2, USART_2_buffer, USART_2_BUFFER_SIZE, (void *)NULL);
	USART_2_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_3_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM3_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_3_PORT_init()
{

	gpio_set_pin_function(PA22, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_function(PA23, PINMUX_PA23C_SERCOM3_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_3_init(void)
{
	USART_3_CLOCK_init();
	usart_async_init(&USART_3, SERCOM3, USART_3_buffer, USART_3_BUFFER_SIZE, (void *)NULL);
	USART_3_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_4_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, CONF_GCLK_SERCOM4_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM4_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_4_PORT_init()
{

	gpio_set_pin_function(PB08, PINMUX_PB08D_SERCOM4_PAD0);

	gpio_set_pin_function(PB09, PINMUX_PB09D_SERCOM4_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_4_init(void)
{
	USART_4_CLOCK_init();
	usart_async_init(&USART_4, SERCOM4, USART_4_buffer, USART_4_BUFFER_SIZE, (void *)NULL);
	USART_4_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_5_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM5_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_5_PORT_init()
{

	gpio_set_pin_function(PA20, PINMUX_PA20C_SERCOM5_PAD2);

	gpio_set_pin_function(PA21, PINMUX_PA21C_SERCOM5_PAD3);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_5_init(void)
{
	USART_5_CLOCK_init();
	usart_async_init(&USART_5, SERCOM5, USART_5_buffer, USART_5_BUFFER_SIZE, (void *)NULL);
	USART_5_PORT_init();
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	hri_mclk_set_APBCMASK_TC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	timer_init(&TIMER_0, TC0, _tc_get_timer());
}

void CAN_0_PORT_init(void)
{

	gpio_set_pin_function(PB11, PINMUX_PB11G_CAN1_RX);

	gpio_set_pin_function(PB10, PINMUX_PB10G_CAN1_TX);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_0_init(void)
{
	hri_mclk_set_AHBMASK_CAN1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN1_GCLK_ID, CONF_GCLK_CAN1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_0, CAN1);
	CAN_0_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA10

	gpio_set_pin_level(Mag_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(Mag_EN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(Mag_EN, GPIO_PIN_FUNCTION_OFF);

	I2C_0_init();
	USART_1_init();
	USART_2_init();
	USART_3_init();
	USART_4_init();
	USART_5_init();

	TIMER_0_init();
	CAN_0_init();
}
