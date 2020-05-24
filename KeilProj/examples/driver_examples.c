/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

void I2C_0_example(void)
{
	struct io_descriptor *I2C_0_io;

	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, 0x12, I2C_M_SEVEN);
	io_write(I2C_0_io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using USART_1 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_1[12] = "Hello World!";

static void tx_cb_USART_1(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void USART_1_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&USART_1, USART_ASYNC_TXC_CB, tx_cb_USART_1);
	/*usart_async_register_callback(&USART_1, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_1, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_1, &io);
	usart_async_enable(&USART_1);

	io_write(io, example_USART_1, 12);
}

/**
 * Example of using USART_2 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_2[12] = "Hello World!";

static void tx_cb_USART_2(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void USART_2_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&USART_2, USART_ASYNC_TXC_CB, tx_cb_USART_2);
	/*usart_async_register_callback(&USART_2, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_2, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_2, &io);
	usart_async_enable(&USART_2);

	io_write(io, example_USART_2, 12);
}

/**
 * Example of using USART_3 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_3[12] = "Hello World!";

static void tx_cb_USART_3(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void USART_3_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&USART_3, USART_ASYNC_TXC_CB, tx_cb_USART_3);
	/*usart_async_register_callback(&USART_3, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_3, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_3, &io);
	usart_async_enable(&USART_3);

	io_write(io, example_USART_3, 12);
}

/**
 * Example of using USART_4 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_4[12] = "Hello World!";

static void tx_cb_USART_4(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void USART_4_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&USART_4, USART_ASYNC_TXC_CB, tx_cb_USART_4);
	/*usart_async_register_callback(&USART_4, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_4, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_4, &io);
	usart_async_enable(&USART_4);

	io_write(io, example_USART_4, 12);
}

/**
 * Example of using USART_5 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_5[12] = "Hello World!";

static void tx_cb_USART_5(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void USART_5_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&USART_5, USART_ASYNC_TXC_CB, tx_cb_USART_5);
	/*usart_async_register_callback(&USART_5, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_5, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_5, &io);
	usart_async_enable(&USART_5);

	io_write(io, example_USART_5, 12);
}

static struct timer_task TIMER_0_task1, TIMER_0_task2;

/**
 * Example of using TIMER_0.
 */
static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_0_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_0_example(void)
{
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_0_task2.interval = 200;
	TIMER_0_task2.cb       = TIMER_0_task2_cb;
	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
}

void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	(void)descr;
}
void CAN_0_rx_callback(struct can_async_descriptor *const descr)
{
	struct can_message msg;
	uint8_t            data[64];
	msg.data = data;
	can_async_read(descr, &msg);
	return;
}

/**
 * Example of using CAN_0 to Encrypt/Decrypt datas.
 */
void CAN_0_example(void)
{
	struct can_message msg;
	struct can_filter  filter;
	uint8_t            send_data[4];
	send_data[0] = 0x00;
	send_data[1] = 0x01;
	send_data[2] = 0x02;
	send_data[3] = 0x03;

	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);
	can_async_enable(&CAN_0);

	/**
	 * CAN_0_tx_callback callback should be invoked after call
	 * can_async_write, and remote device should recieve message with ID=0x45A
	 */
	can_async_write(&CAN_0, &msg);

	msg.id  = 0x100000A5;
	msg.fmt = CAN_FMT_EXTID;
	/**
	 * remote device should recieve message with ID=0x100000A5
	 */
	can_async_write(&CAN_0, &msg);

	/**
	 * CAN_0_rx_callback callback should be invoked after call
	 * can_async_set_filter and remote device send CAN Message with the same
	 * content as the filter.
	 */
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	filter.id   = 0x469;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);

	filter.id   = 0x10000096;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);
}
