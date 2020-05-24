#include "hal_qspi.h"
#include <os.h>
int HAL_QSPI::initialize()
{
	if(!initialFlag)
	{
		qspi_sync_enable( qspi );
		initialFlag = true;
	}
	return 0;
}

int HAL_QSPI::disInitialize(void)
{
	if(initialFlag)
	{
		qspi_sync_disable( qspi );
		initialFlag = false;
	}
	return 0;
}

int HAL_QSPI::setBaudRate( uint32_t cnt_Hz)
{
	uint32_t baudRate = CONF_QSPI_FREQUENCY/cnt_Hz -1;
	
	if( cnt_Hz > CONF_QSPI_FREQUENCY || baudRate > 255)
	{
		return -1;
	}
	
	hri_qspi_write_SCR_SCBR_bf(qspi->dev.prvt, baudRate);
	
	return 0;
}

int HAL_QSPI::writeData(uint8_t _reg, uint8_t pData)
{
	volatile uint8_t dummy;
	
//	CPU_SR_ALLOC();
//	CPU_CRITICAL_ENTER();
	
	gpio_set_pin_level(nCS, false);
	
	while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));
	((Qspi *)qspi->dev.prvt)->QSPI_TDR = _reg;
	while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));

	while(!hri_qspi_get_SR_RDRF_bit(qspi->dev.prvt));
	dummy = ((Qspi *)qspi->dev.prvt)->QSPI_RDR;	
	

	while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));
	((Qspi *)qspi->dev.prvt)->QSPI_TDR = pData ;
	while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));

	while(!hri_qspi_get_SR_RDRF_bit(qspi->dev.prvt));
	dummy = (uint16_t)((Qspi *)qspi->dev.prvt)->QSPI_RDR;

	gpio_set_pin_level(nCS, true);
//	CPU_CRITICAL_EXIT();
	return 0;	
}


int HAL_QSPI::writeData(uint8_t _reg, uint8_t *_pData, uint32_t len)
{
	volatile uint8_t dummy;
	uint8_t* pData = _pData;
	
//	CPU_SR_ALLOC();
//	CPU_CRITICAL_ENTER();
	gpio_set_pin_level(nCS, false);
	
	while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));
	((Qspi *)qspi->dev.prvt)->QSPI_TDR = _reg;
	while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));

	while(!hri_qspi_get_SR_RDRF_bit(qspi->dev.prvt));
	dummy = ((Qspi *)qspi->dev.prvt)->QSPI_RDR;	
	
	for(int i=0; i<len; i++)
	{	
		while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));
		((Qspi *)qspi->dev.prvt)->QSPI_TDR = *pData ;
		while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));

		while(!hri_qspi_get_SR_RDRF_bit(qspi->dev.prvt));
		dummy = (uint16_t)((Qspi *)qspi->dev.prvt)->QSPI_RDR;
		pData ++;
	}
	

	gpio_set_pin_level(nCS, true);
//	CPU_CRITICAL_EXIT();
	return 0;
}


int HAL_QSPI::readData(uint8_t _reg, uint8_t *_pData, uint32_t len)
{
	uint8_t* pData = _pData;
	uint8_t dummyData = 0x00;
	volatile uint32_t dummy;
	
//	CPU_SR_ALLOC();
//	CPU_CRITICAL_ENTER();
	gpio_set_pin_level(nCS, false);
	
	dummy = ((Qspi *)qspi->dev.prvt)->QSPI_RDR;
	while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));
	((Qspi *)qspi->dev.prvt)->QSPI_TDR = _reg ;
	while(!hri_qspi_get_SR_TDRE_bit(qspi->dev.prvt));
	
	while(!hri_qspi_get_SR_RDRF_bit(qspi->dev.prvt));
	dummy = ((Qspi *)qspi->dev.prvt)->QSPI_RDR;
	
	for(int i=0; i<len; i++)
	{	
		while(!hri_qspi_get_SR_TXEMPTY_bit(qspi->dev.prvt));
		((Qspi *)qspi->dev.prvt)->QSPI_TDR = dummyData ;
		while(!hri_qspi_get_SR_TDRE_bit(qspi->dev.prvt));

		while(!hri_qspi_get_SR_RDRF_bit(qspi->dev.prvt));
		*(pData++) = (uint16_t)((Qspi *)qspi->dev.prvt)->QSPI_RDR;
	}
	gpio_set_pin_level(nCS, true);
//	CPU_CRITICAL_EXIT();
	
	return 0;
}

