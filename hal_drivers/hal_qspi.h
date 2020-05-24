#ifndef _HAL_QSPI_H
#define _HAL_QSPI_H
	 
#include <atmel_start.h>
#include "hpl_qspi_config.h"

class HAL_QSPI
{
private:	
	struct qspi_sync_descriptor *qspi;
	uint8_t nCS;

public:
	bool initialFlag;
	
	HAL_QSPI(struct qspi_sync_descriptor *_qspi, uint8_t _nCS)
	{
		qspi = _qspi;
		nCS = _nCS;
		initialFlag = false;
	}
	virtual ~HAL_QSPI(){}
	
	int initialize();
	int disInitialize(void);
	
	int setBaudRate( uint32_t cnt_Hz);

	int writeData(uint8_t _reg, uint8_t pData);
		
	int writeData(uint8_t _reg, uint8_t *_Data, uint32_t len);
		
	int readData(uint8_t _reg, uint8_t *_pData, uint32_t len);
		
};

#endif //_HAL_QSPI_H
