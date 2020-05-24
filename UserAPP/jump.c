////////////////////////////////////////////////////////////
// GDU
// wanglong 
// 2019.7.2
// use for app
////////////////////////////////////////////////////////////
#include "jump.h"

#include "driver_init.h"
////////////////////////////////////////////////////////////
//warning: before use this file you must add
//         Config --hpl_nvmctrl_config.h
//         hal    --hal_flash.c
//				  --hal_flash.h
//				  --hpl_flash.h
//				  --hpl_user_area.h
//         hpl    --hpl_nvmctrl.c
////////////////////////////////////////////////////////////
struct flash_descriptor FLASH_0;
void FLASH_flg(void)
{
	uint8_t flg[1];
	uint8_t check[1];
	flg[0] =0x55;
	
	/* Write data to flash */
	flash_write(&FLASH_0, 0x3f000, flg, 1);
	/* Read data from flash */
	flash_read(&FLASH_0, 0x3f000, check, 1);
	
	while(check[0]!=0x55) //until the address '0x3f000' is '0x55'
	{
		flash_write(&FLASH_0, 0x3f000, flg, 1);
		flash_read(&FLASH_0, 0x3f000, check, 1);
	}
}

void FLASH_CLOCK_init(void)
{
	hri_mclk_set_AHBMASK_NVMCTRL_bit(MCLK);
}

void FLASH_init(void)
{
	FLASH_CLOCK_init();
	flash_init(&FLASH_0, NVMCTRL);
}

uint8_t JumpFun(uint8_t *tempData)
{
	static uint8_t num=0;
	switch (num)
	{
		case 0: if(*tempData==0x03) num=1;
				else num=0;           break;
		case 1: if(*tempData==0x01) num=2;
				else num=0;			  break;
		case 2: if(*tempData==0x01) num=3;
				else num=0;			  break;
		case 3: if(*tempData==0x00) num=4;
				else num=0;			  break;
		case 4: if(*tempData==0xE7) num=5;
				else num=0;			  break;
		
		default: num=0;				  break;
	}
	if(num==5)
	{
		num=0;
		FLASH_init();
		FLASH_flg();
		return 1;
	}
	return 0;
}





