#include "main.h"
#include "spidev_hal.h"
#include "nflash_spi.h"

uint8_t g_spi_tx_buf[10];
uint8_t g_spi_rx_buf[10];

BOOLEAN b_flash_PD_flag;
#define  spi_master_hw_instance_t int
#define SPI_MASTER_0 0 
#define GPIO_SPI_0_CS_NFLASH 0
static void flash_tx_rx(spi_master_hw_instance_t   spi_master_instance,
                          I8U * tx_data_buf, I16U tx_data_size,
                          I8U * rx_data_buf, I16U rx_data_size, I8U pin_cs)
{
	CLASS(SpiDevHal)*p = SpiDevHal_get_instance();
	spi_dev_handle_t hdl  = p->flash_open(p);
	if(hdl  != NULL){
			p->write_read(p, hdl, tx_data_buf, tx_data_size, rx_data_buf, rx_data_size);
	}
	p->close(p, hdl);
}
/*----------------------------------------------------------------------------------
*  Function:	void NOR_init(void)
*
*  Description:
*	Init the SPI interface
*
*----------------------------------------------------------------------------------*/
void NOR_init(void)
{
#ifndef _CLING_PC_SIMULATION_
	I8U buf[10];
	
	// Power up the chip
	NOR_powerDown();
	b_flash_PD_flag = TRUE;
	
	// Read ID
	NOR_readID((I8U *)buf);
	
	cling.whoami.nor[0] = buf[0];
	cling.whoami.nor[1] = buf[1];
#endif
}

void _wait_for_operation_completed()
{
#ifndef _CLING_PC_SIMULATION_
	I32U count = 0;
	I8U ret;
	
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}
	// Maximum delay 2 seconds
	for (count = 0; count < 2000; count ++) {
		ret = NOR_readStatusRegister();
		
		if (ret & SPI_FLASH_WIP) {
			BASE_delay_msec(1);
		} else {
			break;
		}
	}
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	void NOR_writeEnable(void)
*
*  Description:
*	send the write enable command (0x06), all write or erase operation must send
*  the command firstly
*
*----------------------------------------------------------------------------------*/
void NOR_writeEnable(void)
{	
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	g_spi_tx_buf[0] = SPI_FLASH_INS_WREN;
#ifndef _CLING_PC_SIMULATION_
	N_SPRINTF("[spi] 1624");
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 1 ,g_spi_rx_buf,0, GPIO_SPI_0_CS_NFLASH);
#endif
	_wait_for_operation_completed();
}

void NOR_writeDisable(void)
{
    g_spi_tx_buf[0] = SPI_FLASH_INS_WRDI;
#ifndef _CLING_PC_SIMULATION_
		N_SPRINTF("[spi] 123");
    flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 1, g_spi_rx_buf,0, GPIO_SPI_0_CS_NFLASH);
#endif
}
/*----------------------------------------------------------------------------------
*  Function:	I8U NOR_readStatusRegister(void)
*
*  Description:
*	read status register (0x04), return the status register value
*
*----------------------------------------------------------------------------------*/
I8U NOR_readStatusRegister(void)
{
    g_spi_tx_buf[0] = SPI_FLASH_INS_RDSR;
    g_spi_tx_buf[1] = SPI_FLASH_INS_DUMMY;
#ifndef _CLING_PC_SIMULATION_
    flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 2, g_spi_rx_buf,  2,GPIO_SPI_0_CS_NFLASH);
#endif   
    return g_spi_rx_buf[1];
}
/*----------------------------------------------------------------------------------
*  Function:	void NOR_readData(I32U addr, I16U len, I8U *dataBuf)
*
*  Description:
*	read the len's data to dataBuf
*  addr: the 24 bit address
*  len: read length
*  dataBuf: the out data buf
*
*----------------------------------------------------------------------------------*/
void NOR_readData(I32U addr, I16U len, I8U *dataBuf)
{
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}
	g_spi_tx_buf[0] = SPI_FLASH_INS_READ;
	g_spi_tx_buf[1] = (I8U)(addr >> 16);
	g_spi_tx_buf[2] = (I8U)(addr >> 8);
	g_spi_tx_buf[3] = (I8U)(addr);
	
	N_SPRINTF("[NFLASH] %x %x %x %x", g_spi_tx_buf[0], g_spi_tx_buf[1], g_spi_tx_buf[2], g_spi_tx_buf[3]);
	I32U rx_data[65];

#ifndef _CLING_PC_SIMULATION_
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 4, (I8U *)rx_data,len +4 , GPIO_SPI_0_CS_NFLASH);
	memcpy(dataBuf,&rx_data[1],len);
#endif
	
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
		  b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}
/*----------------------------------------------------------------------------------
*  Function:	void NOR_pageProgram(I32U addr, I16U len, I8U *data)
*
*  Description:
*	read the len's data to dataBuf
*  addr: the 24 bit address
*  len: write length, must <=256
*  data: the write data buf
*
*----------------------------------------------------------------------------------*/
static void _page_program_core(I32U addr, I16U len, I8U *data)
{
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

		NOR_writeEnable();
		
		g_spi_tx_buf[0] = SPI_FLASH_INS_PP;
		g_spi_tx_buf[1] = (uint8_t)(addr >> 16);
		g_spi_tx_buf[2] = (uint8_t)(addr >> 8);
		g_spi_tx_buf[3] = (uint8_t)(addr);
	  uint8_t tx_data[65*4];
	  memcpy(tx_data,g_spi_tx_buf,4);
	  memcpy(&tx_data[4],data,len);	
	
#ifndef _CLING_PC_SIMULATION_
		flash_tx_rx(SPI_MASTER_0, (I8U *)tx_data, len + 4, g_spi_rx_buf,0, GPIO_SPI_0_CS_NFLASH);
#endif
		_wait_for_operation_completed();
	
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
	  	b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}

void NOR_pageProgram(I32U addr, I16U len, I8U *data)
{
	I32U start_addr = addr;
	I32U end_addr = addr + len - 1;
	I32U length_1, length_2;
	
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	//
	// Note, page size is 256 bytes, and it can only be programmed within the page.
	//
	// if we cross the page boundary, we have to seperate it into two parts.
	//
	start_addr &= 0xffffff00;
	end_addr   &= 0xffffff00;

	if (start_addr == end_addr) {
		_page_program_core(addr, len, data);
	} else {
		// First page
		length_1 = 256 - (addr - start_addr);
		_page_program_core(addr, length_1, data);
		
		// Second page
		addr += length_1;
		length_2 = len - length_1;
		_page_program_core(addr, length_2, data+length_1);
	}
	
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
		  b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}
/*----------------------------------------------------------------------------------
*  Function:	NOR_erase_block_4k(I32U addr)
*
*  Description:
*	erase the sector(4K Bytes)
*  addr: the 24 bit address
*
*
*----------------------------------------------------------------------------------*/
void NOR_erase_block_4k(I32U addr)
{		
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	NOR_writeEnable();

	g_spi_tx_buf[0] = SPI_FLASH_INS_SE_4K;
	g_spi_tx_buf[1] = (uint8_t)(addr >> 16);
	g_spi_tx_buf[2] = (uint8_t)(addr >> 8);
	g_spi_tx_buf[3] = (uint8_t)(addr);
	
	N_SPRINTF("[NFLASH] %x %x %x %x", g_spi_tx_buf[0], g_spi_tx_buf[1], g_spi_tx_buf[2], g_spi_tx_buf[3]);
#ifndef _CLING_PC_SIMULATION_
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 4, g_spi_rx_buf,  0, GPIO_SPI_0_CS_NFLASH);
#endif
	//wait for the operation finished
	_wait_for_operation_completed();
	
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
	  	b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}

/*----------------------------------------------------------------------------------
*  Function:	NOR_erase_block_32k(I32U addr)
*
*  Description:
*	erase the 32K block
*  addr: the 24 bit address
*
*
*----------------------------------------------------------------------------------*/
void NOR_erase_block_32k(I32U addr)
{	
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	NOR_writeEnable();

	g_spi_tx_buf[0] = (uint8_t)SPI_FLASH_INS_SE_32K;
	g_spi_tx_buf[1] = (uint8_t)(addr >> 16);
	g_spi_tx_buf[2] = (uint8_t)(addr >> 8);
	g_spi_tx_buf[3] = (uint8_t)(addr);
	N_SPRINTF("[NFLASH] %x %x %x %x", g_spi_tx_buf[0], g_spi_tx_buf[1], g_spi_tx_buf[2], g_spi_tx_buf[3]);
#ifndef _CLING_PC_SIMULATION_
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 4, g_spi_rx_buf,  0, GPIO_SPI_0_CS_NFLASH);
#endif
	//wait for the operation finished
	_wait_for_operation_completed();
	
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
		  b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}
/*----------------------------------------------------------------------------------
*  Function:	NOR_erase_block_64k(I32U addr)
*
*  Description:
*	erase the 64K block
*  addr: the 24 bit address
*
*----------------------------------------------------------------------------------*/
void NOR_erase_block_64k(I32U addr)
{	
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	NOR_writeEnable();
    
	g_spi_tx_buf[0] = SPI_FLASH_INS_SE_64K;
	g_spi_tx_buf[1] = (uint8_t)(addr >> 16);
	g_spi_tx_buf[2] = (uint8_t)(addr >> 8);
	g_spi_tx_buf[3] = (uint8_t)(addr);
	N_SPRINTF("[NFLASH] %x %x %x %x", g_spi_tx_buf[0], g_spi_tx_buf[1], g_spi_tx_buf[2], g_spi_tx_buf[3]);
#ifndef _CLING_PC_SIMULATION_
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 4, g_spi_rx_buf,  0, GPIO_SPI_0_CS_NFLASH);
#endif
	N_SPRINTF("[NFLASH] waiting ...");
	//wait for the operation finished
	_wait_for_operation_completed();
	N_SPRINTF("[NFLASH] completed!");
	
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
		  b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}
/*----------------------------------------------------------------------------------
*  Function:	NOR_ChipErase
*
*  Description:
*	erase the 64K block
*
*----------------------------------------------------------------------------------*/
void NOR_ChipErase()
{
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	NOR_writeEnable();

	g_spi_tx_buf[0] = SPI_FLASH_INS_BE;

#ifndef _CLING_PC_SIMULATION_
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 1,  g_spi_rx_buf,  0, GPIO_SPI_0_CS_NFLASH);
#endif
	//wait for the operation finished
	_wait_for_operation_completed();
	
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
	  	b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}
/*----------------------------------------------------------------------------------
*  Function:	void NOR_powerDown()
*
*  Description:
*  power down the nor flash to save the power consumption
*
*----------------------------------------------------------------------------------*/
void NOR_powerDown()
{
    g_spi_tx_buf[0] = SPI_FLASH_INS_DP;

#ifndef _CLING_PC_SIMULATION_
    flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 1, g_spi_rx_buf,0,GPIO_SPI_0_CS_NFLASH);
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	void NOR_releasePowerDown()
*
*  Description:
*  exit the power down mode for normal operation
*
*----------------------------------------------------------------------------------*/
void NOR_releasePowerDown()
{
    g_spi_tx_buf[0] = SPI_FLASH_INS_RES;

#ifndef _CLING_PC_SIMULATION_
    flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 1,g_spi_rx_buf,  0, GPIO_SPI_0_CS_NFLASH);
#endif
}
/*----------------------------------------------------------------------------------
*  Function:	void NOR_readID(I8U *id)
*
*  Description:
*  read the manufacture ID and device ID
*	id: the array of I8U, the length >2
*
*----------------------------------------------------------------------------------*/
void NOR_readID(I8U *id)
{
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	g_spi_tx_buf[0] = SPI_FLASH_INS_RDID;

#ifndef _CLING_PC_SIMULATION_
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 1, g_spi_rx_buf,  6, GPIO_SPI_0_CS_NFLASH);
#endif
	*id = g_spi_rx_buf[4];
	*(id+1) = g_spi_rx_buf[5];

	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
		  b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}
/*----------------------------------------------------------------------------------
*  Function:	void NOR_readUID(I8U *id)
*
*  Description:
*  read the unique ID
*	id: the array of I8U, the length >8
*
*----------------------------------------------------------------------------------*/
void NOR_readUID(I8U *id)
{
	if (b_flash_PD_flag) {
		b_flash_PD_flag = FALSE;
		NOR_releasePowerDown();
	}

	g_spi_tx_buf[0] = (uint8_t)0x4B;
	g_spi_tx_buf[1] = (uint8_t)0;
	g_spi_tx_buf[2] = (uint8_t)0;
	g_spi_tx_buf[3] = (uint8_t)0;
	g_spi_tx_buf[4] = (uint8_t)0;
#ifndef _CLING_PC_SIMULATION_
	flash_tx_rx(SPI_MASTER_0, g_spi_tx_buf, 5,  id,  10, GPIO_SPI_0_CS_NFLASH);
#endif
	if (!OTA_if_enabled()){
	  if (!b_flash_PD_flag) {
		  b_flash_PD_flag = TRUE;
		  NOR_powerDown();
	  }
  }
}
