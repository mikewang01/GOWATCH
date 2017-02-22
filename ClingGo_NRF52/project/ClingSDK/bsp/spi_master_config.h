/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup spi_master_example_main spi_master_config
 * @{
 * @ingroup spi_master_example
 *
 * @brief Defintions needed for the SPI master driver.
 */
 
#ifndef SPI_MASTER_CONFIG_H__
#define SPI_MASTER_CONFIG_H__

#define SPI_OPERATING_FREQUENCY (0x02000000uL << (uint32_t)Freq_8Mbps)  /**< Slave clock frequency. */

// SPI0. 
#define SPI_PSELSCK1            GPIO_SPI_1_SCK                                     /**< SPI clock GPIO pin number. */
#define SPI_PSELMOSI1           GPIO_SPI_1_MOSI                                     /**< SPI Master Out Slave In GPIO pin number. */
#define SPI_PSELMISO1           GPIO_SPI_1_MISO                                     /**< SPI Master In Slave Out GPIO pin number. */
#define SPI_PSELSS1_PPG         GPIO_SPI_1_CS_PPG                               /**< SPI Slave Select GPIO pin number. */

// 
#define SPI_PSELSCK0            GPIO_SPI_0_SCK                                     /**< SPI clock GPIO pin number. */
#define SPI_PSELMOSI0           GPIO_SPI_0_MOSI                                     /**< SPI Master Out Slave In GPIO pin number. */
#define SPI_PSELMISO0           GPIO_SPI_0_MISO                                     /**< SPI Master In Slave Out GPIO pin number. */
#define SPI_PSELSS0_OLED        GPIO_SPI_0_CS_OLED                                     /**< SPI Slave Select GPIO pin number. */
#define SPI_PSELSS0_ACC         GPIO_SPI_0_CS_ACC                                     /**< SPI Slave Select GPIO pin number. */
#define SPI_PSELSS0_NFLASH      GPIO_SPI_0_CS_NFLASH

#define TIMEOUT_COUNTER         0x3000uL                                /**< Timeout for SPI transaction in units of loop iterations. */

#endif // SPI_MASTER_CONFIG_H__
