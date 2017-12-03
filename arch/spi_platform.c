/*
 * Copyright 2011-2013 Ayla Networks, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Ayla Networks, Inc.
 */
#include <string.h>
#include <ayla/mcu_platform.h>
#include "mcu_io.h"
#include "stm32f10x.h"
#include "spi_platform_arch.h"
#include <func.h>

#ifdef USER_SPI2

#ifdef USER_SPI2_RX_IRQ

void NVIC_Configuration_spi2(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}

void SPI1_IRQHandler(void)
{
  if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)
  {
//    spi_rx = SPI2->DR;
  }
}

#endif

void spi_platform_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable GPIOB clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
#ifdef USER_SPI2_RX_IRQ
	NVIC_Configuration_spi2();
#endif	
	/* Enable the SPI2 Pins Software Remapping */


	/*	PB12:	SSN
		PB13:	SCK
		PB14:	MISO
		PB15:	MOSI	*/
		
	/* Configure PB12,13,14,15 pins for SPI2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Configure SPI2 for Master mode */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Clear flags and any RX data */
	SPI_I2S_ReceiveData(SPI2);
	SPI_I2S_GetFlagStatus(SPI2, 0x00);

	SPI_Cmd(SPI2, ENABLE);

}

void spi_platform_intr_init(void)
{
	;
}

/*
 * Select slave
 */
void spi_platform_slave_select(void)
{
	GP_SPI_GPIO->BRR = bit(GP_SPI_NSS);
}

void spi_platform_out(u8 byte)
{
	SPI_TypeDef *spi = SPI2;

	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	SPI_I2S_SendData(SPI2, byte);
}

u8 spi_platform_in(void)
{
	SPI_TypeDef *spi = SPI2;
	u16 sr;

	for (;;) {
		sr = spi->SR;
		if (sr & SPI_I2S_FLAG_OVR) {
			(void)spi->DR;
			(void)spi->SR;
		}
		if (sr & SPI_I2S_FLAG_RXNE) {
			return SPI_I2S_ReceiveData(SPI2);
		}
	}
}

#endif

#ifdef USER_SPI1

#ifdef USER_SPI1_RX_IRQ

void NVIC_Configuration_spi1(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}

void SPI1_IRQHandler(void)
{
  if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)
  {
//    spi_rx = SPI1->DR;
  }
}

#endif

void spi_platform_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable GPIOB clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

#ifdef USER_SPI2_RX_IRQ
	void NVIC_Configuration_spi1(void);
#endif
	/* Enable the SPI2 Pins Software Remapping */

	/*	PA4:	SSN
		PA5:	SCK
		PA6:	MISO
		PA7:	MOSI	*/

	/* Configure PA4, 5, 6, 7 pins for SPI1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable the SPI clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Configure SPI1 for Master mode */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Clear flags and any RX data */
	SPI_I2S_ReceiveData(SPI1);
	SPI_I2S_GetFlagStatus(SPI1, 0x00);

	SPI_Cmd(SPI1, ENABLE);

}

void spi_platform_intr_init(void)
{
	;
}

/*
 * Select slave
 */
void spi_platform_slave_select(void)
{
	GP_SPI_GPIO->BRR = bit(GP_SPI_NSS);
}

/*
 * Deselect slave
 */
void spi_platform_slave_deselect(void)
{
	GP_SPI_GPIO->BSRR = bit(GP_SPI_NSS);
}

void spi_platform_out(u8 byte)
{
	SPI_TypeDef *spi = SPI1;

	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	SPI_I2S_SendData(SPI1, byte);
}

u8 spi_platform_in(void)
{
	SPI_TypeDef *spi = SPI1;
	u16 sr;

	for (;;) {
		sr = spi->SR;
		if (sr & SPI_I2S_FLAG_OVR) {
			(void)spi->DR;
			(void)spi->SR;
		}
		if (sr & SPI_I2S_FLAG_RXNE) {
			return SPI_I2S_ReceiveData(SPI1);
		}
	}
}

#endif
