/*
 * mux.cpp
 *
 * Created: 03/02/2024 20:34:05
 *  Author: GuavTek
 */ 

#include "sam.h"
#include "conf_board.h"

uint32_t muxState = 0;
uint8_t shreg_offset;

void Mux_Init(){
	//Setting the Software Reset bit to 1
	SERCOM5->SPI.CTRLA.bit.SWRST = 1;
	while(SERCOM5->SPI.CTRLA.bit.SWRST || SERCOM5->SPI.SYNCBUSY.bit.SWRST);
	
	// Enable SERCOM clock
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
	
	// Select generic clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ((GCLK_CLKCTRL_ID_SERCOM5_CORE) << GCLK_CLKCTRL_ID_Pos);

	// Set pin functions
	pin_dirset(SHIFT_STROBE, 1);
	pin_outset(SHIFT_STROBE, 1);
	pin_set_peripheral_function(SHIFT_DATA);
	pin_set_peripheral_function(SHIFT_CLK);

	// Set master mode and pad configuration
	SERCOM5->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER | SERCOM_SPI_CTRLA_DOPO(0x1)
	| SERCOM_SPI_CTRLA_DIPO(0x0) | SERCOM_SPI_CTRLA_MODE(0x0);
	
	// Set baud rate
	while(SERCOM5->SPI.SYNCBUSY.bit.CTRLB);
	SERCOM5->SPI.BAUD.reg = (F_CPU/(2 * 8000000)) - 1;
	
	//Enable SPI
	SERCOM5->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
	
	// Send dummy byte
	SERCOM5->SPI.DATA.reg = 69;
	// Wait for transfer to complete
	while(SERCOM5->SPI.INTFLAG.bit.TXC == 0);
	
}

void Mux_Set(uint8_t inLine, uint8_t outLine){
	muxState &= ~(0b111 << (3*outLine));
	muxState |= (inLine & 0b111) << (3*outLine);
}

void Mux_Update(){
	pin_outset(SHIFT_STROBE, 0);
	shreg_offset = 0;
	SERCOM5->SPI.INTENSET.bit.DRE = 1;
}

void SERCOM5_Handler(){
	uint32_t active_ints;
	active_ints = SERCOM5->SPI.INTFLAG.reg;
	active_ints &= SERCOM5->SPI.INTENSET.reg;
	SERCOM5->SPI.INTFLAG.reg = active_ints;
	if (active_ints & SERCOM_SPI_INTFLAG_DRE){
		SERCOM5->SPI.DATA.reg = (muxState >> shreg_offset) & 0xff;
		shreg_offset += 8;
		if (shreg_offset >= 24){
			SERCOM5->SPI.INTENCLR.bit.DRE = 1;
			SERCOM5->SPI.INTENSET.bit.TXC = 1;
		}
	}
	if (active_ints & SERCOM_SPI_INTENSET_TXC){
		SERCOM5->SPI.INTENCLR.bit.TXC = 1;
		pin_outset(SHIFT_STROBE, 1);
	}
}
