/*
 * MIDIBUS_Tangle.cpp
 *
 * Created: 26/06/2021 19:38:07
 * Author : GuavTek
 */ 


#include "samd21g15b.h"
#include <asf.h>
#include "MIDI_Config.h"
#include "MCP2517.h"
#include "MIDI_Driver.h"

MCP2517_C CAN(SERCOM1);

MIDI_C MIDI(2);

void Receive_CAN(CAN_Rx_msg_t* msg);

void MIDI2_Handler(MIDI2_voice_t* msg);
void MIDI1_Handler(MIDI1_msg_t* msg);

void RTC_Init();

void Set_Mux(uint8_t inLine, uint8_t outLine);

// Translate output number to pin levels
const uint8_t MuxOut[8][8] = {
	{0b010, 0b110, 0b011, 0b111, 0b000, 0b100, 0b001, 0b101},
	{0b010, 0b110, 0b011, 0b111, 0b000, 0b100, 0b001, 0b101},
	{0b010, 0b110, 0b011, 0b111, 0b000, 0b100, 0b001, 0b101},
	{0b010, 0b110, 0b011, 0b111, 0b000, 0b100, 0b001, 0b101},
	{0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111},
	{0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111},
	{0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111},
	{0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111}};
		
const uint8_t MuxPins[8][3] = {
	{PIN_PA08, PIN_PA07, PIN_PA06},
	{PIN_PA11, PIN_PA10, PIN_PA09},
	{PIN_PA00, PIN_PA01, PIN_PB08},
	{PIN_PA05, PIN_PA04, PIN_PB09},
	{PIN_PA12, PIN_PA14, PIN_PA13},
	{PIN_PA28, PIN_PB23, PIN_PA27},
	{PIN_PA20, PIN_PA15, PIN_PA21},
	{PIN_PA22, PIN_PB22, PIN_PA23}};
		
uint8_t buttonState;
uint8_t buttonStatePrev;
uint8_t readOffset = 0;

uint8_t buttonNum;
uint8_t buttonHold;

int main(void)
{
	system_init();
	CAN.Init(CAN_CONF, SPI_CONF);
	RTC_Init();
	
	// Set MUX controls as outputs
	PORT->Group[0].DIRSET.reg = 0x18f0fff3;
	PORT->Group[1].DIRSET.reg = 0x00c00300;
	
	// Set button group select as output
	PORT->Group[1].DIRSET.reg = 0x0000000c;
	PORT->Group[1].OUTSET.reg = 1 << 2;
	
	// Set LED as output (Disable for debugging)
	PORT->Group[0].DIRSET.reg = 1 << 31;
	PORT->Group[0].OUTCLR.reg = 1 << 31;
	
	// Enable input on button pins
	PORT->Group[0].PINCFG[2].bit.INEN = 1;
	PORT->Group[0].PINCFG[3].bit.INEN = 1;
	PORT->Group[1].PINCFG[10].bit.INEN = 1;
	PORT->Group[1].PINCFG[11].bit.INEN = 1;
	
	MIDI.Set_handler(MIDI2_Handler);
	MIDI.Set_handler(MIDI1_Handler);
	
	NVIC_EnableIRQ(SERCOM1_IRQn);
	system_interrupt_enable_global();
	
    while (1){
		CAN.State_Machine();
		
		static uint32_t buttonTimer = 0;
		if (RTC->MODE0.COUNT.reg > buttonTimer){
			buttonTimer = RTC->MODE0.COUNT.reg + 10;
			
			// Read Buttons
			uint8_t temp;
			temp = (PORT->Group[1].IN.reg & (1 << 10)) ? 0b0001 : 0;
			temp |= (PORT->Group[1].IN.reg & (1 << 11)) ? 0b0010 : 0;
			temp |= (PORT->Group[0].IN.reg & (1 << 3)) ? 0b0100 : 0;
			temp |= (PORT->Group[0].IN.reg & (1 << 2)) ? 0b1000 : 0;
			
			buttonState &= ~(0xf << readOffset);
			buttonState |= ~temp << readOffset;
			
			uint8_t diff = buttonState ^ buttonStatePrev;
			for (uint8_t i = 0; i < 4; i++){
				if (buttonState & (1 << (i + readOffset))){
					for (uint8_t j = 0; j < 8; j++){
						Set_Mux(i + readOffset, j);
					}
				}
			}
			
			buttonStatePrev = buttonState;
			
			// Switch button select
			if (readOffset){
				readOffset = 0;
				PORT->Group[1].OUTCLR.reg = 1 << 3;
				PORT->Group[1].OUTSET.reg = 1 << 2;
			} else {
				readOffset = 4;
				PORT->Group[1].OUTCLR.reg = 1 << 2;
				PORT->Group[1].OUTSET.reg = 1 << 3;
			}
			
		}
		
    }
}

void RTC_Init(){
	// Enable clock
	PM->APBAMASK.bit.RTC_ = 1;
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_ID_RTC;
	
	RTC->MODE0.READREQ.bit.RCONT = 1;
	
	RTC->MODE0.COUNT.reg = 0;
	
	RTC->MODE0.CTRL.bit.MODE = RTC_MODE0_CTRL_MODE_COUNT32_Val;
	RTC->MODE0.CTRL.bit.PRESCALER = RTC_MODE0_CTRL_PRESCALER_DIV32_Val;
	
	RTC->MODE0.CTRL.bit.ENABLE = 1;
}


void Set_Mux(uint8_t inLine, uint8_t outLine){
	for (uint8_t i = 0; i < 3; i++){
		uint8_t grp = MuxPins[outLine][i] / 32;
		uint8_t pin = MuxPins[outLine][i] - 32 * grp;
		if (MuxOut[outLine][inLine] & (1 << i)){
			PORT->Group[grp].OUTSET.reg = 1 << pin;
		} else {
			PORT->Group[grp].OUTCLR.reg = 1 << pin;
		}
	}
}

void Receive_CAN(CAN_Rx_msg_t* msg){
	uint8_t length = CAN.Get_Data_Length(msg->dataLengthCode);
	MIDI.Decode(msg->payload, length);
}

void MIDI2_Handler(MIDI2_voice_t* msg){
	
}

void MIDI1_Handler(MIDI1_msg_t* msg){
	
}

void SERCOM1_Handler(){
	CAN.Handler();
}
