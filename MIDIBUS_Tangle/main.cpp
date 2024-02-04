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
#include "rww_eeprom.h"

MCP2517_C CAN(SERCOM1);

MIDI_C MIDI(2);

struct Settings_t {
	union{
		uint32_t word;
		struct{
			uint8_t group;
			uint8_t channel;
			uint16_t bank;
		};
	};
};

struct Profile_t {
	union{
		uint32_t word;
		struct{
			uint8_t program;
			uint32_t muxState : 24;
		};
	};
};

void Receive_CAN(CAN_Rx_msg_t* msg);

void MIDI2_Handler(MIDI2_voice_t* msg);
void MIDI1_Handler(MIDI1_msg_t* msg);

void RTC_Init();

void Set_Mux(uint8_t inLine, uint8_t outLine);
void NVM_Init();
void Scan_EEPROM();
Profile_t Load_Profile(uint8_t index);
void Load_EEPROM(uint8_t index);
void Save_EEPROM(uint8_t index);
void Clear_EEPROM(uint8_t index);
void Save_Settings();

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

uint8_t eeprom_buff[RWW_EEPROM_PAGE_SIZE];
uint8_t pageNum;
	
uint32_t muxState = 0;
uint16_t savedProfiles[128*4/RWW_EEPROM_PAGE_SIZE];
		
uint8_t buttonState;
uint8_t buttonStatePrev;
uint8_t readOffset = 0;

uint8_t buttonNum;
uint8_t buttonHold;

Settings_t currentSettings;
uint16_t currentBank = 0;	// The Bank being accessed by MIDI

enum SysState_t{
	idle,
	firstButt,
	secondButt,
	held,
	waitMIDI
	}sysState;
uint8_t firstButton;

int main(void)
{
	system_init();
	CAN.Init(CAN_CONF, SPI_CONF);
	CAN.Set_Rx_Callback(Receive_CAN);
	RTC_Init();
	
	
	// Set button group select as output
	pin_dirset(BUTT_X, 1);
	pin_dirset(BUTT_Y, 1);
	pin_outset(BUTT_Y, 1);
	
	// Set LED as output
	pin_dirset(LED1, 1);
	pin_outset(LED1, 0);
	
	// Enable input on button pins
	pin_cfg(BUTT_A, 1, 0);
	pin_cfg(BUTT_B, 1, 0);
	pin_cfg(BUTT_C, 1, 0);
	pin_cfg(BUTT_D, 1, 0);
	
	// Enable input and pullup for CAN_INT pin
	pin_cfg(CAN_INT, 1, 1);
	
	NVM_Init();
	
	Scan_EEPROM();
	Load_EEPROM(0);
	
	
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
			temp = pin_inget(BUTT_A) ? 0 : 0b0001;
			temp |= pin_inget(BUTT_B) ? 0 : 0b0010;
			temp |= pin_inget(BUTT_C) ? 0 : 0b0100;
			temp |= pin_inget(BUTT_D) ? 0 : 0b1000;
			
			buttonState &= ~(0xf << readOffset);
			buttonState |= (temp & 0xf) << readOffset;
			
			for (uint8_t i = 0; i < 4; i++){
				if (buttonState & (1 << (i + readOffset))){
					if (!(buttonStatePrev & (1 << (i + readOffset)))){
						pin_outset(LED1, 1);
						// Rising edge
						buttonHold = 0;
						if (sysState == SysState_t::firstButt){
							// Second press
							sysState = SysState_t::idle;
							Set_Mux(buttonNum, i + readOffset);
							buttonNum = 32;
						} else {
							// First press
							sysState = SysState_t::firstButt;
							buttonNum = i + readOffset;	
						}
					}
					if ((i + readOffset) == buttonNum){
						if (buttonHold >= 200){
							sysState = SysState_t::waitMIDI;
						} else {
							// Hold timer
							buttonHold++;
						}
						
					}
				}
			}
			
			buttonStatePrev = buttonState;
			
			// Switch button select
			if (readOffset){
				readOffset = 0;
				pin_outset(BUTT_X, 0);
				pin_outset(BUTT_Y, 1);
			} else {
				readOffset = 4;
				pin_outset(BUTT_X, 1);
				pin_outset(BUTT_Y, 0);
			}
			
		}
		
		static uint32_t ledTimer = 0;
		if (sysState == SysState_t::firstButt){
			pin_outset(LED1, 1);
		} else if (sysState == SysState_t::waitMIDI){
			if (ledTimer <= RTC->MODE0.COUNT.reg){
				ledTimer = RTC->MODE0.COUNT.reg + 200;
				pin_outtgl(LED1);
			}
		} else {
			pin_outset(LED1, 0);
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
	muxState &= ~(0b111 << (3*outLine));
	muxState |= (inLine & 0b111) << (3*outLine);
}

void NVM_Init(){
	// Set 1 wait states
	//NVMCTRL->CTRLB.bit.RWS = 1;
	//NVMCTRL->CTRLB.bit.READMODE = 0x1;
	
    /* Setup EEPROM emulator service */
    enum status_code error_code = rww_eeprom_emulator_init();
    if (error_code == STATUS_ERR_NO_MEMORY) {
        while (true) {
            /* No EEPROM section has been set in the device's fuses */
			pin_outset(LED1, 1);
        }
    }
    else if (error_code != STATUS_OK) {
        /* Erase the emulated EEPROM memory (assume it is unformatted or
         * irrecoverably corrupt) */
        rww_eeprom_emulator_erase_memory();
        rww_eeprom_emulator_init();
    }
	
	rww_eeprom_emulator_read_page(0, eeprom_buff);
	pageNum = 0;
}

void Scan_EEPROM(){
	if (pageNum != 0){
		rww_eeprom_emulator_read_page(0, eeprom_buff);
		pageNum = 0;
	}
	
	// Initialize settings
	if (eeprom_buff[RWW_EEPROM_PAGE_SIZE-1] != 69){
		currentSettings.channel = 1;
		currentSettings.bank = 0;
		currentSettings.group = 1;
		Save_Settings();
	} else {
		currentSettings.group = eeprom_buff[0];
		currentSettings.channel = eeprom_buff[1];
		currentSettings.bank = (eeprom_buff[3] << 8) | eeprom_buff[2];
	}
	
	for (uint8_t i = 0; i < 128; i++){
		Profile_t temp = Load_Profile(i);
		
		if (temp.program == i){
			uint16_t tempPage = ((4*i) / RWW_EEPROM_PAGE_SIZE);
			uint8_t tempIndex = i - tempPage * (RWW_EEPROM_PAGE_SIZE/4);
			savedProfiles[tempPage] |= 1 << tempIndex;
		}
	}
	
}

Profile_t Load_Profile(uint8_t index){
	uint16_t tempPage = ((4*index) / RWW_EEPROM_PAGE_SIZE) + 1;
	if (tempPage != pageNum){
		rww_eeprom_emulator_read_page(tempPage, eeprom_buff);
		pageNum = tempPage;
		savedProfiles[tempPage-1] = 0;
	}
	uint8_t tempOffset = 4*index - (tempPage-1)*RWW_EEPROM_PAGE_SIZE;
	Profile_t temp;
	temp.muxState = (eeprom_buff[tempOffset + 2] << 16) | (eeprom_buff[tempOffset + 1] << 8) | eeprom_buff[tempOffset];
	temp.program = eeprom_buff[tempOffset + 3];
	return temp;
}

void Load_EEPROM(uint8_t index){
	uint8_t tempPage = ((4*index) / RWW_EEPROM_PAGE_SIZE);
	uint8_t tempIndex = index - tempPage * (RWW_EEPROM_PAGE_SIZE/4);
	if (savedProfiles[tempPage] & (1 << tempIndex)){
		Profile_t temp = Load_Profile(index);
		
		muxState = temp.muxState;
		for(uint8_t i = 0; i < 8; i++){
			uint8_t source;
			source = (muxState >> (3*i)) & 0b111;
			Set_Mux(source, i);
		}
	}
}

void Save_EEPROM(uint8_t index){
	uint16_t tempPage = ((4*index) / RWW_EEPROM_PAGE_SIZE) + 1;
	if (tempPage != pageNum){
		rww_eeprom_emulator_read_page(tempPage, eeprom_buff);
		pageNum = tempPage;
		savedProfiles[tempPage-1] = 0;
	}
	uint8_t tempOffset = 4*index - (tempPage-1)*RWW_EEPROM_PAGE_SIZE;
	
	eeprom_buff[tempOffset] = muxState & 0xff;
	eeprom_buff[tempOffset + 1] = (muxState >> 8) & 0xff;
	eeprom_buff[tempOffset + 2] = (muxState >> 16) & 0xff;
	eeprom_buff[tempOffset + 3] = index;
	
	rww_eeprom_emulator_write_page(tempPage, eeprom_buff);
	rww_eeprom_emulator_commit_page_buffer();
	
	tempPage = ((4*index) / RWW_EEPROM_PAGE_SIZE);
	uint8_t tempIndex = index - tempPage * (RWW_EEPROM_PAGE_SIZE/4);
	savedProfiles[tempPage] |= 1 << tempIndex;
	
	sysState = SysState_t::idle;
}

void Save_Settings(){
	if (pageNum != 0){
		rww_eeprom_emulator_read_page(0, eeprom_buff);
		pageNum = 0;
	}
	
	eeprom_buff[RWW_EEPROM_PAGE_SIZE-1] = 69;
	eeprom_buff[0] = currentSettings.group;
	eeprom_buff[1] = currentSettings.channel;
	eeprom_buff[2] = currentSettings.bank & 0xff;
	eeprom_buff[3] = currentSettings.bank >> 8;
	
	rww_eeprom_emulator_write_page(0, eeprom_buff);
	rww_eeprom_emulator_commit_page_buffer();
}

void Receive_CAN(CAN_Rx_msg_t* msg){
	uint8_t length = CAN.Get_Data_Length(msg->dataLengthCode);
	MIDI.Decode(msg->payload, length);
}

void MIDI2_Handler(MIDI2_voice_t* msg){
	if ((msg->group != currentSettings.group) || (msg->channel != currentSettings.channel)){
		return;
	}
	if (msg->status == MIDI2_VOICE_E::ProgChange){
		if (msg->options & 1){
			currentBank = msg->bankPC;
		}
		if (currentBank == currentSettings.bank){
			if (sysState == SysState_t::waitMIDI){
				Save_EEPROM(msg->program);
			} else {
				Load_EEPROM(msg->program);
			}
		}
	}
}

void MIDI1_Handler(MIDI1_msg_t* msg){
	Settings_t msgSettings;
	msgSettings.group = msg->group;
	msgSettings.channel = msg->channel;
	msgSettings.bank = currentBank;
	if (msg->status == MIDI1_STATUS_E::ProgChange){
		if (msgSettings.word == currentSettings.word){
			if (sysState == SysState_t::waitMIDI){
				Save_EEPROM(msg->instrument);
			} else {
				Load_EEPROM(msg->instrument);
			}
		}
	} else if (msg->status == MIDI1_STATUS_E::CControl){
		// Change bank???
		
	}
}

void SERCOM1_Handler(){
	CAN.Handler();
}
