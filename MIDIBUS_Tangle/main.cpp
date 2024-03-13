/*
 * MIDIBUS_Tangle.cpp
 *
 * Created: 26/06/2021 19:38:07
 * Author : GuavTek
 */ 


#include "sam.h"
#include <asf.h>
#include "MIDI_Config.h"
#include "SPI_SAMD.h"
#include "MCP2517.h"
#include "eeprom_cat.h"
#include "AM_MIDI2/include/umpProcessor.h"
#include "AM_MIDI2/include/utils.h"
#include "mux.h"
#include "conf_board.h"

SPI_SAMD_C SPI_CAN(SERCOM1);
MCP2517_C CAN(&SPI_CAN);

SPI_SAMD_C SPI_MEM(SERCOM2);
eeprom_cat_c EEPROM(&SPI_MEM);

uint32_t midiID;
bool rerollID = false;
umpProcessor MIDI;

void Button_Handler();

void Receive_CAN(CAN_Rx_msg_t* msg);
void Receive_CAN_Payload(char* data, uint8_t length);
void Check_CAN_Int();

void MIDI_CVM_Handler(struct umpCVM msg);
void MIDI_Stream_Discovery(uint8_t majVer, uint8_t minVer, uint8_t filter);
void MIDI_Data_Handler(struct umpData msg);

void RTC_Init();

void mem_cb();
void Scan_EEPROM();
void Load_Profile(uint8_t program, uint16_t bank, uint8_t channel);
void Save_Profile(uint8_t program, uint16_t bank, uint8_t channel);

uint8_t buttonState;
uint8_t buttonStatePrev;
uint8_t readOffset = 0;

uint8_t buttonNum;
uint8_t buttonHold;

Settings_t currentSettings;
uint16_t memBanks[20];
uint8_t memChans[20];
char memBuff[8];
uint8_t headPend[5];
struct {
	bool wr : 1;
	bool rd : 1;
	bool pendHead : 1;
} memState;
uint8_t validBanks;
uint8_t nextBank;	// If a bank must be overwritten

enum SysState_t{
	idle,
	firstButt,
	secondButt,
	held,
	waitMIDI
}sysState;
uint8_t firstButton;

int main(void){
	system_init();
	
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
	
	// Initialize Hardware drivers
	SPI_CAN.Init(SPI_CAN_CONF);
	SPI_MEM.Init(SPI_MEM_CONF);
	Mux_Init();
	RTC_Init();
	
	// Enable interrupts
	NVIC_EnableIRQ(SERCOM1_IRQn);
	NVIC_EnableIRQ(SERCOM2_IRQn);
	NVIC_EnableIRQ(SERCOM5_IRQn);
	system_interrupt_enable_global();
	
	// Initialize high-level drivers
	EEPROM.init(EEPROM_CONF, EEPROM_SECTIONS, 2);
	EEPROM.set_callback(mem_cb);
		
	CAN.Set_Rx_Header_Callback(Receive_CAN);
	CAN.Set_Rx_Data_Callback(Receive_CAN_Payload);
	CAN.Init(CAN_CONF);
	
	MIDI.setCVM(MIDI_CVM_Handler);
	MIDI.setSysEx(MIDI_Data_Handler);
	MIDI.setMidiEndpoint(MIDI_Stream_Discovery);
	
	// Randomize ID
	midiID = rand();
	rerollID = true;
	
	// Scan memory
	Scan_EEPROM();
	Load_Profile(0, memBanks[0], memChans[0]);
	Mux_Update();
	
    while (1){
		
		if (rerollID){
			rerollID = false;
			CAN_Filter_t tempFilt = CAN_FLT3;
			tempFilt.ID = midiID & 0x7f;
			CAN.Reconfigure_Filter(&tempFilt, 3);
		}
		
		static uint32_t buttonTimer = 0;
		if (RTC->MODE0.COUNT.reg > buttonTimer){
			buttonTimer = RTC->MODE0.COUNT.reg + 10;
			Button_Handler();
		}
		
		if (CAN.Ready()){
			Check_CAN_Int();
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

void Button_Handler(){
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
				// Rising edge
				buttonHold = 0;
				if (sysState == SysState_t::firstButt){
					// Second press
					sysState = SysState_t::idle;
					Mux_Set(buttonNum, i + readOffset);
					Mux_Update();
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
	if (readOffset > 0){
		readOffset = 0;
		pin_outset(BUTT_X, 0);
		pin_outset(BUTT_Y, 1);
	} else {
		readOffset = 4;
		pin_outset(BUTT_X, 1);
		pin_outset(BUTT_Y, 0);
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

void Check_CAN_Int(){
	if (!pin_inget(CAN_INT)){
		CAN.Check_Rx();
	}
}

void mem_cb(){
	if (memState.wr){
		memState.wr = 0;
		// Nuthin?
	}
	if (memState.rd){
		memState.rd = 0;
		// Load config into muxes
		muxState = memBuff[0] | (memBuff[1] << 8) | (memBuff[2] << 16);
		Mux_Update();
	}
	if (memState.pendHead){
		// Figure out which part of the header has changed
		int8_t index;
		index = -1;
		for (uint8_t i = 0; i < 5; i++){
			if (headPend[i]){
				index = i;
				break;
			}
		}
		if (index == -1){
			memState.pendHead = 0;
			return;
		}
		for (uint8_t i = 0; i < 8; i++){
			if (headPend[index] & (1 << i)){
				headPend[index] &= ~(1 << i);
				index = index*8 + i;
				break;
			}
		}
		if (index >= 20){
			uint8_t memIndex = index - 20;
			memBuff[0] = memBanks[memIndex] & 0xff;
			memBuff[1] = (memBanks[memIndex] >> 8) & 0xff;
			memBuff[2] = memChans[memIndex];
			memBuff[7] = 69;
			EEPROM.write_data(&memBuff[0], 0, index);
		} else if (index == 1){
			memBuff[0] = nextBank;
			EEPROM.write_data(&memBuff[0], 0, 1);
		} else if (index == 2){
			memBuff[0] = currentSettings.group;
			EEPROM.write_data(&memBuff[0], 0, 2);
		}
	}
}

void Scan_EEPROM(){
	char temp[8];
	memState.pendHead = 0;
	memState.rd = 0;
	memState.wr = 0;
	// Read version
	EEPROM.read_data(temp, 0, 0);
	while (SPI_MEM.Get_Status() != Idle);
	if (temp[7] != 69){
		// Initialize settings
		temp[0] = 1;
		temp[7] = 69;
		EEPROM.write_data(temp, 0, 0);
		while (SPI_MEM.Get_Status() != Idle);
		
		nextBank = 0;
		for (uint8_t i = 0; i < 8; i++){
			temp[i] = 0;
		}
		EEPROM.write_data(temp, 0, 1);
		while (SPI_MEM.Get_Status() != Idle);
		
		currentSettings.group = 200;
		temp[0] = 200;
		for (uint8_t i = 1; i < 8; i++){
			temp[i] = 0;
		}
		EEPROM.write_data(temp, 0, 2);
		while (SPI_MEM.Get_Status() != Idle);
		
		// Nothing saved
		return;
	}
	
	EEPROM.read_data(temp, 0, 1);
	while (SPI_MEM.Get_Status() != Idle);
	nextBank = temp[0];
	
	EEPROM.read_data(temp, 0, 2);
	while (SPI_MEM.Get_Status() != Idle);
	currentSettings.group = temp[0];
	
	// Check which banks are saved
	for (uint8_t i = 0; i < 20; i++){
		EEPROM.read_data(temp, 0, 20+i);
		while (SPI_MEM.Get_Status() != Idle);
		if (temp[7] == 69){
			validBanks++;
			memBanks[i] = temp[0] | (temp[1] << 8);
			memChans[i] = temp[2];
		} else {
			break;
		}
	}
}

void Load_Profile(uint8_t program, uint16_t bank, uint8_t channel){
	// Check if config exists
	int8_t index = -1;
	for (uint8_t i = 0; i < validBanks; i++){
		if ((bank == memBanks[i]) && (channel == memChans[i])){
			index = i;
			break;
		}
	}
	if (index == -1){
		// Nope
		return;
	}
	
	// Load config from eeprom
	memState.rd = 1;
	EEPROM.read_data(&memBuff[0], 1, index*384+program);
}

void Save_Profile(uint8_t program, uint16_t bank, uint8_t channel){
	// Check if bank is in use
	int8_t index = -1;
	for (uint8_t i = 0; i < validBanks; i++){
		if ((bank == memBanks[i]) && (channel == memChans[i])){
			index = i;
			break;
		}
	}
	if (index == -1){
		if (validBanks < 20){
			// Save new bank
			index = validBanks;
			memBanks[index] = bank;
			memChans[index] = channel;
			uint8_t temp = 20 + index;
			headPend[temp/8] |= 1 << (temp % 8);
			memState.pendHead = 1;
			validBanks++;
		} else {
			// Overwrite oldest bank
			index = nextBank;
			memBanks[index] = bank;
			memChans[index] = channel;
			uint8_t temp = 20 + index - 1;
			headPend[temp/8] |= 1 << (temp % 8);
			headPend[0] |= 1 << 1;
			memState.pendHead = 1;
			nextBank++;
		}
	}
	
	// Write data to EEPROM chip
	memBuff[0] = muxState & 0xff;
	memBuff[1] = (muxState >>  8) & 0xff;
	memBuff[2] = (muxState >> 16) & 0xff;
	EEPROM.write_data(&memBuff[0], 1, index*384+program);
}

void Receive_CAN(CAN_Rx_msg_t* msg){
	if (!msg->extendedID && ((msg->id & 0x7F) == (midiID & 0x7F))){
		// Received an undirected message using the same ID. Reconfigure.
		rerollID = true;
		midiID = rand();
	}
}

void Receive_CAN_Payload(char* data, uint8_t length){
	for (uint8_t i = 0; i < length-3; i += 4){
		uint32_t temp = (data[i] << 24) | (data[i+1] << 16) | (data[i+2] << 8) | data[i+3];
		MIDI.processUMP(temp);
	}
}

void MIDI_CVM_Handler(struct umpCVM msg){
	if (msg.umpGroup != currentSettings.group){
		if (currentSettings.group != 200){
			return;
		}
	}
	if (msg.status == PROGRAM_CHANGE){
		if (msg.flag1){
			// BankValid
			currentSettings.bank = msg.bank;
		}
		if (sysState == SysState_t::waitMIDI){
			if (currentSettings.group == 200){
				currentSettings.group = msg.umpGroup;
				headPend[0] |= 1 << 2;
				memState.pendHead = 1;
			}
			Save_Profile(msg.value, currentSettings.bank, msg.channel);
			sysState = SysState_t::idle;
		} else {
			Load_Profile(msg.value, currentSettings.bank, msg.channel);
		}
	}
}

void MIDI_Stream_Discovery(uint8_t majVer, uint8_t minVer, uint8_t filter){
	/* TODO
	//Upon Recieving the filter it is important to return the information requested
	if(filter & 0x1){ //Endpoint Info Notification
		std::array<uint32_t, 4> ump = mtFMidiEndpointInfoNotify(3, false, true, false, false);
		sendUMP(ump.data(),4);
	}

	if(filter & 0x2) {
		std::array<uint32_t, 4> ump = mtFMidiEndpointDeviceInfoNotify({DEVICE_MFRID}, {DEVICE_FAMID}, {DEVICE_MODELID}, {DEVICE_VERSIONID});
		sendUMP( ump.data(), 4);
	}

	if(filter & 0x4) {
		uint8_t friendlyNameLength = strlen(DEVICE_MIDIENPOINTNAME);
		for(uint8_t offset=0; offset<friendlyNameLength; offset+=14) {
			std::array<uint32_t, 4> ump = mtFMidiEndpointTextNotify(MIDIENDPOINT_NAME_NOTIFICATION, offset, (uint8_t *) DEVICE_MIDIENPOINTNAME,friendlyNameLength);
			sendUMP(ump.data(),4);
		}
	}
	
	if(filter & 0x8) {
		int8_t piiLength = strlen(PRODUCT_INSTANCE_ID);

		for(uint8_t offset=0; offset<piiLength; offset+=14) {
			std::array<uint32_t, 4> ump = mtFMidiEndpointTextNotify(PRODUCT_INSTANCE_ID, offset, (uint8_t *) buff,piiLength);
			sendUMP(ump.data(),4);
		}
	}
	
	if(filter & 0x10){
		std::array<uint32_t, 4> ump = mtFNotifyProtocol(0x1,false,false);
		sendUMP(ump.data(),4);
	}
	*/
}

void MIDI_Data_Handler(struct umpData msg){
	// TODO: implement Capability exchange features
}

void SERCOM1_Handler(){
	SPI_CAN.Handler();
}

void SERCOM2_Handler(){
	SPI_MEM.Handler();
}
