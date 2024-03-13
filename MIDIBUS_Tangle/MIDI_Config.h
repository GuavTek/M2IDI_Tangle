/*
 * MIDI_Config.h
 *
 * Created: 15/10/2021 22:31:05
 *  Author: GuavTek
 */ 

// Configurations for MIDI application

#ifndef MIDI_CONFIG_H_
#define MIDI_CONFIG_H_

#include "SPI_SAMD.h"
#include "MCP2517.h"
#include "eeprom_cat.h"

// Define eeprom configuration
const eeprom_cat_conf_t EEPROM_CONF = {
	.comSlaveNum = 0,
	.maxAddr = 0x1fff
};

const eeprom_cat_section_t EEPROM_SECTIONS[2] = {
	{	// Header, 320 byte
		.offset = 0x0000,
		.objectSize = 8 // Contains 40 chunks
	},
	{	// Main
		.offset = 0x0140,
		.objectSize = 3 // sizeof(muxstate)
	}
};

// Define CAN filters
// Module will need stream type to define function blocks,
// Sysex types for capability exchange, voice types for program change messages

// Regular input. MIDI 1.0 channel (0x2), sysex 7-bit (0x3)
const CAN_Filter_t CAN_FLT0 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = false,
	.ID = 0b0010 << 7,
	.matchBothIDTypes = false,
	.maskID = 0b1110 << 7
};

// Regular input. MIDI 2.0 channel (0x4), sysex 8-bit (0x5)
const CAN_Filter_t CAN_FLT1 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = false,
	.ID = 0b0100 << 7,
	.matchBothIDTypes = false,
	.maskID = 0b1110 << 7
};

// Regular input. Stream data (0xf)
const CAN_Filter_t CAN_FLT2 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = false,
	.ID = 0b1111 << 7,
	.matchBothIDTypes = false,
	.maskID = 0b1111 << 7
};

// Targeted input. Match messages with extended CAN id
const CAN_Filter_t CAN_FLT3 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = true,
	.ID = 69,	// Temporary value, will be changed in runtime
	.matchBothIDTypes = false,
	.maskID = 0x07f
};

// Define FIFO configurations
// Rx FIFO
const CAN_FIFO_t CAN_FIFO1 = {
	.enabled = true,
	.payloadSize = 64,
	.fifoDepth = 18,
	.retransmitAttempt = CAN_FIFO_t::unlimited,
	.messagePriority = 0,
	.txEnable = false,
	.autoRemote = false,
	.receiveTimestamp = false,
	.exhaustedTxInterrupt = false,
	.overflowInterrupt = false,
	.fullEmptyInterrupt = false,
	.halfFullEmptyInterrupt = false,
	.notFullEmptyInterrupt = true
};

// Tx FIFO
const CAN_FIFO_t CAN_FIFO2 = {
	.enabled = true,
	.payloadSize = 64,
	.fifoDepth = 8,
	.retransmitAttempt = CAN_FIFO_t::unlimited,
	.messagePriority = 0,
	.txEnable = true,
	.autoRemote = false,
	.receiveTimestamp = false,
	.exhaustedTxInterrupt = false,
	.overflowInterrupt = false,
	.fullEmptyInterrupt = false,
	.halfFullEmptyInterrupt = false,
	.notFullEmptyInterrupt = false
};


const spi_config_t SPI_CAN_CONF = {
	.sercomNum = 1,
	.dipoVal = 0x0,
	.dopoVal = 0x1,
	.speed = 8000000,
	.pinmux_mosi = PINMUX_PA18C_SERCOM1_PAD2,
	.pinmux_miso = PINMUX_PA16C_SERCOM1_PAD0,
	.pinmux_sck = PINMUX_PA19C_SERCOM1_PAD3,
	.num_cs = 1,
	.pin_cs = {PIN_PA17}
};

const spi_config_t SPI_MEM_CONF = {
	.sercomNum = 2,
	.dipoVal = 0x0,
	.dopoVal = 0x1,
	.speed = 8000000,
	.pinmux_mosi = PINMUX_PA14C_SERCOM2_PAD2,
	.pinmux_miso = PINMUX_PA12C_SERCOM2_PAD0,
	.pinmux_sck = PINMUX_PA15C_SERCOM2_PAD3,
	.num_cs = 1,
	.pin_cs = {PIN_PA13}
};

const CAN_Config_t CAN_CONF = {
	.comSlaveNum = 0,
	.clkOutDiv = CAN_Config_t::clkOutDiv1,
	.sysClkDiv = false,
	.clkDisable = false,
	.pllEnable = false,
	.txBandwidthShare = CAN_Config_t::BW_Share4,
	.opMode = CAN_MODE_E::Normal_FD,
	.txQueueEnable = false,
	.txEventStore = false,
	.listenOnlyOnError = false,
	.restrictRetransmit = false,
	.disableBitrateSwitch = false,
	.wakeFilter = CAN_Config_t::Wake_Filter_40_75ns,
	.enableWakeFilter = false,
	.disableProtocolException = false,
	.enableIsoCrc = true,
	.deviceNetFilterBits = 0,
	.nominalBaudPrescaler = 1,	// Time quanta prescaler Tq = 2/20MHz = 100ns
	.nominalTSEG1 = 6,			// Time segment 1 = 7 Tq
	.nominalTSEG2 = 1,			// Time segment 2 = 2 Tq
	.nominalSyncJump = 0,		// Sync jump width = 1 Tq
	.dataBaudPrescaler = 0,		// Time quanta Tq = 1/20MHz = 50ns
	.dataTSEG1 = 1,				// 2 Tq
	.dataTSEG2 = 1,				// 2 Tq
	.dataSyncJump = 0,			// 1 Tq
	.enableEdgeFilter = true,
	.enableSID11 = false,
	.txDelayCompensation = CAN_Config_t::TdcAuto,
	.txDelayCompOffset = 0,
	.enableIntInvalidMessage = false,
	.enableIntWakeup = false,
	.enableIntBusError = false,
	.enableIntSysError = false,
	.enableIntRxOverflow = false,
	.enableIntTxAttempt = false,
	.enableIntCrcError = false,
	.enableIntEccError = false,
	.enableIntTxEvent = false,
	.enableIntModeChange = false,
	.enableIntTimebaseCount = false,
	.enableIntRxFifo = true,
	.enableIntTxFifo = false
};


#endif /* MIDI_CONFIG_H_ */