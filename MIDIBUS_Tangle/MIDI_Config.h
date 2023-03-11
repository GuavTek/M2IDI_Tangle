/*
 * MIDI_Config.h
 *
 * Created: 15/10/2021 22:31:05
 *  Author: GuavTek
 */ 

// Configurations for MIDI application

#ifndef MIDI_CONFIG_H_
#define MIDI_CONFIG_H_

#include "MCP2517.h"

uint32_t midiID = 123;

// Define CAN filters
CAN_Filter_t CAN_FLT0 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = false,
	.ID = (0 << 10),
	.matchBothIDTypes = false,
	.maskID = 1 << 10
};

CAN_Filter_t CAN_FLT1 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = false,
	.ID = (1 << 10) | (midiID & 0x3ff),
	.matchBothIDTypes = false,
	.maskID = 0x7ff
};

// Define FIFO configurations
CAN_FIFO_t CAN_FIFO1 = {
	.enabled = true,
	.payloadSize = 16,
	.fifoDepth = 31,
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

CAN_FIFO_t CAN_FIFO2 = {
	.enabled = true,
	.payloadSize = 16,
	.fifoDepth = 31,
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


const spi_config_t SPI_CONF = {
	.sercomNum = 1,
	.dipoVal = 0x0,
	.dopoVal = 0x1,
	.speed = 8000000,
	.pin_cs = PIN_PA17,
	.pinmux_mosi = PINMUX_PA18C_SERCOM1_PAD2,
	.pinmux_miso = PINMUX_PA16C_SERCOM1_PAD0,
	.pinmux_sck = PINMUX_PA19C_SERCOM1_PAD3
};

const CAN_Config_t CAN_CONF = {
	.rxMethod = CAN_Config_t::CAN_Rx_RTC,
	.interruptPin = 0,
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