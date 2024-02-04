/*
 * mux.h
 *
 * Created: 03/02/2024 20:34:18
 *  Author: GuavTek
 */ 


#ifndef MUX_H_
#define MUX_H_

extern uint32_t muxState;

// Initialize hardware to control multiplexers
void Mux_Init();

// Select a mux connection between inLine and outLine connectors
void Mux_Set(uint8_t inLine, uint8_t outLine);

// Update the shift registers to change mux connections
void Mux_Update();


#endif /* MUX_H_ */