/**
 * \file
 *
 * \brief User board configuration template
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define F_CPU 48000000

// Define IO pins
#define BUTT_X PIN_PB03
#define BUTT_Y PIN_PB02
#define BUTT_A PIN_PB10
#define BUTT_B PIN_PB11
#define BUTT_D PIN_PA02
#define BUTT_C PIN_PA03
#define BUTT_PROG PIN_PA08
#define SHIFT_STROBE PIN_PA27
#define SHIFT_DATA PINMUX_PB22D_SERCOM5_PAD2
#define SHIFT_CLK PINMUX_PB23D_SERCOM5_PAD3
#define LED1 PIN_PA28
#define CAN_INT PIN_PA20

inline void pin_dirset(uint32_t pinNum, uint8_t dir){
	if (dir){
		PORT->Group[pinNum/32].DIRSET.reg = 1 << (pinNum % 32);
	} else {
		PORT->Group[pinNum/32].DIRCLR.reg = 1 << (pinNum % 32);
	}
}

inline void pin_outset(uint32_t pinNum, uint8_t state){
	if (state){
		PORT->Group[pinNum/32].OUTSET.reg = 1 << (pinNum % 32);
	} else {
		PORT->Group[pinNum/32].OUTCLR.reg = 1 << (pinNum % 32);
	}
}

inline uint8_t pin_inget(uint32_t pinNum){
	return (PORT->Group[pinNum/32].IN.reg >> (pinNum % 32)) & 1;
}

inline void pin_outtgl(uint32_t pinNum){
	PORT->Group[pinNum/32].OUTTGL.reg = 1 << (pinNum % 32);
}

inline void pin_cfg(uint32_t pinNum, uint8_t inEn, uint8_t pullEn){
	PORT->Group[pinNum/32].PINCFG[pinNum % 32].reg = (inEn << PORT_PINCFG_INEN_Pos)| (pullEn << PORT_PINCFG_PULLEN_Pos);
}

static void pin_set_peripheral_function(uint32_t pinmux){
	uint8_t port = (uint8_t)((pinmux >> 16)/32);
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >> 16) & 0x01u)));
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux & 0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));
	PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
}

#endif // CONF_BOARD_H
