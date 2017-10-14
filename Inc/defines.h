/*
 * defines.h
 *
 *  Created on: 09.10.2017
 *      Author: user
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define MAT1_COLS 48

#define ADDR_SETTEXT 0x01

#define STATE_NONE 0x00
#define STATE_RX 0x01
#define STATE_SETTEXT_XFLEN 0x10
#define STATE_SETTEXT_WAITLEN 0x11
#define STATE_SETTEXT_XFTEXT 0x12
#define STATE_SETTEXT_WAITTEXT 0x13

#endif /* DEFINES_H_ */
