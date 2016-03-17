//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//                                                                            x
//      CCCCC    OOOO     NN   N    FFFFF   IIIII    GGGGG       H    H       x
//     C        O    O    N N  N    F         I     G            H    H       x
//     C        O    O    N  N N    FFFF      I     G   GG       HHHHHH       x
//     C        O    O    N   NN    F         I     G    G       H    H       x
//      CCCCC    OOOO     N    N    F       IIIII    GGGG    O   H    H       x
//                                                                            x
//                                                                            x
//    <config.h>  -  "Fuse" settings the Dalf Motor Control Board             x
//                                                                            x
//  (c) Copyright 2006 Embedded Electronics LLC, All rights reserved          x
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


// ********************************
// *** Set configuration bits  ****
// ********************************
//#pragma config	OSC = INTIO7		// Internal Osc, OSC2=ClockOut, OSC1=RA7
#pragma	config	OSC = EC			// External Oscillator.  OSC2 is Clock Out.
#pragma config	FCMEN = OFF			// Fail-Safe Clock Monitor disabled.
#pragma config	IESO = OFF			// Int/Ext Osc Switchover disabled.
#pragma	config	PWRT = ON			// Powerup Timer On.
#pragma	config	BOREN = OFF			// Brown out reset Off
#pragma	config	BORV = 0     		// Brown out reset voltage 4.5V
#pragma	config	WDT = OFF			// Watch Dog disabled - it will be activated at run time via InitWatchdog()
#pragma	config	WDTPS = 512   	    // Watch Dog timeout= WDTPS * 4 ms
#pragma	config	MCLRE = ON			// MCLR# enabled.
#pragma	config	LPT1OSC = OFF		// Low Power Timer1 Operation disabled.
#pragma	config	CCP2MX = PORTE		// CCP2 on RE7
#pragma	config	STVREN = ON			// Stack Overflow Reset enabled.
#pragma	config	LVP = OFF			// Low Voltage ICSP off
#pragma	config	BBSIZ = BB2K		// Boot Block Size
#pragma	config	XINST = OFF			// Don't install extended instruct set.
#pragma	config	DEBUG = ON			// Remote debug enabled


// Code Protect: [0x18000...0x1FFFF]
#pragma	config	CP0 = OFF
#pragma	config	CP1 = OFF
#pragma	config	CP2 = OFF
#pragma	config	CP3 = OFF
#pragma	config	CP4 = OFF
#pragma	config	CP5 = OFF
#pragma	config	CP6 = OFF
#pragma	config	CP7 = OFF
#pragma	config	CPB = OFF			// -- boot block
#pragma	config	CPD = OFF			// -- eeprom

// Write Protection Off (all banks)
#pragma	config	WRT0 = OFF
#pragma	config	WRT1 = OFF
#pragma	config	WRT2 = OFF
#pragma	config	WRT3 = OFF
#pragma	config	WRT4 = OFF
#pragma	config	WRT5 = OFF
#pragma	config	WRT6 = OFF
#pragma	config	WRT7 = OFF
#pragma	config	WRTB = OFF			// -- boot block
#pragma	config	WRTD = OFF			// -- eeprom

// External Blk Tbl Reads Permitted (all banks) ------------------Investigate This with Bootloader!!!
#pragma	config	EBTR0 = OFF
#pragma	config	EBTR1 = OFF
#pragma	config	EBTR2 = OFF
#pragma	config	EBTR3 = OFF
#pragma	config	EBTR4 = OFF
#pragma	config	EBTR5 = OFF
#pragma	config	EBTR6 = OFF
#pragma	config	EBTR7 = OFF
#pragma	config	EBTRB = OFF			// -- boot block


