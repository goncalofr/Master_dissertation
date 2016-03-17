/******************************************************************************
*******************************************************************************
**
**
**                      dalf.h
**      This is the dalf.h file of the Dalf-1F v1.73 firmware, modified
**          for the ROVIM project.
**      See Dalf-1F owner's manual and the ROVIM T2D documentation for more
**          details.
**
**          The ROVIM Project
********************************************************************************
********************************************************************************/

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//                                                                            x
//           D D          AA       L         FFFFFF      H    H               x
//           D   D      A    A     L         F           H    H               x
//           D    D     AAAAAA     L         FFFF        HHHHHH               x
//           D    D     A    A     L         F           H    H               x
//           D  D       A    A     LLLLLL    F      O    H    H               x
//                                                                            x
//                                                                            x
//    <dalf.h>  -  Various definitions for the Dalf Motor Control Board       x
//                                                                            x
//  (c) Copyright 2006 Embedded Electronics LLC, All rights reserved          x
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


///////////////////////////////////////////////////////////////////////////////
//                        -- HISTORY --                                      //
//                                                                           //
//  --Date--   -Who-   ------------Description-----------------------        //
//  05/13/05    SMB    Created File                                          //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#ifndef __DALF_H
#define __DALF_H

#include <stdio.h>


// Logic
#define FALSE   0x00
#define TRUE    ~FALSE              // The truth is not out there, it is here...

// ASCII Control Chrs
#define chr_STX     0x02            // Start of API Pkt[]
#define chr_ETX     0x03            // End of API Pkt[]
#define chr_OK      0xAA            // Successful Command Code

#define HOST_NID    0x00

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xx  C O N D I T I O N A L     F L A G S   xx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
#define Dbg     0x00                // No debugging
//#define   Dbg     0x01                // Debug closed loop continuity


///////////////////////////////////////////////////////////////////////////////
//                      E R R O R     L I S T                                //
///////////////////////////////////////////////////////////////////////////////
#define NoErr       0           // Success
#define eErr        1           // Generic error
#define eParseErr   2           // Syntax (eg; unexpected buffer empty)
#define eNumArgsErr 3           // Arg count
#define eParmErr    4           // Bad parameter value
#define eModeErr    5           // Invalid operating mode
#define eFrameErr   6           // Framing Error
#define eOvRunErr   7           // OverRun Error 
#define eBuffFull   8           // Buffer Full while receiving data.
#define eProtocol   9           // Packet Protocol Error.
#define eChkSum     10          // Check Sum.
#define eTimeOut    11          // Timeout.
#define eDisable    12          // Interface disabled.

//define absolute
#define abs(x) ((x) > 0 ? (x) : -(x))

//////////////////////////////////////////////////////////////
//                    S T R U C T U R E S                   //
//////////////////////////////////////////////////////////////
typedef unsigned char   BYTE, UCHAR;        //  8 bits; [0 .. 255]
typedef unsigned int    WORD, UINT, USHORT; // 16 bits; [0 .. 65,535]
typedef unsigned long   DOUBLE, ULONG;      // 32 bits; [0 .. 4,294,967,295]
typedef long            LONG, DWORD;        // 32 bits signed;
typedef UCHAR BOOL;

/*  I/O expanders pin numbering information for Dalf board.
    -IO expander 1 is connector J5
    -IO expander 2 is connector J6
    -Female I/O connector (attached to board) pin numbering, and translation to male connector 
    pin numbering:
    ---------------------------     \   ---------------------------
    | B1 B3 B5 B7 A6 A4 A2 A0 |   ===\  | 15 13 11 09 07 05 03 01 |
    | B0 B2 B4 B6 A7 A5 A3 A1 |   ===/  | 16 14 12 10 08 06 04 02 |
    -----------     -----------     /   ------------   ------------
                                                   |   |
                                                    ---
    For real world applications, you need only to know the connector number (J5 or J6) and the pin
    number to be able to use them.
*/

typedef enum{
    J5,
    J6
}IOExpander;

typedef enum{
    OUT,
    IN
}IOPinDirection;

typedef enum{
    OFF,
    ON
}IOPinFeature;

#define IO_PIN_NAME_MAX_LEN 30  //max length for a I/O pin name, including the '\0' string
#define IOEXP_REG_BANK_OFFSET(X) (X>8?1:0) //used to access configuration pins for bank b without further logic
#define IOEXP_PIN_BIT_OFFSET(X) (X>8?16-X:X-1) //calculates the number of shifts to get the pin's bit
#define GPIOS_PER_EXPANDER 16 //the number of pins each I/O expander has


//this structures describe a pin in a real aplication in a way an unexperienced user can understand
typedef struct{
    IOExpander      exp;
    BYTE            number;
    IOPinDirection  dir;
    IOPinFeature    pullup;
    IOPinFeature    inverted;
}IOPinConfig;

typedef struct{
    char            name[IO_PIN_NAME_MAX_LEN];  //unique identifier of GPIO
    IOPinConfig     config;
}IOPinDescription;

BOOL SetGPIOConfig(const rom char* name, IOPinConfig* config);
BOOL GetGPIOConfig(const rom char* name, IOPinConfig* config);
BOOL CompareGPIOConfig(IOPinConfig* config1, IOPinConfig* config2);
BOOL SetGPIO(const rom char* name);
BOOL ResetGPIO(const rom char* name);
BOOL ToggleGPIO(const rom char* name);
BOOL GetGPIO(const rom char* name, BYTE* value);
BOOL GetAllGPIO(BYTE* J5A, BYTE* J5B, BYTE* J6A, BYTE* J6B);

extern rom const IOPinDescription* GPIOsDescription;
extern BYTE ngpios;
//-----------------------------------------------------------------------
// 16-bit Timer1 runs at 32,768 Hz and is preloaded with 0x8000 in order to 
// generate periodic 1-sec interrupts.  This timer also supports timeout
// delays specified in seconds and ticks (32,768 ticks/sec) which are
// specified in a TIME data structure. 
//-----------------------------------------------------------------------
typedef struct
{
    ULONG  secs;
    WORD ticks;
} TIME, *PTIME;

extern  BYTE    SECS, MINS, HOURS;      // RTC variables
extern  ULONG   Seconds;                // Seconds since boot 
extern  BYTE    ADC0[7];                    // ADC readings AN0..AN6
WORD    AdcConvert(WORD Adc);       // ADC reading to millivolts
//-----------------------------------------------------------------------

/* TODO: remove
//-----------------------------------------------------------------------
// External functions used to customize the Dalf firmware to run other
// applications on top of it.
//-----------------------------------------------------------------------
typedef void (*greeting)(void);
typedef BYTE (*cmdExtensionDispatch)(void);
typedef void (*serviceIO)(void);

typedef struct
{
    greeting                GreetingFct;
    cmdExtensionDispatch    CmdExtensionDispatchFct;
    serviceIO               ServiceIOFct;
}ExternalAppSupportFcts;*/

//-----------------------------------------------------------------------


////////////////////////////////////////////////////////////////////////////////
//                E X T E R N A L     F U N C T I O N S                       //
//                                                                            //
//      Many (if not all) of these functions reside in <dalf.lib>.            //
////////////////////////////////////////////////////////////////////////////////
//             ** UNCALLED LIBRARY FUNCTIONS **
extern  void    WriteIOExp1(BYTE reg, BYTE data);
extern  void    WriteIOExp2(BYTE reg, BYTE data);
extern  BYTE    ReadIOExp1(BYTE reg);
extern  BYTE    ReadIOExp2(BYTE reg);
extern  void    WriteExtEE_Byte(WORD adrs, BYTE dat);
extern  BYTE    ReadExtEE_Byte(WORD adrs);
extern  void    WriteExtEE_Block(WORD adrs, BYTE *buff, BYTE len); 
extern  void    ReadExtEE_Block(WORD adrs, BYTE *buff, BYTE len);
extern  BYTE    ReadIntEE_Byte(WORD adrs);
extern  void    WriteIntEE_Byte(WORD adrs, BYTE dat);
extern  void    WriteIntEE_Block(WORD adrs, BYTE *buff, BYTE len); 
extern  void    ReadIntEE_Block(WORD adrs, BYTE *buff, BYTE len);
extern  void    WritePot1(BYTE cmd, BYTE data);
extern  void    WritePot2(BYTE cmd, BYTE data);
extern  void    GetTime(PTIME pTime);

//            ** CALLED LIBRARY FUNCTIONS **
    // Interrupt Handlers
extern  void    TMR0_ISR(void);     // TMR0:   Heartbeat
extern  void    INT0_ISR(void);     // AB2INT: Mtr2 encoder
extern  void    INT1_ISR(void);     // AB1INT: Mtr1 encoder
extern  void    TMR1_ISR(void);     // TMR1:   RTC
extern  void    TMR2_ISR(void);     // TMR2:   ADC state machine
extern  void    INT2_ISR(void);     // I2INT:  Mtr2 current sense
extern  void    INT3_ISR(void);     // I1INT:  Mtr1 current sense
extern  void    TX1_ISR(void);      // Cmd interface
extern  void    RX1_ISR(void);      // Cmd interface
extern  void    CCP1_ISR(void);     // EX0 - R/C Channel 1 (Mtr1) Interface
extern  void    CCP2_ISR(void);     // EX1 - R/C Channel 2 (Mtr2) Interface
extern  void    CCP3_ISR(void);     // EX2 - R/C Channel 3 (Alt) Interface
extern  void    SSP2_ISR(void);     // SSP2 - Secondary I2C bus (Dalf Slave).

    // Others
extern  void    SystemInit(void);       // Power Up Initialization
extern  void    SetDelay(PTIME pTime, ULONG DelaySecs, WORD DelayTicks);
extern  int     Timeout(PTIME pTime);   // Check for delay expiration
extern  void    UpdateVelocity1(void);  // Update Mtr1 velocity
extern  void    PendingCmd1(void);      // If Mtr1 stopped, do cmd
extern  void    RampMotor1(void);       // Routine Mtr1 ramping
extern  void    Trajectory1(void);      // Generate Mtr1 waypoint
extern  void    PID1(void);             // Generate Mtr1 command
extern  void    UpdateVelocity2(void);  // Update Mtr2 velocity
extern  void    PendingCmd2(void);      // If Mtr2 stopped, do cmd
extern  void    RampMotor2(void);       // Routine Mtr2 ramping
extern  void    Trajectory2(void);      // Generate Mtr2 waypoint
extern  void    PID2(void);             // Generate Mtr2 command

extern  BYTE    TxChr(char c);          // USART1: XMIT(c)
extern  BYTE    TE_CmdParse(void);      // TE.   Parses ASCIZ string in Tx1_Buff
extern  BYTE    API_CmdParse(void);     // API.  Parses pkt data in Tx1_Buff
extern  BYTE    I2C2_CmdParse(void);    // I2C2. Parses pkt' data in I2C2_Buff
extern  BYTE    TeCmdDispatch(void);    // TE.   Cmd Dispatch
extern  BYTE    ApiCmdDispatch(void);   // API.  Cmd Dispatch
extern  BYTE    I2C2CmdDispatch(void);  // I2C2. Cmd Dispatch
extern  void    DoReset(void);          // Microcontroller reset
extern  void    DoIdle(void);           // Enter Low Power (IDLE) Mode
extern  void    Mtr1FlagFix(void);      // Fixup after OvrCrnt handled by ISR
extern  void    Mtr2FlagFix(void);      // Fixup after OvrCrnt handled by ISR


//XXX: this was moved here without though. Analyse this properly
int     printf(const rom char *fmt, ...);

////////////////////////////////////////////////////////////////////////////////
//                E X T E N D E D     F U N C T I O N S                       //
//                                                                            //
//              System functions used because of the ROVIM Project.           //
////////////////////////////////////////////////////////////////////////////////

// system functions
void SystemInitExt(void);
#ifdef WATCHDOG_ENABLED
void InitWatchdog(void);
void KickWatchdog(void);
void HardReset(void);
#endif
DWORD CalculateDelayMs(PTIME start, PTIME end);
void EmergencyStopMotors(void);
void LockCriticalResourcesAccess(void);
void UnlockCriticalResourcesAccess(void);
BOOL IsStandardCommandLocked(BYTE cmd);
BOOL IsExtendedCommandLocked(BYTE cmd);
BYTE TeCmdDispatchExt(void);
BYTE I2C2CmdDispatchExt(void);
BYTE TeProcessAck(void);
BYTE TeDisableAck(void);
BYTE TE_CmdParseExt(void);

void OpenLoopTune1(void);
void OpenLoopTune2(void);
void SoftStop(BYTE mtr);
void MoveMtrOpenLoop(BYTE mtr, BYTE dir, BYTE spd, BYTE slew);
void MoveMtrClosedLoop(BYTE mtr, short long tgt, WORD v, WORD a);

//support functions
void DEBUG_PrintCmd(void);
void SetVerbosity(BYTE level);
BYTE GetVerbosity(void);
#ifdef HELP_ENABLED
BYTE ShowHelp(void);
#endif
#ifdef LOG_ENABLED
void LOG_LogInit(void);
#endif

void Greeting(void);

// External switches used for pot control modes
#define SWITCH0 (PORTD&0x01)    // PORTD.0: On/Off
#define SWITCH1 (PORTD&0x02)    // PORTD.1: Dir (POTF only)


// Oscillator Speed Definition
#define FOSC    (D'40000000')   // define FOSC to PICmicro

//////////////////////////////////////////////////
//   I2C Device Addresses for On-board Devices  //
//////////////////////////////////////////////////
#define Dev_24LC512     0xA0    // I2C Device Adrs (Microchip 24LC512 EEPROM)
#define Dev_MCP23017_1  0x42    // I2C Device Adrs (Microchip MCP23017 I/O EXP)
#define Dev_MCP23017_2  0x40    // I2C Device Adrs (Microchip MCP23017 I/O EXP)
#define Dev_MAX5478_1   0x50    // I2C Device Adrs (MAXIM, Dual digital pot)
#define Dev_MAX5478_2   0x52    // I2C Device Adrs (MAXIM, Dual digital pot)
//          Dev_DALF    0x60    ; Dalf as slave on I2C2 bus (See Parm Block)

////////////////////////////////////////////////////
//   I2C Register Addresses for MCP23017 Devices  //
//                                                //
//   Note: Reg#s assume IOCON.BANK=0              //
////////////////////////////////////////////////////
// --- PORT A Registers --- 
#define IODIRA      0x00    // I/O Direction          ("0"=Output, "1"=Input)
#define IPOLA       0x02    // Input Polarity         ("1"=Inverted)
#define GPINTENA    0x04    // Int-On-Change Enable   ("0"=Disable, "1"=Enable)
#define DEFVALA     0x06    // Default Compare Reg    (for Int-On-Change)
#define INTCONA     0x08    // Int Control            ("0"=pin, "1"=DEFVAL)
#define IOCON       0x0A    // I/O Config (A & B)   
//#define   IOCONA      0x0A    // I/O Configuration
#define GPPUA       0x0C    // Pullup Enable          ("0"=Disable, "1"=Enable)
#define INTFA       0x0E    // Interrupt Flags        ("1"=IntRequest)
#define INTCAPA     0x10    // Interrupt Capture      (Pin states on interrupt)
#define GPIOA       0x12    // GPIO Port              (Read gives pin states)
#define OLATA       0x14    // Output Latch           (Writes drive outputs)
// --- PORT B Registers ---
#define IODIRB      0x01
#define IPOLB       0x03
#define GPINTENB    0x05
#define DEFVALB     0x07
#define INTCONB     0x09
//#define   IOCONB      0x0B    // Shadow of IOCONA
#define GPPUB       0x0D
#define INTFB       0x0F
#define INTCAPB     0x11
#define GPIOB       0x13
#define OLATB       0x15


///////////////////////////////////////////////////////////////////////
//  I2C Cmds for MAX5478 Devices (Dual Digital 50K Pots) with 256    //
//  tap points.  These are Write Only devices with NonVolatile       //
//  memory for PwrUp initialization.  There are 4 regs per wiper:    //
//                                                                   //
//  Reg             Write Effect                                     //
//  ----          ------------------------------------------------   //
//  VREG          Update wiper position only.                        //
//  NVREG         Update NVREG only (not wiper or VREG).             //
//  NVREGtoVREG   Copy NVREG to VREG & wiper.  No change to NVREG.   //
//  VREGtoNVREG   Copy VREG to NVREG.  No change to VREG or wiper.   //
//                                                                   //
//  NOTE: Apparently the copy operations don't require a data byte.  //
///////////////////////////////////////////////////////////////////////
// Wiper A 
#define VREGA           0x11 
#define NVREGA          0x21
#define NVREGtoVREGA    0x61
#define VREGtoNVREGA    0x51

// Wiper B
#define VREGB           0x12
#define NVREGB          0x22
#define NVREGtoVREGB    0x62
#define VREGtoNVREGB    0x52

// Both Wipers A & B
#define VREG            0x13
#define NVREG           0x23
#define NVREGtoVREG     0x63
#define VREGtoNVREG     0x53






////////////////////////
//    LED Patterns    //
////////////////////////
#define LED_FULLOFF 0x00000000          // Always Off
#define LED_FAST    0x55555555          // Off for 1, On for 1.
#define LED_MED     0x33333333          // Off for 2, On for 2.
#define LED_SLOW    0x0F0F0F0F          // Off for 4, On for 4. 
#define LED_FULLON  0xFFFFFFFF          // Always On

/////////////////////////
//   LED Conditions    //
/////////////////////////
#define LED_MTR_OFF     LED_FULLOFF  // S=0. ---------------------- Motor off
#define LED_MTR_TGA     LED_FAST     // S>0. TGA Active ----------- Moving, PID
#define LED_MTR_OPENLP  LED_SLOW     // S>0. TGA Inactive. V>0.---- Moving, OL
#define LED_MTR_STALL   LED_FULLON   // S>0. TGA Inactive. V=0.---- Stalled
#define LED_OVERCURRENT LED_FAST     // One or more motors overcurrent
#define LED_SIGNAL_LOSS LED_SLOW     // One or both R/C Signal Loss.
#define LED_VBATT       LED_FULLON   // Low VBATT
#define ROVIM_T2D_LED_LOCKDOWN    LED_SLOW     // ROVIM is in lockdown

#define LED_PERIOD  0x64                // 100 msec
#define LED_HYSTERESIS  500             // 500 mV Vbatt Hysteresis for red LED.

extern  BYTE    LedErr;                     // "1" bits flag err cond'n

//////////////////////////////////////
//  Macros for Fan and LED control  //
//////////////////////////////////////
//      FAN1   (F.3)
#define _FAN1_ON            LATF |=  0x08
#define _FAN1_OFF           LATF &= ~0x08
#define _FAN1_TOGGLE        LATF ^=  0x08

//      FAN2   (F.4)
#define _FAN2_ON            LATF |=  0x10
#define _FAN2_OFF           LATF &= ~0x10
#define _FAN2_TOGGLE        LATF ^=  0x10

//       LED1  (F.6)
#define _LED1_ON            LATF |=  0x40
#define _LED1_OFF           LATF &= ~0x40
#define _LED1_TOGGLE        LATF ^=  0x40

//       LED2  (F.7)
#define _LED2_ON            LATF |=  0x80
#define _LED2_OFF           LATF &= ~0x80
#define _LED2_TOGGLE        LATF ^=  0x80

//       LED3  (E.1)
#define _LED3_ON            LATE |=  0x02
#define _LED3_OFF           LATE &= ~0x02
#define _LED3_TOGGLE        LATE ^=  0x02



///////////////////////////////////////////////////
//    Delay constants for fractional seconds     //
//               in SetDelay()                   //
//                                               //
//         TMR1 tick freq of 32,768 Hz           //
///////////////////////////////////////////////////
#define usec_31     1       // 31  uSec
#define usec_305    10      // 305 uSec
#define usec_457    15      // 457 uSec

#define msec_1      33      //   1 mSec
#define msec_5      164     //   5 mSec
#define msec_10     328     //  10 mSec
#define msec_20     655     //  20 msec
#define msec_50     1638    //  50 mSec
#define msec_100    3277    // 100 mSec
#define msec_200    6554    // 200 msec
#define msec_300    9830    // 300 msec
#define msec_400    13107   // 400 msec
#define msec_500    16384   // 500 mSec
#define msec_750    24576   // 750 msec
#define sec_1       32768   // 1   sec

// TIMESVC_REQ Bits
#define Vsp1Msk     0x01    // Bit0
#define Vsp2Msk     0x02    // Bit1
#define PspMsk      0x04    // Bit2
#define RcspMsk     0x08    // Bit3
#define CmdspMsk    0x10    // Bit4 

//                      DispSvc Bits
// DispSvc0 (Low Byte)
#define CHRreqMsk       0x01    // Bit0 - xCHR request.
#define EreqMsk         0x02    // Bit1 - Motor position request.
#define VreqMsk         0x04    // Bit2 - Motor velocity request.
#define ADCreqMsk       0x08    // Bit3 - ADC readings request.
#define RCreqMsk        0x10    // Bit4 - Radio control request.
#define HMSreqMsk       0x20    // Bit5 - Hours:Mins:Secs request.
#define MEMBYTEreqMsk   0x40    // Bit6 - Memory Byte request.
#define IOBYTEreqMsk    0x80    // Bit7 - IOEXP Byte request.

// DispSvc1 (Hi Byte)
#define STATreqMsk      0x0100  // Bit8 - Motor Status request.
#define PIDreqMsk       0x0200  // Bit9 - PID Motor Parameters request.
#define RAMBLKreqMsk    0x0400  // BitA - RAM Block request.
#define EXTEEBLKreqMsk  0x0800  // BitB - External EEPROM Block request.
#define INTEEBLKreqMsk  0x1000  // BitC - Internal EEPROM Block request.
#define RESETreqMsk     0x2000  // BitD - Microcontroller RESET request.


// CmdSource Bits
#define     RC_SrcMsk       0x01    // Bit0 - R/C source for CMD/ARG[]
#define     POT_SrcMsk      0x02    // Bit1 - Pots source for CMD/ARG[]
#define     TE_SrcMsk       0x04    // Bit2 - TE source for CMD/ARG[]
#define     API_SrcMsk      0x08    // Bit3 - API source for CMD/ARG[]
#define     I2C2_SrcMsk     0x10    // Bit4 - I2C2 source for CMD/ARG[]
#define     RS232_SrcMsk    TE_SrcMsk+API_SrcMsk    // Either RS232

// ISR_Flags Bits - Set and cleared by ISR.  Read only outside ISR!
#define     OverC2Msk   0x02        // Bit1 - Mtr2 OverCurrent detected by ISR
#define     OverC1Msk   0x01        // Bit0 - Mtr1 OverCurrent detected by ISR

// I2C2_PktStatus Bits
#define     i2c2_AutoMsk    0x80    // Bit7 - I2C2 Pkt[] ready.  Xmit AutoStart
#define     i2c2_ERRMsk     0x40    // Bit6 - I2C2 Error.  Clear after reported.

// Mtr1_Flags1, Mtr2_Flags1 bits
#define     cpend_Msk       0x80    // Bit7 - Cmd pending awaiting motor stop
#define     ConstantV       0x40    // Bit6 - Constant Velocity (closed loop).
#define     OverCurrent_Msk 0x20    // Bit5 - Over current condition
#define     sumhoa_Msk      0x10    // Bit4 - PID Err Summation HoldOff active
#define     pida_Msk        0x08    // Bit3 - PID motor control active
#define     trev_Msk        0x04    // Bit2 - Traj reverse direction required
#define     tga_Msk         0x02    // Bit1 - Trajectory generator active
#define     triga_Msk       0x01    // Bit0 - Waiting for Closed Loop Trigger

// Mtrx_Flags2 bits
// Mtrx_Flags2 uses more bits than shown here (highest value caught = 0x1E).
// Q: Should I use another flag carrier, to be sure i'm not overwritting anything?
#define     OL_stepresp     0x80    // Bit7 - "1"=Measuring motor open loop step response
#define     _MtrD_mask      0x20    // Bit5 - "1"=Reverse
#define     DisableMsk      0x01    // Bit0 - "1"=Disable all Mtr commands


// SYSMODE Bits
#define     IdleMsk     0x80    // Bit7 - '1'=Enable Low Power (IDLE) Mode
#define     CmdToMsk    0x40    // Bit6 - '1'=Enable Serial Cmd Timeout

// MTRx_MODE1 (ERAM) Bits
#define fanMsk          0x40    // Bit6 - Powerup Fan State (0=Off, 1=On)
#define analogfbMsk     0x20    // Bit5 - "1"=Analog feedback (not incr encoder)
#define sumhoMsk        0x10    // Bit4 - "1" = PID errsum holdoff !MODE!
#define relMsk          0x08    // Bit3 - "1" = Target Relative mode
#define trigMsk         0x04    // Bit2 - "1" = Closed Loop Trigger Mode
#define aleadsbMsk      0x02    // Bit1 - Mtr2 Encoder:"1" if A Leads B for Fwd
#define disactiveloMsk  0x01    // Bit0 - "1" = DIS signal active low


// MTRx_MODE2 (ERAM) Bits
#define PotcMsk         0x80    // Bit7 - '1' = POT Centered mode
#define PotfMsk         0x40    // Bit6 - '1' = POT + Reversing Switch mode
#define RcNrmMsk        0x20    // Bit5 - '1' = R/C Interface mode
#define RcMixMsk        0x10    // Bit4 - '1' = R/C Mixing Active
#define RcServoMsk      0x08    // Bit3 - '1' = R/C Servo mode
#define PotServoMsk     0x04    // Bit2 - '1' = POT Servo mode
#define PotMixMsk       0x02    // Bit1 - '1' = POT Mix mode
                                // Bit0 - //UNUSED//
#define PotMsk      PotcMsk+PotfMsk+PotServoMsk+PotMixMsk
#define RcMsk       RcNrmMsk+RcMixMsk+RcServoMsk
#define ManualMsk   PotMsk+RcMsk


// MTRx_MODE3 (ERAM) Bits
                                // Bit7 - //UNUSED//
#define AnalogDirMsk    0x40    // Bit6 - "1" = Reverse analog encoder direction
#define OsmcMsk         0x20    // Bit5 - '1' = Motor PWM complement for rev
#define OcieMsk         0x10    // Bit4 - '1' = Over Current Int Enable 
#define OcFastOffMsk    0x08    // Bit3 - "1" = Immediate OFF on OverCurrent
#define VelDecMask      0x04    // Bit2 - "1" = TE disp velocity in decimal
#define PosDecMsk       0x02    // Bit1 - "1" = TE disp position in decimal
#define VerboseMsk      0x01    // Bit0 - "1" = Verbose PID output enabled.


// SCFG States
#define TEcfg       0x00        // Terminal Emulator Interface (RS232)
#define APIcfg      0x01        // Application Programming Interface (RS232)


// NewPulse Bits (Inactivity Detect)
#define NewRC1msk   0x80    // Bit7
#define NewRC2msk   0x40    // Bit6
#define NewRC3msk   0x20    // Bit5
#define NewRCmsk    (NewRC1msk | NewRC2msk | NewRC3msk)


// LedErr Bits
#define VBATTmsk    0x80        // Bit7: Low VBATT voltage
#define OC1msk      0x40        // Bit6: Over Current Mtr 1
#define OC2msk      0x20        // Bit5: Over Current Mtr 2
#define SL1msk      0x10        // Bit4: R/C SignalLoss Mtr1
#define SL2msk      0x08        // Bit3: R/C SignalLoss Mtr2
#define Lckmsk      0x04        // Bit2: ROVIM is in Lockdown


// Motor Equates
#define FORWARD     0x00    // Direction
#define REVERSE     0x01    // Direction
#define NEUTRAL     0x02    // Movement type
#define HILLHOLD    0x03    // Movement type
#define SPEEDZERO   0x00    // Speed

//Step Response
#define MAXSAMPLES 0x3e7    //999 - Maximum samples collected when measuring step response

//Command line extension definitions
#define CUSTOM_CMD_ID_OFFSET   16    //identifier of the first custom command (not bellonging to the extended dalf application), "G 'X'"

extern void (*AckCallback)(void);  //pointer to ack callback

typedef enum{
    config,
    set,
    reset,
    toggle1,
    toggle2,
    get,
    getOneAtATime1,
    getOneAtATime2,
    idle
}GPIOTestSM;

typedef enum{
    out,
    outPullup,
    outInverted,
    outPullupInverted,
    in,
    inPullup,
    inInverted,
    inPullupInverted,
    end
}GPIOConfigSM;

////////////////////////////////////////////////////
//          Other variables declarations          //
////////////////////////////////////////////////////

#ifdef WATCHDOG_ENABLED
    extern WORD watchdogcount;
#endif
extern  BYTE    CMD,ARG[16],ARGN;           // parsed command info
extern  BYTE    SCFG;                       // Serial Configuration (1..3)
extern  BYTE    Mtr2_Flags2;                // Motor2 flags2
extern  BYTE    Mtr2_Flags1;                // Motor2 flags1
extern  short long  encode1;                // Mtr1 position encoder
extern  short long  V1;                   // Mtr1 Velocity
extern  short long  V2;                     // Mtr2 Velocity
extern BYTE MTR2_MODE1, MTR2_MODE2, MTR2_MODE3;
extern BYTE VMIN1, VMAX1;
extern WORD TPR1;
extern BYTE VSP1;
extern BYTE VSP2;
extern WORD ACC2, VMID2;
extern  BYTE    S2,Power2;                  // SPD: [0..100%], [0..VMAX2%]
extern  BYTE    S1,Power1;                  // SPD: [0..100%], [0..VMAX1%]
extern  BYTE    CmdSource;

// LED variables
extern ULONG   ledshift;       // On/Off LED bit shifter (shifted bit is time period)
extern ULONG   grn1pattern;    // LED1: On/Off LED bit pattern
extern ULONG   grn2pattern;    // LED2: On/Off LED bit pattern 
extern ULONG   redpattern;     // LED3: On/Off LED bit pattern

#define TIME_TO_MSEC(x) ((x.secs*1000) + (x.ticks>>5))

//Verbosity level mask
#define VERBOSITY_DISABLED              0x00
#define VERBOSITY_LEVEL_ERROR           0x01
#define VERBOSITY_LEVEL_WARNING         0x02
#define VERBOSITY_LEVEL_STATUS          0x04
#define VERBOSITY_LEVEL_DEBUG           0x08
#define VERBOSITY_LEVEL_MSG             0x0F //Prints if any of the previous four is enabled

#define VERBOSITY_USE_CALL_INFO         0x40
#define VERBOSITY_USE_TIMESTAMP         0x80

//Verbosity print macros
#define FATAL_ERROR_MSG(ARGS)       do { \
    BYTE auxVerbosity = GetVerbosity(); \
    SetVerbosity(VERBOSITY_LEVEL_ERROR | VERBOSITY_USE_CALL_INFO | VERBOSITY_USE_TIMESTAMP); \
    PRINT_VERBOSITY_MSG("FATAL ERROR!!:\t",VERBOSITY_LEVEL_ERROR,ARGS); \
    SetVerbosity(auxVerbosity); \
} while(0)
#define ERROR_MSG(ARGS)             PRINT_VERBOSITY_MSG("ERROR!: \t",VERBOSITY_LEVEL_ERROR,ARGS)
#define WARNING_MSG(ARGS)           PRINT_VERBOSITY_MSG("WARNING:\t",VERBOSITY_LEVEL_WARNING,ARGS)
#define STATUS_MSG(ARGS)            PRINT_VERBOSITY_MSG("STATUS: \t",VERBOSITY_LEVEL_STATUS,ARGS)
#define DEBUG_MSG(ARGS)             PRINT_VERBOSITY_MSG("DEBUG:  \t",VERBOSITY_LEVEL_DEBUG,ARGS)
#define MSG(ARGS)                   PRINT_VERBOSITY_MSG("",VERBOSITY_LEVEL_MSG,ARGS)

/* prints and the test module occupy a lot of space. You may reach a point where program memory is
full. In that case, you may need to not compile some of those traces */
#ifdef REMOVE_DEBUG_PRINTS
#undefine DEBUG_MSG
#endif
#ifdef REMOVE_STATUS_PRINTS
#undefine STATUS_MSG
#endif

/*Remember that the __LINE__ macro says it is one line above the actual line on the .c, because of
the 1st line workaround; so we fix it here*/
#define PRINT_VERBOSITY_MSG(TYPE,VERBOSITY_LEVEL, ARGS) do { \
    if(SCFG != TEcfg) break;\
    if(GetVerbosity() & VERBOSITY_LEVEL) { \
        printf(TYPE); \
        if (GetVerbosity() & VERBOSITY_USE_CALL_INFO) { \
            printf("File: "__FILE__"; Line:%d:\t",(__LINE__-1)); \
        } \
        if (GetVerbosity() & VERBOSITY_USE_TIMESTAMP) { \
            printf("Elapsed Time: %lu s\t",Seconds); \
        } \
    printf(ARGS); \
    } \
} while(0)

#endif /*__DALF_H*/
