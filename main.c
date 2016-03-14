#include <p33Fxxxx.h>
#include <string.h>

//
//  DMCC motorDriver
//
//  Copyright (C) 2016 by Exadler Technologies Inc., All Rights Reserved.
//
//  Custom modified version to use the power limits for the PID control
//  if the power is set to something other than 0
//
//  Portions of this code are copyright by Microchip and may only be
//  used on their devices.
//
// THIS SOFTWARE IS PROVIDED IN AN 'AS IS' CONDITION. NO WARRANTIES,
// WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO,
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE
// FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
//
// Mk.7 Pinout MAP
//
//    RC4 - U2_INA
//    RC5 - U2_INB
//
//    RC2 - U2_ENA/DIAGA
//    RC3 - U2_ENB/DIAGB
//
//    PWM1H2 - U2_PWM
//
//    AN5 - U2_CS
//
//    RA0 - U1_INA
//    RA1 - U1_INB
//
//    RA2 - U1_ENA/DIAGA
//    RA3 - U1_ENB/DIAGB
//
//    PWM1H1 - U1_PWM
//
//    AN4 - U1_CS
//
//    RA4 - EEPROM WP
//    RA8 - EEPROM A0
//    RA9 - EEPROM A1
//
//    AN6 - Motor VCC Supply Voltage/11
//
//    RC6/RP22 - QEI1A
//    RC7/RP23 - QEI1B
//
//    RC8/RP24 - QEI2A
//    RC9/RP25 - QEI2B
//
//    New for Mk.7
//    RB5/RP5 - Status LED RED
//    RB6/RP6 - Status LED GREEN
//





// Configuration bits
_FOSCSEL(FNOSC_FRCPLL)			// clock is internal OSC

_FOSC(OSCIOFNC_OFF & POSCMD_NONE)	// no clock output, external OSC disabled

_FWDT(FWDTEN_OFF)			// disable watchdog timer

_FICD(JTAGEN_OFF & ICS_PGD1)		// disable JTAG, enable debugging


//Functions prototype

void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void);

extern unsigned char RAMBuffer[256];	//RAM area which will work as EEPROM for Master I2C device

struct FlagType
{
	unsigned char AddrFlag:1;
	unsigned char DataFlag:1;
};
extern struct FlagType Flag;

#define I2C_BASE_ADDR (0x2C)


#define POWER_MOTOR_1 (0x02)
#define POWER_MOTOR_1_MSB (0x03)
#define POWER_MOTOR_2 (0x04)
#define POWER_MOTOR_2_MSB (0x05)

#define MOTOR_SUPPLY_VOLTAGE_LSB (0x06)
#define MOTOR_SUPPLY_VOLTAGE_MSB (0x07)

#define PID_LIMIT_1 (0x08)
#define PID_LIMIT_1_MSB (0x09)

#define PID_LIMIT_2 (0x0a)
#define PID_LIMIT_2_MSB (0x0b)


#define QEI1_LSB (0x10)
#define QEI1_MSB (0x11)
#define QEI2_LSB (0x14)
#define QEI2_MSB (0x15)

#define VELOCITY_MOTOR_1 (0x18)
#define VELOCITY_MOTOR_2 (0x1A)

#define CURRENT_MOTOR_1_LSB (0x1c)
#define CURRENT_MOTOR_1_MSB (0x1d)
#define CURRENT_MOTOR_2_LSB (0x1e)
#define CURRENT_MOTOR_2_MSB (0x1f)

#define PID_POS1 (0x20)
#define PID_POS2 (0x24)
#define PID_VEL1 (0x28)
#define PID_VEL2 (0x2A)

#define PID_POS1_KP (0x30)
#define PID_POS1_KI (0x32)
#define PID_POS1_KD (0x34)

#define PID_VEL1_KP (0x36)
#define PID_VEL1_KI (0x38)
#define PID_VEL1_KD (0x3A)

#define PID_POS2_KP (0x40)
#define PID_POS2_KI (0x42)
#define PID_POS2_KD (0x44)

#define PID_VEL2_KP (0x46)
#define PID_VEL2_KI (0x48)
#define PID_VEL2_KD (0x4A)

#define CAPEID (0xE0)

#define PTR_WORD_POWER_MOTOR_1 ((int *) &RAMBuffer[POWER_MOTOR_1])
#define PTR_WORD_POWER_MOTOR_2 ((int *) &RAMBuffer[POWER_MOTOR_2])

#define PTR_DWORD_QEI1 ((unsigned long *) &RAMBuffer[QEI1_LSB])
#define PTR_DWORD_QEI2 ((unsigned long *) &RAMBuffer[QEI2_LSB])

#define PTR_DWORD_POS1 ((unsigned long *) &RAMBuffer[PID_POS1])
#define PTR_DWORD_POS2 ((unsigned long *) &RAMBuffer[PID_POS2])

#define PTR_WORD_VEL1 ((int *) &RAMBuffer[PID_VEL1])
#define PTR_WORD_VEL2 ((int *) &RAMBuffer[PID_VEL2])

#define PTR_WORD_PID_POS1_KP ((int *) &RAMBuffer[PID_POS1_KP])
#define PTR_WORD_PID_POS1_KI ((int *) &RAMBuffer[PID_POS1_KI])
#define PTR_WORD_PID_POS1_KD ((int *) &RAMBuffer[PID_POS1_KD])

#define PTR_WORD_PID_VEL1_KP ((int *) &RAMBuffer[PID_VEL1_KP])
#define PTR_WORD_PID_VEL1_KI ((int *) &RAMBuffer[PID_VEL1_KI])
#define PTR_WORD_PID_VEL1_KD ((int *) &RAMBuffer[PID_VEL1_KD])

#define PTR_WORD_PID_POS2_KP ((int *) &RAMBuffer[PID_POS2_KP])
#define PTR_WORD_PID_POS2_KI ((int *) &RAMBuffer[PID_POS2_KI])
#define PTR_WORD_PID_POS2_KD ((int *) &RAMBuffer[PID_POS2_KD])

#define PTR_WORD_PID_VEL2_KP ((int *) &RAMBuffer[PID_VEL2_KP])
#define PTR_WORD_PID_VEL2_KI ((int *) &RAMBuffer[PID_VEL2_KI])
#define PTR_WORD_PID_VEL2_KD ((int *) &RAMBuffer[PID_VEL2_KD])


// A/D adjustments to convert raw readings to voltagess
#define SUPPLY_VOLTAGE_SLOPE (1096)
#define SUPPLY_VOLTAGE_OFFSET (6)

#define POWER_MIN (-10000)
#define POWER_MAX (10000)

#define PWM_MAX (4444)

#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define MIN(x,y) (((x) > (y)) ? (y) : (x))

#define MAKEWORD(x) ((x >= 32767) ? 0x7fff : ((x < -32767) ? -0x7fff : x ))


unsigned char RAMBuffer[256];	//RAM area which will work as EEPROM for Master I2C device
unsigned char *RAMPtr;			//Pointer to RAM memory locations
unsigned char RAMindex = 0;
struct FlagType Flag;

int *pMotorPower;


unsigned int motor1Current = 0;
unsigned int motor2Current = 0;
unsigned int motorSupplyVoltage = 0;

int invertMotor1 = 0;
int invertMotor2 = 0;


unsigned int lastQEI1 = 0;
unsigned int lastQEI2 = 0;

int v1 = 0;
int v2 = 0;

unsigned int t1count = 0;

//
// Define PID modes
//  0 = PID off (motor power control only)
//  1 = PID position
//  2 = PID velocity
//
int PID1Mode = 0;
int PID2Mode = 0;

int e1_0 = 0;
int e1_1 = 0;
int e1_2 = 0;
int e1_3 = 0;

long int delta1 = 0;
long int sum1 = 0;

int e2_0 = 0;
int e2_1 = 0;
int e2_2 = 0;
int e2_3 = 0;

long int delta2 = 0;
long int sum2 = 0;


#define USE_I2C_Clock_Stretch


// Define ADC Buffers
// Taken from CE120_ADC Microchip sample code
// Probably don't need all these buffers
//
#define  MAX_CHNUM	13	// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	8	// Size of the input buffer per analog input
#define  NUM_CHS2SCAN	4	// Number of channels enabled for channel scan

// Number of locations for ADC buffer = 14 (AN0 to AN13) x 8 = 112 words
// Align the buffer to 128 words or 256 bytes. This is needed for peripheral indirect mode
int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));

void resetPID(int motor) {
    if (motor & 1) {
        e1_0 = 0;
        e1_1 = 0;
        e1_2 = 0;
        e1_3 = 0;
        sum1 = 0;
    }
    if (motor & 2) {
        e2_0 = 0;
        e2_1 = 0;
        e2_2 = 0;
        e2_3 = 0;
        sum2 = 0;
    }
}




/*****************************************************************
		Init I2C1 Bus
*****************************************************************/
void i2c1_init(int i2cAddr) {
#if !defined(USE_I2C_Clock_Stretch)
    I2C1CON = 0x8000; //Enable I2C1 module
#else
    I2C1CON = 0x9040; //Enable I2C1 module, enable clock stretching
#endif


    I2C1ADD = i2cAddr; // 7-bit I2C slave address must be initialised here.

    IFS1 = 0;
    RAMPtr = &RAMBuffer[0]; //set the RAM pointer and points to beginning of RAMBuffer
    RAMindex = 0;
    Flag.AddrFlag = 0; //Initlize AddFlag
    Flag.DataFlag = 0; //Initlize DataFlag


    // Enable interrupts on I2C1
    _SI2C1IE = 1;
}

void setMotorDir(int motorNumber, int dir)
{
    if (motorNumber == 1) {
        if (invertMotor1) {
            dir = -dir;
        }
        if (dir == 1) {
            // Motor 1 forward
            PORTA = (PORTA & 0xfff0) | 0x00d;
        } else if (dir == -1) {
            // Motor 1 reverse
            PORTA = (PORTA & 0xfff0) | 0x00e;
        } else if (dir == 0) {
            // Motor 1 Brake to GND
            PORTA = (PORTA & 0xfff0) | 0x00c;
        }

    } else if (motorNumber == 2) {
        if (invertMotor2) {
            dir = -dir;
        }
        if (dir == 1) {
            // Motor 2 forward
            PORTC = (PORTC & 0xffc3) | 0x01c;
        } else if (dir == -1) {
            // Motor 2 reverse
            PORTC = (PORTC & 0xffc3) | 0x02c;
        } else if (dir == 0) {
            // Motor 2 Brake to GND
            PORTC = (PORTC & 0xffc3) | 0x00c;
        }
    }
}

void setMotorPower(int motorNumber, int power)
{
    long int pwmDuty;

    if (power < 0) {
        pwmDuty = (((long int) (-MAX(power,POWER_MIN))) * PWM_MAX) / POWER_MAX;
        if (motorNumber == 2) {
            P1DC2 = (int) pwmDuty;
            setMotorDir(2, -1);
        } else if (motorNumber == 1) {
            P1DC1 = (int) pwmDuty;
            setMotorDir(1, -1);
        }
    } else if (power > 0) {
        pwmDuty = (((long int) (MIN(power,POWER_MAX))) * PWM_MAX) / POWER_MAX;
        if (motorNumber == 2) {
            P1DC2 = (int) pwmDuty;
            setMotorDir(2, 1);
        } else if (motorNumber == 1) {
            P1DC1 = (int) pwmDuty;
            setMotorDir(1, 1);
        }
    } else {
        if (motorNumber == 2) {
            setMotorDir(2,0);
            P1DC2 = 0;
        } else if (motorNumber == 1){
            setMotorDir(1,0);
        }
    }
}


void updateRAMData(void)
{
    unsigned int temp;
    temp = POS1CNT;
    RAMBuffer[QEI1_LSB] = temp & 0x00ff;
    RAMBuffer[QEI1_MSB] = (temp >> 8) & 0x00ff;

    temp = POS2CNT;
    RAMBuffer[QEI2_LSB] = temp & 0x00ff;
    RAMBuffer[QEI2_MSB] = (temp >> 8) & 0x00ff;

    temp = motor1Current;
    RAMBuffer[CURRENT_MOTOR_1_LSB] = temp & 0x00ff;
    RAMBuffer[CURRENT_MOTOR_1_MSB] = (temp >> 8) & 0x00ff;

    temp = motor2Current;
    RAMBuffer[CURRENT_MOTOR_2_LSB] = temp & 0x00ff;
    RAMBuffer[CURRENT_MOTOR_2_MSB] = (temp >> 8) & 0x00ff;

    temp = motorSupplyVoltage + SUPPLY_VOLTAGE_OFFSET;
    // Adjust A/D to voltage
    unsigned long ulTemp;
    ulTemp = __builtin_mulss(10000,temp);
    temp = __builtin_divud(ulTemp , SUPPLY_VOLTAGE_SLOPE);
    RAMBuffer[MOTOR_SUPPLY_VOLTAGE_LSB] = temp & 0x00ff;
    RAMBuffer[MOTOR_SUPPLY_VOLTAGE_MSB] = (temp >> 8) & 0x00ff;

    temp = v1;
    unsigned int *pTemp;
    pTemp = (unsigned int *)&RAMBuffer[VELOCITY_MOTOR_1];
    *pTemp = temp;

    temp = v2;
    pTemp = (unsigned int *)&RAMBuffer[VELOCITY_MOTOR_2];
    *pTemp = temp;

}

void processCommand(void)
{
    switch (RAMBuffer[255]) {
        case 0:     // Refresh Data
            updateRAMData();
            break;
        case 1:     // Set Motor 1 Power
            pMotorPower = (int *) &RAMBuffer[POWER_MOTOR_1];
            setMotorPower(1, *pMotorPower);
            resetPID(1);
            PID1Mode = 0;
            break;
        case 2:     // Set Motor 2 Power
            pMotorPower = (int *) &RAMBuffer[POWER_MOTOR_2];
            setMotorPower(2, *pMotorPower);
            resetPID(2);
            PID2Mode = 0;
            break;
        case 3:     // Set Motor 1 and 2 Power
            pMotorPower = (int *) &RAMBuffer[POWER_MOTOR_1];
            // set motor power pwm 1
            setMotorPower(1, *pMotorPower);
            pMotorPower = (int *) &RAMBuffer[POWER_MOTOR_2];
            // set motor power pwm 2
            setMotorPower(2, *pMotorPower);
            // Reset both PID (bits 0 and 1)
            resetPID(3);
            PID1Mode = 0;
            PID2Mode = 0;
            break;


        case 0x11:  // Set Motor1 PID Position Mode
            resetPID(1);
            PID1Mode = 1;
            break;
        case 0x12:  // Set Motor1 PID Position Mode
            resetPID(2);
            PID2Mode = 1;
            break;
        case 0x13:  // Set Motor1 and Motor 2 to PID Position Mode
            resetPID(3);
            PID1Mode = 1;
            PID2Mode = 1;
            break;
        case 0x21:  // Set Motor1 to Velocity PID Mode
            resetPID(1);
            PID1Mode = 2;
            break;
        case 0x22:  // Set Motor2 to Velocity PID Mode
            resetPID(2);
            PID2Mode = 2;
            break;
        case 0x23:  // Set Motor1 and Motor2 to Velocity PID Mode
            resetPID(3);
            PID1Mode = 2;
            PID2Mode = 2;
            break;

        case 0x30:  // Clear QEI1
            POS1CNT = 0;
            *PTR_DWORD_QEI1 = 0;
            // Clear PID modes
            resetPID(1);
            PID1Mode = 0;
            break;
        case 0x31:  // Clear QEI2
            POS2CNT = 0;
            *PTR_DWORD_QEI2 = 0;
            // Clear PID modes
            resetPID(2);
            PID2Mode = 0;
            break;
        case 0x32:  // Clear QEI1 and QEI2
            POS1CNT = 0;
            POS2CNT = 0;
            *PTR_DWORD_QEI1 = 0;
            *PTR_DWORD_QEI2 = 0;
            // Clear PID modes
            resetPID(3);
            PID1Mode = 0;
            PID2Mode = 0;
            break;
        default:
            break;
    }
}

/*
Function Name: SI2C1Interrupt
Description : This is the ISR for I2C1 Slave interrupt.
Arguments	 : None
 */
void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void) {
    unsigned char Temp; //used for dummy read

    if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0)){ //Address matched
        Temp = I2C1RCV; //dummy read
        Flag.AddrFlag = 1; //next byte will be address
    } else if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 1)){ //check for data
        if (Flag.AddrFlag) {
            Flag.AddrFlag = 0;
            Flag.DataFlag = 1; //next byte is data
            //RAMPtr = RAMPtr + I2C1RCV;
            RAMindex = (unsigned char) I2C1RCV;
#if defined(USE_I2C_Clock_Stretch)
            I2C1CONbits.SCLREL = 1; //Release SCL1 line
#endif
        } else if (Flag.DataFlag) {
            RAMBuffer[RAMindex] = (unsigned char) I2C1RCV; // store data into RAM
            Flag.AddrFlag = 0; //end of tx
            Flag.DataFlag = 0;
            if (RAMindex == 255) {
                processCommand();
            } else if (RAMindex == 1) {
                // On any update, turn off PID modes
                PID1Mode = 0;
                PID2Mode = 0;
                // And turn off motors
                setMotorPower(1,0);
                setMotorPower(2,0);
                Temp = RAMBuffer[1];
                invertMotor1 = Temp & 0x01;
                invertMotor2 = (Temp & 0x02) >> 1;
                QEI1CONbits.SWPAB = (Temp & 0x04) >> 2;
                QEI1CONbits.SWPAB = (Temp & 0x08) >> 3;

            }
            RAMindex = 0;
            //RAMPtr = &RAMBuffer[0]; //reset the RAM pointer
#if defined(USE_I2C_Clock_Stretch)
            I2C1CONbits.SCLREL = 1; //Release SCL1 line
#endif
        }
    } else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0)) {
        Temp = I2C1RCV;
        I2C1TRN = RAMBuffer[RAMindex]; //Read data from RAM & send data to I2C master device
        I2C1CONbits.SCLREL = 1; //Release SCL1 line
        while (I2C1STATbits.TBF); //Wait till all
        RAMindex = 0;
        //RAMPtr = &RAMBuffer[0]; //reset the RAM pointer
    } else if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0)) {
        Temp = I2C1RCV;
        I2C1TRN = RAMBuffer[RAMindex]; //Read data from RAM & send data to I2C master device
        I2C1CONbits.SCLREL = 1; //Release SCL1 line
        while (I2C1STATbits.TBF); //Wait till all
        RAMindex = 0;
    }
    _SI2C1IF = 0; //clear I2C1 Slave interrupt flag
}


void ADC1_init(void)
{
    //
    //
    //

    AD1CON1bits.ADON  = 0;  // Disable ADC

    AD1CON1bits.FORM  = 0;  // Data Output Format: Integers
    AD1CON1bits.SSRC  = 2;  // Sample Clock Source: GP Timer3 starts conversion
    AD1CON1bits.ASAM  = 1;  // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 1;  // 10-bit = 0, 12 = 1 ADC operation

    AD1CON2bits.CSCNA = 1;  // Scan Input Selections for CH0+ during Sample A bit
    AD1CON2bits.CHPS  = 0;  // Converts CH0

    AD1CON3bits.ADRC = 0;   // ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS = 63;  // ADC Conversion Clock Tad=Tcy*(ADCS+1) =
                            //          (1/40MHz)*64 = 1.6us (625Khz)
                            // ADC Conversion Time for 10-bit Tc=12*Tad = 19.2us
                            // ADC Conversion Time for 12-bit Tc=14*Tad = 22.4us

    AD1CON1bits.ADDMABM = 0;// DMA are in scatter/gather mode
    AD1CON2bits.SMPI = 2;   // 3 ADC Channels are scanned
    AD1CON4bits.DMABL = 3;  // Each buffer contains 8 words

    AD1CSSL = 0;            // Initialize it to 0 (no channels)
    AD1CSSLbits.CSS4=1;     // Enable AN4 for channel scan
    AD1CSSLbits.CSS5=1;     // Enable AN5 for channel scan
    AD1CSSLbits.CSS6=1;     // Enable AN6 for channel scan

    //AD1PCFGH/AD1PCFGL: Port Configuration Register
    AD1PCFGL=0xFFFF;
    AD1PCFGLbits.PCFG4 = 0; // AN4 as Analog Input
    AD1PCFGLbits.PCFG5 = 0; // AN5 as Analog Input
    AD1PCFGLbits.PCFG6 = 0; // AN6 as Analog Input
	
    IFS0bits.AD1IF = 0;     // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0;     // Disable A/D interrupt, as we use DMA
    AD1CON1bits.ADON = 1;   // Turn on the A/D converter

}

/*=============================================================================
Timer 3 is setup to time-out every 125 microseconds (8Khz Rate). As a result, the module
will stop sampling and trigger a conversion on every Timer3 time-out, i.e., Ts=125us.
=============================================================================*/
void TMR3_init()
{
    TMR3 = 0x0000;
    PR3  = 4999;	// Trigger ADC1 every 125usec
    IFS0bits.T3IF = 0;	// Clear Timer 3 interrupt
    IEC0bits.T3IE = 0;	// Disable Timer 3 interrupt
    T3CONbits.TON = 1;	//Start Timer 3
}


// DMA0 configuration
// Direction: Read from peripheral address 0-x300 (ADC1BUF0) and write to DMA RAM
// AMODE: Peripheral Indirect Addressing Mode
// MODE: Continuous, Ping-Pong Mode
// IRQ: ADC Interrupt

void DMA0_init(void)
{
    DMA0CONbits.AMODE = 2;  // Configure DMA for Peripheral indirect mode
    DMA0CONbits.MODE  = 2;  // Configure DMA for Continuous Ping-Pong mode
    DMA0PAD=(int)&ADC1BUF0;
    DMA0CNT = (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;					
    DMA0REQ = 13;           // Select ADC1 as DMA Request source

    DMA0STA = __builtin_dmaoffset(BufferA);		
    DMA0STB = __builtin_dmaoffset(BufferB);

    IFS0bits.DMA0IF = 0;    //Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1;    //Set the DMA interrupt enable bit

    DMA0CONbits.CHEN=1;     // Enable DMA
}


/*=============================================================================
_DMA0Interrupt(): ISR name is chosen from the device linker script.
=============================================================================*/

unsigned int DmaBuffer = 0;

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
    if(DmaBuffer == 0)
    {
        motor1Current =      BufferA[4][0];
        motor2Current =      BufferA[5][0];
        motorSupplyVoltage = BufferA[6][0];
    }
    else
    {
        motor1Current =      BufferB[4][0];
        motor2Current =      BufferB[5][0];
        motorSupplyVoltage = BufferB[6][0];
    }

    DmaBuffer ^= 1;

    //tglPin();					// Toggle RA6	
    IFS0bits.DMA0IF = 0;		// Clear the DMA0 Interrupt Flag
}


void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt(void)
{
    unsigned int *pQEI;
    pQEI = (unsigned int *) &RAMBuffer[QEI1_LSB + 2];
    if (QEI1CONbits.CNTERR) {
        // Find if we are having an underflow or overflow
        // use a large positive number in case the motor is spinning fast
        if (POS1CNT < 10000) {
            *pQEI = *pQEI + 1;
        } else {
            *pQEI = *pQEI - 1;
        }
        // Reset count error
        QEI1CONbits.CNTERR = 0;

    }
    IFS3bits.QEI1IF = 0;        // Clear QEI1 Interrupt Flag
}
void __attribute__((interrupt, no_auto_psv)) _QEI2Interrupt(void)
{
   unsigned int *pQEI;
    pQEI = (unsigned int *) &RAMBuffer[QEI2_LSB + 2];
    if (QEI2CONbits.CNTERR) {
        // Find if we are having an underflow or overflow
        // use a large positive number in case the motor is spinning fast
        if (POS2CNT < 10000) {
            *pQEI = *pQEI + 1;
        } else {
            *pQEI = *pQEI - 1;
        }
        // Reset count error
        QEI2CONbits.CNTERR = 0;
    }
    IFS4bits.QEI2IF = 0;        // Clear QEI1 Interrupt Flag
}


void QEI_init(void)
{
    // Setup the PPS to enable the QEI ports on RP22,RP23 and RP24,RP25
    //
    // Setup QEI1 PPS
    //   RP     23          22
    //   bits   0001 0111   0001 0110
    //   hex       1    7      1    6
    //
    RPINR14 = 0x1716;

    // Setup QEI2 PPS
    //   RP     25          24
    //   bits   0001 1001   0001 1000
    //   hex       1    9      1    8
    //
    RPINR16 = 0x1918;

    // Setup QEIxCON
    //
    // 15 CNTERR  = 0
    // 14 N/A     = 0
    // 13 QEISIDL = 0
    // 12 INDEX   = 0
    // 11 UPDN    = 1
    // 10 QEIM    = 1   4x with index pulse
    //  9  "      = 1
    //  8  "      = 0
    //  7 SWPAB   = 1   A/B Swaped
    //  6 PCDOUT  = 0
    //  5 TQGATE  = 0
    //  4 TQCKPS  = 0
    //  3   "     = 0
    //  2 POSRES  = 0   Index pulse does not reset Position Counter
    //  1 TQCS    = 0
    //  0 UPDN_SRC= 0   UPDN (QEIxCON<11>) defines direction
    //
    // Bits   0000 1110 1000 0000
    // Hex       0    e    8    0
    //
    QEI1CON = 0x0e80;
    QEI2CON = 0x0e80;

    // Enable interrupts on count errors
    DFLT1CON = 0;
    DFLT2CON = 0;

    // Clear the interrupt bits
    IFS3bits.QEI1IF = 0;
    IFS4bits.QEI2IF = 0;

    // Enable interrupts
    IEC3bits.QEI1IE = 1;
    IEC4bits.QEI2IE = 1;

}

void TMR1_init(void)
{
    TMR1 = 0; // Reset timer counter
    T1CONbits.TON = 0; // Turn off timer 1
    T1CONbits.TSIDL = 0; // Continue operation during sleep
    T1CONbits.TGATE = 0; // Gated timer accumulation disabled
    T1CONbits.TCS = 0; // use Tcy as source clock
    T1CONbits.TCKPS = 2; // Tcy / 64 as input clock
    PR1 = 31250; // Interrupt period = 0.05 sec with a 64 prescaler
    IFS0bits.T1IF = 0; // Clear timer 1 interrupt flag
    IEC0bits.T1IE = 1; // Enable timer 1 interrupts
    T1CONbits.TON = 1; // Turn on timer 1
    return;
}



void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt (void)
{
    unsigned int p1;
    unsigned int p2;

    long int lTemp;
    int temp;
    long int lPower;

    unsigned int *pPID_limit;
    unsigned int uPID_limit;

    p1 = POS1CNT;
    p2 = POS2CNT;

    v1 = p1 - lastQEI1;
    v2 = p2 - lastQEI2;

    lastQEI1 = p1;
    lastQEI2 = p2;

    if (PID1Mode == 1) {
        e1_3 = e1_2;
        e1_2 = e1_1;
        e1_1 = e1_0;

        temp = POS1CNT;
        RAMBuffer[QEI1_LSB] = temp & 0x00ff;
        RAMBuffer[QEI1_MSB] = (temp >> 8) & 0x00ff;
        //TODO: check for CNTERR overflow
        // Calculate the error term
        lTemp = (long int) ((*PTR_DWORD_QEI1) - (*PTR_DWORD_POS1));
        // Make sure it fits in 16 bits and convert it to 16 bits
        e1_0 = (int) MAKEWORD(lTemp);

        // calculate the D term
        delta1 = ((long int) e1_0) - ((long int) e1_3);
        delta1 = MAKEWORD(delta1);
        // Calculate the I term
        temp = (int) sum1;
        sum1 = __builtin_mulss(temp,31) + (((long int) e1_0)<<2);
        sum1 = sum1 >> 5;
        // Make sure it fits in 16 bits
        sum1 = MAKEWORD(sum1);

        // Calculate Kp * e
        lTemp = __builtin_mulss(e1_0, *PTR_WORD_PID_POS1_KP);
        lTemp = lTemp >> 8;
        // Make sure it fits in 16 bits
        lTemp = MAKEWORD(lTemp);

        // Calculate Ki * Integral(e)
        lPower = lTemp;
        temp = (int) sum1;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_POS1_KI);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);
        
        // Calculate Kd * Derivative(e)
        temp = (int) delta1;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_POS1_KD);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);

        temp = (int) lPower;

        // Limit changes 14-Mar-2016
        pPID_limit = (unsigned int *) &RAMBuffer[PID_LIMIT_1];
        uPID_limit = *pPID_limit;

        if (uPID_limit != 0) {
            // Only limit the system if not zero
            if (uPID_limit > 10000) {
                uPID_limit = 10000;
            }
            if (temp >= 0) {
                if (temp > ((int) uPID_limit)) {
                    temp = (int) uPID_limit;
                }
            } else {
                if (temp < -((int) uPID_limit)) {
                    temp = -((int) uPID_limit);
                }
            }
        }


        // set the power level
        setMotorPower(1, temp);

    } else if (PID1Mode == 2) {
        e1_3 = e1_2;
        e1_2 = e1_1;
        e1_1 = e1_0;

        e1_0 = v1 - *PTR_WORD_VEL1;

        // calculate the D term
        delta1 = ((long int) e1_0) - ((long int) e1_3);
        delta1 = MAKEWORD(delta1);
        // Calculate the I term
        temp = (int) sum1;
        sum1 = __builtin_mulss(temp,31) + (((long int) e1_0)<<2);
        sum1 = sum1 >> 5;

        // Calculate Kp * e
        lTemp = __builtin_mulss(e1_0, *PTR_WORD_PID_VEL1_KP);
        lTemp = lTemp >> 8;

        // Calculate Ki * Integral(e)
        lPower = lTemp;
        temp = (int) sum1;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_VEL1_KI);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);

        // Calculate Kd * Derivative(e)
        temp = (int) delta1;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_VEL1_KD);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);

        temp = (int) lPower;

        // Limit changes 14-Mar-2016
        pPID_limit = (unsigned int *) &RAMBuffer[PID_LIMIT_1];
        uPID_limit = *pPID_limit;

        if (uPID_limit != 0) {
            // Only limit the system if not zero
            if (uPID_limit > 10000) {
                uPID_limit = 10000;
            }
            if (temp >= 0) {
                if (temp > ((int) uPID_limit)) {
                    temp = (int) uPID_limit;
                }
            } else {
                if (temp < -((int) uPID_limit)) {
                    temp = -((int) uPID_limit);
                }
            }
        }


        // set the power level
        setMotorPower(1, temp);

    }
    if (PID2Mode == 1) {

        e2_3 = e2_2;
        e2_2 = e2_1;
        e2_1 = e2_0;

        temp = POS2CNT;
        RAMBuffer[QEI2_LSB] = temp & 0x00ff;
        RAMBuffer[QEI2_MSB] = (temp >> 8) & 0x00ff;
        //TODO: check for CNTERR overflow
        // Calculate the error term
        lTemp = (long int) ((*PTR_DWORD_QEI2) - (*PTR_DWORD_POS2));
        // Make sure it fits in 16 bits and convert it to 16 bits
        e2_0 = (int) MAKEWORD(lTemp);

        // calculate the D term
        delta2 = ((long int) e2_0) - ((long int) e2_3);
        delta2 = MAKEWORD(delta2);
        // Calculate the I term
        temp = (int) sum2;
        sum2 = __builtin_mulss(temp,31) + (((long int) e2_0)<<2);
        sum2 = sum2 >> 5;
        // Make sure it fits in 16 bits
        sum2 = MAKEWORD(sum2);

        // Calculate Kp * e
        lTemp = __builtin_mulss(e2_0, *PTR_WORD_PID_POS2_KP);
        lTemp = lTemp >> 8;
        // Make sure it fits in 16 bits
        lTemp = MAKEWORD(lTemp);

        // Calculate Ki * Integral(e)
        lPower = lTemp;
        temp = (int) sum2;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_POS2_KI);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);

        // Calculate Kd * Derivative(e)
        temp = (int) delta2;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_POS2_KD);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);

        temp = (int) lPower;

        // Limit changes 14-Mar-2016
        pPID_limit = (unsigned int *) &RAMBuffer[PID_LIMIT_2];
        uPID_limit = *pPID_limit;

        if (uPID_limit != 0) {
            // Only limit the system if not zero
            if (uPID_limit > 10000) {
                uPID_limit = 10000;
            }
            if (temp >= 0) {
                if (temp > ((int) uPID_limit)) {
                    temp = (int) uPID_limit;
                }
            } else {
                if (temp < -((int) uPID_limit)) {
                    temp = -((int) uPID_limit);
                }
            }
        }


        // set the power level
        setMotorPower(2, temp);

    } else if (PID2Mode == 2) {
        e2_3 = e2_2;
        e2_2 = e2_1;
        e2_1 = e2_0;

        e2_0 = v2 - *PTR_WORD_VEL2;

        // calculate the D term
        delta2 = ((long int) e2_0) - ((long int) e2_3);
        delta2 = MAKEWORD(delta2);
        // Calculate the I term
        temp = (int) sum2;
        sum2 = __builtin_mulss(temp,31) + (((long int) e2_0)<<2);
        sum2 = sum2 >> 5;

        // Calculate Kp * e
        lTemp = __builtin_mulss(e2_0, *PTR_WORD_PID_VEL2_KP);
        lTemp = lTemp >> 8;

        // Calculate Ki * Integral(e)
        lPower = lTemp;
        temp = (int) sum2;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_VEL2_KI);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);

        // Calculate Kd * Derivative(e)
        temp = (int) delta2;
        lTemp = __builtin_mulss(temp, *PTR_WORD_PID_VEL2_KD);
        lTemp = lTemp >> 8;
        lPower = lPower + lTemp;
        lPower = MAKEWORD(lPower);

        temp = (int) lPower;

        // Limit changes 14-Mar-2016
        pPID_limit = (unsigned int *) &RAMBuffer[PID_LIMIT_2];
        uPID_limit = *pPID_limit;

        if (uPID_limit != 0) {
            // Only limit the system if not zero
            if (uPID_limit > 10000) {
                uPID_limit = 10000;
            }
            if (temp >= 0) {
                if (temp > ((int) uPID_limit)) {
                    temp = (int) uPID_limit;
                }
            } else {
                if (temp < -((int) uPID_limit)) {
                    temp = -((int) uPID_limit);
                }
            }
        }


        // set the power level
        setMotorPower(2, temp);

    }

    // clear interrupts
    IFS0bits.T1IF = 0;

    t1count++;
}




//
// U1,U2 (VNH5019A) operation
//
// INA INB DIAGA/ENA DIAGB/ENB OUTA OUTB  CS          Operating mode
//  1   1      1         1       H    H   High imp.   Brake to VCC
//  1   0      1         1       H    L   IOUT/K      Clockwise (CW)
//  0   1      1         1       L    H   IOUT/K      Counterclockwise (CCW)
//  0   0      1         1       L    L   High imp.   Brake to GND



int main()
{
    // PORT A map
    // RA0 - U1 INA
    // RA1 - U1 INB
    // RA2 - U1 ENA/DIAGA
    // RA3 - U1 ENB/DIAGB
    // RA4 - EEPROM WP
    // RA5 - N/A
    // RA6 - N/A
    // RA7 - N/C
    // RA8 - EEPROM A0
    // RA9 - EEPROM A1
    // RA10 - N/C
    //
    // Bits for input/output (1 = input)
    //    011 0110 0000
    //      3    6    0

    TRISA = 0xb60;

    // PORT B map
    // RB0 - ICD PGD
    // RB1 - ICD PGC
    // RB2 - AN4 Motor 1 Current Sense (A/D)
    // RB3 - AN5 Motor 2 Current Sense (A/D)
    // RB4 - N/C
    // RB5 - Status LED RED
    // RB6 - Status LED Green
    // RB7 - N/C
    
    TRISB = 0x9f;



    // PORT C map
    // RC0 - (used as AN6) Motor VCC Supply Voltage / 11
    // RC1 - N/C
    // RC2 - U2 ENA/DIAGA
    // RC3 - U2 ENB/DIAGB
    // RC4 - U2 INA
    // RC5 - U2 INB
    // RC6 - (RP22) QEI1A
    // RC7 - (RP23) QEI1B
    // RC8 - (RP24) QEI2A
    // RC9 - (RP25) QEI2B
    //
    // Bits for input/output (1 = input)
    //    11 1100 0001
    //     3    c    1


    TRISC = 0xfc1;


    // Fosc = Fin * ( M / (N1 * N2) )
    //      = 7.37MHz * ( 152 / ( 7 * 2 ) )
    //      = 80.017 MHz
    //
    // PLLFBD = M - 2 (register definition)
    //        = 152 - 2 = 150
    PLLFBD=150;
    // PLLPOST  = 0 for N2 = 2,
    //          = 1 for N2 = 4,
    //          = 3 for N2 = 8
    CLKDIVbits.PLLPOST = 0;
    // PLLPRE = N1 - 2
    //        = 7 - 2 = 5
    CLKDIVbits.PLLPRE = 5;   // N2 = 17

    // Wait for oscillator to be ready
    while (OSCCONbits.LOCK != 1)
        ;

    //===================================
    // Now initialize the PWM
    // PMW1L1 is on the same pin as RB15

    // FOSC = 80.017, FCY = FOSC/2 = 40.0085
    // P1TPER = (FCY / (FPWM * P1TMRPRE)) - 1
    //        = (40MHz / (18KHz * 1 ) - 1
    //        = 2222
    P1TPER = 2222;
    // P1TMR Prescale = 1
    P1TCONbits.PTCKPS = 0;

    PWM1CON1bits.PEN2H = 1;

    PWM1CON1bits.PEN1H = 1;

    // Set initial duty cycle
    P1DC2 = 10;
    P1DC1 = 10;

    // Turn off both motors
    setMotorPower(1,0);
    setMotorPower(2,0);

    // Changed 14-Mar-2016
    // Set up PID limits to zero
    RAMBuffer[PID_LIMIT_1] = 0;
    RAMBuffer[PID_LIMIT_1_MSB] = 0;
    RAMBuffer[PID_LIMIT_2] = 0;
    RAMBuffer[PID_LIMIT_2_MSB] = 0;

    // Enable PWM
    P1TCONbits.PTEN = 1;

    //===================================
    // Now initialize the ADC
    //-----------------------------------
    ADC1_init();
    DMA0_init();
    TMR3_init();

    // Enable PWM to trigger ADC
//    P1SECMPbits.SEVTDIR = 1;        // trigger on up count
//    PWM1CON2bits.SEVOPS = 0;         // postscale 1:1
//    P1SECMPbits.SEVTCMP = 100;      // Trigger at count = 100

    QEI_init();

    // Get cape selection id
    int capeID;
    capeID = (PORTA & 0x0300) >> 8;

    char *capeIDstr;
    capeIDstr = (char *) &RAMBuffer[CAPEID];
    char versionString[] = "DMCC Mk.07c1";
    
    strcpy (capeIDstr, versionString);

    i2c1_init( capeID + I2C_BASE_ADDR);
    TMR1_init();


    LATBbits.LATB5 = 0;
    LATBbits.LATB6 = 1;

    while(1);


    return 0;
}
