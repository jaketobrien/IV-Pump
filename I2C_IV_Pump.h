/*********************************************************************
 *             I2C_Test   Firmware Version 1.0
 *********************************************************************
 * FileName:        project_defs.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC16F877
 * Compiler:        Hitec C compiler
 * Company:         

 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                       15/11/11       .
 *			updated		Nov 2022 by JM
 **********************************************************************/

/***********USER FUNCTIONS***********

1.	i2c_init()
				- Must be called to initialise I2C device.
				- Note what SFRs are effected (TRISC) and be sure not to overwrite these in yourt program initialisation.


*/




#include <pic.h>
#define _XTAL_FREQ 4000000
unsigned char data[20];


/******************typedef for data types *****************************/
typedef signed char     BYTE;
typedef signed short    WORD;
typedef signed long     DWORD;
typedef float           FLOAT;
typedef unsigned char   UBYTE;
typedef unsigned int    UWORD;
typedef unsigned long   UDWORD;

#define TRUE                1
#define FALSE               0
#define HIGH	            1
#define LOW		            0	
#define RX_BUFFER_SIZE     20

/** T R I S *********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0


void i2c_init(void);
unsigned char i2c_write( unsigned char i2cWriteData );
int i2c_read( unsigned char ack );
void i2c_stop(void);
void i2c_repStart(void);
void i2c_start(void);
void i2c_waitForIdle(void);

/******************************************************************************************/

/******************************************************************************************/

void i2c_waitForIdle(void)
{
 while (( SSPCON2 & 0x1F ) | R_nW ) {}; // wait for idle and not writing
}
/******************************************************************************************/
void i2c_start(void)
{
 i2c_waitForIdle();
 SEN=1;
}
/******************************************************************************************/
void i2c_repStart(void)
{
 i2c_waitForIdle();
 RSEN=1;
}
/******************************************************************************************/
void i2c_stop(void)
{
 i2c_waitForIdle();
 PEN=1;
}
/******************************************************************************************/

int i2c_read( unsigned char ack )
{
unsigned char i2cReadData;
i2c_waitForIdle();
 RCEN=1;
 i2c_waitForIdle();
 i2cReadData = SSPBUF;
 i2c_waitForIdle();
 if ( ack )
  {
  ACKDT=0;				//ACK
  }
 else
  {
  ACKDT=1;				//NACK
  }
  ACKEN=1;               // send acknowledge sequence
 return( i2cReadData );
}
/******************************************************************************************/

unsigned char i2c_write( unsigned char i2cWriteData )
{
 i2c_waitForIdle();
 SSPBUF = i2cWriteData;
return ( ! ACKSTAT  ); // function returns '1' if transmission is acknowledged
}
/******************************************************************************************/


void i2c_init(void)
{
 // Do in main code TRISC = 0b00011000;       // set SCL and SDA pins as inputs
 SSPCON = 0x38;            // set I2C master mode
 SSPCON2 = 0x00;
 SSPADD = 0x0A;            // 100k at 4Mhz clock
 CKE=1;                    // use I2C levels      
 SMP=1;                    // disable slew rate control  
 PSPIF=0;                  // clear SSPIF interrupt flag
 BCLIF=0;                  // clear bus collision flag
}

/*****************************************************************************************/

