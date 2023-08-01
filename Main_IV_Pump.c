/**
* C File        Main_IV_Pump.c    	
* Platform:     PICmicro PIC16F877A @ 4 Mhz	
* Written by:   Jake O'Brien		
*
* Date:         24/11/2022
*
* Function:	    A c file to make use of the USART peripheral onboard
*               the PIC16F877A board to send the string "Hello" to
*		        the Putty on the PC
*/

// CONFIG____________________________________________________________________

#pragma config FOSC = XT 	// Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF 	// Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF 	// Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF 	// Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF 	// Low-Voltage (Single-Supply) In-Circuit Serial 
                        // Programming Enable bit (RB3 is digital I/O, HV
                        // on MCLR must be used for programming)
#pragma config CPD = OFF 	// Data EEPROM Memory Code Protection bit 
                        // (Data EEPROM code protection off)
#pragma config WRT = OFF 	// Flash Program Memory Write Enable bits (Write
                        // protection off; all program memory may be 
                        // written to by EECON control)
#pragma config CP = OFF 	// Flash Program Memory Code Protection bit 
                        // (Code protection off)
#include <xc.h> 			// Include standard PIC library
#include <string.h>
#include "ee302lcd.h"
#include "I2C_EE302.h"
#ifndef _XTAL_FREQ
                // Unless already defined assume 4MHz system frequency
                // This definition is required to calibrate the delay functions, __delay_us() and __delay_ms();
#define _XTAL_FREQ 4000000
#endif

// Definitions____________________________________________________________

#define OPEN 1
#define CLOSED 0		// Define switch action "Closed" as 0
#define SW1 RB0			// Assign Label SW1 to PortB bit 0 (RB0)
#define SW2 RB1			// Assign Label SW2 to PortB bit 1 (RB1)
#define RESET RB2       // Assign Label RB2 to PortB bit 2 (RB2))
#define LED1	RC0	//bit 0 of PORTC
#define LED2	RC1	//bit 1 of PORTC
#define ON 1            // Define ON as 1
#define OFF 0           // Define OFF as 0


// Globals _____________________________________________________

	unsigned char gOutString[16];       // unsigned char global variable string 16 in length
    char gChar = 'S';
    double gAdcValue;
    double gDosage;
    
// Prototypes___________________________________________________

    void setup(void);           // Declare setup function
    void lcdTitle(void);        // Declare LCD title function
    void loop(void);            // Declare loop function
    void setRC(void);
    void switchPowerOn(void);	// Declare data to LCD function
    void ledPowerOn(void);
    void ledAdminDrug(void);
    void displayPowerOn(void);
    void displayPatientRequest(void);
    void clear_outString(void); // Declare clear string function
    void write_eeprom_patientNum(char *string);	// write eeprom function declared
    unsigned char read_eeprom_patientNum(void);	// read eeprom function declared
    void transmitString(char *string);
    char drugAdministered(void);
    int calculateADC(void);
    void adcDisplayDosage(void);
    void activeScreen(void);
    int fluidLevels(void);
    void displayTermination(void);

// Main program_________________________________________________  
void main(void) {
    
    setup();				// Call initialisation
	lcdTitle();             // Call LCDTitle
    
    //Superloop
	for(;;)
	{
		loop();		// Write data to LCD
	}
    
    return;
}

void setup(void)
	{
	Lcd8_Init();		// Required initialisation of LCD to 8-bit mode
    i2c_init();					// I2C initialisation function
	TRISB=0x07;			// Set PORTB bit 0 and 1 as inputs
    	TRISD 	= 0x00;			//	Port D bits 0 and 7 as outputs 
	PORTD	= 0xff;			//	Turn off all LEDs on PORTD
		//	T0CS = 0: Timer 0 internal clock
							//	PSA	= 0: Prescaler assined to Timer0
							//	PS2 = 0: Prescaler bits set to 0 1 0 (x8)
							//	PS1 = 1
							//	PS0 = 0

	INTCON 	= 0b11000000;         //	T0IE = 1: enable interrupt on TMR0 overflow
                            //	TMR0IF = 0: Timer0 interrupt flag cleared
                            // GIE = 1; // Global interrupt enable
    T1CON = 0x21;
    PIE1 = 0x01;

	TMR1 = 0;               //	Initialise TMR1 to 0x00
    TRISC = 0xC0; 		// RC6 and RC7 must be set to inputs for USART
	PORTC = 0b00000111;         // set all PORTC outputs to 0
    TXSTA = 0b00100100; 		// Set TXEN bit to enable transmit.
                // Set BRGH bit for Baud Rate table selection.
    RCSTA = 0b10010000; 		// Set CREN bit for continuous read.
                // Set SPEN bit to enable serial port.
    SPBRG = 0b00011001; 		// Set Baud Rate to 9600
//    ADCON0 = 0b00010001;    // Set ADCON0 bit 0 and 4 as inputs
//    ADCON1 = 0b01000010;    // Set ADCON1 bit 1 and 6 as inputs
    ADCON0 = 0b01000101;    // Set ADCON0 bit 0, 2 and 6 as inputs
    ADCON1 = 0b10001110;    // Set ADCON1 bit 1 and 6 as inputs
	}

void lcdTitle(void)
	{
	Lcd8_Write_String("- EE302 Project -");		// print "LCD Demo" on line 1 of LCD
	Lcd8_Set_Cursor(2,1);                   // select line 2
	Lcd8_Write_String("- IV Pump -");             // print "LCD Demo" on line 2 of LCD
	}

void loop(void)                             
{
    switchPowerOn();                       // switch commands function
    displayDrugRequest();
    write_eeprom_drug(drugAdministered());	// write eeprom function called with string argument inserted
    read_eeprom_drug();                  // read eeprom function called
    transmitString();               // transmit string function called
    adcDisplayDosage();
    displayActiveScreen();
}

void clear_outString(void){
    int i;                                  // int i declared
    
    for(i = 0; i<16; i++){                  // for i incrementing through 16 char string clear
        gOutString[i] = 0x00;
    }
}

void displayPowerOn(){
    Lcd8_Clear();                               // clear LCD display
    clear_outString();                          // clear outString array
    Lcd8_Shift_Right();                         // shift string right
    sprintf(gOutString,"- IV Pump -");         // define string as "Int Value is"
    Lcd8_Write_String(gOutString);              // print string to LCD
    Lcd8_Set_Cursor(2,1);                       // select line 2 of LCD
    sprintf(gOutString,"Power On");             // define intvalue as a char in outString
    Lcd8_Write_String(gOutString);              // print string to LCD
}

void displayDrugRequest(){
    Lcd8_Clear();                               // clear LCD display
    clear_outString();                          // clear outString array
    Lcd8_Shift_Right();                         // shift string right
    sprintf(gOutString,"Insert Drug");         // define string as "Int Value is"
    Lcd8_Write_String(gOutString);              // print string to LCD
    Lcd8_Set_Cursor(2,1);                       // select line 2 of LCD
    sprintf(gOutString,"Being Administered");             // define intvalue as a char in outString
    Lcd8_Write_String(gOutString);              // print string to LCD
}

void switchPowerOn(){
    
    if (SW1 == CLOSED){                       // if KEY3 is pressed
        displayPowerOn();                       // display to LCD
        LED1 = ON;
    }
}

void write_eeprom_drug(char *drugString){    // write eeprom function initialised
    i2c_start();                    // Start
    i2c_write(0xA0);                // Control byte
    i2c_write(0x01);                // High address byte
    i2c_write(0x55);                // Low address byte
    
    for (int i = 0; i <= 7; i++){     // for loop to iterate through bytes
    i2c_write(drugString[i]);           // Data byte
    }
    
    i2c_stop();                     // Stop
    __delay_ms(5);                  // Necessary 5ms delay for write to propagate
}

unsigned char read_eeprom_drug(void){    // read eeprom function initialisation
    i2c_start();                    // Start
    i2c_write(0xA0);                // Control byte
    i2c_write(0x01);                // High address byte
    i2c_write(0x55);                // Low address byte
    
    i2c_repStart();                 // Restart
    i2c_write(0xA1);                // Control byte
    
    for (int i = 0; i <= 7; i++){   // for loop to iterate through bytes
    gOutString[i] = i2c_read(1);    // Data set to read ACK
    }
    gOutString[11] = i2c_read(0);   // Data set to read NACK
    i2c_stop();                     // Stop
}

char drugAdministered(){
    char drugString;
    
    setRC();
    
    if (gChar == 'S'){
        //transmitString("Saline");
        drugString = "Saline";
    }
    
    if (gChar == 'D'){
        // transmitString("Dextrose");
        drugString = "Dextrose";
    }
    
    return drugString;
}

void transmitString(char *string)			// transmit string function initialised
{
	for(int i = 0; string[i] != '\0'; i++){	// for iteration through to end of string
        while(!TXIF);					// while TXIF is set
        TXREG = string[i];				// ste TXIF equal to string index
    }
}
    
void setRC(){
    while(!RCIF);						// while RCIF is set
    gChar = RCREG;					// char type 'x' set equal to RCREG
}

int calculateADC(){
    unsigned int adcValue = 0;                  // unsigned int var declared and initialised
    GO_nDONE = 1;                               // GO_nDONE set to 1
    
    while(GO_nDONE) {continue;}                 // wait until done polling
    
    adcValue += ((unsigned int)ADRESH) << 2;    // var is an addition compound assignment of the value from ADRESH bitmasked 2 shifts left
    adcValue = adcValue + ADRESL;
    return adcValue;                            // return value associated with adcValue
}

void adcDisplayDosage(){
    __delay_ms(100);                                // 100 ms delay
    double dosage;                                 // var declared as a double
    GO_nDONE = 1;                                   // GO_nDONE set to 1
    
    while (GO_nDONE) {continue;}                    // wait until done polling
    dosage = (double) calculateADC() * 5/1024;     // var initialised being set equal to the product of calculation
    
    Lcd8_Write_String(gOutString, "Insert Dosage");
    Lcd8_Set_Cursor(2,3);                           // select second line and over 6 positions
    sprintf(gOutString, "= %.1f", dosage);         // print to LCD - value to one decimal place
    Lcd8_Set_Cursor(2,5);                           // select second line and over 6 positions
    Lcd8_Write_String(gOutString);                  // print string to LCD
    Lcd8_Write_String("ml/hr");                         // print to LCD
    
    if (SW2 == CLOSED){
        gAdcValue = dosage;
        LED1 = OFF;
        LED2 = ON;
    }
    
    gDosage = dosage;
}

void displayActiveScreen(){
    Lcd8_Clear();                               // clear LCD display
    clear_outString();                          // clear outString array
    Lcd8_Shift_Right();                         // shift string right
    sprintf(gOutString, drugAdministered());         // define string as "Int Value is"
    Lcd8_Write_String(gOutString);              // print string to LCD
    Lcd8_Set_Cursor(2,1);                       // select line 2 of LCD
    sprintf(gOutString, "= %.1f", gDosage);         // print to LCD - value to one decimal place
    Lcd8_Set_Cursor(2,2);                           // select second line and over 6 positions
    Lcd8_Write_String(gOutString);                  // print string to LCD
    Lcd8_Write_String("ml/hr");                         // print to LCD
    Lcd8_Write_String(gOutString);              // print string to LCD
    Lcd8_Set_Cursor(2,5);                           // select second line and over 6 positions
    Lcd8_Write_String(fluidLevels());                  // print string to LCD
    Lcd8_Set_Cursor(2,6);                           // select second line and over 6 positions
    Lcd8_Write_String("%");                         // print to LCD
}

int fluidLevels(){
    int currentFluidLevel = 100;
    
    while(currentFluidLevel >= 0){
        
        if (gAdcValue <= (gAdcValue += 0.2) && gAdcValue <= (gAdcValue -= 0.2)){        // Real time application
            __delay_ms(3);
            if (gAdcValue <= (gAdcValue += 0.2) && gAdcValue <= (gAdcValue -= 0.2)){
                __delay_ms(100);
                currentFluidLevel -= 1;
            }
        }
        else{
            displayTermination();
        }
    }
    return currentFluidLevel;
}

void displayTermination(){
    Lcd8_Clear();                               // clear LCD display
    clear_outString();                          // clear outString array
    //Lcd8_Shift_Right();                         // shift string right
    sprintf(gOutString,"Termination - Measure");         // define string as "Int Value is"
    Lcd8_Write_String(gOutString);              // print string to LCD
    Lcd8_Set_Cursor(2,1);                       // select line 2 of LCD
    sprintf(gOutString,"Out Of Range");             // define intvalue as a char in outString
    Lcd8_Write_String(gOutString);              // print string to LCD
}

