#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <c8051f38x.h>

#define pi 3.14
#define MHZ 1000000L
#define SYSCLK (24*MHZ)
#define BAUDRATE 115200L
#define VDD      3.325 // The measured value of VDD in volts
#define NUM_INS  4
#define zerocross0 //reference zero cross
#define zerocross1

// ANSI colors
#define	COLOR_BLACK		0
#define	COLOR_RED		1
#define	COLOR_GREEN		2
#define	COLOR_YELLOW		3
#define	COLOR_BLUE		4
#define	COLOR_MAGENTA		5
#define	COLOR_CYAN		6
#define	COLOR_WHITE		7

// Some ANSI escape sequences
#define CURSOR_ON "\x1b[?25h"
#define CURSOR_OFF "\x1b[?25l"
#define CLEAR_SCREEN "\x1b[2J"
#define GOTO_YX "\x1B[%d;%dH"
#define CLR_TO_END_LINE "\x1B[K"

/* Black foreground, white background */
#define BKF_WTB "\x1B[0;30;47m"
#define FORE_BACK "\x1B[0;3%d;4%dm"
#define FONT_SELECT "\x1B[%dm"

// C8051F381_ADC_multiple_inputs.c:  Shows how to use the 10-bit ADC and the
// multiplexer.  This program measures the voltages applied to pins P2.0 to P2.3.
//
// (c) 2008-2014, Jesus Calvino-Fraga
//
// ~C51~ 

char _c51_external_startup (void)
{
	PCA0MD&=(~0x40) ;    // DISABLE WDT: clear Watchdog Enable bit
	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == (12*MHZ))
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
	#elif (SYSCLK == (24*MHZ))
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == (48*MHZ))
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12MHZ, 24MHZ, or 48MHZ
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency
	
	// Configure P2.0 to P2.3 as analog inputs
	P2MDIN &= 0b_1111_0000; // P2.0 to P2.3
	P2SKIP |= 0b_0000_1111; // Skip Crossbar decoding for these pins

	// Init ADC multiplexer to read the voltage between P2.0 and ground.
	// These values will be changed when measuring to get the voltages from
	// other pins.
	// IMPORTANT: check section 6.5 in datasheet.  The constants for
	// each pin are available in "c8051f38x.h" both for the 32 and 48
	// pin packages.
	AMX0P = LQFP32_MUX_P2_0; // Select positive input from P2.0
	AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
	
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN=0b_0000_1000; //Select VDD as the voltage reference for the converter
  	
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	P0MDOUT|=0x10;     // Enable Uart TX as push-pull output
	XBR0=0x01;         // Enable UART on P0.4(TX) and P0.5(RX)
	XBR1=0x40;         // Enable crossbar and weak pull-ups
	
	#if (SYSCLK/BAUDRATE/2L/256L < 1)
		TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
		CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
		CKCON |=  0x08;
	#elif (SYSCLK/BAUDRATE/2L/256L < 4)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 01                  
		CKCON |=  0x01;
	#elif (SYSCLK/BAUDRATE/2L/256L < 12)
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 00
	#else
		TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
		CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 10
		CKCON |=  0x02;
	#endif
	
	TL1 = TH1;     // Init timer 1
	TMOD &= 0x0f;  // TMOD: timer 1 in 8-bit autoreload
	TMOD |= 0x20;                       
	TR1 = 1;       // Start timer1
	SCON = 0x52;
	
	return 0;
}

void PORT_Init (void)
{
	P0MDOUT |= 0x10; // Enable UTX as push-pull output
	XBR0     = 0x01; // Enable UART on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0x40; // Enable crossbar and weak pull-ups
}

void SYSCLK_Init (void)
{
	CLKSEL|=0b_0000_0011; // SYSCLK derived from Internal HF Osc / 1.
	OSCICN |= 0x03;   // Configure internal oscillator for its maximum 
	RSTSRC  = 0x04;   // Enable missing clock detector
}
 
void UART0_Init (void)
{
	SCON0 = 0x10;
	TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
	CKCON &= ~0x0B; // T1M = 1; SCA1:0 = xx
	CKCON |=  0x08;
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
}
 

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON:
	CKCON|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN & 0x80));  // Wait for overflow
		TMR3CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN = 0 ;                   // Stop Timer3 and clear overflow flag
}


void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}


//Measure half period at pin P1.0 using timer 0
TR0=0; //Stop timer 0
TMOD=0B_0000_0001; //Set timer 0 as 16-bit timer
TH0=0; TL0=0; //Reset the timer
while (P1_0==1); //Wait for the signal to be zero
while (P1_0==0); //Wait for the signal to be one
TR0=1; //Start timing
while (P1_0==1); //Wait for the signal to be zero
TR0=0; //Stop timer 0
//[TH0,TL0] is half the period in multiples of 12/CLK, so:
Period=(TH0*0x100+TL0)*2; //Assume Period is unsigned int 

//wait for zero cross
waitms(Period/4);
//measure peak voltage of reference, V0
//wait for zero cross
waitms(Period/4);
//measure peak voltage of other signal, V1
//measure time difference between zero cross of both signals
//convert peak ADC values to RMS and display
V1rms = V1/sqrt(2);
V0rms = V0/sqrt(2);
//display V1rms and V0rms
printf("rms reference voltage = %.3f,\n, rms input voltage = %.3f," V0rms, V1rms);
///convert time diff between the zero cross of both signals to degrees and display
//convert period to frequency and display
frequency = 2*pi/Period 
printf("frequency = %.3f", frequency);



void main (void)
{
	float v;
	unsigned char j;
	unsigned int Period;
	unsigned int halfperiod;
	unsigned int quarterperiod;
	unsigned int time;
	unsigned int time0;
	unsigned int time1;
	unsigned int timediff;
	float phase;
	float V0peak;
	float V1peak;
	
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	PORT_Init();     // Initialize Port I/O
	SYSCLK_Init ();  // Initialize Oscillator
	UART0_Init();    // Initialize UART0
	
	printf ("ADC/Multiplexer test program\n"
	        "Apply analog voltages to P2.0, P2.1, P2.2, and P2.3\n"
	        "File: %s\n"
	        "Compiled: %s, %s\n\n",
	        __FILE__, __DATE__, __TIME__);
	        
	//Measure half period at pin P1.0 using timer 0
TR0=0; //Stop timer 0
TMOD=0B_0000_0001; //Set timer 0 as 16-bit timer
TH0=0; TL0=0; //Reset the timer
while (P1_0==1); //Wait for the signal to be zero
while (P1_0==0); //Wait for the signal to be one
TR0=1; //Start timing
while (P1_0==1); //Wait for the signal to be zero
TR0=0; //Stop timer 0
//[TH0,TL0] is half the period in multiples of 12/CLK, so:
Period=(TH0*0x100+TL0)*2; //Assume Period is unsigned int 
time = (overflow_count * 65536.0 + TH0 * 256.0 + TL0) * (12.0 / SYSCLK); 

halfperiod = Period/2;
quarterperiod = Period/4;
while (zerocross0==1); //wait for zero cross to hit 0
waitms(halfperiod);
waitms(quarterperiod); 

//now want to measure peak voltage of the reference
	for(j=0; j<NUM_INS; j++)
		{
			AD0BUSY = 1; // Start ADC 0 conversion to measure previously selected input
			
			// Select next channel while ADC0 is busy
			switch(j)
			{
				case 0:
					AMX0P=LQFP32_MUX_P2_1;
				break;
				case 1:
					AMX0P=LQFP32_MUX_P2_2;
				break;
				case 2:
					AMX0P=LQFP32_MUX_P2_3;
				break;
				case 3:
					AMX0P=LQFP32_MUX_P2_0;
				break;
			}
			
			while (AD0BUSY); // Wait for conversion to complete
			v = ((ADC0L+(ADC0H*0x100))*VDD)/1023.0; // Read 0-1023 value in ADC0 and convert to volts
			
			// Display measured values
			switch(j)
			{
				case 0:
					printf("V0=%5.3fV, ", v);
					V0peak = v;
				break;
				case 1:
					printf("V1=%5.3fV, ", v);
				break;
				case 2:
					printf("V2=%5.3fV, ", v);
				break;
				case 3:
					printf("V3=%5.3fV", v);
				break;
			}

		}
//V0peak = V0;

while (zerocross0==1); //wait for zero cross
waitms(quarterperiod);

//measure peak voltage of test input

		for(j=0; j<NUM_INS; j++)
		{
			AD0BUSY = 1; // Start ADC 0 conversion to measure previously selected input
			
			// Select next channel while ADC0 is busy
			switch(j)
			{
				case 0:
					AMX0P=LQFP32_MUX_P2_1;
				break;
				case 1:
					AMX0P=LQFP32_MUX_P2_2;
				break;
				case 2:
					AMX0P=LQFP32_MUX_P2_3;
				break;
				case 3:
					AMX0P=LQFP32_MUX_P2_0;
				break;
			}
			
			while (AD0BUSY); // Wait for conversion to complete
			v = ((ADC0L+(ADC0H*0x100))*VDD)/1023.0; // Read 0-1023 value in ADC0 and convert to volts
			
			// Display measured values
			switch(j)
			{
				case 0:
					printf("V0=%5.3fV, ", v);
				break;
				case 1:
					printf("V1=%5.3fV, ", v);
					V1peak = v;
				break;
				case 2:
					printf("V2=%5.3fV, ", v);
				break;
				case 3:
					printf("V3=%5.3fV", v);
				break;
			}

		}
//V1peak = V1;

//measure time difference between zero cross of both signals

//while (zerocross0==1);
//time0 = time;
//while (zerocross1==1);
//time1 = time;
//timediff = time0-time1;

//convert peak ADC values to RMS and display
V1rms = V1peak/sqrt(2);
V0rms = V0peak/sqrt(2);

//convert time diff between zero cross of both signals to degrees
phase = timediff*360/Period;
printf("phase = %0.3f", phase);

//convert period to frequency and display
frequency = 2*pi/Period 
printf("frequency = %.3f", frequency);

	// Start the ADC in order to select the first channel.
	// Since we don't know how the input multiplexer was set up,
	// this initial conversion needs to be discarded.
	AD0BUSY=1;
	while (AD0BUSY); // Wait for conversion to complete

	while(1)
	{
		printf("\x1B[6;1H"); // ANSI escape sequence: move to row 6, column 1
		//try putting algorithm within while loop here 

		for(j=0; j<NUM_INS; j++)
		{
			AD0BUSY = 1; // Start ADC 0 conversion to measure previously selected input
			
			// Select next channel while ADC0 is busy
			switch(j)
			{
				case 0:
					AMX0P=LQFP32_MUX_P2_1;
				break;
				case 1:
					AMX0P=LQFP32_MUX_P2_2;
				break;
				case 2:
					AMX0P=LQFP32_MUX_P2_3;
				break;
				case 3:
					AMX0P=LQFP32_MUX_P2_0;
				break;
			}
			
			while (AD0BUSY); // Wait for conversion to complete
			v = ((ADC0L+(ADC0H*0x100))*VDD)/1023.0; // Read 0-1023 value in ADC0 and convert to volts
			
			// Display measured values
			switch(j)
			{
				case 0:
					printf("V0=%5.3fV, ", v);
				break;
				case 1:
					printf("V1=%5.3fV, ", v);
				break;
				case 2:
					printf("V2=%5.3fV, ", v);
				break;
				case 3:
					printf("V3=%5.3fV", v);
				break;
			}

		}
		printf("\x1B[K"); // ANSI escape sequence: Clear to end of line
		waitms(100);  // Wait 100ms before next round of measurements.
	 }  
}	

