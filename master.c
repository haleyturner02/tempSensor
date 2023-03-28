#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


volatile int i, n, sampleCount, current, sample = 0;
volatile float avg = 0;
volatile float ADC[9];
char packet[] = {0x03, 0x04};
volatile unsigned char col_holding, row_holding, pressed_key;
volatile unsigned char transmit_key;

void initI2C_master() {
    UCB1CTLW0 |= UCSWRST;       // SW RESET ON

    UCB1CTLW0 |= UCSSEL_3;      // SMCLK
    UCB1BRW = 10;               // Prescalar = 10

    UCB1CTLW0 |= UCMODE_3;      // Put into I2C mode
    UCB1CTLW0 |= UCMST;         // Set as MASTER
    UCB1CTLW0 |= UCTR;          // Put into Tx mode

    UCB1CTLW1 |= UCASTP_2;      // Enable automatic stop bit
    UCB1TBCNT = 1;              // Transfer byte count

    P6DIR |= (BIT6 | BIT5 | BIT4);      // Set P6.6-P6.4 as outputs
    P6OUT &= ~(BIT6 | BIT5 | BIT4);     // Clear P6.6-P6.4

    P4SEL1 &= ~BIT7;            // P4.7 SCL
    P4SEL0 |= BIT7;

    P4SEL1 &= ~BIT6;            // P4.6 SDA
    P4SEL0 |= BIT6;

    PM5CTL0 &= ~LOCKLPM5;       // Enable digital I/O
    UCB1CTLW0 &= ~UCSWRST;      // SW RESET OFF
}

void initTimerB0compare() {
    TB0CTL |= TBCLR;            // Clear TB0
    TB0CTL |= TBSSEL__ACLK;     // Select ACLK
    TB0CTL |= MC__UP;           // UP mode
}

void initTimerB1compare() {
    TB1CTL |= TBCLR;            // Clear TB1
    TB1CTL |= TBSSEL__SMCLK;    // Select SMCLK
    TB1CTL |= MC__UP;           // UP mode
    TB1CCR0 = 347985;           // Set CCR0 value (read sensor every 333ms)
    TB1CCTL0 &= ~CCIFG;         // Clear TB1 flag
    TB1CCTL0 |= CCIE;           // Enable TB1 interrupt
}

void configureAdc() {

    SYSCFG2 |= 0x0001 << ADCINCH_8;         // Configure ADC A8 pin (P5.0)
    ADCCTL0 &= ~ADCENC;                     // Disable ADC
    ADCCTL0 |= ADCSHT_2 | ADCON;            // ADC ON, 16 clocks
    ADCCTL1 = ADCSHP;                       // Sampling timer
    ADCCTL2 = ADCRES;                       // 10 bit resolution
    ADCMCTL0 = ADCINCH_8 | ADCSREF_0;       // ADC A8 input select, Vref of AVCC
    ADCIE = ADCIE0;                         // Enable ADC

}

void columnInput(){
    P3DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Initialize pins as input
    P3REN |= (BIT0 | BIT1 | BIT2 | BIT3);   // Enable pull up/down resistor
    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Configure resistors as pull down

    P3DIR |= (BIT4 | BIT5 | BIT6 | BIT7);   // Initialize pins as outputs
    P3OUT |= (BIT4 | BIT5 | BIT6 | BIT7);   // Set as outputs

    P3IES &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // L-H edge sensitivity
    P3IE |= (BIT0 | BIT1 | BIT2 | BIT3);    // Enable IRQs

    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Clear the P3 interrupt flags
}

void rowInput(){
    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7);  // Initialize pins as input
    P3REN |= (BIT4 | BIT5 | BIT6 | BIT7);   // Enable pull up/down resistor
    P3OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);  // Configure resistors as pull down

    P3DIR |= (BIT0 | BIT1 | BIT2 | BIT3);  // Initialize pins as outputs
    P3OUT |= (BIT0 | BIT1 | BIT2 | BIT3);  // Set as outputs
}

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	initI2C_master();
	initTimerB0compare();       // Initialize Timer B0 for keypad input
	initTimerB1compare();       // Initialize Timer B1 for sampling temperature sensor every 333ms
	configureAdc();             // Initialize ADC for receiving input signal from temperature sensor

	columnInput();

	PM5CTL0 &= ~LOCKLPM5;       // Turn on digital I/O

	__enable_interrupt();

	while(1) {

	    while(sample == 0) {}

	    TB1CCTL0 &= ~CCIE;

	    float unrounded_avg = 0;

	    if(sampleCount >= (n - 1) && n!=0) {                        // Transmit temperature average if enough samples have been recorded for desired window size

	        for(i = current; i > current - n; i--) {                // Calculate moving average for window size n
	            avg = avg + ADC[i];
	        }
	        unrounded_avg = avg / n;
	        avg = round(unrounded_avg * 10) / 10;                   // Round Celsius temperature to tenths place
	        transmitTemperature();                                  // Transmit rounded Celsius temperature
	        avg = round(((unrounded_avg + 273.15) * 10) / 10);      // Round Kelvin temperature to tenths place
	        transmitTemperature();                                  // Transmit reoundedKelvin temperature
	    } else if(sampleCount > 2) {                                // Transmit temperature average with window of 3 if at least 3 samples have been recorded, but user has not set n/set n > 3
	        unrounded_avg = (ADC[0] + ADC[1] + ADC[2]) / 3;         // Store unrounded average for accuracy in both Celsius and Kelvin temperatures
	        avg = round(unrounded_avg * 10) / 10;                   // Round Celsius temperature to tenths place
	        transmitTemperature();                                  // Transmit rounded Celsius temperature
	        avg = round(((unrounded_avg + 273.15) * 10) / 10);      // Round Kelvin temperature to tenths place
	        transmitTemperature();                                  // Transmit rounded Kelvin temperature
	    }

	    avg = 0;                                                    // Reset moving average
	    sample = 0;                                                 // Reset sample indicator

	    TB1CCTL0 |= CCIE;
	}
	
	return 0;
}

void delay1000() {
    int i;
    for(i = 0; i <= 1000; i++){}
}

void keyPressedAction() {
                                        // Use keypad input to set n for number values, or transmit */# to LCD slave
    switch(pressed_key) {
        case 0x87:          // 1
            n = 1;
            break;
        case 0x83:          // 2
            n = 2;
            break;
        case 0x81:          // 3
            n = 3;
            break;
        case 0x47:          // 4
            n = 4;
            break;
        case 0x43:          // 5
            n = 5;
            break;
        case 0x41:          // 6
            n = 6;
            break;
        case 0x27:          // 7
            n = 7;
            break;
        case 0x23:          // 8
            n = 8;
            break;
        case 0x21:          // 9
            n = 9;
            break;
        case 0x17:          // *
            n = 0;
            packet[0] = pressed_key;            // Transmit *
            UCB1I2CSA = 0x0068;                 // LCD Slave address = 0x68;
            UCB1CTLW0 |= UCTXSTT;               // Generate START condition
            UCB1TXBUF = packet[0];              // Place packet in Tx buffer
            UCB1IFG |= UCSTPIFG;                // Set stop condition flag
            UCB1IFG &= ~UCSTPIFG;               // Clear STOP flag
            UCB1IE &= ~UCTXIE0;                 // Clear transmit
            delay1000();
            break;
        case 0x11:
            n = 0;
            packet[0] = pressed_key;            // Transmit #
            UCB1I2CSA = 0x0068;                 // LCD Slave address = 0x68;
            UCB1CTLW0 |= UCTXSTT;               // Generate START condition
            UCB1TXBUF = packet[0];              // Place packet in Tx buffer
            UCB1IFG |= UCSTPIFG;                // Set stop condition flag
            UCB1IFG &= ~UCSTPIFG;               // Clear STOP flag
            UCB1IE &= ~UCTXIE0;                 // Clear transmit
            break;
        default:
            n = -1;
            break;

    }

    columnInput();                          // Set columns back to inputs
    P3IFG &= ~(BIT3 | BIT2 | BIT1 | BIT0);
}

void ADC_start() {
    ADCCTL0 |= ADCENC | ADCSC;              // Start sampling and conversion
    __bis_SR_register(LPM3_bits | GIE);     // Enter LPM0, ADC_ISR force exits
}

void ADC_stop() {
    ADCCTL0 &= ~(ADCENC | ADCON);           // Stop sampling
}

void sampleSensor() {

    if(sampleCount >= 8) {
        ADC[8] = ADC[7];
    }
    if(sampleCount >= 7) {
        ADC[7] = ADC[6];
    }
    if(sampleCount >= 6) {
        ADC[6] = ADC[5];
    }
    if(sampleCount >= 5) {
        ADC[5] = ADC[4];
    }
    if(sampleCount >= 4) {
        ADC[4] = ADC[3];
    }
    if(sampleCount >= 3) {
        ADC[3] = ADC[2];
    }
    if(sampleCount >= 2) {
        ADC[2] = ADC[1];
    }
    if(sampleCount >= 1) {
        ADC[1] = ADC[0];
    }

    delay1000();                    // Wait for ADC ref
    ADC_start();                    // Start sample from ADC
    ADC_stop();                     // Stop sample from ADC

    // Convert DN (ADCMEM0) to voltage to temperature (Celsius)

    ADC[0] = -1481.96 + sqrt( 2.1962*pow(10, 6) + (1.8639 - (ADCMEM0/(pow(2, 10))*3.3)) / (3.88*pow(10, -6))  );

    current = i;                    // Keep track of current sample

}

void getCharKey(int d) {
    switch(d) {                         // Set transmit key based on digit
        case 1:
            transmit_key = 0x87;
            break;
        case 2:
            transmit_key = 0x83;
            break;
        case 3:
            transmit_key = 0x81;
            break;
        case 4:
            transmit_key = 0x47;
            break;
        case 5:
            transmit_key = 0x43;
            break;
        case 6:
            transmit_key = 0x41;
            break;
        case 7:
            transmit_key = 0x27;
            break;
        case 8:
            transmit_key = 0x23;
            break;
        case 9:
            transmit_key = 0x21;
            break;
        case 0:
            transmit_key = 0x13;
            break;
        default:
            transmit_key = 0;
            break;
    }
}

void transmitTemperature() {

    int digit;

    if((avg / 100) > 1) {
        i = 4;                              // Temperature has 4 digits (XXX.X)
    } else if((avg / 10) > 1) {
        i = 3;                              // Temperature has 3 digits (XXX.X)
    } else {
        i = 2;                              // Temperature has 2 digits (X.X)
    }

    for(i; i > 0; i--) {                    // Get each individual digit in temperature, starting with greatest place value
        if(i == 4) {
            digit = round(avg / 100);
            avg = avg - digit * 100;
        } else if (i == 3) {
            digit = round(avg / 10);
            avg = avg - digit * 10;
        } else if (i == 2) {
            digit = round(avg);
            avg = avg - digit;
        } else {
            digit = round(avg * 10);
            avg = 0;
        }

        getCharKey(digit);                  // Get ASCII key for digit to transmit to LCD slave
        packet[0] = transmit_key;           // Place key in packet


        UCB1I2CSA = 0x0068;                 // LCD Slave address = 0x68;
        UCB1CTLW0 |= UCTXSTT;               // Generate START condition
        UCB1TXBUF = packet[0];              // Place packet in Tx buffer
        UCB1IFG |= UCSTPIFG;                // Set stop condition flag
        UCB1IFG &= ~UCSTPIFG;               // Clear STOP flag

        UCB1IE &= ~UCTXIE0;                 // Clear transmit
        delay1000();

    }
}

#pragma vector = PORT3_VECTOR
__interrupt void ISR_PORT3(void) {

    TB0CCTL0 |= CCIE;                       // Local IRQ enable for CCR0
    TB0CCR0 = 500;                          // Set CCR0 value (period)
    TB0CCTL0 &= ~CCIFG;                     // Clear CCR0 flag to begin count
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void) {
    TB0CCTL0 &= ~CCIE;                      // Disable TB0

    col_holding = P3IN;                     // Store keypad input column

    rowInput();                             // Set rows to inputs
    row_holding = P3IN;                     // Store keypad input row

    pressed_key = col_holding + row_holding;    // Add column and row to get key pressed

    keyPressedAction();                     // Use key pressed to set n or transmit */#
}

#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void) {
    TB1CCTL0 &= ~CCIE;                                          // Disable TB1
    sample = 1;                                                 // Set sample indicator

    sampleCount++;
    sampleSensor();                                             // Sample temperature sensor

    TB1CCTL0 &= ~CCIFG;                                         // Clear flag
    TB1CCTL0 |= CCIE;                                           // Enable TB1

}

#pragma vector = ADC_VECTOR
__interrupt void ISR_ADC(void) {
    switch(__even_in_range(ADCIV, ADCIV_ADCIFG)) {
        case ADCIV_ADCIFG:
            ADCIFG &= ~ADCIFG0;                             // Clear ADC flag
            __bic_SR_register_on_exit(LPM3_bits + GIE);     // Exit low power mode
            break;
        default:
            break;
    }
}

#pragma vector = EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_TX_ISR(void) {
    UCB1TXBUF = packet[0];
}
