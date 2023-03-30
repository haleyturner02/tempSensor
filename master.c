#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


volatile int i, j, n, sampleCount, sample = 0;
volatile float avg, unrounded_avg = 0;
volatile float ADC[9];
char packet[] = {0x12, 0x12, 0x12, 0x12};
volatile unsigned char col_holding, row_holding, pressed_key;
volatile unsigned char transmit_key;

void initI2C_master() {
    UCB1CTLW0 |= UCSWRST;               // SW RESET ON

    UCB1CTLW0 |= UCSSEL_3;              // SMCLK
    UCB1BRW = 10;                       // Prescalar = 10

    UCB1CTLW0 |= UCMODE_3;              // Put into I2C mode
    UCB1CTLW0 |= UCMST;                 // Set as MASTER
    UCB1CTLW0 |= UCTR;                  // Put into Tx mode

    UCB1CTLW1 |= UCASTP_2;              // Enable automatic stop bit
    UCB1TBCNT = sizeof(packet);         // Transfer byte count

    //UCB1I2CSA = 0x0068;                 // LCD Slave address = 0x68;

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

void transmit() {                       // Need to transmit to LED bar as well
    UCB1IE |= UCTXIE0;                  // Local Tx interrupt enable
    UCB1I2CSA = 0x0068;                 // LCD Slave address = 0x68;
    UCB1CTLW0 |= UCTXSTT;               // Generate START condition
    for(i = 0; i <= 5000; i++){}
    UCB1IE &= ~UCTXIE0;                 // Clear transmit
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


	    getAverage();


	    TB1CCTL0 |= CCIE;
	}
	
	return 0;
}

void delay1000() {
    int i;
    for(i = 0; i <= 1000; i++){}
}

void resetPacket() {
    packet[] = {0x12, 0x12, 0x12, 0x12};
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
            resetPacket();
            packet[0] = pressed_key;            // Transmit *
            transmit();
            break;
        case 0x11:
            n = 0;
            resetPacket();
            packet[0] = pressed_key;            // Transmit #
            transmit();
            break;
        case 0x80:
            resetPacket();
            packet[0] = pressed_key;            // Transmit A
            transmit();
            break;
        case 0x40:
            resetPacket();
            packet[0] = pressed_key;            // Transmit B
            transmit();
            break;
        case 0x20:
            resetPacket();
            packet[0] = pressed_key;            // Transmit C
            transmit();
            break;
        case 0x10:
            resetPacket();
            packet[0] = pressed_key;
            transmit();
            break;
        default:
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

}

void getCharKey(int d) {
    switch(d) {                         // Set transmit key based on digit
        case 1:
            transmit_key = 0x01;
            break;
        case 2:
            transmit_key = 0x02;
            break;
        case 3:
            transmit_key = 0x03;
            break;
        case 4:
            transmit_key = 0x04;
            break;
        case 5:
            transmit_key = 0x05;
            break;
        case 6:
            transmit_key = 0x06;
            break;
        case 7:
            transmit_key = 0x07;
            break;
        case 8:
            transmit_key = 0x08;
            break;
        case 9:
            transmit_key = 0x09;
            break;
        case 0:
            transmit_key = 0x00;
            break;
        case -1:                        // '.'
            transmit_key = 0x10;
            break;
        case -2:
            transmit_key = 0x12;        // Space holder for Kelvin temperature (no decimal)
            break;
        default:
            transmit_key = 0;
            break;
    }
}

void transmitTemperature() {

    int digit, j;

    avg = fabs(avg);
    unrounded_avg = fabs(unrounded_avg);

    if((unrounded_avg / 100) > 1) {
        for(i = 0; i < 4; i++) {
            if(i == 0) {
                digit = avg / 100;
                avg = avg - digit*100;
            } else if(i == 1) {
                digit = avg / 10;
                avg = avg - digit*10;
            } else if(i == 2) {
                digit = avg;
                avg = 0;
            } else if(i == 3) {
                digit = -2;
            }

            getCharKey(digit);
            packet[i] = transmit_key;
            delay1000();

        }
    } else if((unrounded_avg / 10) > 1) {
        //avg = avg * 10;
        for(i = 0; i < 4; i++) {
            if(i == 0) {
                digit = avg / 100;
                avg = avg - digit*100;
            } else if(i == 1) {
                digit = avg / 10;
                avg = avg - digit*10;
            } else if(i == 2) {
                digit = -1;
            } else if(i == 3) {
                digit = avg;
                avg = 0;
            }

            getCharKey(digit);
            packet[i] = transmit_key;
            delay1000();

        }
    }

    transmit();                             // Transmit temperature packet to LCD slave

}

void getAverage() {

    if(sampleCount > n - 1  && n!=0) {                          // Transmit temperature average if enough samples have been recorded for desired window size

            for(i = 0; i < n; i++) {                                // Calculate moving average for window size n
                avg = avg + ADC[i];
            }
            unrounded_avg = (avg / n) + 273.15;                     // Store unrounded average

            avg = round(unrounded_avg);                             // Calculate Kelvin temperature and round to ones place
            transmitTemperature();
            unrounded_avg = unrounded_avg - 273.15;
            avg = round((unrounded_avg) * 10);              // Calculature Celsius temperature and round to tenths place
            transmitTemperature();

        } else if(sampleCount > 2) {                                            // Transmit temperature average with window of 3 if at least 3 samples have been recorded, but user has not set n/set n > 3
            unrounded_avg = ((ADC[0] + ADC[1] + ADC[2]) / 3) + 273.15;          // Store unrounded average

            avg = round(unrounded_avg);                                         // Calculate Kelvin temperature and round to ones place
            transmitTemperature();
            unrounded_avg = unrounded_avg - 273.15;
            avg = round((unrounded_avg) * 10);                         // Calculature Celsius temperature and round to tenths place
            transmitTemperature();

        }

        avg = 0;                                                    // Reset moving average
        sample = 0;                                                 // Reset sample indicator
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

    keyPressedAction();                     // Use key pressed to set n or transmit */# to LCD, A/B/C/D to LED
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

    //UCB1TXBUF = packet[i];

    if(j == sizeof(packet) - 1)  {
        UCB1TXBUF = packet[j];
        j = 0;
    } else {
        UCB1TXBUF = packet[j];
        j++;
    }

}
