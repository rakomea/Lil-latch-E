/*
* Team: Lil Latch E
* 12/10/2019
* Team members:     
*               Clara Martinez Rubio
*               Soma Mizobuchi
*               Richard Akomea
*               Ronaldo Naveo
*/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <proc/p32mx795f512l.h>
#include <math.h>
#include <string.h>

#define SYS_FREQ 80000000
#define GAMEPAD_OFFSET 127
#define SAMPLE_TIME 10

//run at 80MHz
#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config POSCMOD = HS
#pragma config FNOSC = PRIPLL
#pragma config FPBDIV = DIV_1

char message[100]; //message received from UART

unsigned int pin = 13;   // global variable for current pin to connect to ADC. Pin 42

//for encoder use
volatile unsigned int new17 = 0, new18 = 0, new13 = 0, new14 = 0; //for the pins
unsigned int state = 0, oldstate = 0, state2 = 0, oldstate2 = 0; //state of the encoders
int count = 0, count2 = 0; //counter for each encoder

unsigned int sample_adc(int pin); //global function definition


//state of either encoder lines change (encoder A and B)
void __ISR(_CHANGE_NOTICE_VECTOR, IPL6SOFT) Ext2ISR( void ){ //interrupt with priority 6
  
	//MOTOR RIGHT
    //CN20 -RF4 (pin 47)
    new17 = PORTDbits.RD14; //CN pins that are being monitored
    //CN21 -RF5 (pin 48)
    new18 = PORTDbits.RD15;

    if (new17 == 0 && new18 == 0){ //encoder = 00
        state = 1; //state 00
        if (oldstate == 2){ //from 10 to 00
           count++; //increment
        }else if (oldstate == 4){ //from 01 to 00
           count--; //decrement
        }
        oldstate = state; //keep track of state to compare
    }
    if (new17 == 1 && new18 == 0){ //encoder = 10
       state = 2; //state 10
        if(oldstate == 1){  //from 00 to 10
            count--; //decrement
        }
        else if(oldstate == 3){ //from 11 to 10
            count++; //increment
        }
       oldstate = state; //keep track of state to compare
    } 
    if (new17 == 1 && new18 == 1){ //encoder = 11
        state = 3; //state 11
        if(oldstate == 4){ //from 01 to 11
            count++; //increment
        } 
        else if(oldstate == 2){ //from 10 to 11
            count--; //decrement
        }
        oldstate = state; //keep track of state to compare
    }
    if (new17 == 0 && new18 == 1){ //encoder = 01
        state = 4; //state 01
        if(oldstate == 1){ //from 00 to 01
            count++; //increment
        }
        else if(oldstate == 2){ //from 10 to 01
            count--; //decrement
        }
        oldstate = state; //keep track of state to compare
    }

    //MOTOR LEFT
    //CN16 -RD7 (pin 84)
    new13 = PORTDbits.RD7; //CN pins that are being monitored
    //CN15 -RD6 (pin 83)
    new14 = PORTDbits.RD6;

    if (new13 == 0 && new14 == 0){ //encoder = 00
    	state2 = 1; //state 00
    	if (oldstate2 == 2){ //from 10 to 00
       		count2++; //increment
    	}else if (oldstate2 == 4){ //from 01 to 00
       		count2--; //decrement
    	}
    	oldstate2 = state2; //keep track of state to compare
    }
    if (new13 == 1 && new14 == 0){ //encoder = 10
       	state2 = 2; //state 10
        if(oldstate2 == 1){  //from 00 to 10
            count2--; //decrement
        }
        else if(oldstate2 == 3){ //from 11 to 10
            count2++; //increment
        }
       oldstate2 = state2; //keep track of state to compare
    } 
    if (new13 == 1 && new14 == 1){ //encoder = 11
        state2 = 3; //state 11
        if(oldstate2 == 4){ //from 01 to 11
            count2++; //increment
        }
        else if(oldstate2 == 2){ //from 10 to 11
            count2--; //decrement
        }
        oldstate2 = state2; //keep track of state to compare
    }
    if (new13 == 0 && new14 == 1){ //encoder = 01
        state2 = 4; //state 01
        if(oldstate2 == 1){ //from 00 to 01
            count2++; //increment
        }
        else if(oldstate2 == 2){ //from 10 to 01
            count2--; //decrement
        }
        oldstate2 = state2; //keep track of state to compare
    }
 
    IFS1bits.CNIF = 0; //clear CN interrupt flag status bit  
}

void delay (int steps){ //delay method
    int i = 0; //initialize i 
    for (i = 0; i<steps; i++){} //specified delay
    return; //return control
}

void uart_read(char * message){ //read from UART method
    char dat = 0; //data being sent
    int complete = 0, num_bytes = 0; //is it complete
    while(!complete){ //wait till full
       if(U2STAbits.URXDA){ //when set indicates that the receive buffer contains data
           dat = (char)U2RXREG; //put data in the register
           if((dat == '\n' ) || (dat == '\r')){ //end of line is read
              complete = 1; //set complete, to mark end of message
           }else { //if data not received
             message[num_bytes] = dat; //load data into message array
             ++num_bytes; //increment array pointer (# of message)
           }
       }
    }
    message[num_bytes] = '\0'; //end of message array
 }

 void uart_initialize(int baud){ //initialize UART method
    
    TRISFbits.TRISF4 = 1;  //U2RX pin 49
    U2STAbits.URXEN = 1; //enables RX pin
    U2BRG = ((SYS_FREQ / baud) / 16) - 1; //80M/(16*baud)-1 
      
    U2MODEbits.UEN = 0b00; //only RX and TX
    U2MODEbits.BRGH = 0; // M = 16
    U2MODEbits.PDSEL = 0b00; //8 data bits, no parity   
    U2MODEbits.ON = 1; //enables UART
    U2MODEbits.STSEL = 0; //1 stop bit
 }

void LED_indicator(int data){ //led for the distance sensor
    // variable storing sequential bits of On LEDs from left to right
    int LED_values[9] = {0b00000000, 0b10000000, 0b11000000, 0b11100000, 0b11110000, 0b11111000, 0b11111100, 0b11111110, 0b11111111};
    int idx = data * 8 / 1023;  // derive the index(int) of the number of LEDs to turn on
    LATA = LED_values[idx];     // Set Latch A to values corresponding to # of LEDs.
}

unsigned int sample_adc(int pin){ //analog to digital converter
    AD1CHSbits.CH0SA = pin;     // connect pin to MUXA
    _CP0_SET_COUNT(0);  // reset CPU core timer
    unsigned int current = _CP0_GET_COUNT();    // get CPUs current value of core timer
    AD1CON1bits.SAMP = 1;       // start sampling
    while(_CP0_GET_COUNT() < current + SAMPLE_TIME){;} // sample for set sample time
    AD1CON1bits.SAMP = 0;       // stop sampling
    while(!AD1CON1bits.DONE){;} // wait until AD conversion finishes
    return ADC1BUF0;            // Return 10bit value stored in buffer
}

__ISR(_TIMER_2_VECTOR, IPL4SOFT) T2ISR(void){ //interrupt routine for ADC
    int dataAdc = 0;           // variable to store digital values from ADC
    dataAdc = sample_adc(pin);             // sample analog input from specified pin (2: pot, 3: temp)
    LED_indicator(dataAdc);                // update LED value
    
    if(dataAdc > 900){ //when within ~15cm of object
        //drive backwards
        //left motor 
        LATCbits.LATC1 = 1; //pin 6
        LATCbits.LATC2 = 0; //pin 7
        //right motor
        LATBbits.LATB9 = 1; //pin 32
        LATBbits.LATB10 = 0; //pin 33
        
        OC2RS = 0.75*PR2; // 75% PWM left motor
        OC3RS = 0.75*PR2; // 75% PWM right motor
        
        while(sample_adc(pin) > 850); //wait until is done backing up
        OC2RS = 0; //stop left motor 
        OC3RS = 0; //stop right motor
    }
    
    IFS0bits.T2IF = 0; //clear interrupt flag
}

int main(void){
    
    // LED init
    TRISA = 0xFF00; // Tri-State Buffer: output
    LATA = 0x00;  // Latch values: off
    
    //change notification pins
	TRISDbits.TRISD6 = 1; //for CN15 (pin 83) --not use for U2RX
    TRISDbits.TRISD7 = 1; //for CN16 (pin 84)
    TRISDbits.TRISD14 = 1; //for CN20 (pin 47) 
    TRISDbits.TRISD15 = 1; //for CN21 (pin 48) 
     
    TRISBbits.TRISB13 = 1;   // AN13 as input (pin 42)
    AD1PCFGbits.PCFG13 = 0;  // AN13 pin is an analog output
    // Analog Digital Converter Setup
    AD1CON3bits.ADCS = 3;   // 2 x k x Tpb, Tpb = 12.5ns @ 80MHz, Tad > 65 -> 2x3x12.5 = 75ns
    AD1CON1bits.ADON = 1;   // Turn on AD converter
    
    DDPCONbits.JTAGEN = 0; //DDPCON to 0
    
    __builtin_disable_interrupts(); //disable CPU interrupts
    INTCONbits.MVEC = 0x1; //multi-vector enabled
    CNCONbits.ON = 1; //configure Cn int, turn CN on
    //if 1 CN would be triggered when the program is called
    CNPUEbits.CNPUE20 = 1; //internal pull-up
    CNPUEbits.CNPUE21 = 1; //internal pull-up
    CNENbits.CNEN20 = 1; //enable use CN15
    CNENbits.CNEN21 = 1; // enable use CN16
    
    CNPUEbits.CNPUE16 = 1; //internal pull-up
    CNPUEbits.CNPUE15 = 1; //internal pull-up
    CNENbits.CNEN16 = 1; //enable use CN15
    CNENbits.CNEN15 = 1; // enable use CN16
    
    IPC6bits.CNIP = 6; //priority
    IPC6bits.CNIS = 2; //sub-priority
    IFS1bits.CNIF = 0; //clear the interrupt flag //if set to 1 it is forced into the interrupt inmediately
    IEC1bits.CNIE = 1; //enable the CN interrupts
    
    IPC2bits.T2IP = 4;  //priority 
    IPC2bits.T2IS = 1;  //sub-priority
    IFS0bits.T2IF = 0; //clear interrupt flag
    IEC0bits.T2IE = 1; //enable Timer 2 interrupts
    
    __builtin_enable_interrupts(); //re-enable CPU interrupts

    T2CONbits.ON = 1; //turn on Timer2
    
    // Output Compare 2: Motor 1 (L)
    OC2CONbits.ON = 0;      // Turn OC off during setup
    OC2CONbits.OCM = 0b110; // PWM Mode w/o fault pin
    OC2CONbits.OCTSEL = 0;  // Timer Select: Timer 2
    OC2R = 3999;               // Initial Duty Cycle: 0%
    OC2RS = 0; //duty cycle 

    // Output Compare 3: Motor 2 (R)
    OC3CONbits.ON = 0;      // Turn OC off during setup
    OC3CONbits.OCM = 0b110; // PWM Mode w/o fault pin
    OC3CONbits.OCTSEL = 0;  // Timer Select: Timer 2
    OC3R = 3999;               // Initial Duty Cycle: 0%
    OC3RS = 0; //duty cycle
   
    // Timer 2 Setup (TYPE B TIMER)
    T2CONbits.TCKPS = 2;   // Pre-scaler set to 1: 1 Pulse / 2 input. T = 25ns
    int Fsck = 10000; //frequency of clock to calculate PR
    PR2 = (SYS_FREQ / 2) / Fsck - 1;  // (80MHz / 2) /10kHz - 1 = 3999
    TMR2 = 0; // initialize Timer 2
    // Turn on modules:
    T2CONbits.ON = 1;   // Timer 2: ON
    OC2CONbits.ON = 1;  // OC2: ON
    OC3CONbits.ON = 1;  // OC3: ON
    
    uart_initialize(9600); // UART2 initialize baud rate = 9600

    /*==========================================*
     *              GPIO Pins                   *
     * ++++++++++++++++++++++++++++++++++++++++ *
     *      / Motor 1 (Left):                   *
     *          RC1  -> p6                      *
     *          RC2  -> p7                      *
     *      / Motor 2 (Right):                  *
     *          RB9 -> p32                      *
     *          RB10 -> p33                     *
     * =========================================*/
    /* NOTE:
     *  - Left and Right motors will have inverted M1Nx and M2Nx values respectively
     *    since they are arranged symmetrically with respect to the wheel base. I.E.
     *    for forward movement, M2 will technically be backwards rotation*/
    // Configure GPIO pins for output (5V)
    TRISCbits.TRISC1 = 0; //pin 6
    TRISCbits.TRISC2 = 0; //pin 7
    TRISBbits.TRISB9 = 0; //pin 32
    TRISBbits.TRISB10 = 0; //pin 33
    
    //initialize all pin to low
    //left motor
    LATCbits.LATC1 = 0; //pin 6
    LATCbits.LATC2 = 0; //pin 7
    //right motor
    LATBbits.LATB9 = 0; //pin 32
    LATBbits.LATB10 = 0; //pin 33
   
    char XY[2]; //for UART message parsing
    
    char dataStr[4];  // char array to store alphanumeric representation of digital values

    while(1){ //loop infinitely
        
        uart_read(message); //read the message sent by the master (raspberry pi)
        delay(10); //10 cycle delay
 
        if(message[0] == 'X'){ //if movement in x-axis 
            XY[0] = message[1]; //direction part of the message
            
        }else{ //if movement in y-axis
            XY[1] = message[1]; //direction part of the message
        } 
        
        
        if(XY[1] == 'C'){ //direction is center
            //left motor
            LATCbits.LATC1 = 0; //pin 6
            LATCbits.LATC2 = 1; //pin 7
            //right motor
            LATBbits.LATB9 = 0; //pin 32
            LATBbits.LATB10 = 1; //pin 33
            
            if(XY[0] == 'C'){  //center center condition          
                OC2RS = 0; //stop motor left
                OC3RS = 0; //stop motor right
            }
            //reverse polarity of right motor
            if(XY[0] == 'R'){ //center right condition 
                //right motor
                LATBbits.LATB9 = 1; //pin 32
                LATBbits.LATB10 = 0; //pin 33
                OC2RS = PR2; //full speed
                OC3RS = PR2; //full speed
            }
            //reverse polarity of left motor
            if(XY[0] == 'L'){ //center left condition 
            //left motor
                LATCbits.LATC1 = 1;  //pin 6
                LATCbits.LATC2 = 0;  //pin 7
                OC2RS = PR2; //full speed
                OC3RS = PR2; //full speed
            }
        }
        // Set each motor to opposing polarity to move in forward direction
        //move forward
        if(XY[1] == 'F'){ //forward condition
            //left motor
            LATCbits.LATC1 = 0; //pin 6
            LATCbits.LATC2 = 1;  //pin 7
            //right motor
            LATBbits.LATB9 = 0; //pin 32
            LATBbits.LATB10 = 1; //pin 33
            //straight back
            if(XY[0] == 'C'){               
                OC2RS = PR2; //full speed
                OC3RS = PR2; //full speed
            }
            //right back diagonal
            if(XY[0] == 'R'){
                OC2RS = 0; //stop motor left
                OC3RS = PR2; //full speed
            }
            //left back diagonal
            if(XY[0] == 'L'){
                OC2RS = PR2; //full speed
                OC3RS = 0; //stop motor right
            }
        }
        if(XY[1] == 'B'){ //backwards condition
            //left motor
            LATCbits.LATC1 = 1; //pin 6
            LATCbits.LATC2 = 0; //pin 7
            //right motor
            LATBbits.LATB9 = 1; //pin 32
            LATBbits.LATB10 = 0; //pin 33
            //straight back
            if(XY[0] == 'C'){             
                OC2RS = PR2; //full speed
                OC3RS = PR2; //full speed
            }
            if(XY[0] == 'R'){ //right back diagonal
                OC2RS = 0; //stop motor left
                OC3RS = PR2; //full speed
            }
            if(XY[0] == 'L'){ //left back diagonal
                OC2RS = PR2; //full speed
                OC3RS = 0; //stop motor right
            }
        }
       
    }
    
    return 0;
}

