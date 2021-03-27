#include <msp430.h>
#include "ADC.h"
#include <string.h>

//Cablage des captures
#define capIR BIT4                             //Capteur infrarouge P1.4

//SPI master clock data in/out
#define SCK         BIT5            // Serial Clock
#define DATA_OUT    BIT6            // DATA out
#define DATA_IN     BIT7            // DATA in

//Les valeurs preconception
#define vmaxG 1000                              //vitesse de roue gauche (0-1000)
#define vmaxD 1000                              //vitesse de roue droite (500-1000)

//Fonnction pour initialiser
void init_Infrarouge(void);
void init_Moteur(void);
void init_UART(void);
void direction_control(void);
void UART_Send_String(char *pbuff);
void init_SPI(void);



int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	             // stop watchdog timer
    //CLOCK
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

	ADC_init();

	init_Moteur();

	init_Infrarouge();

	init_UART();

	init_SPI();


	__delay_cycles(100000);
	__enable_interrupt();
	while(1);
}


//====================================interrupt============================================

//TX interrupt
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    //while(!(IFG2 & UCA0TXIFG));
    IFG2&=~UCA0TXIFG;
}



//RX interrupt
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
   //while (!(IFG2&UCA0TXIFG));
   IFG2&=~UCA0RXIFG;                // USCI_A0 TX buffer ready?
   //UCA0TXBUF = UCA0RXBUF;
   direction_control();
}


#pragma vector = TIMER0_A1_VECTOR
__interrupt void stop(void) {
    //stop
    P1DIR &= ~capIR;
    ADC_Demarrer_conversion(4);         //P1.4
    int res;
    res = ADC_Lire_resultat();
    int v1, v2;
    v1 = TA1CCR1;
    v2 = TA1CCR2;
    while (res > 500){
        TA1CCR1 = 0;
        TA1CCR2 = 500;
        ADC_Demarrer_conversion(4);
        res = ADC_Lire_resultat();
    }
    TA1CCR1 = v1;
    TA1CCR2 = v2;

    TA0CTL &= ~TAIFG;
}


//initialisation d'infrarouge
void init_Infrarouge(void)
{
    TA0CTL = 0 | (TASSEL_2 | ID_3 | MC_1 | TAIE);       //source SMCLK, prediviseur 8, mode up, enable interrupteur
    TA0CCR0 = 12500;                                    //100ms
    TA0CTL &= ~TAIFG;                                   //flag=0
}

//initialisation de moteur
void init_Moteur(void)
{
    P2SEL &= ~(BIT1|BIT4);                   //p2.1 p2.4 mode selectionner E/S    P2SEL2 &= ~(BIT1|BIT4);

    P2SEL |= BIT2|BIT5;                      //p2.2 p2.5 timer PWM
    P2SEL2 &= ~(BIT2|BIT5);
    P2DIR |= BIT1|BIT2|BIT4|BIT5;            //p2.1 p2.2 p2.4 p2.5 Output

    P2OUT &= ~BIT1;                          //roue gauche avancer
    P2OUT |= BIT4;                           //roue droite avancer

    TA1CTL = 0x0210;                         //mode up, source SMCLK
    TA1CCR0 = 1000;

    TA1CCTL1 = 0x00E0;                       //mode 7 Reset/Set
    TA1CCTL2 = 0x00E0;                       //mode 7 Reset/Set
    TA1CCR1 = 0;                             //roue gauche arreter
    TA1CCR2 = 500;                           //roue droite arreter
}



void init_UART(void){

      //GPIO
      P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
      P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD

      UCA0CTL1 |= UCSSEL_2;                     // SMCLK
      UCA0BR0 = 104;                            // 1MHz 9600
      UCA0BR1 = 0;                              // 1MHz 9600
      UCA0MCTL =UCBRS_0;                        // Modulation UCBRSx = 1
      UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

      IE2 |= UCA0RXIE+UCA0TXIE;                 //enable interrupt
      _enable_interrupts();
      //_bis_SR_register(LPM0_bits);
 }

void init_SPI( void )
{
    UCB0CTL1 |= UCSWRST;
    UCB0CTL1 |= UCSSEL_2;
    P1SEL |= (BIT5 | BIT6 | BIT7);
    P1SEL2 |= (BIT5 | BIT6 | BIT7);
    UCB0CTL0 &= ~(UCCKPH | UCCKPL);
    UCB0CTL0 |= (UCMODE_0 | UCMST | UCSYNC);
    UCB0CTL0 &= ~(UCMSB | UC7BIT);
    UCB0BR0 = 0x0A;
    UCB0BR1 = 0x00;
    UCB0CTL1 &= ~UCSWRST;
    IFG2 &= ~UCB0RXIFG;
}


void direction_control(){
    char com;
    com=UCA0RXBUF;
    switch(com){
        case 'h':
            UART_Send_String("h----help\n\r");
            UART_Send_String("8----run\n\r");
            UART_Send_String("5----stop\n\r");
            UART_Send_String("2----back\n\r");
            UART_Send_String("4----turn left\n\r");
            UART_Send_String("6----turn right\n\r");
            UART_Send_String("q----camera turn left\n\r");
            UART_Send_String("e----camera turn right\n\r");
            UART_Send_String("w----camera reposition\n\r");
            break;
        case '8':
            UART_Send_String("run\n\r");
            TA1CCR1 = 500;             //positif 0-1000
            TA1CCR2 = 750;             //positif 500-1000
            break;
        case '4':
            UART_Send_String("turn left\n\r");
            TA1CCR1 = 0;             //positif 0-1000
            TA1CCR2 = 750;             //positif 500-1000
            break;
        case '5':
            UART_Send_String("stop\n\r");
            P2OUT &= ~BIT1;
            TA1CCR1 = 0;             //positif 0-1000
            TA1CCR2 = 500;             //positif 500-1000
            break;
        case '2':
            UART_Send_String("back\n\r");
            P2OUT |= BIT1;
            TA1CCR1 = 500;
            TA1CCR2 = 250;
            break;
        case '6':
            UART_Send_String("turn right\n\r");
            TA1CCR1 = 500;             //positif 0-1000
            TA1CCR2 = 500;             //positif 500-1000
            break;
        case'q':
            UART_Send_String("camera turn left\n\r");
            UCB0TXBUF = 0x31;
            break;
        case'e':
            UART_Send_String("camera turn right\n\r");
            UCB0TXBUF = 0x32;
            break;
        case'w':
            UART_Send_String("camera reposition\n\r");
            UCB0TXBUF = 0x33;
            break;
        default:
            UART_Send_String("Command does not exist, use 'h' for help\n\r");
            break;
        }
        __delay_cycles(1000);

}

void UART_Send_String(char *pbuff)
{
    while(*pbuff != '\0'){
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = *pbuff;
        pbuff++;
        __delay_cycles(10000);
    }
}



