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
void SPI_Send_char(unsigned char carac);



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
    //-------UART
    if (IFG2 & UCA0RXIFG){
        //while (!(IFG2&UCA0TXIFG));
        IFG2&=~UCA0RXIFG;                // USCI_A0 TX buffer ready?
        //UCA0TXBUF = UCA0RXBUF;
        direction_control();
    }

    //--------------- SPI
    else if (IFG2 & UCB0RXIFG)
    {
        while( (UCB0STAT & UCBUSY) && !(UCB0STAT & UCOE) );
        while(!(IFG2 & UCB0RXIFG));
    }

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
      _bis_SR_register(LPM0_bits);
 }

void init_SPI( void )
{
    // Waste Time, waiting Slave SYNC
    __delay_cycles(250);

    // SOFTWARE RESET - mode configuration
    UCB0CTL0 = 0;
    UCB0CTL1 = (0 + UCSWRST*1 );

    // clearing IFg /16.4.9/p447/SLAU144j
    // set by setting UCSWRST just before
    IFG2 &= ~(UCB0TXIFG | UCB0RXIFG);

    // Configuration SPI (voir slau144 p.445)
    // UCCKPH = 0 -> Data changed on leading clock edges and sampled on trailing edges.
    // UCCKPL = 0 -> Clock inactive state is low.
    //   SPI Mode 0 :  UCCKPH * 1 | UCCKPL * 0
    //   SPI Mode 1 :  UCCKPH * 0 | UCCKPL * 0  <--
    //   SPI Mode 2 :  UCCKPH * 1 | UCCKPL * 1
    //   SPI Mode 3 :  UCCKPH * 0 | UCCKPL * 1
    // UCMSB  = 1 -> MSB premier
    // UC7BIT = 0 -> 8 bits, 1 -> 7 bits
    // UCMST  = 0 -> CLK by Master, 1 -> CLK by USCI bit CLK / p441/16.3.6
    // UCMODE_x  x=0 -> 3-pin SPI,
    //           x=1 -> 4-pin SPI UC0STE active high,
    //           x=2 -> 4-pin SPI UC0STE active low,
    //           x=3 -> i²c.
    // UCSYNC = 1 -> Mode synchrone (SPI)
    UCB0CTL0 |= ( UCMST | UCMODE_0 | UCSYNC );
    UCB0CTL0 &= ~( UCCKPH | UCCKPL | UCMSB | UC7BIT );
    UCB0CTL1 |= UCSSEL_2;

    UCB0BR0 = 0x0A;     // divide SMCLK by 10
    UCB0BR1 = 0x00;

    // SPI : Fonctions secondaires
    // MISO-1.6 MOSI-1.7 et CLK-1.5
    // Ref. SLAS735G p48,49
    P1SEL  |= ( SCK | DATA_OUT | DATA_IN);
    P1SEL2 |= ( SCK | DATA_OUT | DATA_IN);

    UCB0CTL1 &= ~UCSWRST;                                // activation USCI
}


void direction_control(){
    char com;
    com=UCA0RXBUF;
    switch(com){
        case 'h':
            UART_Send_String("h----help\n\r");
            UART_Send_String("w----run\n\r");
            UART_Send_String("s----stop\n\r");
            SPI_Send_char(0x31);
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


void SPI_Send_char(unsigned char data)
{
    while ((UCB0STAT & UCBUSY));   // attend que USCI_SPI soit dispo.
    while(!(IFG2 & UCB0TXIFG)); // p442
    UCB0TXBUF = data;         // Put character in transmit buffer
}
