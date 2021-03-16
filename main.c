#include <msp430.h>
#include "ADC.h"

//Cablage des captures
#define boutonStart BIT3                       //Bouton de commencer P1.3
#define capIR BIT4                             //Capteur infrarouge P1.4
#define capSLG BIT5                            //Capteur Suiveur Ligne Gauche P1.5
#define capSLD BIT6                            //Capteur Suiveur Ligne Droite P1.6
#define capLUMG BIT1                           //Capteur Lumiere Gauche P1.1
#define capLUMD BIT0                           //Capteur Lumiere Droite P1.0

//Les valeurs preconception
#define vmaxG 900                              //vitesse de roue gauche (0-1000)
#define vmaxD 950                              //vitesse de roue droite (500-1000)
#define lumSeilG 950                            //Seuil du capteur lumiere gauche
#define lumSeilD 700                            //Seuil du capteur lumiere droite

//Les valeurs intermediaire
int start=0;                                    //etat du robot 0:arrete; 1:commence
int lumG, lumD;                                 //Les valeurs detecte par les capteurs lumieres

//Fonnction pour initialiser
void initBoutonStart(void);
void initInfrarouge(void);
void initMoteur(void);
void initSuiveurLigne(void);
void initLum(void);
void USCI_A0_init(void);
void direction_control();
void UARTSendString(char *pbuff);



int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	             // stop watchdog timer
    //CLOCK
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

	ADC_init();
	
	initMoteur();
	//initSuiveurLigne();
	initInfrarouge();
	//initLum();
	//initBoutonStart();
	USCI_A0_init();

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

#pragma vector = PORT1_VECTOR
__interrupt void tourner(void){
    if (((P1IFG & boutonStart) == boutonStart) && (start == 0))
    {
        TA1CCR1 = 1000;             //positif 0-1000
        TA1CCR2 = 1000;             //positif 500-1000
        //while (((P1IN & capSLG) == 0) | ((P1IN & capSLD) == 0));
        __delay_cycles(100000);
        start = 1;
        P1IFG &= ~(boutonStart);
    }
    /*
    else if (((P1IFG & capSLG) == capSLG) && (start == 1))
    {
        TA1CCR1 = 0;
        TA1CCR2 = 750;
        while ((P1IN & capSLG) == 0)
        {
            if (TA1CCR2 < vmaxD)
                TA1CCR2 = TA1CCR2+5;
            __delay_cycles(10000);
            if ((TA1CCR2 >= vmaxD-100) && ((P1IN & capSLD) == 0))
            {
                start = 0;
            }
        }
        if (start == 0)
        {
            TA1CCR2 = 250;
            while ((P1IN & capSLD) == 0);
            TA1CCR1 = vmaxG-100;
            TA1CCR2 = vmaxD;
            __delay_cycles(200000);
            while (((P1IN & capSLG) == 0) && ((P1IN & capSLD) == 0));
            TA1CCR1 = 0;
            TA1CCR2 = 500;
            P1IFG &= ~(capSLG|capSLD);
        }
        else
        {
            TA1CCR1 = vmaxG;
            TA1CCR2 = vmaxD;
            P1IFG &= ~(capSLG|capSLD);
        }

    }
    else if (((P1IFG & capSLD) == capSLD) && (start == 1))
    {
        TA1CCR1 = 500;
        TA1CCR2 = 500;
        while ((P1IN & capSLD) == 0)
        {
            if (TA1CCR1 < vmaxG)
                TA1CCR1 = TA1CCR1+10;
            __delay_cycles(10000);
            if ((TA1CCR1 >= vmaxG-200) && ((P1IN & capSLG) == 0))
            {
                start = 0;
            }
        }
        if (start == 0)
        {
            P2OUT |= BIT1;
            TA1CCR1 = 500;
            while ((P1IN & capSLG) == 0);
            P2OUT &= ~BIT1;
            TA1CCR1 = vmaxG;
            TA1CCR2 = vmaxD-50;
            __delay_cycles(200000);
            while (((P1IN & capSLG) == 0) && ((P1IN & capSLD) == 0));
            TA1CCR1 = 0;
            TA1CCR2 = 500;
            P1IFG &= ~(capSLG|capSLD);
        }
        else
        {
            TA1CCR1 = vmaxG;
            TA1CCR2 = vmaxD;
            P1IFG &= ~(capSLG|capSLD);
        }

    }
    */
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void stopOrTurn(void) {
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

    /*
    //turn
    ADC_Demarrer_conversion(1);
    lumG = ADC_Lire_resultat();
    ADC_Demarrer_conversion(0);
    lumD = ADC_Lire_resultat();
    if ((lumG > lumSeilG) && (lumD < lumSeilD) && (start == 1))
    {
        TA1CCR1 = 0;
        TA1CCR2 = 750;
        while (lumG > lumSeilG -5)
        {
            ADC_Demarrer_conversion(1);
            lumG = ADC_Lire_resultat();
        }
        TA1CCR1 = vmaxG;
        TA1CCR2 = vmaxD;
    }
    else if ((lumD > lumSeilD) && (lumG < lumSeilG) && (start == 1))
    {
        TA1CCR1 = 500;
        TA1CCR2 = 500;
        while (lumD > lumSeilD -5)
        {
            ADC_Demarrer_conversion(0);
            lumD = ADC_Lire_resultat();
        }
        TA1CCR1 = vmaxG;
        TA1CCR2 = vmaxD;
    }
    */
    TA0CTL &= ~TAIFG;
}







//initialisation de bouton
void initBoutonStart(void)
{
    P1SEL &= ~(boutonStart);                //mode selectionner E/S
    P1SEL2 &= ~(boutonStart);
    P1DIR &= ~(boutonStart);                //Input
    P1REN |= boutonStart;                   //enable resistance interne
    P1OUT |= boutonStart;                   //Pull-up
    P1IE |= boutonStart;                    //enable interrupteur
    P1IES |= boutonStart;                   //detect front montant
    P1IFG &= ~(boutonStart);                //flag=0
}

//initialisation d'infrarouge
void initInfrarouge(void)
{
    TA0CTL = 0 | (TASSEL_2 | ID_3 | MC_1 | TAIE);       //source SMCLK, prediviseur 8, mode up, enable interrupteur
    TA0CCR0 = 12500;                                    //100ms
    TA0CTL &= ~TAIFG;                                   //flag=0
}

//initialisation de moteur
void initMoteur(void)
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

//initialisation de suiveur-ligne
void initSuiveurLigne(void)
{
    P1SEL &= ~(capSLG|capSLD);             //mode selectionner E/S
    P1SEL2 &= ~(capSLG|capSLD);
    P1DIR &= ~(capSLG|capSLD);             //Input
    P1IE |= capSLG|capSLD;                 //enable interrupteur
    P1IES |= capSLG|capSLD;                //detect front montant
    P1IFG &= ~(capSLG|capSLD);             //flag=0
}

//initialisation de capteur lumiere
void initLum(void)
{
    P1SEL &= ~(capLUMG | capLUMD);           //mode selectionner E/S
    P1SEL2 &= ~(capLUMG | capLUMD);
    P1DIR &= ~(capLUMG | capLUMD);           //Input
}

void USCI_A0_init(void){

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


void direction_control(){
    char com;
    com=UCA0RXBUF;
    switch(com){
    case 'h':
        UARTSendString("h----help\n\r");
        UARTSendString("w----run\n\r");
        UARTSendString("s----stop\n\r");
        //UARTSendString("s----change state of LED\n\r");
        break;
    case '8':
        UARTSendString("run\n\r");
        TA1CCR1 = 1000;             //positif 0-1000
        TA1CCR2 = 1000;             //positif 500-1000
        break;
    case '4':
        UARTSendString("turn left\n\r");
        TA1CCR1 = 0;             //positif 0-1000
        TA1CCR2 = 750;             //positif 500-1000
        break;
    case '5':
        UARTSendString("stop\n\r");
        P2OUT &= ~BIT1;
        TA1CCR1 = 0;             //positif 0-1000
        TA1CCR2 = 500;             //positif 500-1000
        break;
    case '2':
        UARTSendString("back\n\r");
        P2OUT |= BIT1;
        TA1CCR1 = 500;
        TA1CCR2 = 250;
        break;
    case '6':
        UARTSendString("turn right\n\r");
        TA1CCR1 = 500;             //positif 0-1000
        TA1CCR2 = 500;             //positif 500-1000
        break;

    default:
        UARTSendString("Command does not exist, use 'h' for help\n\r");
        break;
    }
    __delay_cycles(1000);
}

void UARTSendString(char *pbuff)
{
    while(*pbuff != '\0'){
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = *pbuff;
        pbuff++;
        __delay_cycles(10000);
    }
}
