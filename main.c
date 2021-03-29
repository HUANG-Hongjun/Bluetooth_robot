#include <msp430.h>
#include "ADC.h"
#include <string.h>

/*Cablage des captures*/
#define capIR BIT4                             /*Capteur infrarouge P1.4*/

/*obstacle 0 = false 1= true*/
int IsObstacleInfront = 0;
int IsObstacleLeft = 0;
int IsObstacleRight = 0;

/*position camare
 *1=left 2=middle 3=right*/
int PositionCamera =2; 


/*Fonnction pour initialiser*/
void init_Infrarouge(void);
void init_Moteur(void);
void init_UART(void);
void command_reaction(void);
void UART_Send_String(char *pbuff);
void init_SPI(void);
void robot_stop(void);



int main(void)
{
	/* stop watchdog timer*/
	WDTCTL = WDTPW | WDTHOLD;	             
	/*CLOCK*/
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
	while(1){};
}


/*====================================interrupt============================================*/

/*TX interrupt*/
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	//while(!(IFG2 & UCA0TXIFG));
	IFG2&=~UCA0TXIFG;
}



/*RX interrupt*/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
   	//while (!(IFG2&UCA0TXIFG));
	/* USCI_A0 TX buffer ready? */
   	IFG2&=~UCA0RXIFG;                
   	//UCA0TXBUF = UCA0RXBUF;
   	command_reaction();
}


#pragma vector = TIMER0_A1_VECTOR
__interrupt void stop(void) {
    P1DIR &= ~capIR;
    ADC_Demarrer_conversion(4);         
    SINT_32 res;
    res = ADC_Lire_resultat();

    /*test if obstacle*/
    if (res > 500){
        /*obstacle infront*/
        if (PositionCamera == 2){
            robot_stop();
            if (IsObstacleInfront == 0){
                IsObstacleInfront = 1;
                UART_Send_String("Obstacle infront\n\r");
            }
        }
        /*obstavle left*/
        else if (PositionCamera == 1){
            robot_stop();
            if (IsObstacleLeft == 0){
                IsObstacleLeft = 1;
                UART_Send_String("Obstacle Left\n\r");
            }
        }
        /*obstacle right*/
        else{
            robot_stop();
            if (IsObstacleRight == 0){
                IsObstacleRight = 1;
                UART_Send_String("Obstacle Right\n\r");
            }
        }
    }
    /*no obstalce*/
    else{
        /*infront*/
        if (PositionCamera == 2){
            if(IsObstacleInfront == 1){
                IsObstacleInfront = 0;
                UART_Send_String("No obstacle Infront\n\r");
            }
        }
        /*left*/
        else if (PositionCamera == 1){
            if(IsObstacleLeft == 1){
                IsObstacleLeft = 0;
                UART_Send_String("No obstacle Left\n\r");
            }
        }
        /*right*/
        else{
            if(IsObstacleRight == 1){
                IsObstacleRight = 0;
                UART_Send_String("No obstacle Right\n\r");
            }
        }
    }

    TA0CTL &= ~TAIFG;
}


/*initialisation d'infrarouge*/
void init_Infrarouge(void)
{
	/*source SMCLK, prediviseur 8, mode up, enable interrupteur*/
    	TA0CTL = 0 | (TASSEL_2 | ID_3 | MC_1 | TAIE);
	/*100ms*/       
    	TA0CCR0 = 12500;
	/*flag=0*/                                    
    	TA0CTL &= ~TAIFG;                                   
}

/*initialisation de moteur*/
void init_Moteur(void)
{
	/*p2.1 p2.4 mode selectionner E/S    */
    	P2SEL &= ~(BIT1|BIT4);                   
	P2SEL2 &= ~(BIT1|BIT4);
	
	/*p2.2 p2.5 timer PWM*/
    	P2SEL |= BIT2|BIT5;                      
    	P2SEL2 &= ~(BIT2|BIT5);
	/*p2.1 p2.2 p2.4 p2.5 Output*/
    	P2DIR |= BIT1|BIT2|BIT4|BIT5;            
	
	/*roue gauche avancer*/
    	P2OUT &= ~BIT1;
	/*roue droite avancer*/                          
    	P2OUT |= BIT4;                           
	
	/*mode up, source SMCLK*/
    	TA1CTL = 0x0210;                         
    	TA1CCR0 = 1000;
	/*mode 7 Reset/Set*/
    	TA1CCTL1 = 0x00E0;                       
    	TA1CCTL2 = 0x00E0; 
      /*robot stop*/               
    	TA1CCR1 = 0;                             
    	TA1CCR2 = 500;                  
}


/*initialisation bluetooth*/
void init_UART(void){

      /*GPIO
	 *P1.1 = RXD, P1.2=TXD*/
      P1SEL = BIT1 + BIT2 ;                     
      P1SEL2 = BIT1 + BIT2 ;                    

	/* SMCLK*/
      UCA0CTL1 |= UCSSEL_2;
	/* 1MHz 9600*/                     
      UCA0BR0 = 104;                            
      UCA0BR1 = 0;
	/* Modulation UCBRSx = 1*/                              
      UCA0MCTL =UCBRS_0;
	/*Initialize USCI state machine*/                        
      UCA0CTL1 &= ~UCSWRST;                     

	/*enable interrupt*/
      IE2 |= UCA0RXIE+UCA0TXIE;                 
      _enable_interrupts();
      //_bis_SR_register(LPM0_bits);
 }

/*initialisation SPI*/
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


void command_reaction(void){
    char com;
    com=UCA0RXBUF;
    /*switch command
     * h == help
     * 8 == run
     * 5 == stop
     * 2 == back
     * 4 == turn left
     * 6 == turn right
     * q == camera turn left
     * e == camera turn right
     * w == camera reposition*/
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
            if(IsObstacleInfront == 0){
               	UART_Send_String("run\n\r");
		    	/*positif 0-1000*/
                	TA1CCR1 = 500;
			/*positif 500-1000*/             
                	TA1CCR2 = 750;             
            }
            else{
                	UART_Send_String("Obstacle Infront\n\r");
                	robot_stop();
            }
            break;
        case '4':
            if(IsObstacleLeft == 0){
                	UART_Send_String("turn left\n\r");
                	TA1CCR1 = 0;             
                	TA1CCR2 = 750;             
            }
            else{
                	UART_Send_String("Obstacle Left\n\r");
                	robot_stop();
            }
            break;
        case '5':
            UART_Send_String("stop\n\r");
            P2OUT &= ~BIT1;
            robot_stop();
            break;
        case '2':
            UART_Send_String("back\n\r");
            P2OUT |= BIT1;
            TA1CCR1 = 500;
            TA1CCR2 = 250;
            break;
        case '6':
            if(IsObstacleRight == 0){
                UART_Send_String("turn right\n\r");
                TA1CCR1 = 500;             
                TA1CCR2 = 500;             
            }
            else{
                UART_Send_String("Obstacle Right\n\r");
                robot_stop();
            }
            break;
        case'q':
            UART_Send_String("camera turn left\n\r");
            PositionCamera = 1;
            UCB0TXBUF = 0x31;
            break;
        case'e':
            UART_Send_String("camera turn right\n\r");
            PositionCamera = 3;
            UCB0TXBUF = 0x32;
            break;
        case'w':
            UART_Send_String("camera reposition\n\r");
            PositionCamera = 2;
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
        while(UCA0STAT & UCBUSY){};
        UCA0TXBUF = *pbuff;
        pbuff++;
    }
}

void robot_stop(void){
    TA1CCR1 = 0;
    TA1CCR2 = 500;
}



