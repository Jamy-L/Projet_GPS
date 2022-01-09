#include <string.h>
#include "LPC8xx.h"
#include "syscon.h"
#include "lib_ENS_II1_lcd.h"

#include "LPC8xx.h"
#include "syscon.h"
#include "swm.h"
#include "syscon.h"
#include "utilities.h"
#include "uart.h"
#include "chip_setup.h"

#define RX_BUFFER_SIZE 90
#define WaitForUART0txRdy  while(((LPC_USART0->STAT) & (1<<2)) == 0)
#define maxlen 82

unsigned char mot[RX_BUFFER_SIZE];
int indice_etoile=-3;

static uint32_t rx_char_counter = 0;

unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char chaine_nettoyee[RX_BUFFER_SIZE];
unsigned char chaine_ordonnee[RX_BUFFER_SIZE];

unsigned char chaine_heure[3];
unsigned char chaine_minute[3];
unsigned char chaine_seconde[3];
unsigned char chaine_longitude[9];
unsigned char chaine_latitude[9];
unsigned char chaine_direction_longitude[2];
unsigned char chaine_direction_latitude[2];
unsigned char texte_ecran[30];


enum {arret, marche} etat_lecture;
volatile enum {false, true} handshake;
enum {echec_heure, echec_position,succes} succes_lecture;


//
// Function name: UART0_IRQHandler
// Description:	  UART0 interrupt service routine.
//                This ISR reads one received char from the UART0 RXDAT register,
//                appends it to the rx_buffer array, and echos it back via the
//                UART0 transmitter. If the char. is 0xD (carriage return),
//                new line char (0xA) is appended to the array and echoed,
//                then a NUL char (0x0) is appended to the array to terminate the string
//                for future use.
// Parameters:    None
// Returns:       void
//
void UART0_IRQHandler() {
  unsigned char temp;

  temp = LPC_USART0->RXDAT;
  rx_buffer[rx_char_counter] = temp;        // Append the current character to the rx_buffer

  if ((temp=='*')&&(etat_lecture==marche)) indice_etoile=temp;
  if (temp=='$') etat_lecture=marche;

  if ((rx_char_counter==indice_etoile+2)&&(etat_lecture==marche)) {   // Fin de chaine.
    rx_buffer[rx_char_counter+1] = 0x0A;    // Append a new line character to rx_buffer.
    rx_buffer[rx_char_counter+2] = 0x00;    // Append a NUL terminator character to rx_buffer to complete the string.

    handshake = true;                       // Set handshake for main()
    indice_etoile=-3;
    rx_char_counter = 0;                    // Clear array index counter
    etat_lecture=arret;
  }
  else {                                    // Current character is not CR, keep collecting them.
    rx_char_counter++;                      // Increment array index counter.

    if (rx_char_counter == RX_BUFFER_SIZE){  // If the string overruns the buffer, stop here before all hell breaks lose.
    	rx_char_counter=0; //remise à zéro du chariot
    	indice_etoile=-3;
    	etat_lecture=arret;
    }
  }

  return;
}

void copier_nettoyer(unsigned char* chaine_origine,unsigned char* chaine_sortie){
	// copier une version nettoyée de la chaine d'origine dans la chaine de sortie
	unsigned char *ptr_origine=chaine_origine;
	unsigned char *ptr_sortie=chaine_sortie;
	while (*ptr_origine != '$')ptr_origine++; //on se débarasse des premiers char. Par construction, on est sur de trouver une dollard

	*ptr_sortie='$';
	ptr_origine++;ptr_sortie++;

	while (*ptr_origine!='*'){ //par construction, on est sur de trouver *
		*ptr_sortie=*ptr_origine;
		ptr_sortie++; ptr_origine++;
	}

	*ptr_sortie='*';
	ptr_sortie++; ptr_origine++;
	*ptr_sortie=*ptr_origine;
	ptr_sortie++; ptr_origine++;
	*ptr_sortie=*ptr_origine;
	ptr_sortie++;
	*ptr_sortie=0;//fin de chaine
}

void ordonner_chaine(unsigned char* chaine_origine, unsigned char* chaine_heure, unsigned char* chaine_minute, unsigned char* chaine_seconde, unsigned char* chaine_latitude,unsigned char* chaine_longitude, unsigned char* chaine_direction_latitude, unsigned char* chaine_direction_longitude){
	// extrait les données importantes et renvoit les données sous forme de chaines.
	//prend en entrée une chaine valide au format GPGGA
	int i=0;
	while (chaine_origine[i] != ',')i++;;
	i++;
	if (chaine_origine[i]==','){ //un champ est vide : lecture impossible
		succes_lecture = echec_heure;
	}

	else{
		succes_lecture=succes;
		chaine_heure[0]=chaine_origine[i++];
		chaine_heure[1]=chaine_origine[i++];
		chaine_heure[2]=0; //fin de chaine
		chaine_minute[0]=chaine_origine[i++];
		chaine_minute[1]=chaine_origine[i++];
		chaine_minute[2]=0;
		chaine_seconde[0]=chaine_origine[i++];
		chaine_seconde[1]=chaine_origine[i++];
		chaine_seconde[2]=0;
		while (chaine_origine[i] != ',')i++; //on ignore les 100èmes de seconde
		i++;
		if (chaine_origine[i]==','){ //pas de signal
			succes_lecture = echec_position;
		}
		else {
			chaine_latitude[0]=chaine_origine[i++];
			chaine_latitude[1]=chaine_origine[i++];
			chaine_latitude[2]=chaine_origine[i++];
			chaine_latitude[3]=chaine_origine[i++];
			chaine_latitude[4]=chaine_origine[i++];
			chaine_latitude[5]=chaine_origine[i++];
			chaine_latitude[6]=chaine_origine[i++];
			chaine_latitude[7]=chaine_origine[i++];
			chaine_longitude[8]=0;
			i++;
			chaine_direction_latitude[0]=chaine_origine[i++];
			i++;

			chaine_longitude[0]=chaine_origine[i++];
			chaine_longitude[1]=chaine_origine[i++];
			chaine_longitude[2]=chaine_origine[i++];
			chaine_longitude[3]=chaine_origine[i++];
			chaine_longitude[4]=chaine_origine[i++];
			chaine_longitude[5]=chaine_origine[i++];
			chaine_longitude[6]=chaine_origine[i++];
			chaine_longitude[7]=chaine_origine[i++];
			chaine_longitude[8]=0;
			i++;
			chaine_direction_longitude[0]=chaine_origine[i];

			}

		}
	}
//
// Main routine
//
int main(void) {
  // Enable clocks to relevant peripherals
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (UART0 | SWM);

  // Connect UART0 TXD, RXD signals to port pins
  ConfigSWM(U0_TXD, P0_18);
  ConfigSWM(U0_RXD, P0_20);

  // Configure UART0 for 9600 baud, 8 data bits, no parity, 1 stop bit.
  // For asynchronous mode (UART mode) the formula is:
  // (BRG + 1) * (1 + (m/256)) * (16 * baudrate Hz.) = FRG_in Hz.
  //
  // We proceed in 2 steps.
  // Step 1: Let m = 0, and round (down) to the nearest integer value of BRG for the desired baudrate.
  // Step 2: Plug in the BRG from step 1, and find the nearest integer value of m, (for the FRG fractional part).
  //
  // Step 1 (with m = 0)
  // BRG = ((FRG_in Hz.) / (16 * baudrate Hz.)) - 1
  //     = (15,000,000/(16 * 9600)) - 1
  //     = 96.66
  //     = 96 (rounded)
  //
  // Step 2.
  // m = 256 * [-1 + {(FRG_in Hz.) / (16 * baudrate Hz.)(BRG + 1)}]
  //   = 256 * [-1 + {(15,000,000) / (16*9600)(96+1)}]
  //   = 1.73
  //   = 2 (rounded)

  // Configure FRG0
  LPC_SYSCON->FRG0MULT = 0;
  LPC_SYSCON->FRG0DIV = 255;

  // Select main_clk as the source for FRG0
  LPC_SYSCON->FRG0CLKSEL = FRGCLKSEL_MAIN_CLK;

  // Select frg0clk as the source for fclk0 (to UART0)
  LPC_SYSCON->UART0CLKSEL = FCLKSEL_FRO_CLK;

  // Give USART0 a reset
  LPC_SYSCON->PRESETCTRL0 &= (UART0_RST_N);
  LPC_SYSCON->PRESETCTRL0 |= ~(UART0_RST_N);

  // Configure the USART0 baud rate generator
  LPC_USART0->BRG = 97;

  // Configure the USART0 CFG register:
  // 8 data bits, no parity, one stop bit, no flow control, asynchronous mode
  LPC_USART0->CFG = DATA_LENG_8|PARITY_NONE|STOP_BIT_1;

  // Configure the USART0 CTL register (nothing to be done here)
  // No continuous break, no address detect, no Tx disable, no CC, no CLRCC
  LPC_USART0->CTL = 0;

  // Clear any pending flags, just in case
  LPC_USART0->STAT = 0xFFFF;

  // Enable USART0
  LPC_USART0->CFG |= UART_EN;

  // Enable the USART0 RX Ready Interrupt
  LPC_USART0->INTENSET = RXRDY;
  NVIC_EnableIRQ(UART0_IRQn);


	char text[32];
	init_lcd();
	//sprintf(text,"UE441 - M1 - ENS");
	lcd_gohome();
	lcd_puts(text);

  while(1) {

    handshake = false;                                   // Clear handshake flag, will be set by ISR at end of user input
    while (handshake == false);                        // Wait here for handshake from ISR
    LPC_USART0->INTENSET &= ~RXRDY; //rx_buffer est occupé
    copier_nettoyer(rx_buffer, chaine_nettoyee);//copie et isole la ligne de rx_buffer dans chaine
    LPC_USART0->INTENSET |= RXRDY; //rx_buffer est dispo
    if ((*(chaine_nettoyee+3)=='G')&&(*(chaine_nettoyee+4)=='G')&&(*(chaine_nettoyee+5)=='A')){ //trame GNGGA
    	lcd_gohome();
    	ordonner_chaine(chaine_nettoyee, chaine_heure, chaine_minute, chaine_seconde, chaine_latitude, chaine_longitude, chaine_direction_latitude, chaine_direction_longitude);
    	if (succes_lecture == echec_position){
    		sprintf(texte_ecran,"%s:%s%:%s",
    		    	chaine_heure,chaine_minute, chaine_seconde);
    		lcd_puts(texte_ecran);
    		lcd_position(1,0);
    		lcd_puts("pas de position");
    	}
    	else if (succes_lecture == echec_heure){
    		lcd_puts("pas d'heure");
    		lcd_position(1,0);
    		lcd_puts("pas de position");

    	}

    	else {
    		sprintf(texte_ecran,"%s:%s%:%s",
    				chaine_heure,chaine_minute, chaine_seconde);
    		lcd_puts(texte_ecran);
    		lcd_position(1,0);
    		sprintf(texte_ecran, "%s%s %s%s",
					chaine_latitude,chaine_direction_latitude,
					chaine_longitude,chaine_direction_longitude);
    		lcd_puts(texte_ecran);

    		//lcd_puts(chaine_nettoyee);


    	}
    }

    }
  }



