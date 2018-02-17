#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "pageESP8.h"
#include "sha1.h"
#include "base64.h"

#define F_CPU 14745600UL
#define BAUD_RATE 115200
#define UBRR (unsigned long) (((F_CPU/(BAUD_RATE * 16UL))) - 1)

void init_UART();
void init_ADC();
void init_INT();
void uart_send(uint8_t data);
void uart_send_string(char *str);
int uart_send_string_P(const char *str, long index);
int readResponseFromBuffer(char *line, int max);
void gestionCommandAT(char *etat, char *cmd, char *reponsePositive, char etatSuivantPositive, char *reponseNegative, char etatSuivantNegative, char etatTimeout);

//Buffer circulaire de reception
#define MAX_BUFFER 256
char BUFFER[MAX_BUFFER];
//Buffer pour stocker une ligne de reception
#define MAX_BUFFER_LINE 130
char BUFFER_LINE[MAX_BUFFER_LINE];
//Index pour la reception
int BUFFER_INDEX_WRITE = 0;
int BUFFER_INDEX_READ = 0;
int NB_RECEIVED_LINE = 0;
//Buffer pour la transmission des commandes AT comportant des variables (ex:nb char envoyés)
#define MAX_BUFFER_TX 100
char BUFFER_TX[MAX_BUFFER_TX];

char DATA_TX[8];

//Etats pour la fonction gestionDesCommandesAT
int CMD_STATE = 0;
#define AT_CMD_SEND	0
#define AT_CMD_LISTEN	1
#define AT_CMD_SUCCESS	2
#define AT_CMD_FAIL	3

//Etat de la machine
#define S_INIT_UC 	0
#define S_TEST_AT 	1
#define S_RESET_AT 	2
#define S_WIFI_MODE     3
#define S_AP_PARAM      4
#define S_MULTI_CONNECT 5
#define S_START_SERVER  6
#define S_SET_TIMEOUT   7
#define S_LISTEN_REQUEST     8
#define S_WAIT_END_RECEPTION 9
#define S_SEND_PAGE     10
#define S_SEND_DATA     11
#define S_CLOSE         12
#define S_OPEN_WS       13
#define S_SEND_OVER_WS  14
#define S_ARRET		100
#define S_TIMOUT        101
#define S_GET_STATUS    102

int CHANNEL_ID = 0;
int LENGHT = 0;
char KEY[26];

#define MAGIC_KEY "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
int NB_CHAR_RECEIVED = 0;

//Les types de requetes
#define R_NONE 0
#define R_PAGE 1
#define R_DATA 2
#define R_WS   3
#define R_SEND_OVER_WS 4

//Etat de l'envoie
#define E_INIT 0
#define E_SEND 1
#define E_TIME_OUT 2
#define E_CLOSE 3
#define E_ERROR_SEND 4
#define E_COMPLETE_SEND 5
#define E_CHECK_CONNECTION_STATUS 6
#define E_WEBSOCKET_CONNECTED 7


//----------------------------------------------------MAIN-----------------------------------------------
//----------------------Debut de la fonction main qui fait rouler la machine d'etat----------------------
//-------------------------------------------------------------------------------------------------------

//-------------------------ADC VARIABLES-------------------------
uint16_t previousValue = 0;

uint8_t webSocketWaitConnection = 0;
uint8_t webSocketConnected = 0;
uint8_t webSocketChannelID = 255;
uint8_t checkIsNewRequestIncomming = 0;
uint8_t nbReceivedChar = 0;

struct Frame {
	uint8_t isMasked;
	uint8_t isFinal;
	uint8_t opCode;
	uint8_t mask[4];
	uint8_t length; 
    char dataReceived[256];
} frame;

//--------------------------------------------------MAIN LOOP--------------------------------------------
int main(void){

  //Etat initiaux
  int etatMachine = S_INIT_UC;
  int requestType = R_NONE;
  int sendState = E_INIT;
  
  long indexCharTX = 0;
  long nbCharTX = 0;

	char uncodedKey[60];
	char codedKey[20];
	char codedKeyb64[28];
	char header[150];
  //Boucle infinie
  while(1){
    switch(etatMachine){
//Etapes d'initialisation---------------------------------------------------------------------------
    case S_INIT_UC:
      DDRC |= 1 << PINC0;
      DDRC &= ~(1 << PINC1);	//PC1 as input
      PORTC |= 1 << PINC1;	//Pullup
      init_UART();
      init_ADC();
      init_INT();
      etatMachine = S_TEST_AT;
      break;
    case S_TEST_AT:
      gestionCommandAT(&etatMachine, "AT\r\n", "OK\r\n", S_RESET_AT, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
    case S_RESET_AT:
      gestionCommandAT(&etatMachine, "AT+RST\r\n", "ready\r\n", S_AP_PARAM, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
    case S_WIFI_MODE:
      gestionCommandAT(&etatMachine, "AT+CWMODE=2\r\n", "OK\r\n", S_MULTI_CONNECT, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
    case S_AP_PARAM:
      gestionCommandAT(&etatMachine, "AT+CWSAP=\"ESP8266_seb\",\"123456789\",1,2\r\n", "OK\r\n", S_WIFI_MODE, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
    case S_MULTI_CONNECT:
      gestionCommandAT(&etatMachine, "AT+CIPMUX=1\r\n", "OK\r\n", S_START_SERVER, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
    case S_START_SERVER:
      gestionCommandAT(&etatMachine, "AT+CIPSERVER=1,80\r\n", "OK\r\n", S_SET_TIMEOUT, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
    case S_SET_TIMEOUT:
      gestionCommandAT(&etatMachine, "AT+CIPSTO=10\r\n", "OK\r\n", S_GET_STATUS, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
    case S_GET_STATUS:
      gestionCommandAT(&etatMachine, "AT+CIPSTATUS\r\n", "OK\r\n", S_LISTEN_REQUEST, "ERROR\r\n", S_ARRET, S_TIMOUT);
      break;
//Ecoute des requetes---------------------------------------------------------------------------------------------
    case S_LISTEN_REQUEST:
      //For debug purpose
      PORTC &= ~(1 << PINC0);
      //---------------------
      NB_CHAR_RECEIVED = 0;
      indexCharTX = 0;
      nbCharTX = 0;
      
      //On attend d'avoir une ligne dans BUFFER_LINE
      if(readResponseFromBuffer(BUFFER_LINE, MAX_BUFFER_LINE)){
	  	//On cherche la ligne +IPD, channelID, lenght:GET / HTTP/1.1
	  	if(sscanf(BUFFER_LINE,"+IPD,%d,%d:GET",&CHANNEL_ID,&LENGHT)==2){
		    //La prochaine etape est d attendre la fin de la reception
		    etatMachine = S_WAIT_END_RECEPTION;	    
		    
		    //On compte le nombre de char dans la premiere ligne a partir de :	    
		    NB_CHAR_RECEIVED = strlen(BUFFER_LINE);

		    int i = 0;
		    while(BUFFER_LINE[i] != '\0' && BUFFER_LINE[i] != ':'){
		      i++;
		      NB_CHAR_RECEIVED--;
		    }
		    NB_CHAR_RECEIVED--; //On enleve les : qui compte pour un char

		    //Determine le type de requete
		    if(strstr(BUFFER_LINE, "page") != NULL){
		      requestType = R_PAGE;
		    }
		    else if(strstr(BUFFER_LINE, "data") != NULL){
		      requestType = R_DATA;
		    }
		    else if(strstr(BUFFER_LINE, "websocket") != NULL){
		      requestType = R_WS;
		    }
		    else{
		      requestType = R_NONE;
		    }
	  	}
	  }
    break;
    case S_WAIT_END_RECEPTION:
      //On attend que tous les char aient ete recu avant de passer a l etape suivante
      if(NB_CHAR_RECEIVED < LENGHT){	
		if(readResponseFromBuffer(BUFFER_LINE, MAX_BUFFER_LINE)){
			//On cherche la clé pour la connexion WebSocket
			if(sscanf(BUFFER_LINE, "Sec-WebSocket-Key: %s",KEY)==1){
				//PORTC |= 1 << PINC0;
			}

	     	NB_CHAR_RECEIVED+=strlen(BUFFER_LINE);
		}
      }
      else{
		switch(requestType){
		case R_PAGE:
	  		//PORTC |= 1 << PINC3;	  
	  		etatMachine = S_SEND_PAGE;
	  		sendState = E_INIT;
	  	break;
		case R_DATA:
	  		//PORTC |= 1 << PINC2;
	  		etatMachine = S_SEND_DATA;
	  		sendState = E_INIT;
	  	break;
		case R_WS:
	  		etatMachine = S_OPEN_WS;
	  		sendState = E_INIT;
	  	break;
		case R_NONE:
	  		etatMachine = S_CLOSE;
	  		sendState = E_INIT;
	  	break;
	  	case R_SEND_OVER_WS:
	  		etatMachine = S_SEND_OVER_WS;
	  		sendState = E_INIT;
	  	break;
		}
      }
    break;
//Envoie d'une page internet---------------------------------------------------------------------
    case S_SEND_PAGE:
      switch(sendState){
      	case E_INIT:
			//Determine le nombre de char a envoyer (<2048)
			nbCharTX = strlen_P(PAGE_HTML);
			if(nbCharTX-indexCharTX > 2048){
			  sprintf(BUFFER_TX, "AT+CIPSEND=%d,2048\r\n",CHANNEL_ID);
			}
			else{
			  sprintf(BUFFER_TX, "AT+CIPSEND=%d,%d\r\n",CHANNEL_ID,nbCharTX-indexCharTX);	  
			}
			gestionCommandAT(&sendState, BUFFER_TX, "OK\r\n", E_SEND, "ERROR\r\n", E_ERROR_SEND, E_TIME_OUT); 
		break;

	    case E_SEND:
			indexCharTX=uart_send_string_P(PAGE_HTML, indexCharTX);
			//Si il reste des char a envoyer on recommence la fonction init
			if(indexCharTX<nbCharTX){
			  gestionCommandAT(&sendState, "", "SEND OK\r\n", E_INIT, "SEND FAIL\r\n", E_ERROR_SEND, E_ERROR_SEND);
			}
			//Sinon on ferme la connection
			else{
			  gestionCommandAT(&etatMachine, "", "SEND OK\r\n", S_CLOSE, "SEND FAIL\r\n", S_LISTEN_REQUEST, S_TIMOUT);
			}	        
		break;

	    case E_COMPLETE_SEND:
			gestionCommandAT(&etatMachine, "", "SEND OK\r\n", S_CLOSE, "SEND FAIL\r\n", S_LISTEN_REQUEST, S_TIMOUT);
		break;

	    case E_ERROR_SEND:
			etatMachine = S_LISTEN_REQUEST;
		break;
      }
    break;
//ENVOYER des Donnees -------------------------------------------------------------------------------
    case S_SEND_DATA:
      switch(sendState){
      	case E_INIT:
			itoa(previousValue, DATA_TX, 10);
			strcat(DATA_TX, "\r\n"); 	
			sprintf(BUFFER_TX, "AT+CIPSEND=%d,%d\r\n", CHANNEL_ID, strlen(DATA_TX));
			gestionCommandAT(&sendState, BUFFER_TX, "OK\r\n", E_SEND, "ERROR\r\n", E_ERROR_SEND, E_TIME_OUT);
		break;

      	case  E_SEND:
			uart_send_string(DATA_TX);
			sendState = E_COMPLETE_SEND;
		break;

      	case E_COMPLETE_SEND:
			gestionCommandAT(&etatMachine, "", "SEND OK\r\n", S_CLOSE, "SEND_FAIL\r\n", S_LISTEN_REQUEST, S_TIMOUT);
		break;
      }
    break;
//Ouvrir un webSocket -------------------------------------------------------------------------------
    case S_OPEN_WS:
      switch(sendState){
      	case E_INIT:

      		sprintf(uncodedKey, "%s%s", KEY, MAGIC_KEY);

      		SHA1Context sha;
    		SHA1Reset(&sha);
			SHA1Input(&sha, (unsigned char *) uncodedKey, strlen(uncodedKey));
			SHA1Result(&sha, codedKey);

			base64_encode(codedKeyb64, codedKey, strlen(codedKey));

			sprintf(header, "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n", codedKeyb64);
      		sprintf(BUFFER_TX, "AT+CIPSEND=%d,%d\r\n", CHANNEL_ID, strlen(header));
			gestionCommandAT(&sendState, BUFFER_TX, "OK\r\n", E_SEND, "ERROR\r\n", E_ERROR_SEND, E_TIME_OUT);
      	break;

      	case E_SEND:
      		uart_send_string(header);
			sendState = E_COMPLETE_SEND;
      	break;

      	case E_COMPLETE_SEND:
      		gestionCommandAT(&sendState, "", "SEND OK\r\n", E_CHECK_CONNECTION_STATUS, "ERROR\r\n", S_LISTEN_REQUEST, S_TIMOUT);
      		webSocketWaitConnection = 1;
      	break;
      	case E_CHECK_CONNECTION_STATUS:
      		gestionCommandAT(&sendState, "AT+CIPSTATUS\r\n", "OK\r\n", E_WEBSOCKET_CONNECTED, "ERROR\r\n", S_ARRET, S_TIMOUT);
      	break;
      	case E_WEBSOCKET_CONNECTED :
      		if(webSocketConnected == 1){
      			etatMachine = S_SEND_OVER_WS;
      			sendState = E_INIT;
      		}
      	break;
      }
    break;
//Send data over WebSocket----------------------------------------------------------------------------
    case S_SEND_OVER_WS:
      switch(sendState){

      	case E_INIT: ;
	    	char dataToSend[]= {0x81, 0x85, 0x37, 0xfa, 0x21, 0x3d, 0x7f, 0x9f, 0x4d, 0x51, 0x58};
	    	sprintf(BUFFER_TX, "AT+CIPSEND=%d,%d\r\n", CHANNEL_ID, sizeof(dataToSend));
	    	gestionCommandAT(&sendState, BUFFER_TX, "OK\r\n", E_SEND, "ERROR\r\n", E_ERROR_SEND, E_TIME_OUT);
    	break;

    	case E_SEND:
    		uart_send_string(dataToSend);
			sendState = E_COMPLETE_SEND;
    	break;

    	case E_COMPLETE_SEND:
    		gestionCommandAT(&etatMachine, "", "SEND OK\r\n", S_LISTEN_REQUEST, "SEND_FAIL\r\n", S_LISTEN_REQUEST, S_TIMOUT);
    	break;
      }
    break;

    case S_CLOSE:
      sendState = E_INIT;      
      sprintf(BUFFER_TX,"AT+CIPCLOSE=%d\r\n",CHANNEL_ID);
      gestionCommandAT(&etatMachine, BUFFER_TX, "OK\r\n", S_LISTEN_REQUEST, "ERROR\r\n", S_ARRET, E_TIME_OUT);
    break;
    case S_ARRET:

      break;
    case S_TIMOUT:

      break;
    }
  }

  return 1;
}

//---------------------------------------------------------------INIT_UART----------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
void init_UART(){
	UCSRB |= ((1 << RXEN) | (1 << TXEN));
	UCSRC |= ((1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0));

	UBRRH = (unsigned char)(UBRR) >> 8;
	UBRRL = (unsigned char) UBRR;

	UCSRB |= (1 << RXCIE);

	sei();
	_delay_ms(1000);
}

void init_INT(){
	GICR |= 1 << INT0;
	MCUCR |= 1 << ISC00;  
}

void uart_send(uint8_t data){
	while(!(UCSRA & (1 << UDRE))){
	//On ne fait rien
	}
	UDR = data;

}

void uart_send_string(char *str){
	while(*str != '\0'){
		uart_send(*str++);
	}
}

int uart_send_string_P(const char *str, long index){
  long i = 0;
  while((pgm_read_byte(str+index) != 0x00) && i < 2048){
    uart_send(pgm_read_byte(str+index));
    index++;
    i++;
  }
  return index;
}


void gestionCommandAT(char *etat, char *cmd, char *reponsePositive, char etatSuivantPositive, char *reponseNegative, char etatSuivantNegative, char etatTimeout){
	
	switch(CMD_STATE){
		case AT_CMD_SEND:
		  if(strlen(cmd) > 0){
		    uart_send_string(cmd);
		  }
		  CMD_STATE = AT_CMD_LISTEN;
		  break;
		case AT_CMD_LISTEN:
			if(readResponseFromBuffer(BUFFER_LINE, MAX_BUFFER_LINE)){
				if(strcmp(BUFFER_LINE, reponsePositive)==0){
					CMD_STATE = AT_CMD_SUCCESS;
				}
				else if(strcmp(BUFFER_LINE, reponseNegative)==0){
					CMD_STATE = AT_CMD_FAIL;
				}
				else if(sscanf(BUFFER_LINE,"+CIPSTATUS:%d,",&webSocketChannelID)==1){
					webSocketConnected = 1;
				}
				else{
				  
				}
			}

			break;
		case AT_CMD_SUCCESS:
			CMD_STATE = AT_CMD_SEND;
			*etat = etatSuivantPositive;
			break;
		case AT_CMD_FAIL:
			CMD_STATE = AT_CMD_SEND;
			*etat = etatSuivantNegative;
			break;
	}
}

int readResponseFromBuffer(char *line, int max){
  
  int nbLineReceivedCopy = 0;
  int i = 0;
  int endOfLine = 0;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    nbLineReceivedCopy = NB_RECEIVED_LINE;
  }
  
  if(nbLineReceivedCopy <= 0){
    endOfLine = 0;
  }
  else{
    do{
      if(BUFFER_INDEX_READ>=MAX_BUFFER-1){
		BUFFER_INDEX_READ = 0;
      }
      line[i++]=BUFFER[BUFFER_INDEX_READ++];
    }while(line[i-1] != '\n'&& i < max - 1);
    line[i]='\0';

    //Si une ligne a ete lu on decremente le nombre de ligne dans BUFFER
    if(line[i-1]=='\n'){
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	NB_RECEIVED_LINE--;
      }
    }
    endOfLine = 1;
  }

  return endOfLine;
}

void init_ADC(){
	//**************Configure AD Conversion*******************
	
	//ADC is slow so we must enable a prescaler determined by internal clock
	//input clock frequency between 50kHz and 200kHz
	// ADPS2 is a 16 prescaler
	ADCSRA |= 1 << ADPS2;	
	
	//8bits or 10bits results (Left shift or right shift one the results)
	ADMUX |= 1 << ADLAR;
	
	//Setup the voltage reference to AVCC with external capacitor at AREF pin
	ADMUX |= 1 << REFS0;
	
	//Enable interrupts function in ADC	
	ADCSRA |= 1 << ADIE;	
		
	//Turn the ADC On
	ADCSRA |= 1 << ADEN;
	
	//Enable global Interrupts
	sei();
		
	//Start the first conversion
	ADCSRA |= 1<<ADSC;
}

ISR(ADC_vect)
{
//*****************Add interrupts routine
	//Variable to store the lower bit of the ADC data register
	uint8_t lowerADC_bit = ADCL;
	
	//Put the ADCL and ADCH into the same 10 bit value
	uint16_t tenBitValue = ADCH << 2 | lowerADC_bit >> 6;
	uint8_t line = 0;

	/*switch(ADMUX){
		case(0x60):
			line = 0;
			ADMUX = 0x61;
		break;
		case(0x61):
			line = 1;
			ADMUX = 0x62;		
		break;
		case(0x62):
			line = 2;
			ADMUX = 0x60;		
		break;
		default:
		break;
	}*/
	if(tenBitValue != previousValue){
		previousValue = tenBitValue;
		
		//Lcd string variable declaration

	}

	//Start the next conversion
	ADCSRA |= 1 << ADSC;
//******************************
}

ISR(USART_RXC_vect){

  BUFFER[BUFFER_INDEX_WRITE] = UDR;

  /*if(checkIsNewRequestIncomming == 1 && webSocketWaitConnection == 1){
  
	if(BUFFER[BUFFER_INDEX_WRITE] == '+'){
		webSocketConnected = 1;
		nbReceivedChar++;
	}
	else{
		nbReceivedChar++;
	}
  }
*/
  if(BUFFER[BUFFER_INDEX_WRITE] == '\n'){
    NB_RECEIVED_LINE++;
    //checkIsNewRequestIncomming = 1;
  }

  BUFFER_INDEX_WRITE++;

  if(BUFFER_INDEX_WRITE >= MAX_BUFFER-1){
    BUFFER[MAX_BUFFER-1]='\0';
    BUFFER_INDEX_WRITE = 0;
  }
}

ISR(INT0_vect){
	//PORTC ^= 1 << PINC0;
}
