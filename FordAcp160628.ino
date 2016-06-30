/*
 * Original code for Yampp -  Andrew Hammond <andyh@mindcode.co.uk>
 *
 * For Arduino Mega 2560 - Krzysztof Pintscher <niou.ns@gmail.com>
 * Full schematics and connection by Krzysztof Pintscher
 * please keep our credits in header :)
 *
 *
 ******************************************************************
 * Revision
 *
 * 2013-12-17 version 0.9 - public relase
 *
 * 2016-06-30 version 0.91
 *    Sends buttoncodes via I2C. Can disable LCD. 
 * Based on http://www.instructables.com/id/Ford-CD-Emulator-Arduino-Mega/?ALLSTEPS
 * I2C <-> RPI, https://oscarliang.com/raspberry-pi-arduino-connected-i2c/
 */

//#define LCD   // Activate if LCD is used

#include <avr/interrupt.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/delay.h>
#ifdef LCD
#include <LiquidCrystal.h>
#endif
#include <TimerOne.h>

// For I2C:
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
// Set D8 high when requesting to send a message tp the Rpi
int i2cInt = 8;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#define outp(VAL,ADRESS) ((ADRESS) = (VAL))
#define inp(VAL) (VAL)

typedef unsigned char u08;
typedef unsigned short u16;
typedef unsigned long  u32;

unsigned char acp_status;
uint8_t acp_rx[12];
uint8_t acp_tx[12];
uint8_t acp_rxindex;
uint8_t acp_txindex;
unsigned char acp_txsize;
unsigned char acp_timeout;
unsigned char acp_checksum;
unsigned char acp_retries;
unsigned char acp_mode;
uint16_t acp_ltimeout;
unsigned char isPlaying;
unsigned char bLoudness;
unsigned char bRandom;
unsigned char bFRSearch;
int wPlayTime = 0;
int currentTrack = 1;
boolean reset_timer = false;

#define UART_BAUD_RATE   9600
#define UART_BAUD_SELECT ((u32)((u32)16000000/(UART_BAUD_RATE*16L)-1))

#define ACP_LISTEN  0
#define ACP_SENDACK 1
#define ACP_MSGREADY 2
#define ACP_WAITACK 3
#define ACP_SENDING 4

// Arduino's Pin 28 Connected to Pins 2 & 3 (RE/DE) on MAX485
int switchPin = 28;

#ifdef LCD
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
#endif

SIGNAL(USART1_RX_vect)
{
  u08 eod;
  uint8_t ch;
  
  eod = (inp(UCSR1B) & _BV(RXB81));
  ch = inp(UDR1);
  if (acp_status != ACP_LISTEN)
  {
    return; // ignore incoming msgs if still busy processing
  }
  if(!eod)
  {
    acp_checksum += ch;
  }
  acp_rx[acp_rxindex++] = ch;


  if(acp_rxindex>12)
  {          
    acp_reset();    
  } 

  else if (eod) 
  {
    if (acp_checksum == ch)    // Valid ACP message - send an ack
    {              
      acp_status=ACP_SENDACK;
      acp_handler();
    }
    else 
    {      
      acp_reset();    // Abort message
    }
  }
  return; 
}

void acp_uart_init(unsigned short baud)
{
  cli();
  outp((u08)(baud>>8), UBRR1H);                  
  outp((u08)baud, UBRR1L);
  sbi(UCSR1B, RXEN1);
  sbi(UCSR1B, TXEN1);
  sbi(UCSR1B, RXCIE1);  
  cbi(UCSR1B, UDRIE1);
  sbi(UCSR1B, UCSZ12);
  sei();
  #ifdef LCD
  lcd.setCursor(0,1);
  lcd.print("Waiting...");  
  #endif

}

void acp_txenable(boolean enable)
{
  if (enable){
    PORTA |= (1<<PA6); //ustawia stan wysoki na pinie PD4
  }
  else {
    PORTA &= ~(1<<PA6); //ustawia stan niski na pinie PD4
  }
  asm volatile("nop");
}

void acp_sendack(void)
{
  while(!(inp(PORTD) & 0x01)) // Wait till RX line goes high (idle)
    cbi(UCSR1B, RXCIE1);
  acp_txenable(true);  
  sbi(UCSR1A, TXC1);    // Set TXC to clear it!!
  cbi(UCSR1B, TXB81);
  outp(0x06, UDR1);
  while(!(inp(UCSR1A) & _BV(TXC1)));      // wait till buffer clear
  acp_txenable(false);
}

void acp_reset(void)
{
  acp_retries=0;
  acp_timeout=0;
  acp_checksum=0;
  acp_rxindex=0;
  acp_txindex=0;
  acp_txsize=0;
  acp_status=ACP_LISTEN;
}

void acp_sendmsg(void)
{
  u08 i;
  while(!(inp(PORTD) & 0x01)) // Wait till RX line goes high (idle)
    cbi(UCSR1B, RXCIE1);
  acp_txenable(true);
  /* Delay 1664us (16 Bit Time Intervals) to signify start of message (ACP spec says 14, but doesn't work!)  
     as was written before - should be 1664 = 104*16 (it was working on yampp), but it wasn't working on Arduino Mega, so I've tried with 104*16
  */
  _delay_us(104*17);
  for(i=0; i <= acp_txsize; i++)
  {
    while(!(inp(UCSR1A) & _BV(UDRIE1)));      // wait till tx buffer empty
    cbi(UCSR1B, TXB81);
    if(i==acp_txsize) 
    {
      sbi(UCSR1A, TXC1);  // set TXC to clear it!
      sbi(UCSR1B, TXB81);  // set 9th bit at end of message
    }
    outp(acp_tx[i], UDR1);
  }
  while(!(inp(UCSR1A) & _BV(TXC1))){
  }; // wait till last character tx complete
  acp_txenable(false); // switch back to rx mode immediately
  acp_status = ACP_WAITACK; // wait for ACK
  sbi(UCSR1B, RXCIE1);
}

void acp_handler()
{
  if (acp_status == ACP_LISTEN)
  {
    if(++acp_ltimeout == 1000)  // Send FFWD/FREW events periodically 
    {
      acp_ltimeout=0;

      /*switch(bFRSearch)
       {
       case 0x03:
       set_event(EV_FFWD);
       break;
       
       case 0x05:
       set_event(EV_FREW);
       }*/
    }
  }

  if (acp_status == ACP_SENDACK)
  {
    acp_sendack();
    acp_status=ACP_MSGREADY;
  }
  else if (acp_status == ACP_WAITACK)
  {
      //lcd.setCursor(0,0);
      //lcd.print("Wait ack");
    acp_reset(); // HU does not seem to return an ACK? So just listen for next msg.
  }

  if (acp_status == ACP_MSGREADY)
  {
    acp_status = ACP_SENDING;
    acp_process(); // Process message   
  }
  else if (acp_status == ACP_SENDING)
  {
    acp_timeout++;
    if(!acp_timeout)
    {
      acp_reset();
      acp_txenable(false);
    }   
  }
}

void acp_process(void)
{ 
  acp_timeout = 0;
  acp_tx[0] = 0x71;   // medium/low priority
  acp_tx[1] = 0x9b;
  acp_tx[2] = 0x82;     // Message from CD Changer (yampp3)
  acp_tx[3] = acp_rx[3];

  if (acp_rx[2] == 0x80)  // Message from Head Unit
  {
    if(acp_rx[1]==0x9a || acp_rx[1]==0x9b) // CD Changer functional address 
    {
      
      switch (acp_rx[3])
      {
      case 0xc8:    // Handshake 3 - CD Changer now recognised
        acp_tx[4] = acp_rx[4];
        acp_chksum_send(5);
        break;

      case 0xfc:    // Handshake 2
        acp_tx[4] = acp_rx[4];
        acp_chksum_send(5);
        break;

      case 0xe0:    // Handshake 1
        acp_tx[4] = 0x04;
        acp_chksum_send(5);
        #ifdef LCD
        lcd.setCursor(0,0);
        lcd.print("S ack 1");
        #endif
        break;

      case 0xff:    // Current disc status request - responses are
        acp_tx[4]= 0x00;
        // 00 - Disc OK
        // 01 - No disc in current slot
        // 02 - No discs
        // 03 - Check disc in current slot
        // 04 - Check all discs!    
        acp_chksum_send(5);
        #ifdef LCD
        lcd.setCursor(0,1);
        lcd.print("Playing...");
        #endif

        Timer1.initialize(1000000);
        Timer1.attachInterrupt(PlayTime);        
        break;

      case 0xc2:    // Current disc number
      case 0xd0:
        if (acp_rx[1] == 0x9a)  // Command to change disc
        {
          //u08 disc = plist_change(acp_rx[4]);
          #ifdef LCD
          lcd.setCursor(0,1);
          lcd.print("Change disc");
          #endif
          acp_tx[4] = 1 & 0x7f; 
          acp_chksum_send(5);
          //if(disc & 0x80)
          //acp_nodisc();
          break;
        } 
        else        // Request current disc
        {
          //acp_tx[4] = get_disc();
        }
        if (acp_rx[3]!=0xd0) acp_chksum_send(5);
        break;

      case 0xc1:    // Command

        acp_mode = acp_rx[4];
        //if((acp_mode & 0x40) && !isPlaying) set_event(EV_PLAY); 
        if((acp_mode & 0x40)) 
        {
          #ifdef LCD
          lcd.setCursor(0,0);
          lcd.print("Play!!");  
          #endif
        }
        //if(!(acp_mode & 0x40) && isPlaying) set_event(EV_STOP);
        //if((acp_mode & 0x10) && !bRandom) set_event(EV_RANDOM);
        //if(!(acp_mode & 0x10) && bRandom) set_event(EV_RANDOM);
        //if((acp_mode & 0x20) && !bLoudness) set_event(EV_LDS);
        //if(!(acp_mode & 0x20) && bLoudness) set_event(EV_LDS);
        //if(acp_mode & 0x01) // Fast search mode
        //{
        //bFRSearch = acp_mode & 0x07;
        //} 
        //else bFRSearch=0;

        acp_mode = (acp_mode & 0x40);
        acp_tx[4] = acp_mode;
        acp_chksum_send(5);
        break;

      case 0xc3:
        change_track(true); // Next Track
        #ifdef LCD
        lcd.setCursor(0,0);
        lcd.print("Next");
        #endif
        acp_tx[4] = BCD(currentTrack);
        acp_chksum_send(5);
        break;

      case 0x43:
        change_track(false);  // Prev Track
        #ifdef LCD
        lcd.setCursor(0,0);
        lcd.print("Prev");
        #endif
        acp_tx[4] = BCD(currentTrack);
        acp_chksum_send(5);
        break;

      default:
        // unknown - ignore
        acp_reset();
      }


    }
    else
    { // Ignore all other acp messages
      acp_reset();
    }

  }
  #ifdef LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(acp_rx[3]);    
    lcd.setCursor(0,5);
    lcd.print(acp_rx[4]);
    lcd.setCursor(1,0);
    lcd.print(acp_rx[5]);
   #endif

      // Send code to Rpi
      digitalWrite(i2cInt, HIGH);
      number = acp_rx[5];
      sendData();
      delay(50);
      digitalWrite(i2cInt, LOW);
}

void change_track(boolean next)
{
  if(next != true){
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    if(currentTrack == 1) {
    } 
    else {
      currentTrack--; 
    }
  }
  else {
    currentTrack++;
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
  }
  reset_timer = true;
}

void acp_displaytime()
{
  if(reset_timer == true){
    wPlayTime = 0;
    reset_timer = false; 
  }
  acp_tx[0] = 0x71 ;    
  acp_tx[1] = 0x9b ;
  acp_tx[2] = 0x82 ;
  acp_tx[3] = 0xd0 ;
  acp_tx[4] = BCD(1);
  acp_tx[5] = BCD(currentTrack) ; //Track
  acp_tx[6] = BCD(wPlayTime/60); 
  acp_tx[7] = _BV(7) | BCD (wPlayTime % 60); 
  acp_chksum_send(8);
}

void acp_nodisc(void)
{
  acp_tx[0] = 0x71 ;    
  acp_tx[1] = 0x9b ;
  acp_tx[2] = 0x82 ;
  acp_tx[3] = 0xff ;
  acp_tx[4] = 0x01 ;
  acp_chksum_send(5);
}

void acp_chksum_send(unsigned char buffercount)
{
  u08 i;
  u08 checksum = 0;

  for(i=0;i<buffercount;i++)
  {
    checksum += acp_tx[i];
  }
  acp_txsize = buffercount;
  acp_tx[acp_txsize] = checksum;
  acp_sendmsg();
}

uint8_t BCD(unsigned char val)
{
  return ((val/10)*16) + (val % 10) ;
}

// callback for received data
void receiveData(int byteCount){
  while(Wire.available()) {
    number = Wire.read();
    //Serial.print(“data received: “);
    //Serial.println(number);

    if (number == 1){

      if (state == 0){
        digitalWrite(13, HIGH); // set the LED on
        state = 1;
      }
      else{
        digitalWrite(13, LOW); // set the LED off
        state = 0;
      }
    }
  }
 }

// callback for sending data
void sendData(){
  Wire.write(number);
}

void setup(){
  outp(0xff, PORTD);
  outp(0xC0, DDRD);
  pinMode(switchPin, OUTPUT);
  digitalWrite(switchPin, LOW);
  #ifdef LCD
    lcd.begin(8, 2);
    delay(1000);
    lcd.print("FORD EMULATOR");
    lcd.setCursor(0,1);
    lcd.print("v 1.0"); 
  #endif

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  acp_uart_init(UART_BAUD_SELECT);
  delay(1);
  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  // Send message to Rpi
  pinMode(i2cInt, OUTPUT);
  digitalWrite(i2cInt, HIGH);
  number = 1;
  sendData();
  delay(50);
  digitalWrite(i2cInt, LOW);
}

void loop()
{
  acp_handler();
}

void PlayTime(){
  wPlayTime++;
  acp_displaytime();
}
