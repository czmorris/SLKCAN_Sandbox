/*
Note: This sketch uses the MCP2551 library (Alexander Entinger).

This sketch was created by Zach M. for the purpose of acting as a sandbox for the SLKCAN
board. This could be used as a jumping off point for anyone interested in working with the board.
This is not a commercial product and no warranty is provided. This code is not guaranteed to be 
error free. It is quick code intended to test the features of the board. 

This code may change frequently as a testbed for the board.
*/

#include <SPI.h>
#include <ArduinoMCP2515.h>

void    spi_select           ();
void    spi_deselect         ();
uint8_t spi_transfer         (uint8_t const);
void    onExternalEvent      ();
void    onReceiveBufferFull  (uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

static int const MKRCAN_MCP2515_CS_PIN  = 15;
static int const MKRCAN_MCP2515_INT_PIN = 35; // Note: Caused bluetooth issues on 4, SLKCAN01. 
/* 20210408 Moved to IO35, This goes up to 5v so it really should have been tri-stated/shifted. Oops... 
On IO35 it doesn't seem to bother bluetooth. This needs to be tested with the bike later
to make sure that it actually works!*/

SPIClass spiH(HSPI);    // HSPI is used as to not conflict with the internal SPI flash on the ESP32

ArduinoMCP2515 mcp2515(spi_select,
                       spi_deselect,
                       spi_transfer,
                       micros,
                       onReceiveBufferFull,
                       nullptr);


void SetupCAN()
{
  // Level Translator IC Enable. (Active High)
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);

  spiH.begin(14,12,13,15); //CLK,MISO,MOIS,SS

  /* Setup SPI access */
  spiH.begin();  // Note: Why again?
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);

  // CZM, During testing I found that the ISR was getting hammered and interupt watchdog resets would happen.
  // I elected to use a dedicated task for handling messages as fast as it could instead of high interrupt load.
  // Note that not every message will be seen. At some point it might be a good idea to try buffering with interupt.
  // I suspect the buffer would be overrun if that happened. The high priority items on the bus happen very fast.
  //attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_500kBPS_8MHZ);
  mcp2515.setNormalMode(); 
}

// The task handler for the MCP2551.
// Instead of interrupts it just checks if the interrupt pin is low.
void mcpTask( void * parameter )
{
  // Get MCP messages as fast as possible...
  // but only as fast as possible. Does not allow ISR bog. (but can come with its own problems)
  // Note: This will not catch every single message!
  for(;;)
  {
#ifdef SIM

    SimVariables();
    delay(1000);

#else
  
    if(digitalRead(MKRCAN_MCP2515_INT_PIN)==LOW)
    {
      mcp2515.onExternalEventHandler();
    }    
    delay(1);

#endif
  }
}

void spi_select()
{digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);}

void spi_deselect()
{digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);}

uint8_t spi_transfer(uint8_t const data)
{return spiH.transfer(data);}

void onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

void onReceiveBufferFull(uint32_t const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  char * pek;
  int i = 0;

// Only stream the can messages to serial when the define is enabled. 
// See the top of this file.
#ifdef STREAMCANMSGS
  //Clear the buffer
  for(i = 0; i < 200; i++)
  {printbuff[i] = 0x00;}

  // Setup the pointer to the beginning
  pek = &printbuff[0];

  // Print the start of the line, adjust the buffer
  pek += sprintf(pek, "%d %08X %d ", timestamp_us, id, len);

  // Print each data byte, adjust the buffer
  for(i = 0; i < len; i++)
  {
    pek += sprintf(pek, "%02X ", data[i]);
  }

  // Finally print the entire buffer.
  //Serial.println(printbuff);

  // Note: Add ifdef?
  // Note: Could possibly slow down CAN transfers.
  writeSDCard(printbuff);   
#endif
   
  // Lets pick out some data... 
  switch(id)
  {
    case 0xA0:
    
      gearmode = data[0];   // ECO, POWER, REV, PARK
      
      break;
    case 0x19:
    
      BatterySoc = data[1];
      debugcnt++;          // For debugging timing.
      amps = (float)((data[6] + (255.0 * data[7])) / 100.0);   
      watts = (float)(amps * 72.0);     // This estimate is based on the rated voltage so its likely low. voltage can be in the 80s or low.
      
      break;
    case 0x98:
    
      //repthrottle = data[3];  // This doesn't appear to be speed. Maybe some kind of demand signal. Not certain its throttle.
      
      break;
    case 0x2D0:

      // Odometer in Kilometers
      odo = (float)((255 * data[3]) + data[2]);

      // Trip Meter. (Kilometers)
      tripkm = (float)(((255 * data[6]) + data[5]) / 10.0);

      break;
    case 0x101:
    
      ssStatus = data[5];                       // Possible Side Stand Status (or a bitfield of status indicators?) 
      rawkph = data[1];                         // kph before factor applied
      corrkph = ((float)rawkph * 1.275);        // still not sure why the 1.275 is needed.
      posTemp = (float)(data[6] - 40.0);        // offset by 40 C
      
      break;   
    default:
      break;
  }

}
