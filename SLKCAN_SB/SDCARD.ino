#include "SPI.h"
#include "SD.h"

#define SD_CS 5  // Micro-SD card chip select. 

SPIClass spiV(VSPI);    // VSPI is used with the SD Card.

void SDSetup ()
{
  SDIsInit = false;

  // VSPI is used for the SD Card.
  spiV.begin(18, 19, 23, 5);  // CLK, MISO, MOSI, SS
}

// Try to reinit the sd card.
void trySDinit()
{
  if(!SD.begin(SD_CS, spiV))
  {
    //Serial.println("Card Mount Failed!");
    SDIsInit = false;
  }
  else
  {
    //Serial.println("SD Card Init!");
    SDIsInit = true;
  }
}

// Quick hack, nothing special. Needs more work.
void writeSDCard(char * data)
{

    // SD Card not initialized. Try
  if(SDIsInit == false)
  {
    trySDinit();
  }
  else
  {
    // Note: Add better checking here. This is just for testing.
   File file = SD.open("/TESTLOG.txt", FILE_APPEND);

   if(file)
   {        
     file.println(data);
     file.close();
   }
   else
   {
     SDIsInit = false; // Try to force a reinit on the next cycle... 
      //Serial.println("Could not write to the SD Card!");
   }     
  }
  
}
