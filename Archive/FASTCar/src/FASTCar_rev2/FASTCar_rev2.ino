#include <TinyGPS.h>
#include <FASTSerial.h>
TinyGPS gps;

#define RXPIN 3
#define TXPIN 2
SoftwareSerial nss(RXPIN, TXPIN);

void setup() {
  
}
  
void loop()
{
  while (nss.available()) //crap I didn't realize this while loop is here. This has the potential to hang like crazy if the GPS is sending us stuff.
  {
    int c = nss.read();
    if (gps.encode(c))
    {
      // process new gps info here
    }
  }
}
