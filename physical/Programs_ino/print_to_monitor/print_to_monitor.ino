
#include <SCServo.h>

SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

void setup()
{
  Serial.begin(9600);
}

void loop()
{
Serial.println("bubush");
}
