//Setup routine

#define RECEIVER_INPUT_PIN 7

const int ledPin6 = 3;       // green LED
const int ledPin5 = 4;      // yellow LED
const int ledPin4 = 2;      // red LED
//const int threshold3 = 1800;   // Autonomous Mode (down on TX)
//const int threshold2 = 1400;   // Manual Mode (middle on TX)
//const int threshold1 = 1100;   // Full Stop (up on TX)

void setup()
{
  // initialize the LED pin as an output:
  pinMode(ledPin6, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  Serial.begin(9600);
}

//Main program loop
void loop(){
  int receiver_input_channel_1 = pulseIn(RECEIVER_INPUT_PIN,HIGH);
  Serial.println(receiver_input_channel_1);

  
  // if the analog value is high enough, turn on the LED:
  //if (receiver_input_channel_1 > 1700 )
  //3 green is on
  if (receiver_input_channel_1 >1870 && receiver_input_channel_1 <1890) 
  {
    digitalWrite(ledPin6, HIGH);
    //digitalWrite(ledPin6, LOW);
    digitalWrite(ledPin5, LOW);
    digitalWrite(ledPin4, LOW);
   }
   
   //if (receiver_input_channel_1 > 1499 && receiver_input_channel_1 < 1600) 
   // 2 yellow is on
   if (receiver_input_channel_1 > 1486 && receiver_input_channel_1 < 1489)
   {
    digitalWrite(ledPin5, HIGH);
    digitalWrite(ledPin6, LOW);
    //digitalWrite(ledPin5, LOW);
    digitalWrite(ledPin4, LOW);
   }
   
// if (receiver_input_channel_1 > 1400 && receiver_input_channel_1 < 1499)
// 4 red is on
 if (receiver_input_channel_1 > 1495 && receiver_input_channel_1 < 1498) 
 {
    digitalWrite(ledPin4, HIGH);
    digitalWrite(ledPin6, LOW);
    digitalWrite(ledPin5, LOW);
    //digitalWrite(ledPin4, LOW);
  }

}
