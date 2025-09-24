#define l_encoder_pinA 2
#define l_encoder_pinB 9

long  pulse_l = 0;


void counter_l()
{

  if( digitalRead(l_encoder_pinB) == 0 ) {
    if ( digitalRead(l_encoder_pinA) == 0 ) {
      // A fell, B is low
      pulse_l--; // moving reverse
    } else {
      // A rose, B is low
      pulse_l++; // moving forward
    }
  }

}





void setup()
{

  //Encoder setup
  
  pinMode(l_encoder_pinA, INPUT);
  pinMode(l_encoder_pinB, INPUT);


  //Initialize Value
  pulse_l = 0;


  attachInterrupt(digitalPinToInterrupt(2), counter_l,CHANGE);
 

  
  
  Serial.begin(57600);
 
  delay(20);
}

void loop()
{
  
  Serial.println(pulse_l);
   // wait for a second
   delay(20);
   
}
