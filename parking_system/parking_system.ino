#define LED 5
#define LIGHT_SENSOR A0

void setup(){
  pinMode(LED,OUTPUT);
  pinMode(LIGHT_SENSOR,INPUT);
  Serial.begin(9600);
}

void loop(){
  int readlight=analogRead(LIGHT_SENSOR);
  Serial.println(readlight);
  delay(200);

  if (readlight>820){
    digitalWrite(LED,HIGH);
    delay(2000);
    digitalWrite(LED,LOW);
    delay(2000);
  }
  else{
    digitalWrite(LED,LOW);
  }
}