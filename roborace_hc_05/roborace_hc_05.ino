/*
  CONNECTIONS - modify according to wheel rotation
  IN1 - PIN4
  IN2 - PIN5
  IN3 - PIN6
  IN4 - PIN7

  use PWM to control speed
  EN_A - PIN9
  EN_B - PIN10

*/

#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7

int motorSpeedA = 220;
int motorSpeedB = 220;

char x;
void setup()
{
  Serial.begin(9600);
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
}

void loop()
{
  x = 's';
  if (Serial.available() > 0)
  {
    x = Serial.read();
    Serial.println(x);
  }
  
  delay(10);
  
  if (x == 'f')
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(200);
  }
  else if (x == 'l')
  { digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(200);
  }
  else if (x == 'r')
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(200);
  }
  else if (x == 'b')
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(200);
  }
  else if (x == 's')
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(200);
  }
}
