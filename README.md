# robotix
This repository contains projects and templates related to robotics, Arduino, AVR and similar. 
## Table of Contents
* [ATtiny](#attiny)
* [Reference](#reference)
## ATtiny 
<img src="images/attiny_blink.gif" width="450" />

```arduino
void setup() 
{
  pinMode(0, OUTPUT); // set pin0 as output
}

void loop()
{
  digitalWrite(0, HIGH); 
  delay(1000); // Wait for 1 second
  digitalWrite(0, LOW);
  delay(1000); // Wait for 1 second
}

```

## Reference
* https://www.arduino.cc/
* https://www.tinkercad.com/
