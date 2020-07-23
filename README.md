# robotix
This repository contains projects and templates related to robotics, Arduino, AVR and similar. Intention behind this repository is to provide beginners easier access to code and circuit diagrams. 
## Table of Contents
* [Introduction](#introduction)
* [ATtiny](#attiny)
* [Reference](#reference)

## Introduction 
You may have Arduino Board or may have not, but not having the board can't stop you from learnig cool stuff and making building future technology.
You can use Tinkercad to simulate circuits (link is in the ~bio~ reference!).  

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
