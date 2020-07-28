# ROBOTIX
This repository contains projects and templates related to robotics, Arduino, AVR and similar. Intention behind this repository is to provide beginners easier access to code and circuit diagrams. 

## Table of Contents
* [Introduction](#introduction)
* [Power Supply Pins](#power-supply-pins)
* [Potentiometer](#potentiometer)
* [ATtiny](#attiny)
* [555 timer IC](#555-timer-ic)
* [Push-Button](#push-button)
* [ATmega 2560](atmega-2560.md)
* [7-Segment Display](7-segment-display.md)
* [Motor-Drivers](motor-drivers.md)
* [Bitwise Operators](bitwise.md)
* [Clipper Circuits](clipper.md)
* [BJT](bjt.md)
* [Op-Amp](op-amp.md)
* [References](#references)

## Introduction 

You may have Arduino Board or may have not, but not having the board can't stop you from learnig cool stuff and making building future technology.
You can use [Tinkercad](https://www.tinkercad.com/) to simulate circuits. If you have board then you can download [Arduino IDE](https://www.arduino.cc/en/Main/Software) or can use Arduino Online Editor. 

<img src="images/hello-world.png" width="550" />

## Power Supply Pins

| Type of Supply  | BJT | FET | Traditional  | Op-Amp |
| --------------- | --------------- | --------------- | --------------- | --------------- |
| Positive supply voltage | V<sub>CC</sub> / V<sub>BB</sub> | V<sub>DD</sub>  | V+ | V<sub>S+</sub> |
| Negative supply voltage | V<sub>EE</sub>                  | V<sub>SS</sub>  | V- | V<sub>S-</sub> |
| Ground                  | GND                             | GND             | 0  | 0              |


## Potentiometer

Potentiometer is three-terminal device which we will use in our projects to adjust voltage. 

<img src="images/potentiometer.png" width="250" /> 

Terminals are when connected to Vcc and GND (in any manner), output at the wiper will change depending upon rotation of shaft.

<img src="images/potentiometer.gif" width="550" />

## ATtiny 

<img src="images/attiny_blink.gif" width="450" />

```c++
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

## 555 timer IC

<img src="images/555_ic.gif" width="450" />

## Push-Button 

<img src="images/push-button-simple.gif" width="450" />


<img src="images/button.gif" width="450" />

```C++

int buttonState = 0;

void setup()
{
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
}

void loop()
{
  buttonState = digitalRead(2);
  if (buttonState == HIGH) {
    digitalWrite(13, HIGH);
  } 
  else {
    digitalWrite(13, LOW);
  }
  delay(10); 
}
```

## References

* https://www.arduino.cc/
* https://www.tinkercad.com/
