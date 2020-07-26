# Motor Drivers 
## L293D 
<img src="images/l293d.gif" width="650" />

```c++
void setup()
{
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);r
}

void loop()
{
  //clockwise
  digitalWrite(11, HIGH);
  digitalWrite(9, LOW);
  delay(2000);
  //anti-clockwise
  digitalWrite(11, LOW);
  digitalWrite(9, HIGH);
  delay(2000);
}
```

<img src="images/motor-driver-dual.gif" width="650" />

```C++
void setup()
{
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  pinMode(9, OUTPUT);    
}

void loop()
{ 
  //clockwise 
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH); 
  digitalWrite(9, LOW);
  delay(2000);
  
  //anti-clockwise
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  delay(2000);
}
```

## L298 
<img src="images/l298.gif" width="650" />

```C++
void setup() {
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
}

void loop() {
  digitalWrite(11,HIGH);
  digitalWrite(10,LOW);
  digitalWrite(9,HIGH);
  digitalWrite(8,LOW);
}
```
