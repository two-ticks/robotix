```c++
#include <avr/io.h>
#define F_CPU 8000000
#include <util/delay.h>


int main(void)
{   
	DDRB |= 1 << PINA1 ; //BITWISE ORing
	
    while(1)
    {
       PORTA ^= 1 << PINA1;
	   _delay_ms(100);
    }
}
```
