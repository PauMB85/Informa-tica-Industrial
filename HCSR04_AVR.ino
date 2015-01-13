//Librerias.
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Pines Ultrasonic HCSR04
#define trig  PORTD0   //es un pin de salida
#define echo  PORTD1   //es un pin de entrada 
#define	estadoD	PIND   //para la lectura de los pines de entrada
#define greenLed  PORTD2  //es un pin de salida
#define redLed  PORTD3    //es un pin de salida

double r;
boolean fin = false;
//uint32_t result;
int result;

void setup() {

  //Inicializar los pines del sensor (trig y echo)
  Serial.begin(9600);
  while(!Serial);
  HCSR04_init();
  LEDS_init();
  TIMER3_init();

}

void loop() {

  //Enviamos el pulso trigger
  TRIGGER();
  //esperamos la respuesta del pulso enviado, echo
  r = ECHO();
  double d;
  d=(r/58.0);
  //Calculamos la distancia que hay con el objeto detectado	
  Serial.print("La distancia es: ");
  Serial.println(d);
  _delay_ms(500);
  //Handle Errors
  if( (d > -1) & (d < 200))
  {
      //Al detectar un obstaculo significa que hay un vehiculo 
      //por lo tanto el led verde debe estar apagado y el led rojo encendido
      PORTD &= ~(1 << greenLed);
      PORTD |= (1 << redLed); 
  }
  else
  {

    //Al no haber un obtaculo significa que la plaza esta libre
    //por lo tanto el led verde debe estar encendido y el led rojo apagado
    PORTD |= (1 << greenLed);
    PORTD &= ~(1 << redLed);
  }

}

void HCSR04_init()
{
  //Inicializamos el puerto D0
  DDRD |= (1 << trig);  //como solo trig es de salida ponemos un 1 en PD0
  PORTD = 0; //ponemos un cero todos los pines del puerto D, para inicializar
}

void TRIGGER()
{
  //Enviamos un pulso TTL
  PORTD |= (1 << trig);
  _delay_us(15); //esperamos 15us
  PORTD &= ~(1 << trig);//finalizamos el pulso.
}

int ECHO()
{
  result = 0;

  //esperamos a que se detecte el pulso de entrada, se espera a que sea nivel alto
  while(!(estadoD & (1<<echo)));
 
  TCNT3=0x00;//Iniciamos el timer a 0

  //Calculamos el ancho del pulso
  while(estadoD & (1<<echo))
  {
     if(TCNT3 > 60000)
     {
       //si superamos los 30ms es debido a que no se encuentra ningun objeto
       return -1;
     }
  }
  
  result = TCNT3;
  return (result>>1);//devolvemos el resultado ya dividido entre 2.
}

void LEDS_init()
{
  //Inicializamos los puertos PD3(led verde) y el PD2(led rojo)
  DDRD |= (1 << greenLed) | (1 << redLed);
  PORTD = 0; //ponemos todos los pines del puerto D a zero
  
  //iniciamos los leds con el led verde encendido y el led rojo apagado
  PORTD |= (1 << greenLed);
  PORTD &= ~(1 << redLed);
  
}

void TIMER3_init()
{
  //iniciamos el timer3 para calcular el tiempo que dura el pulso
  TCCR3A=0;
  TCCR3B=(1<<CS31);	//Prescaler = Fcpu/8
}


