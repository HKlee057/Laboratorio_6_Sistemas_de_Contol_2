//Clase: Sistemas de Control 2
//Sección: 11
//Catedratico: Luis Rivera
//Por: Juan Diego Castillo Amaya -17074 y Hector Alejandro Klée Gonzales - 17118
//Laboratorio 6
//Parte 2
#include <SPI.h>
#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

#define Timers 1000
#define frecdiv (80*Timers) //se define la freciencia por la que se va a dividir la frecuencia de 80MHz
#define COTA_SUP 3.3
#define COTA_INF 0
#define Gain 1
#define CASO 1
#if CASO == 1
const int Nbar = 3.5753;
float K_pp[3] = {0.1801,    2.3952,  -21.7918};
#endif
#if CASO == 2
const int Nbar = 6.8756;
float K_pp[3] = {0.4801,    5.3955,  -57.1948}; 
#endif


const int ss = PD_1; //se espesifica el salve select

//-----------------------Se especifican Pines de entrada analogicos---------------------------
const int IN = A0;
const int C1 = A1; 
const int C2 = A2;
const int C3_1 = A3;
const int C3_2 = A4;

//---------------------Se inicializan las variables de estado---------------------------------
float Vc1 = 0;
float Vc2 = 0;
float Vc3 = 0;
float Vc3_1 = 0;
float Vc3_2 = 0;
//--------------------se especifica la referencia y salida del sistema-----------------------
float Ref = 0;
float u = 0;
int U = 0;

void setup() {
  configureTimer1A();
  pinMode(ss,OUTPUT); //se especifica el pin PB_5 como salida
  pinMode(IN,INPUT);
  pinMode(C1,INPUT);
  pinMode(C2,INPUT);
  pinMode(C3_1,INPUT);
  pinMode(C3_2,INPUT);
  SPI.begin(); //se inicializa el SPI
  digitalWrite(ss,HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
//------------------------------------Se crea la funcion para escribir al DAC------------------------------------
void Write_DAC(int value, int slave_select) {
  //se apaga el pin para seleccionar el chip a utilizar
  digitalWrite(slave_select,LOW);
  byte primero = (byte)(((value>>8) & 0b00001111) | (Gain<<5) | (0b01100000));// con el operador>> se realiza un shift, en este caso de 8 bits por 
                                           // lo que se descartan los 8 bits menos significativos y se realiza un 
                                           // and con el valor 0b00001111. Esto permite que los bits en 1 se mantengan 
                                           //como 1 y los que estan en 0 se coloquen como 0
  byte segundo = (byte)(value & 0x00FF); //este realiza un and con los primero 8 bits del valor leido y reliza un and
                                       //esto con el proposito de que los primero 8 bits del valor values se asignen
                                       //en la variable segundo
  SPI.transfer(primero);//se escribe los primero 4 bits de value
  SPI.transfer(segundo);//se escriben los primero 8 bits de values
  // take the SS pin high to de-select the chip:
  digitalWrite(slave_select,HIGH);
}

//----------------------------------Configuracion del Timer----------------------------------------------------
void configureTimer1A(){
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock
  ROM_IntMasterEnable(); // Enable Interrupts
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  
  // Configure Timer Frequency
  // El tercer argumento ("CustomValue") de la siguiente función debe ser un entero, no un float.
  // Ese valor determina la frecuencia (y por lo tanto el período) del timer.
  // La frecuecia está dada por: MasterClock / CustomValue
  // En el Tiva C, el MasterClock es de 80 MHz.
  // Ejemplos:
  // Si se quiere una frecuencia de 1 Hz, el CustomValue debe ser 80000000. 80MHz/80M = 1 Hz
  // Si se quiere una frecuencia de 1 kHz, el CustomValue debe ser 80000. 80MHz/80k = 1 kHz
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, frecdiv); // El último argumento es el CustomValue

  // Al parecer, no hay función ROM_TimerIntRegister definida. Usar la de memoria FLASH
  // El prototipo de la función es:
  //    extern void TimerIntRegister(uint32_t ui32Base, uint32_t ui32Timer, void (*pfnHandler)(void));
  // Con el tercer argumento se especifica el handler de la interrupción (puntero a la función).
  // Usar esta función evita tener que hacer los cambios a los archivos internos de Energia,
  // sugeridos en la página de donde se tomó el código original.
  TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1AHandler);
  
  ROM_IntEnable(INT_TIMER1A);  // Enable Timer 1A Interrupt
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Timer 1A Interrupt when Timeout
  ROM_TimerEnable(TIMER1_BASE, TIMER_A); // Start Timer 1A
}

// Handler (ISR) de la interrupción del Timer
void Timer1AHandler(void){
  //Required to launch next interrupt
  ROM_TimerIntClear(TIMER1_BASE, TIMER_A);
//-----------------------------Se leen los voltajes de las terminales de los capacitores-----------------------------
  Ref = analogRead(IN)*3.3/4095;
  Vc1 = analogRead(C1)*3.3/4095;
  Vc2 = analogRead(C2)*3.3/4095;
  Vc3_1 = analogRead(C3_1)*3.3/4095;
  Vc3_2 = analogRead(C3_2)*3.3/4095;
  Vc3 = Vc3_1 - Vc3_2;
//-----------------------------------------Se implementa el controlador con retroalimenctacion y sus variables de estado---------------------------------------
    u=Ref*Nbar-((K_pp[0]*Vc1)+(K_pp[1]*Vc2)+(K_pp[2]*Vc3));
//------------------------------------------Se especifica limite superior para la salida del controlador--------------------------------------------
    if (u>COTA_SUP)
    {
      u=COTA_SUP;
    }
//------------------------------------------Se especifica limite inferior para la salida del controlador--------------------------------------------
    if (u<COTA_INF)
    {
      u=COTA_INF;
    }
//-----------------------------------------Se ingresas la salida del sistema al DAC-------------------------------------------------
    U=(int)((u-COTA_INF)*4095/(COTA_SUP-COTA_INF));
    Write_DAC(U, ss);
}
