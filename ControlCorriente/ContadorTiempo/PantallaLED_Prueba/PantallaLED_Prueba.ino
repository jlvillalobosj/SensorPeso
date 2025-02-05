/**
 Mostrar información en una pantalla LCD de 16x2
**/
#include <LiquidCrystal.h>
unsigned long milisTime;
int hours = 0, minutes = 0, seconds = 0;
int interval = 1000;
unsigned long  previousMillis = 0;

//Declarar LCD y pines
LiquidCrystal lcd(2,3,4,5,6,7);
void setup() {
 //Definir las dimensiones del LCD (16x2)
 lcd.begin(16,2);
}
void loop() {
  contador();
  displayLCD();
}

void contador() {
  
  unsigned long currentMillis = millis();  // Obtener el tiempo actual en milisegundos
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Actualizar el tiempo de referencia
    
    // Incrementar los segundos
    seconds++;
    
    // Si los segundos llegan a 60, resetear y sumar 1 minuto
    if (seconds >= 60) {
      seconds = 0;
      minutes++;
    }

    // Si los minutos llegan a 60, resetear y sumar 1 hora
    if (minutes >= 60) {
      minutes = 0;
      hours++;
    }

    // Si las horas llegan a 24, resetear a 0
    if (hours >= 24) {
      hours = 0;
    }

  }
}
void displayLCD(){
 //Seleccionamos en que columna y en que linea empieza a mostrar el texto
 lcd.setCursor(0,0);
 //Mostramos el texto deseado
 lcd.print("Hola mundo");

 lcd.setCursor(0,1);
 //Mostramos el texto deseado
 lcd.print("Time=");
 if (hours < 10) lcd.print("0");
 lcd.print(hours);
 lcd.print(":");
 if (minutes < 10) lcd.print("0");
 lcd.print(minutes);
 lcd.print(":");
 if (seconds < 10) lcd.print("0");
 lcd.print(seconds);

}


