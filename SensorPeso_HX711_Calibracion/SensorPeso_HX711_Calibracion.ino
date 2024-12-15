#include "HX711.h"

#define DT 8   // Pin de datos
#define SCK 9  // Pin de reloj

HX711 celdaCarga;

void setup() {
  Serial.begin(9600);
  celdaCarga.begin(DT, SCK); // Data en D3, Clock en D2
  Serial.println("Obtención del factor de escala:");

  celdaCarga.set_scale();
  celdaCarga.tare();
  Serial.println("Colocar peso conocido (10 Segundos)");
  delay(10000);
  Serial.print("El valor de calibracion es: ");
  Serial.println(celdaCarga.get_units(10));
  Serial.println("Pasa al siguiente programa ");

}

void loop() {
}
