#include <LiquidCrystal.h>
#include "HX711.h"

#define DT 8   // Pin de datos
#define SCK 9  // Pin de reloj

HX711 celdaCarga;
LiquidCrystal lcd(7,6,5,4,3,2);

void setup() {
  Serial.begin(9600);
  lcd.begin(16,2); //LCD Size
  celdaCarga.begin(DT, SCK); // Data en D3, Clock en D2

  Serial.println("Balanza celda de carga.");
  celdaCarga.set_scale(2213.f);
  celdaCarga.tare();
}

void loop() {
  long weight = celdaCarga.get_units(10)/1000;
  Serial.println(weight, 1);

  lcd.setCursor(0,0);
  lcd.print("Peso:");
  lcd.print(weight,1);
  lcd.print("Kg");

  celdaCarga.power_down();
  delay(1000);
  celdaCarga.power_up();
}
