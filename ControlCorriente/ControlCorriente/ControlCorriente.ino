#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

// Configura el objeto de la pantalla LCD (dirección 0x27 es la más común)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Dirección 0x27, 16 columnas y 2 filas

const int potPin = A0;  // Pin del potenciómetro
const int buttonPin = 2; // Pin del botón para cambiar modos
const int mosfetPin = 9; // Pin del MOSFET

Adafruit_INA219 ina219;

int mode = 0;  // 0: solo lectura, 1: control de corriente
int potValue = 0; // Valor del potenciómetro
float Setpoint = 0.0;
double Input = 0, Output = 0;
double Kp = 2, Ki = 3.7, Kd = 0; // Definir los valores de los parámetros PID


// Crear el controlador PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600);
  ina219.begin();  // Inicializa el INA219
  lcd.begin(16, 2);    // Inicializa la pantalla LCD
  lcd.backlight();  // Activa la luz de fondo

  pinMode(buttonPin, INPUT_PULLUP);  // Configura el botón
  pinMode(mosfetPin, OUTPUT);        // Configura el MOSFET

  digitalWrite(mosfetPin, LOW);     // Asegúrate que el MOSFET esté apagado al inicio

  // Configurar el PID
  myPID.SetMode(AUTOMATIC);  // Establece el PID en modo automático
  myPID.SetOutputLimits(0, 255); // Limita la salida del PID (para control de un MOSFET)

}

void loop() {
  // Lee el estado del botón (cambia el modo cuando se presiona)
  if (digitalRead(buttonPin) == LOW) {
    mode = 1 - mode; // Alterna entre 0 y 1
    delay(500);  // Evita rebotes del botón
  }

  // ---------------MODO SOLO LECTURA------------------------
  // Lee los valores de corriente y voltaje
  float voltage = ina219.getBusVoltage_V();   // Voltaje
  float current = ina219.getCurrent_mA() / 1000.0;  // Corriente

  if (mode == 0) {

    // No controlar corriente, el MOSFET está apagado
    digitalWrite(mosfetPin, LOW);
  }
  else if (mode == 1) {
    // ---------------MODO CONTROL DE CORRIENTE-------------------
    // Lee el valor del potenciómetro
    potValue = analogRead(potPin);
    Setpoint = map(potValue, 0, 1023, 0, 100);  // Mapea el valor de 0-1023 a 0-100

    // Ajusta el MOSFET para controlar la corriente
    // El valor de 'currentControl' es la referencia de corriente
    // Este valor puede ser usado para ajustar el MOSFET y la carga


    // Leer el valor actual de la corriente (Input)
    Input = analogRead(current);   

    // Calcular la salida del PID
    myPID.Compute();

    //int mosfetValue = map(currentControl, 0, 100, 0, 255);  // Mapea a rango de control del MOSFET (PWM)
    analogWrite(mosfetPin, Output);  // Ajusta el MOSFET con PWM
  }

  readLCD(voltage, current);

  delay(500);  // Espera un poco antes de la siguiente lectura
}

void readLCD(float voltage, float current){

    // Muestra los resultados en la pantalla LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Volt: ");
    lcd.print(voltage);
    lcd.print(" V");
    lcd.setCursor(0, 1);
    lcd.print("Current: ");
    lcd.print(current);
    lcd.print(" A");

}

