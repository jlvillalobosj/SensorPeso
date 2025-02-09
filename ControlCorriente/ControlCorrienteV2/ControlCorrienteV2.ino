#include <Wire.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

#define         NUM_READINGS            (50) // Number of readings to calculate the average
#define         CHANGE_THRESHOLD        (0.05) //  Change of threshold (ppm)

// Configura el objeto de la pantalla LCD (dirección 0x27 es la más común)
LiquidCrystal  lcd(7,6,5,4,3,2);  // Dirección 0x27, 16 columnas y 2 filas

const int potPin = A0;  // Pin del potenciómetro
const int analogPinSensorCurrent = A1;   // Pin analógico donde está conectado el sensor
const int voltagePin = A2;   // Pin analógico donde está conectado el sensor
const int buttonPin = 8; // Pin del botón para cambiar modos
const int mosfetPin = 9; // Pin del MOSFET

float voltageSensor = 0, current = 0;
float voltage = 0, inputVoltage = 0;


int mode = 0;  // 0: solo lectura, 1: control de corriente
int potValue = 0; // Valor del potenciómetro
double Setpoint = 0.0;
double Input = 0, Output = 0;
double Kp = 2, Ki = 3.7, Kd = 0; // Definir los valores de los parámetros PID


// Crear el controlador PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// Temporizador
unsigned long milisTime;
unsigned long  previousMillis = 0;
int hours = 0, minutes = 0, seconds = 0;
int interval = 1000, intervalSeconds = 1;
int previousSeconds = 0;

// Calculo estabilidad
float readings[NUM_READINGS]; // Array to keep readings
int index = 0;                // Current index in array
float average = 0;            // Current average
float stableValue_current = 0;        // Stable valuue to display


// Calibracion sensores de tension y corriente
float baseVoltage_ACS712; // cALIBRATE voltage sensor de corriente ACS712
const float R1 = 22000.0;   // 22kΩ
const float R2 = 21400.0;   // 21.4kΩ
const float FACTOR_DIVISION = (R1 + R2) / R2; // Factor de corrección del divisor
float baseVoltage_Calibrate = 0.0; // cALIBRATE voltage sensor de corriente ACS712


void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);    // Inicializa la pantalla LCD


  lcd.setCursor(0, 0);
  lcd.print("Calibrando...");

  pinMode(buttonPin, INPUT_PULLUP);  // Configura el botón
  pinMode(mosfetPin, OUTPUT);        // Configura el MOSFET

  digitalWrite(mosfetPin, LOW);     // Asegúrate que el MOSFET esté apagado al inicio

  // Configurar el PID
  myPID.SetMode(AUTOMATIC);  // Establece el PID en modo automático
  myPID.SetOutputLimits(0, 255); // Limita la salida del PID (para control de un MOSFET)

  readAverageVoltageACS712();
  delay(1000);

}

void loop() {

  contador();

  mode = digitalRead(buttonPin); 

  if (seconds - previousSeconds >= intervalSeconds) {

    previousSeconds = seconds;

    // Lectura de tensión
    int sensorVoltageValue = analogRead(voltagePin);         // Leer el valor analógico (0-1023)
    inputVoltage = (sensorVoltageValue * 5.0) / 1024.0;           // Convertir a voltaje (0-5V)
    voltage = abs((inputVoltage) * FACTOR_DIVISION);                       // Multiplicar por 2 para obtener el voltaje real (0-10V)


    // Lecture de corriente
    int sensorCurrentValue = analogRead(analogPinSensorCurrent);
    voltageSensor = ((sensorCurrentValue * 5.0) / 1024.0);  // Convertir a voltaje
    //current = abs((voltageSensor - 2.535) / 0.066);       // 0.066V por A para ACS712 30A
    current = abs((voltageSensor - baseVoltage_ACS712) / 0.066)-0.05;

  }

  

  if (mode == LOW) {
    
    // ---------------MODO SOLO LECTURA------------------------

    // No controlar corriente, el MOSFET está apagado
    digitalWrite(mosfetPin, LOW);
  }
  else {
    // ---------------MODO CONTROL DE CORRIENTE-------------------
    // Lee el valor del potenciómetro
    
    potValue = analogRead(potPin);
    Setpoint = map(potValue, 0, 1023, 0, 255);  // Mapea el valor de 0-1023 a 0-100
    //Setpoint = 0.5;
    // Ajusta el MOSFET para controlar la corriente
    // El valor de 'currentControl' es la referencia de corriente
    // Este valor puede ser usado para ajustar el MOSFET y la carga


    // Leer el valor actual de la corriente (Input)
    Input = current;   
    Serial.print("pot:");
    Serial.print(Setpoint);

    // Calcular la salida del PID
    myPID.Compute();

    //int mosfetValue = map(currentControl, 0, 100, 0, 255);  // Mapea a rango de control del MOSFET (PWM)
    analogWrite(mosfetPin, Setpoint);  // Ajusta el MOSFET con PWM
    //analogWrite(mosfetPin, LOW);  // Ajusta el MOSFET con PWM
  }

  stableValue_current = averageCalculateCurrent(current, stableValue_current);
  Serial.print("Tensión:");
  Serial.print(inputVoltage);
  Serial.print(", Corriente:");
  Serial.print(voltageSensor);
  Serial.print(", CorrienteStale:");
  Serial.println(stableValue_current);

  readLCD(voltage, stableValue_current);

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
      previousSeconds = 0;
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


float averageCalculateCurrent(float current, float stableValueCurrent) {
  // UUpdate the array with the last readings
  readings[index] = current;
  index = (index + 1) % NUM_READINGS;

  // Calculate the average of the lasts readings
  float sum = 0;
  for (int i = 0; i < NUM_READINGS; i++) {
    sum += readings[i];
  }
  average = sum / NUM_READINGS;

  // Set a stable value if the average changes significantly
  if (abs(average - stableValueCurrent) > CHANGE_THRESHOLD) {
    return  average;
  } 
  else return stableValueCurrent;
}

// Average voltage in ACS712 Sensor
void readAverageVoltageACS712() {
  const int numReadings = 50; 
  float totalVoltage = 0.0;
  float totalVoltageResistors = 0.0;

  for (int i = 0; i < numReadings; i++) {
    int rawValue = analogRead(analogPinSensorCurrent);
    int rawValueResistors = analogRead(voltagePin);
    totalVoltage += rawValue * (5.0 / 1024.0); // Convert ADC to voltage
    totalVoltageResistors += rawValueResistors * (5.0 / 1024.0); // Convert ADC to voltage
    delay(10); 
  }
  
  baseVoltage_ACS712 = totalVoltage / numReadings;
  baseVoltage_Calibrate = totalVoltageResistors / numReadings;
}

void readLCD(float voltage, float current){

    // Muestra los resultados en la pantalla LCD
    lcd.setCursor(0, 0);
    lcd.print("V:");
    lcd.print(voltage);
    lcd.print("V  ");
    lcd.print("I:");
    lcd.print(current);
    lcd.print("A");

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
