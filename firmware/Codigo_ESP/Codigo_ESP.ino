#include <Arduino.h>

// =====================================================
// CONFIGURACIÓN DEL PWM
// =====================================================

constexpr uint8_t PWM_RES = 12;
constexpr uint16_t PWM_MAX = (1UL << PWM_RES) - 1;

// =====================================================
// ASIGNACIÓN DE PINES
// =====================================================

const byte SENSOR_PIN = A3;
const byte PWM_PIN = 9;

const byte IN1_PIN = 7;
const byte IN2_PIN = 8;

// =====================================================
// CONFIGURACIÓN GENERAL
// =====================================================

const unsigned long TIEMPO_MUESTREO_MS = 10;
const float ts = 0.010;

const int OFFSET_ADC = 410;

unsigned long tiempoInicio = 0;

// =====================================================
// VARIABLES DE MEDICIÓN
// =====================================================

float anguloGrados = 0.0;
float anguloRadianes = 0.0;

// =====================================================
// VARIABLES DE CONTROL
// =====================================================

float Ref = 90.0;          // Referencia en grados
float U_t = 0.0;           // Salida saturada del PID
uint16_t pwmDuty = 0;      // Valor PWM final

const float Uunits = 100.0;  // 100 equivale al 100 % de PWM

// =====================================================
// PID
// =====================================================

float k_p = 0.8;
float k_i = 1.3;
float k_d = 0.25;

int N = 100;

float e_n = 0.0;
float e_n_1 = 0.0;

float u_p = 0.0;
float u_i = 0.0;
float u_d = 0.0;
float u_n = 0.0;

float u_n_1_i = 0.0;
float u_n_1_d = 0.0;

// =====================================================
// ENTRADA SERIAL PARA CAMBIAR REFERENCIA
// Formato: <30>
// =====================================================

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// =====================================================
// PROTOTIPOS
// =====================================================

void setupPWMadj();
void analogWriteADJ(uint8_t pin, uint16_t valor);
void recvWithStartEndMarkers();
void parseData();

// =====================================================
// CONFIGURACIÓN INICIAL
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Dirección fija del motor
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  setupPWMadj();

  analogWriteADJ(PWM_PIN, 0);

  Serial.println("tiempo_s,adc,angulo_deg,error,u_p,u_i,u_d,u_n,pwm_porcentaje,pwm_valor");

  delay(3000);

  tiempoInicio = millis();
}

// =====================================================
// BUCLE PRINCIPAL
// =====================================================

void loop() {
  static unsigned long tiempoAnterior = 0;

  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= TIEMPO_MUESTREO_MS) {
    tiempoAnterior += TIEMPO_MUESTREO_MS;

    // =====================================================
    // LECTURA DEL SENSOR
    // =====================================================

    int lecturaADC = analogRead(SENSOR_PIN);

    int lecturaCorregida = lecturaADC - OFFSET_ADC;

    anguloGrados = (lecturaCorregida * 360.0) / 1023.0;
    anguloRadianes = anguloGrados * PI / 180.0;

    // =====================================================
    // ERROR
    // =====================================================

    e_n = Ref - anguloGrados;

    // =====================================================
    // PID DISCRETO
    // =====================================================

    u_p = k_p * e_n;

    u_i = (k_i * ts * e_n_1) + u_n_1_i;

    u_d = (k_d * N * e_n)
        - (k_d * N * e_n_1)
        - (N * ts * u_n_1_d)
        + u_n_1_d;

    u_n = u_p + u_i + u_d;

    /*
    // =====================================================
    // MEJORA FUTURA: ANTI-WINDUP (CLAMPING)
    // =====================================================
    // INSTRUCCIONES DE USO:
    // Si el péndulo físico presenta un sobrepico masivo tras 
    // saturarse el motor, comenta las variables "u_i" y "u_n" 
    // de arriba y descomenta este bloque para activar la protección.
    
    float u_i_tentativo = (k_i * ts * e_n_1) + u_n_1_i;
    float u_n_tentativo = u_p + u_i_tentativo + u_d;

    if ((u_n_tentativo >= Uunits && e_n > 0) || (u_n_tentativo <= 0.0 && e_n < 0)) {
        u_i = u_n_1_i; // Saturado: Congelar el integrador
    } else {
        u_i = u_i_tentativo; // No saturado: Integrar normal
    }

    u_n = u_p + u_i + u_d;
    */

    // =====================================================
    // SATURACIÓN Y CONVERSIÓN A PWM
    // =====================================================

    U_t = constrain(u_n, 0.0, Uunits);

    pwmDuty = round((U_t / Uunits) * PWM_MAX);
    pwmDuty = constrain(pwmDuty, 0, PWM_MAX);

    analogWriteADJ(PWM_PIN, pwmDuty);

    // =====================================================
    // ACTUALIZAR MEMORIAS DEL PID
    // =====================================================

    e_n_1 = e_n;
    u_n_1_i = u_i;
    u_n_1_d = u_d;

    // =====================================================
    // MONITOR SERIAL
    // =====================================================

    float tiempoSegundos = (tiempoActual - tiempoInicio) / 1000.0;
    float pwmPorcentaje = (pwmDuty * 100.0) / PWM_MAX;

    Serial.print("Tiempo:");
    Serial.print(tiempoSegundos, 3);
    Serial.print(" , ");
    Serial.print("Angulo:");
    Serial.print(anguloGrados, 3);
    Serial.print(" , ");
    Serial.print("Error:");
    Serial.print(e_n, 3);
    Serial.print(" , ");
    Serial.print("U:");
    Serial.print(u_n, 3);
    Serial.print(" , ");
    Serial.print("PWM:");
    Serial.println(pwmPorcentaje, 2);
  }

  // Leer nueva referencia por Serial
  recvWithStartEndMarkers();

  if (newData == true) {
    parseData();
    newData = false;
  }
}

// =====================================================
// TIMER1: FAST PWM CON ICR1 COMO TOP
// =====================================================

void setupPWMadj() {
  // Pin 9 = PB1 = OC1A
  // Pin 10 = PB2 = OC1B
  DDRB |= _BV(PB1) | _BV(PB2);

  // Fast PWM, modo 14, salidas no inversoras
  TCCR1A =
      _BV(COM1A1) |
      _BV(COM1B1) |
      _BV(WGM11);

  // Sin preescalador
  TCCR1B =
      _BV(WGM13) |
      _BV(WGM12) |
      _BV(CS10);

  ICR1 = PWM_MAX;

  OCR1A = 0;
  OCR1B = 0;
}

// =====================================================
// ANALOG WRITE AJUSTADO A 12 BITS
// =====================================================

void analogWriteADJ(uint8_t pin, uint16_t valor) {
  valor = constrain(valor, 0, PWM_MAX);

  switch (pin) {
    case 9:
      OCR1A = valor;
      break;

    case 10:
      OCR1B = valor;
      break;

    default:
      break;
  }
}

// =====================================================
// ENTRADA SERIAL CON FORMATO <valor>
// Ejemplo: <30>
// =====================================================

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;

  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;

        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } 
      else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } 
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  Ref = atof(receivedChars);

  Serial.print("Nueva referencia: ");
  Serial.println(Ref);
}