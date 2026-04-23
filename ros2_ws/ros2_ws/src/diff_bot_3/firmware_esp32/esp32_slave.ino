/*
 * esp32_slave.ino  -  ESCLAVO del robot (Raspberry Pi 5 es maestro).
 *
 * Subsistemas controlados:
 *   - Motor paso a paso 12V con controlador DRV8825 (subir/bajar montacargas)
 *     *** Control SOLO por numero de pasos y direccion (sin finales
 *     de carrera) ***
 *   - Servo MG996 (palas / aspas del montacargas)
 *
 * Protocolo (texto por USB CDC, 115200 baud, lineas terminadas en '\n')
 *
 *   RPi -> ESP32:
 *     FORKS_OPEN            -> palas expandidas
 *     FORKS_CLOSE           -> palas recogidas
 *     STEP_UP <n>           -> mueve n pasos hacia arriba
 *     STEP_DOWN <n>         -> mueve n pasos hacia abajo
 *     STEP_STOP             -> cancela el movimiento actual
 *     STATUS?               -> pide estado
 *
 *   ESP32 -> RPi:
 *     READY                 -> enviado al arrancar
 *     OK                    -> comando aceptado
 *     DONE                  -> operacion finalizada
 *     ERR:<causa>           -> error
 *     STATUS:pos=<n>,run=<0|1>
 *
 * Nota: "pos" es un contador relativo.  Vale 0 al encender la ESP32
 * y el MAESTRO es responsable de recordar la referencia (por eso es
 * importante siempre arrancar el sistema con el montacargas en su
 * posicion BAJA / home).
 */

#include <ESP32Servo.h>

// -------------------- PINES (ESP32) --------------------
#define PIN_STEP       18    // DRV8825 STEP
#define PIN_DIR        19    // DRV8825 DIR
#define PIN_ENABLE     21    // DRV8825 ENABLE (activo en LOW)
#define PIN_SERVO      13    // MG996 signal

// -------------------- PARAMETROS -----------------------
// Periodo entre pulsos (define la velocidad del stepper).
// Con 400us (2.5 kHz) y 1/16 micro-pasos el montacargas sube suave y
// sin perder pasos incluso con el cubo (15x15x15 cm).  Ajustar segun
// la mecanica.
const unsigned long STEP_PULSE_US = 400;

// Angulos del servo MG996 (0..180)
const int SERVO_RETRAIDO   = 30;
const int SERVO_EXPANDIDO  = 120;

// -------------------- ESTADO ---------------------------
Servo servoPalas;
long  lift_pos       = 0;     // contador relativo de pasos (+up, -down)
bool  lift_running   = false;
int   lift_dir       = 0;     // +1 up, -1 down
long  lift_remaining = 0;     // pasos que faltan en la orden en curso
unsigned long last_step_us = 0;

// -------------------- UTIL -----------------------------
void sendLine(const String& s) {
  Serial.print(s);
  Serial.print('\n');
}

void enableDriver(bool on) {
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);  // ENABLE activo en LOW
}

void startSteps(int dir, long n) {
  if (n <= 0) { sendLine("DONE"); return; }
  lift_dir = dir > 0 ? +1 : -1;
  lift_remaining = n;
  digitalWrite(PIN_DIR, lift_dir > 0 ? HIGH : LOW);
  enableDriver(true);
  lift_running = true;
  last_step_us = micros();
}

void stopLift() {
  lift_running = false;
  lift_dir = 0;
  lift_remaining = 0;
  enableDriver(false);
}

// Movimiento no bloqueante: se invoca en cada loop().
void stepperTick() {
  if (!lift_running) return;
  unsigned long now = micros();
  if ((now - last_step_us) < STEP_PULSE_US) return;
  last_step_us = now;

  // Pulso STEP
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_STEP, LOW);

  lift_pos += lift_dir;
  lift_remaining--;

  if (lift_remaining <= 0) {
    stopLift();
    sendLine("DONE");
  }
}

// -------------------- COMANDOS -------------------------
long parseLongAfter(const String& line, const String& head) {
  int sp = line.indexOf(' ');
  if (sp < 0) return -1;
  String num = line.substring(sp + 1);
  num.trim();
  return num.toInt();
}

void cmdForksOpen() {
  servoPalas.write(SERVO_EXPANDIDO);
  delay(450);
  sendLine("DONE");
}

void cmdForksClose() {
  servoPalas.write(SERVO_RETRAIDO);
  delay(450);
  sendLine("DONE");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "FORKS_OPEN")        { sendLine("OK"); cmdForksOpen(); }
  else if (line == "FORKS_CLOSE")  { sendLine("OK"); cmdForksClose(); }
  else if (line.startsWith("STEP_UP")) {
    long n = parseLongAfter(line, "STEP_UP");
    if (n <= 0) { sendLine("ERR:bad_steps"); return; }
    sendLine("OK");
    startSteps(+1, n);
  }
  else if (line.startsWith("STEP_DOWN")) {
    long n = parseLongAfter(line, "STEP_DOWN");
    if (n <= 0) { sendLine("ERR:bad_steps"); return; }
    sendLine("OK");
    startSteps(-1, n);
  }
  else if (line == "STEP_STOP") {
    sendLine("OK");
    stopLift();
    sendLine("DONE");
  }
  else if (line == "STATUS?") {
    String s = "STATUS:pos=" + String(lift_pos) +
               ",run="      + String(lift_running ? 1 : 0);
    sendLine(s);
  }
  else {
    sendLine("ERR:unknown_cmd");
  }
}

// -------------------- ARDUINO SETUP/LOOP ----------------
void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);     // driver deshabilitado al inicio

  servoPalas.setPeriodHertz(50);
  servoPalas.attach(PIN_SERVO, 500, 2400);
  servoPalas.write(SERVO_RETRAIDO);

  Serial.begin(115200);
  delay(200);
  sendLine("READY");
}

static String rx;

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx.length() > 0) { handleLine(rx); rx = ""; }
    } else {
      rx += c;
      if (rx.length() > 64) rx = "";
    }
  }
  stepperTick();
}
