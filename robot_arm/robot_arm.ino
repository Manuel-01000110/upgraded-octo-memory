/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║   Robot Arm 4 DOF — Arduino + PCA9685                   ║
 * ║   Riceve comandi dal Teach Pendant (Python/PyQt5)        ║
 * ║   Formato: "B090 J1+00 J2+30 J3-15\n"                   ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * Connessioni PCA9685:
 *   VCC  → 3.3V o 5V Arduino
 *   GND  → GND
 *   SDA  → A4 (Uno) / 20 (Mega)
 *   SCL  → A5 (Uno) / 21 (Mega)
 *   V+   → Alimentazione esterna 5-6V per i servo
 *
 * Canali PCA9685:
 *   CH0 → Servo BASE        (rotazione orizzontale)
 *   CH1 → Servo SPALLA      (giunto 1)
 *   CH2 → Servo GOMITO      (giunto 2)
 *   CH3 → Servo POLSO       (giunto 3)
 *   CH4 → Elettromagnete    (via relè o MOSFET)
 *
 * Librerie richieste (Library Manager):
 *   - Adafruit PWM Servo Driver Library
 *   - Wire (built-in)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── PCA9685 ────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);  // indirizzo I2C default

// ─── Configurazione servo ────────────────────────────────────────────────────
// Modifica questi valori in base ai tuoi servo specifici
// (misurali con un oscilloscopio o per tentativi)
#define SERVO_FREQ      50      // Hz — standard per servo analogici
#define SERVOMIN        102     // Pulse minimo (≈0°)
#define SERVOMAX        496     // Pulse massimo (≈180°)
#define SERVO_MID       299     // Pulse centro (≈90°)

// ─── Canali PCA9685 ──────────────────────────────────────────────────────────
#define CH_BASE         0
#define CH_SPALLA       1
#define CH_GOMITO       2
#define CH_POLSO        3
#define CH_MAGNETE      4

// ─── Pin elettromagnete (opzionale, se non usi CH4) ──────────────────────────
#define PIN_MAGNETE     2       // Pin digitale per relè / MOSFET

// ─── Angoli attuali (per il smoothing) ───────────────────────────────────────
float cur_base    = 90.0;
float cur_j1      = 0.0;
float cur_j2      = 0.0;
float cur_j3      = 0.0;

// ─── Limiti angoli per sicurezza ─────────────────────────────────────────────
#define BASE_MIN    0
#define BASE_MAX    180
#define JOINT_MIN  -90
#define JOINT_MAX   90

// ─── Smoothing ────────────────────────────────────────────────────────────────
// Fattore 0.0–1.0: più basso = più lento ma più fluido
#define SMOOTH_FACTOR   0.15f

// ─── Parsing seriale ─────────────────────────────────────────────────────────
String inputBuffer = "";
bool   msgReady    = false;

// target ricevuti
float tgt_base = 90.0;
float tgt_j1   = 0.0;
float tgt_j2   = 0.0;
float tgt_j3   = 0.0;


// ════════════════════════════════════════════════════════════════════════════
//  Utility: angolo → pulse PCA9685
// ════════════════════════════════════════════════════════════════════════════
uint16_t angleToPulse(float angle_deg) {
  // Mappa 0–180° → SERVOMIN–SERVOMAX
  angle_deg = constrain(angle_deg, 0.0f, 180.0f);
  return (uint16_t)map((long)(angle_deg * 100), 0, 18000,
                        SERVOMIN, SERVOMAX);
}

// ─── Muovi un canale ──────────────────────────────────────────────────────────
void setServo(uint8_t channel, float angle_deg) {
  uint16_t pulse = angleToPulse(angle_deg);
  pca.setPWM(channel, 0, pulse);
}

// ─── Elettromagnete ───────────────────────────────────────────────────────────
void setMagnete(bool on) {
  // Via PCA9685 ch4 (full-on / full-off)
  if (on) {
    pca.setPin(CH_MAGNETE, 4095, false);   // full ON
  } else {
    pca.setPin(CH_MAGNETE, 0, false);      // full OFF
  }
  // Via pin digitale (relè / MOSFET)
  digitalWrite(PIN_MAGNETE, on ? HIGH : LOW);
}


// ════════════════════════════════════════════════════════════════════════════
//  Parsing: "B090 J1+00 J2+30 J3-15"
// ════════════════════════════════════════════════════════════════════════════
void parseMessage(String msg) {
  msg.trim();

  // ── BASE: "B090" ──────────────────────────────────────────────────────────
  int bi = msg.indexOf('B');
  if (bi >= 0 && bi + 4 <= (int)msg.length()) {
    String bstr = msg.substring(bi + 1, bi + 4);
    float val = (float)bstr.toInt();
    val = constrain(val, BASE_MIN, BASE_MAX);
    tgt_base = val;
  }

  // ── JOINT 1: "J1±XX" ─────────────────────────────────────────────────────
  int j1i = msg.indexOf("J1");
  if (j1i >= 0 && j1i + 5 <= (int)msg.length()) {
    String jstr = msg.substring(j1i + 2, j1i + 6);
    float val = (float)jstr.toInt();
    val = constrain(val, JOINT_MIN, JOINT_MAX);
    tgt_j1 = val;
  }

  // ── JOINT 2: "J2±XX" ─────────────────────────────────────────────────────
  int j2i = msg.indexOf("J2");
  if (j2i >= 0 && j2i + 5 <= (int)msg.length()) {
    String jstr = msg.substring(j2i + 2, j2i + 6);
    float val = (float)jstr.toInt();
    val = constrain(val, JOINT_MIN, JOINT_MAX);
    tgt_j2 = val;
  }

  // ── JOINT 3: "J3±XX" ─────────────────────────────────────────────────────
  int j3i = msg.indexOf("J3");
  if (j3i >= 0 && j3i + 5 <= (int)msg.length()) {
    String jstr = msg.substring(j3i + 2, j3i + 6);
    float val = (float)jstr.toInt();
    val = constrain(val, JOINT_MIN, JOINT_MAX);
    tgt_j3 = val;
  }

  // ── ELETTROMAGNETE opzionale: "M1" accendi, "M0" spegni ──────────────────
  if (msg.indexOf("M1") >= 0) setMagnete(true);
  if (msg.indexOf("M0") >= 0) setMagnete(false);

  // Feedback su seriale
  Serial.print("OK B");
  Serial.print((int)tgt_base);
  Serial.print(" J1");
  Serial.print((int)tgt_j1);
  Serial.print(" J2");
  Serial.print((int)tgt_j2);
  Serial.print(" J3");
  Serial.println((int)tgt_j3);
}


// ════════════════════════════════════════════════════════════════════════════
//  Smoothing: interpolazione esponenziale verso target
// ════════════════════════════════════════════════════════════════════════════
void smoothStep() {
  cur_base += (tgt_base - cur_base) * SMOOTH_FACTOR;
  cur_j1   += (tgt_j1   - cur_j1)   * SMOOTH_FACTOR;
  cur_j2   += (tgt_j2   - cur_j2)   * SMOOTH_FACTOR;
  cur_j3   += (tgt_j3   - cur_j3)   * SMOOTH_FACTOR;
}

// ─── Angoli relativi → angoli assoluti servo ──────────────────────────────────
// I giunti ricevono angoli RELATIVI (±90°), i servo vogliono angoli ASSOLUTI (0–180°)
// Centro meccanico del servo = 90° = 0° relativo
void applyAngles() {
  setServo(CH_BASE,    cur_base);          // già assoluto (0–180°)
  setServo(CH_SPALLA,  cur_j1 + 90.0f);   // da ±90° a 0–180°
  setServo(CH_GOMITO,  cur_j2 + 90.0f);
  setServo(CH_POLSO,   cur_j3 + 90.0f);
}


// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("=== Robot Arm 4 DOF — Teach Pendant ready ===");
  Serial.println("Waiting for commands: B000 J1+00 J2+00 J3+00");

  // PCA9685
  pca.begin();
  pca.setOscillatorFrequency(27000000);  // calibra l'oscillatore interno
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Pin elettromagnete
  pinMode(PIN_MAGNETE, OUTPUT);
  digitalWrite(PIN_MAGNETE, LOW);

  // Posizione iniziale — centro su tutti i giunti
  tgt_base = 90.0; cur_base = 90.0;
  tgt_j1   = 0.0;  cur_j1   = 0.0;
  tgt_j2   = 0.0;  cur_j2   = 0.0;
  tgt_j3   = 0.0;  cur_j3   = 0.0;
  applyAngles();
  Serial.println("Home position set.");
}


// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════
void loop() {

  // ── Leggi seriale ──────────────────────────────────────────────────────────
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      msgReady = true;
    } else {
      inputBuffer += c;
    }
  }

  if (msgReady) {
    parseMessage(inputBuffer);
    inputBuffer = "";
    msgReady = false;
  }

  // ── Smoothing + scrittura servo ───────────────────────────────────────────
  smoothStep();
  applyAngles();

  delay(20);   // ~50 Hz, uguale alla frequenza PWM servo
}
