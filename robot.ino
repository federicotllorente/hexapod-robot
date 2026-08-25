#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Bluepad32.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define POW2(x)       ((x)*(x))

// ── Modo de control ──────────────────────────────────────
// Cambia esta línea para elegir el modo activo:
// CONTROL_RADIO    → usa el control nRF24 que hiciste
// CONTROL_BT       → usa mando Xbox/PS5 por Bluetooth
#define CONTROL_BT
// #define CONTROL_RADIO
// ─────────────────────────────────────────────────────────

// ── PCA9685 ──────────────────────────────────────────────
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);

// #define SERVO_MIN_PULSE  150
#define SERVO_MIN_PULSE  125
// #define SERVO_MAX_PULSE  600
#define SERVO_MAX_PULSE  575

int angleToPulse(float angle) 
{
  angle = constrain(angle, 0, 180);
  return map((int)angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void writeServo(int legIndex, int servoIndex, float angle)
{
  int channel = (legIndex % 3) * 3 + servoIndex;
  int pulse = angleToPulse(angle);
  if (legIndex < 3)
    pca1.setPWM(channel, 0, pulse);
  else
    pca2.setPWM(channel, 0, pulse);
}
// ─────────────────────────────────────────────────────────

// ── Radio nRF24 ──────────────────────────────────────────
#ifdef CONTROL_RADIO
  RF24 radio(4, 5);
  const uint64_t pipe = 0xE8E8F0F0E1LL;
  int radioData[10] = { 500, 500, 500, 500, 500, 500, 0, 0, 0, 0 };
  unsigned long lastRadioReceived = 0;
  const unsigned long RADIO_TIMEOUT_MS = 500;
#endif
// ─────────────────────────────────────────────────────────

// ── Bluepad32 (Bluetooth) ────────────────────────────────
#ifdef CONTROL_BT
  GamepadPtr btGamepad = nullptr;

  void onConnectedGamepad(GamepadPtr gp)
  {
    btGamepad = gp;
    Serial.println("Mando conectado!");
  }

  void onDisconnectedGamepad(GamepadPtr gp)
  {
    btGamepad = nullptr;
    Serial.println("Mando desconectado.");
  }
#endif
// ─────────────────────────────────────────────────────────

struct Vector
{
  float x, y, z;
  Vector(float _x = 0.0, float _y = 0.0, float _z = 0.0)
    : x(_x), y(_y), z(_z) {}
};

struct Leg
{
  Vector home, origin, target;
  int legIndex;

  Leg(int _legIndex, float _x, float _y)
    : legIndex(_legIndex), origin(_x, _y, 0) {}

  void init()
  {
    origin.z = atan2(origin.y, origin.x);
    for (int i = 0; i < 3; i++)
      writeServo(legIndex, i, 90);
  }
};

// ── Dimensiones ──────────────────────────────────────────
const float COXA_LENGTH  = 38.0;
const float FEMUR_LENGTH = 86.25;
const float TIBIA_LENGTH = 160.5;

const float STEP_LENGTH  = 60.0;
const float STEP_HEIGHT  = 100.0;
const float DEFAULT_Z    = -40.0;
// const float DEFAULT_Z    = -120.0;

const float WALK_SMOOTHING_FACTOR = 0.4;
const float STOP_SMOOTHING_FACTOR = 0.4;
const float ROTATION_STEP_RAD    = DEG_TO_RAD(15.0);

// Calibración de postura — ajusta AY_REST hasta que las patas
// queden extendidas hacia abajo con el cuerpo elevado
// const float AY_REST = 60.0;  // empieza aquí, sube de 5 en 5
const float AY_REST = -60.0;
const float AZ_REST = 120.0; // ángulo tibia, ajusta si es necesario

// Control de velocidad
float walkSpeed = 0.008;
const float WALK_SPEED_MIN = 0.003;
const float WALK_SPEED_MAX = 0.020;
const float WALK_SPEED_STEP = 0.002;
// ─────────────────────────────────────────────────────────

Leg legs[6] =
{
  Leg(0,  100.94, -57.11),   // RF
  Leg(1,    0.0,  -105.72),  // RM
  Leg(2, -100.94, -57.11),   // RR
  Leg(3, -100.94,  57.11),   // LR
  Leg(4,    0.0,   105.72),  // LM
  Leg(5,  100.94,  57.11)    // LF
};

float t = 0;

void calculate_fk(const Vector& angles, const Vector& pos, Vector res[4]);
void calculate_ik(const Leg& leg, Vector& res);
Vector get_foot_target(float t, size_t legIndex, float direction, float rotation);
void set_foot_positions(float direction, float rotation);

void setup()
{
  Serial.begin(115200);

  // I2C en pines ESP32
  Wire.begin(21, 22);

  // PCA9685
  pca1.begin();
  pca1.setOscillatorFrequency(27000000);
  pca1.setPWMFreq(50);

  pca2.begin();
  pca2.setOscillatorFrequency(27000000);
  pca2.setPWMFreq(50);

  delay(10);

  // ── Init radio ─────────────────────────────────────────
  #ifdef CONTROL_RADIO
    SPI.begin(18, 19, 23, 5); // SCK, MISO, MOSI, CSN
    if (!radio.begin()) {
      Serial.println("ERROR: nRF24 no detectado. Revisa cableado.");
      // No bloqueamos con while(1) para que el robot igualmente inicie
    } else {
      Serial.println("nRF24 OK");
      radio.setPALevel(RF24_PA_LOW);
      radio.openReadingPipe(1, pipe);
      radio.startListening();
    }
  #endif

  // ── Init Bluetooth ─────────────────────────────────────
  #ifdef CONTROL_BT
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys(); // Limpia emparejamientos viejos
    Serial.println("Bluetooth listo. Enciende el mando en modo emparejamiento.");
  #endif

  // ── Init patas ─────────────────────────────────────────
  Vector neutralPoints[4] = {};
  for (size_t i = 0; i < 6; i++)
  {
    legs[i].init();
    // const Vector neutralAngles(0, M_PI / 6, -M_PI / 3);
    const Vector neutralAngles(0, DEG_TO_RAD(AY_REST), DEG_TO_RAD(-(180.0 - AZ_REST)));
    calculate_fk(neutralAngles, legs[i].origin, neutralPoints);
    legs[i].home = neutralPoints[3];
    // legs[i].home.z = DEFAULT_Z;
    legs[i].target = legs[i].home;
    
    // DEBUG
    // Serial.print("Pata "); Serial.print(i);
    // Serial.print(" home x="); Serial.print(legs[i].home.x);
    // Serial.print(" y="); Serial.print(legs[i].home.y);
    // Serial.print(" z="); Serial.println(legs[i].home.z);
  }

  delay(1000);
}

void loop()
{
  float direction = 0.0;
  float rotation  = 0.0;

  // ── Leer control nRF24 ─────────────────────────────────
  #ifdef CONTROL_RADIO
    if (radio.available()) {
      radio.read(radioData, sizeof(radioData));
      lastRadioReceived = millis();
    }

    if (millis() - lastRadioReceived > RADIO_TIMEOUT_MS) {
      set_foot_positions(0.0, 0.0);
      delay(10);
      return;
    }

    if (radioData[0] < 300)       direction = -1.0;
    else if (radioData[0] > 700)  direction =  1.0;
    if (radioData[3] < 300)       rotation  =  1.0;
    else if (radioData[3] > 700)  rotation  = -1.0;
  #endif

  // ── Leer mando Bluetooth ───────────────────────────────
  #ifdef CONTROL_BT
    BP32.update();

    if (btGamepad && btGamepad->isConnected())
    {
      // Joystick izquierdo: adelante/atrás
      // Rango Bluepad32: -512 a 511
      int ly = btGamepad->axisY(); // eje Y joystick izquierdo
      int rx = btGamepad->axisRX(); // eje X joystick derecho

      if (ly < -200)       direction =  1.0; // adelante
      else if (ly > 200)   direction = -1.0; // atrás

      if (rx > 200)        rotation  = -1.0; // girar derecha
      else if (rx < -200)  rotation  =  1.0; // girar izquierda

      // Cruceta arriba/abajo → velocidad
      int dpad = btGamepad->dpad();
      if (dpad & DPAD_UP) {
        walkSpeed = constrain(walkSpeed + WALK_SPEED_STEP, WALK_SPEED_MIN, WALK_SPEED_MAX);
        Serial.print("Velocidad: "); Serial.println(walkSpeed);
        delay(200); // debounce
      }
      if (dpad & DPAD_DOWN) {
        walkSpeed = constrain(walkSpeed - WALK_SPEED_STEP, WALK_SPEED_MIN, WALK_SPEED_MAX);
        Serial.print("Velocidad: "); Serial.println(walkSpeed);
        delay(200);
      }

      // Debug opcional:
      // Serial.print("ly="); Serial.print(ly);
      // Serial.print(" rx="); Serial.println(rx);
    }
    else
    {
      // Sin mando conectado → quieto
      set_foot_positions(0.0, 0.0);
      delay(10);
      return;
    }
  #endif

  set_foot_positions(direction, rotation);
  delay(10);
}

// ── FK ────────────────────────────────────────────────────
void calculate_fk(const Vector& angles, const Vector& pos, Vector res[4])
{
  const float alpha = angles.x + pos.z;
  const float beta  = angles.y;
  const float gamma = angles.z;

  const float ca  = cos(alpha), sa  = sin(alpha);
  const float cb  = cos(beta),  sb  = sin(beta);
  const float cbg = cos(beta + gamma), sbg = sin(beta + gamma);

  res[0] = Vector(pos.x, pos.y, 0);
  res[1].x = res[0].x + COXA_LENGTH * ca;
  res[1].y = res[0].y + COXA_LENGTH * sa;
  res[1].z = res[0].z;
  res[2].x = res[1].x + FEMUR_LENGTH * cb * ca;
  res[2].y = res[1].y + FEMUR_LENGTH * cb * sa;
  res[2].z = res[1].z + FEMUR_LENGTH * sb;
  res[3].x = res[1].x + (FEMUR_LENGTH * cb + TIBIA_LENGTH * cbg) * ca;
  res[3].y = res[1].y + (FEMUR_LENGTH * cb + TIBIA_LENGTH * cbg) * sa;
  res[3].z = res[1].z + (FEMUR_LENGTH * sb + TIBIA_LENGTH * sbg);
}

// ── IK ────────────────────────────────────────────────────
void calculate_ik(const Leg& leg, Vector& res)
{
  Vector rel(
    leg.target.x - leg.origin.x,
    leg.target.y - leg.origin.y,
    leg.target.z
  );

  const float c = cos(-leg.origin.z);
  const float s = sin(-leg.origin.z);
  const float tx = rel.x, ty = rel.y;
  rel.x = tx * c - ty * s;
  rel.y = tx * s + ty * c;

  const float l_xy = sqrt(POW2(rel.x) + POW2(rel.y));
  const float l_forward = l_xy - COXA_LENGTH;
  float D = sqrt(POW2(l_forward) + POW2(rel.z));

  if (D > (FEMUR_LENGTH + TIBIA_LENGTH))
    D = FEMUR_LENGTH + TIBIA_LENGTH - 0.001;
  if (D < fabs(FEMUR_LENGTH - TIBIA_LENGTH))
    D = fabs(FEMUR_LENGTH - TIBIA_LENGTH) + 0.001;

  res.z = RAD_TO_DEG(acos((POW2(FEMUR_LENGTH) + POW2(TIBIA_LENGTH) - POW2(D))
          / (2.0 * FEMUR_LENGTH * TIBIA_LENGTH)) - M_PI);

  const float beta1 = atan2(rel.z, l_forward);
  const float beta2 = acos((POW2(FEMUR_LENGTH) + POW2(D) - POW2(TIBIA_LENGTH))
                     / (2.0 * FEMUR_LENGTH * D));
  res.y = RAD_TO_DEG(beta1 + beta2);
  res.x = RAD_TO_DEG(atan2(rel.y, rel.x));
}

// ── Gait ──────────────────────────────────────────────────
Vector get_foot_target(float t, size_t legIndex, float direction, float rotation)
{
  float phase = fmod(t, 1.0);
  if ((legIndex % 2) != 0)
    phase = fmod(phase + 0.5, 1.0);

  Vector home_pos = legs[legIndex].home;

  if (direction == 0 && rotation == 0)
    return home_pos;

  float step_x_move = 0.0, step_rot_rad = 0.0, step_z_move = 0.0;

  if (phase < 0.5)
  {
    float pn = phase * 2.0;
    step_x_move  = direction * (0.5 - pn) * STEP_LENGTH;
    step_rot_rad = rotation  * (0.5 - pn) * ROTATION_STEP_RAD;
  }
  else
  {
    float pn = (phase - 0.5) * 2.0;
    step_x_move  = direction * (pn - 0.5) * STEP_LENGTH;
    step_rot_rad = rotation  * (pn - 0.5) * ROTATION_STEP_RAD;
    step_z_move  = sin(pn * M_PI) * STEP_HEIGHT;
  }

  float c_rot = cos(step_rot_rad);
  float s_rot = sin(step_rot_rad);

  Vector res;
  res.x = home_pos.x * c_rot - home_pos.y * s_rot + step_x_move;
  res.y = home_pos.x * s_rot + home_pos.y * c_rot;
  res.z = home_pos.z + step_z_move;
  return res;
}

void set_foot_positions(float direction, float rotation)
{
  // t += 0.015;
  t += walkSpeed;

  Vector angles;
  for (size_t i = 0; i < 6; i++)
  {
    const Vector goalTarget = get_foot_target(t, i, direction, rotation);
    const float sf = (direction == 0) ? STOP_SMOOTHING_FACTOR : WALK_SMOOTHING_FACTOR;

    legs[i].target.x += (goalTarget.x - legs[i].target.x) * sf;
    legs[i].target.y += (goalTarget.y - legs[i].target.y) * sf;
    legs[i].target.z += (goalTarget.z - legs[i].target.z) * sf;

    calculate_ik(legs[i], angles);

    float ax = constrain(90 + angles.x, 45, 135);
    // float ay = 90 + angles.y;
    // float az = fabs(angles.z);
    float ay = constrain(90 + angles.y, 0, 180);  // añadiendo constrain
    float az = constrain(fabs(angles.z), 0, 120); // añadiendo constrain

    writeServo(i, 0, ax);
    writeServo(i, 1, ay);
    writeServo(i, 2, az);

    // DEBUG
    Serial.print("Pata "); Serial.print(i);
    Serial.print(" ax="); Serial.print(ax);
    Serial.print(" ay="); Serial.print(ay);
    Serial.print(" az="); Serial.println(az);
  }
}
