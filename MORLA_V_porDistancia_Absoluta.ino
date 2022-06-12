#include <AccelStepper.h>
#include "AccelStepperWithDistance.h"

#define ENB_LBW 24
#define ENB_LFW 30
#define ENB_RBW 38
#define ENB_RFW 56
#define FRONT_SPEED  1800
#define SIDE_SPEED 2200
#define TURN_SPEED 2000
#define MAX_SPEED 3500
#define MIN_SPEED     0
#define MAX_DIST  10000
#define ACCEL      1000
#define stepsPerRotation 800
#define distancePerRotation 303
#define mm/100steps  19
#define led 4

//     En este caso estoy usando una RAMPS 1.3, mapeo de pines al canto, el modulo BT esta en Serial2, rX 16 y tX 17
AccelStepperWithDistance LeftBackWheel(1, 26, 28);   // (Type:driver, STEP, DIR) - Stepper1 ENB_24
AccelStepperWithDistance LeftFrontWheel(1, 36, 34);  // Stepper2                            ENB_30
AccelStepperWithDistance RightBackWheel(1, 54, 55);  // Stepper3                            ENB_38
AccelStepperWithDistance RightFrontWheel(1, 60, 61); // Stepper4
//                                                     Variables de datos por BT
char BluetoothData;

//                                                     Contadores para tiempos de paro
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
int cambioDir = 2000;
int paroMot = 1600;

//                                                      Variables de joystick o medidas y mezcla ruedas
int RX, RY, LZ, LY, MX, MY ;
int speed_base, speed_side, speed_turn;
int RFW_speed, RBW_speed, LFW_speed, LBW_speed;
int mode;

void setup() {
  // Setup enablePins en la RAMPS 1.3
  LeftBackWheel.setEnablePin(ENB_LBW);
  LeftFrontWheel.setEnablePin(ENB_LFW);
  RightBackWheel.setEnablePin(ENB_RBW);
  RightFrontWheel.setEnablePin(ENB_RFW);

  LeftBackWheel.setPinsInverted(false, false, true);
  LeftFrontWheel.setPinsInverted(false, false, true);
  RightBackWheel.setPinsInverted(false, false, true);
  RightFrontWheel.setPinsInverted(false, false, true);

  // Setup de velocidad maxima y accel.
  LeftFrontWheel.setMaxSpeed (MAX_SPEED);
  LeftBackWheel.setMaxSpeed  (MAX_SPEED);
  RightFrontWheel.setMaxSpeed(MAX_SPEED);
  RightBackWheel.setMaxSpeed (MAX_SPEED);

  LeftFrontWheel.setAcceleration (ACCEL);
  LeftBackWheel.setAcceleration  (ACCEL);
  RightFrontWheel.setAcceleration(ACCEL);
  RightBackWheel.setAcceleration (ACCEL);

  //  Setup de distancias
  LeftBackWheel.setStepsPerRotation(stepsPerRotation);
  LeftFrontWheel.setStepsPerRotation(stepsPerRotation);
  RightBackWheel.setStepsPerRotation(stepsPerRotation);
  RightFrontWheel.setStepsPerRotation(stepsPerRotation);

  LeftBackWheel.setDistancePerRotation(distancePerRotation);
  LeftFrontWheel.setDistancePerRotation(distancePerRotation);
  RightBackWheel.setDistancePerRotation(distancePerRotation);
  RightFrontWheel.setDistancePerRotation(distancePerRotation);



  Serial.begin(115200);                //   Serial debug
  Serial1.begin(115200);                 // HC 05 en Serial1
  Serial1.setTimeout(10);
  // Serial2.begin (115200);             // Nano en Serial2
  // Serial2.setTimeout (10);


  /*
    LeftFrontWheel.setSpeed(0);
    LeftBackWheel.setSpeed(0);
    RightFrontWheel.setSpeed(0);
    RightBackWheel.setSpeed(0);
  */
}

void loop() {
  readBluetooth ();
  bateriaEstado ();
  switch (mode) {
    case 1:
      escribeVelocidad ();
      break;
    case 2:
      escribeDistancia ();
      break;
    default:
      paroMotores ();
      break;
  }
  getDistance();
  // serialDebug ();
}

void readBluetooth () {
  int modoPasosVel, modoMov;
  //                                                   Leemos Serial1
  if (Serial1.available()) {
    BluetoothData = Serial1.read();
    if (BluetoothData == 'J') {
      mode = 1;
    }
    if (BluetoothData == 'D') {
      mode = 2;
    }
    if (BluetoothData == 'R') {
      if (Serial1.available()) {
        BluetoothData = Serial1.read();
        if (BluetoothData == 'X') {
          RX = Serial1.parseInt();
          while (BluetoothData != '*') {
            if (Serial1.available()) {
              BluetoothData = Serial1.read();
              if (BluetoothData == 'Y') RY = Serial1.parseInt();
            }
          }
        }
      }
    }
    if (BluetoothData == 'L') {
      if (Serial1.available()) {
        BluetoothData = Serial1.read();
        if (BluetoothData == 'X') {
          LZ = Serial1.parseInt();
          while (BluetoothData != '*') {
            if (Serial1.available()) {
              BluetoothData = Serial1.read(); //Get next character from bluetooth
              if (BluetoothData == 'Y') LY = Serial1.parseInt();
            }
          }
        }
      }
    }
    /*     G0--- Movimiento lineal en EJE X e Y.
           G1--- Movimiento lineal diagonales netas mm.
           G2--- Rotación sobre eje Z, en grados.
           G3--- Arco, definir posicion final y radio.
           G28-- Regresa todos los motores al paso 0.
           G90-- Posicionamiento absoluto, en mm.
           G91-- Posicionamiento relativo, suma mm. a la posicion actual.
    */
    //   La orden GX__,Y__* computa movimientos por medidas
    if (BluetoothData == 'G') {
      while (true) {
        if (Serial1.available()) {
          BluetoothData = Serial1.read();
          if (BluetoothData == 'X') MX = Serial1.parseInt();
          if (BluetoothData == 'Y') MY = Serial1.parseInt();
          if (BluetoothData == '*') break;
        }
      }
    }
  }
}

void escribeVelocidad () {
  //                                                           Mezcla por rueda de 4 canales de los dos joysticks
  speed_base = map ( RY, -100, 100 , FRONT_SPEED, -FRONT_SPEED );
  speed_side = map ( RX, -100, 100 , SIDE_SPEED, -SIDE_SPEED );
  speed_turn = map ( LZ, -100, 100 , TURN_SPEED, -TURN_SPEED );

  LFW_speed = (( speed_base - speed_turn ) - speed_side );
  LBW_speed = (( speed_base - speed_turn ) + speed_side );
  RFW_speed = (( speed_base + speed_turn ) + speed_side );
  RBW_speed = (( speed_base + speed_turn ) - speed_side );

  //                                                               Desarmamos motores en velocidad 0 en todas las ruedas
  if (LFW_speed == 0 && LBW_speed == 0 && RFW_speed == 0 && RBW_speed == 0) {
    LeftFrontWheel.disableOutputs();
    LeftBackWheel.disableOutputs();
    RightFrontWheel.disableOutputs();
    RightBackWheel.disableOutputs();
  } else {
    LeftFrontWheel.enableOutputs();
    LeftBackWheel.enableOutputs();
    RightFrontWheel.enableOutputs();
    RightBackWheel.enableOutputs();
  }

  //                                                                  Escribimos valores en motores
  LeftFrontWheel.setSpeed(LFW_speed);
  LeftBackWheel.setSpeed(LBW_speed);
  RightFrontWheel.setSpeed(RFW_speed);
  RightBackWheel.setSpeed(RBW_speed);

  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
}
void escribeDistancia () {

  long LBW_dist, LFW_dist, RBW_dist, RFW_dist;

  if (MX != 0) {
    LeftFrontWheel.enableOutputs();
    LeftBackWheel.enableOutputs();
    RightFrontWheel.enableOutputs();
    RightBackWheel.enableOutputs();

    LeftFrontWheel.setMaxSpeed (3000);
    LeftBackWheel.setMaxSpeed  (3000);
    RightFrontWheel.setMaxSpeed(3000);
    RightBackWheel.setMaxSpeed (3000);

    LeftFrontWheel.setAcceleration (800);
    LeftBackWheel.setAcceleration  (800);
    RightFrontWheel.setAcceleration(800);
    RightBackWheel.setAcceleration (800);

    LBW_dist =  MY + (MX * -1);
    RFW_dist =  MY + (MX * -1);       // X positivo hacia Derecha
    LFW_dist =  MY + (MX * 1);
    RBW_dist =  MY + (MX * 1);        // Y positivo hacia Delante

  }
  //    RFW_dist son distancias absolutas, en cada orden sumaremos/restaremos pasos a cada rueda
  LeftFrontWheel.moveToDistance (LFW_dist);
  LeftBackWheel.moveToDistance  (LBW_dist);
  RightFrontWheel.moveToDistance(RFW_dist);
  RightBackWheel.moveToDistance (RBW_dist);

  LeftFrontWheel.run();
  LeftBackWheel.run();
  RightFrontWheel.run();
  RightBackWheel.run();
}


void serialDebug () {
  Serial.print (LFW_speed); Serial.print ("   "); Serial.print (LBW_speed); Serial.print ("   "); Serial.print (RFW_speed); Serial.print ("   "); Serial.println (RBW_speed);
  Serial.print("Left Front Pos: ");
  Serial.println( LeftFrontWheel.getCurrentPositionDistance() );
  Serial.print("Left Back Pos: ");
  Serial.println( LeftBackWheel.getCurrentPositionDistance() );
  Serial.print("Right Front Pos: ");
  Serial.println( RightFrontWheel.getCurrentPositionDistance() );
  Serial.print("Right Back Pos: ");
  Serial.println( RightBackWheel.getCurrentPositionDistance() );
  Serial.println();
}

void bateriaEstado () {
  // Monitorea la bateria
  int sensorValue = analogRead(A3);
  float voltage = sensorValue * (5.0 / 1023.00) * 3;
  // Serial.print ("Bateria    "); Serial.println(voltage); Serial.println();
  // Si el voltage esta debajo de 7,0V enciende LED rojo
  if (voltage < 7.00) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}

long getPostTimestamp;
long getPosDuration = 500;
void getDistance() {
  if ( millis() - getPostTimestamp > getPosDuration ) {
    getPostTimestamp = millis();
    Serial.print("Left Front Pos: ");
    Serial.println( LeftFrontWheel.getCurrentPositionDistance() );
    Serial.print("Left Back Pos: ");
    Serial.println( LeftBackWheel.getCurrentPositionDistance() );
    Serial.print("Right Front Pos: ");
    Serial.println( RightFrontWheel.getCurrentPositionDistance() );
    Serial.print("Right Back Pos: ");
    Serial.println( RightBackWheel.getCurrentPositionDistance() );
    Serial.println();
  }
}

void paroMotores () {
  LeftFrontWheel.disableOutputs();
  LeftBackWheel.disableOutputs();
  RightFrontWheel.disableOutputs();
  RightBackWheel.disableOutputs();
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
