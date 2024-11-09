#include <QTRSensors.h>
//Portas driver motor
#define PININ1 2
#define PININ2 4
#define PININ3 5
#define PININ4 7
#define PINENA 3
#define PINENB 6
//define a velocidade base
#define SPEED0 255
// Valor para executar o percurso 
int P;
int I;
int D;
int lastError = 0;
float   Kp = 0.25; 
float 	Ki = 0.0008; 
float   Kd = 0.04;
float 	dt = 0.1;
int setPoint = 2500;
int position;
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{
  // Definições das portas digitais
  pinMode(PININ1, OUTPUT);
  pinMode(PININ2, OUTPUT);
  pinMode(PININ3, OUTPUT);
  pinMode(PININ4, OUTPUT);
  pinMode(PINENA, OUTPUT);
  pinMode(PINENB, OUTPUT);
 
  // configura os sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // liga a luz para indicar que está em modo de calibragem

  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // desliga a luz para indicar que a calibragem acabou

  // mostra no Serial Monitor os valores de calibragem minima
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // mostra no Serial Monitor os valores de calibragem maxima
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
    position = qtr.readLineWhite(sensorValues);
    PIDcontroller(position, setPoint);
}

void PIDcontroller(int position, int setPoint){
  int error = position - setPoint;
  P = error;
  I += (error*dt);
  D = (error - lastError)/dt;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; 
  int motorspeeda = SPEED0 + motorspeed;
  int motorspeedb = SPEED0 - motorspeed;

  //velocidade maxima
  if (motorspeeda > SPEED0) {
    motorspeeda = SPEED0;
   }
  if (motorspeedb > SPEED0) {
    motorspeedb = SPEED0;
  }
   //velocidade minima
   if (motorspeeda < -255) {
    motorspeeda = -255;
  }
  if (motorspeedb < -255)   {
    motorspeedb = -255;
  } 
  motorControl(motorspeeda, motorspeedb);
}

void motorControl(int speedLeft, int speedRight) {
  //Robo vai pra frente;
  if (speedLeft > 0 && speedRight > 0){
    digitalWrite (PININ3, HIGH);
    digitalWrite (PININ4, LOW);

    digitalWrite (PININ1, HIGH);
    digitalWrite (PININ2, LOW);
  }
  //robo vira para esquerda
  else if(speedLeft < 0 && speedRight > 0) {
    speedLeft *= -1;
    digitalWrite (PININ3, HIGH);
    digitalWrite (PININ4, LOW);

    digitalWrite (PININ1, LOW);
    digitalWrite (PININ2, HIGH);
  }
  //motor vira para a direita
  else if(speedLeft > 0 && speedRight < 0){
    speedRight *= -1;
    digitalWrite (PININ3, LOW);
    digitalWrite (PININ4, HIGH);

    digitalWrite (PININ1, HIGH);
    digitalWrite (PININ2, LOW);
  } 
  analogWrite (PINENA, speedLeft);
  analogWrite (PINENB, speedRight);
}