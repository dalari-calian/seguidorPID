#include <QTRSensors.h>

// Portas driver motor
#define PININ1 2
#define PININ2 4
#define PININ3 5
#define PININ4 7
#define PINENA 3
#define PINENB 6

// Portas led rgb
#define PINLEDR 9
#define PINLEDG 11
#define PINLEDB 10

// Portas sensor QTR
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5

// Valores de ajustes para o seguidor de linha MIF
#define TRESHOLD 600                       // Valor de referencia para cor da linha branca
#define SPEED0 255                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 0 0) 
#define SPEED1 220                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 1 0) 

#define SPEED2 150                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 0 0) 
#define SPEED3 100                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 0)  
#define SPEED4 80                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 1) 

#define SPEED5 50                            // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 0) 
#define SPEED6 0                            // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 1) 
#define SPEED7 200                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 0 1) 

#define RUNTIME 24500                      // Valor para executar o percurso 
int P;
int I;
int D;
int lastError = 0;

float   Kp = 0.2; 
float 	Ki = 0.0008; 
float   Kd = 0.9; 
float 	dt = 0.5;
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
int setPoint = 2500;


void setup() {
  /*
  Serial.begin(9600);
  ledControl(13, true, 500);
  ledControl(13, false, 500);
  ledControl(13, true, 500);
  ledControl(13, false, 500);
  */
  pinMode(PININ1, OUTPUT);
  pinMode(PININ2, OUTPUT);
  pinMode(PININ3, OUTPUT);
  pinMode(PININ4, OUTPUT);
  pinMode(PINENA, OUTPUT);
  pinMode(PINENB, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  int position = qtr.readLineWhite(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);
  // Serial.println(position - setPoint);
  // delay(250);
  PIDcontroller(position, setPoint);
}

void motorControl(int speedLeft, int speedRight) {
  // Função para controle do driver de motor
  // Ajustes motor da esquerda
  if (speedLeft <= 0) {
    speedLeft = -speedLeft;
    digitalWrite (PININ3, LOW);
    digitalWrite (PININ4, LOW);
  } else {
    digitalWrite (PININ3, LOW);
    digitalWrite (PININ4, HIGH);
  }

  // Ajustes motor da direita
  if (speedRight < 0) {
    speedRight = -speedRight;
    digitalWrite (PININ1, LOW);
    digitalWrite (PININ2, LOW);
  } else {
    digitalWrite (PININ1, LOW);
    digitalWrite (PININ2, LOW);
  }
  // delay(100);
  // Serial.println(speedLeft);
  // Serial.print("\t");
  // Serial.print(speedRight);
  analogWrite (PINENB, speedLeft);
  analogWrite (PINENA, speedRight);
}

void PIDcontroller(int position, int setPoint){
  int error = position - setPoint;
  P = error;
  I += 0;//(error*dt);
  D = 0;//(error - lastError)/dt;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  int motorspeeda = SPEED0 + motorspeed;
  int motorspeedb = SPEED0 - motorspeed;
  if (motorspeeda > SPEED0) {
    motorspeeda = SPEED0;
   }
  if (motorspeedb > SPEED0) {
    motorspeedb = SPEED0;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0)   {
    motorspeedb = 0;
  } 
  // Serial.prAintln(motorspeeda);
  // Serial.println(motorspeedb);
  // delay(100);
  motorControl(motorspeeda, motorspeedb);
}
// bool motorStop(long runtime, long currentTime) {
//   // Função de parada do robô
//   if (millis() >= (runtime + currentTime)) {
//     motorControl(0, 0);
//     int cont = 0;
//     while (true) {
//       ledControl(13, true, 250);
//       ledControl(13, false, 250);
//       cont++;
//     }
//     return false;
//   }
//   return true;
// }

/*
void rgbControl(int red, int green, int blue, long rumtime) {
  // Função para controle do led rgb
  pinMode(PINLEDR, OUTPUT);
  pinMode(PINLEDG, OUTPUT);
  pinMode(PINLEDB, OUTPUT);

  digitalWrite(PINLEDR, HIGH);
  digitalWrite(PINLEDG, HIGH);
  digitalWrite(PINLEDB, HIGH);

  analogWrite(PINLEDR, red);
  analogWrite(PINLEDG, green);
  analogWrite(PINLEDB, blue);
  delay(rumtime);
}
*/

/*
void ledControl(int led, bool status, long rumtime) {
  // Função para controle do led
  pinMode(led, OUTPUT);
  if (status) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }
  delay(rumtime);
}
*/

// void readSensors(bool readSerial, int *sensors) {
//   // Função para leitura dos sensores
//   sensors[0] = analogRead(S1);
//   sensors[1] = analogRead(S2);
//   sensors[2] = analogRead(S3);
//   sensors[3] = analogRead(S4);
//   sensors[4] = analogRead(S5);
//   sensors[5] = analogRead(S6);
//   if (readSerial) {
//     Serial.print(sensors[0]);
//     Serial.print(' ');
//     Serial.print(sensors[1]);
//     Serial.print(' ');
//     Serial.print(sensors[2]);
//     Serial.print(' ');
//     Serial.print(sensors[3]);
//     Serial.print(' ');
//     Serial.print(sensors[4]);
//     Serial.print(' ');
//     Serial.println(sensors[5]);
//   }
// }
/*
int sensorPondSum(int *sensors){
  int sum = 0;
  if(sensors[0] >= TRESHOLD){
    sum += 200;
  }
  if(sensors[1] >= TRESHOLD){
    sum += 100;
  }
  if(sensors[2] >= TRESHOLD){
    sum += 50;
  }
  if(sensors[3] >= TRESHOLD){
    sum += -50;
  }
  if(sensors[4] >= TRESHOLD){
    sum += -100;
  }
  if(sensors[5] >= TRESHOLD){
    sum += -200;
  }
  return sum;
}*/



// void followLineMEF(void) {
//   // Função para controle do seguidor de linha em modo de maquina de estado finita
//   bool flag = true;
//   while (flag) {
//     // Flag para verificar a parada
//     flag = motorStop(RUNTIME, currentTime);

//     // Leitura sensores
//     int s[6];
//     readSensors(false, s);
//     PIDcontroller(sensorPondSum(s));

// }}