// Pruebas Servo Robotica.ino
#include <Servo.h>

/*==============================
=            Servos            =
==============================*/
Servo SERVO_BASE;
Servo SERVO_ROTACION1;
Servo SERVO_ROTACION2;

void setup() 
{
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  SERVO_BASE.attach(9);
  SERVO_ROTACION1.attach(10);
  SERVO_ROTACION2.attach(11);
  Serial.begin(9600);
}

void loop() 
{
  /*=================================
  =            variables            =
  =================================*/
  
  int ROTACION_BASE = map(analogRead(A1), 0, 1023, 0, 180);
  int ROTACION_ESLABONES = map(analogRead(A2), 0, 1023, 0, 180);
  int TRASLACION_BRAZO = map(analogRead(A0), 0, 1023, 180, 90);
  int MAPEO_ESLABON = map(analogRead(A2), 0, 1023, 0, 180);

  /*================================
  =            Acciones            =
  =================================*/

  float a = 0;  // variable base 20 veces filtro
  float b = 0;  // variable eslabon 20 veces filtro
  float c =0;   // variable traslacion 20 veces filtro
  float TMP = 0; // variable mapeo de eslabon - 90
  
 for(int k = 0 ; k < 20 ; k++ )
 {
 	a =+ map(analogRead(A1), 0, 1023, 0, 180);   //BASE
 	b = + map(analogRead(A0), 0, 1023, 20, 180);
 	c =+ map(analogRead(A0), 0, 1023, 180, 90);  // TRASLACION
 	MAPEO_ESLABON = map(analogRead(A2), 0, 1023, 0, 180);
 }
  TMP = MAPEO_ESLABON - 90;
  
  SERVO_BASE.write(a/0.5);
  SERVO_ROTACION1.write((c + TMP)/0.08);    // 0.07
  SERVO_ROTACION2.write(b/0.06);   //0.081
  
  Serial.print(a);
  Serial.print('\t');
  Serial.print(ROTACION_BASE);
  Serial.print('\t');
  Serial.print(ROTACION_ESLABONES);
  Serial.print('\t');
  Serial.print(TRASLACION_BRAZO);
  Serial.println();
  
}

