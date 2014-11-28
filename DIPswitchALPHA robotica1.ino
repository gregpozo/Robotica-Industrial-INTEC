/*===================================================
=         Practica 1 Robotica industrial            =
====================================================*/
/*****************************************************************************************************************
Gregory Pozo 	  1045331 
Angel Berroa 	  1040705
Hector Moreta   1045643
Programa que utiliza disco giratorio y sensores para determinar velocidad y posicion de un motor (ENCODER)(GITHUB)
******************************************************************************************************************/

/*==========  Declaracion de librerias  ===========*/
#include <QTRSensors.h>                                               // Libreria sensores array pololu																                
#include <LiquidCrystal.h>                                            // Libreria pantallas LCD				

/*==========  Declaracion de variables  ===========*/
#define BOTONDER       53
#define BOTONIZQ       52
#define ENABLE_MOTOR   2
#define RIGHT_MOTOR1   3
#define RIGHT_MOTOR2   4
#define RIGHT_MOTORPWM 5
#define PWMW           120
#define PWMC           100
#define NUM_SENSORS    5
#define TIMEOUT        2500                                           // waits 2500 microseconds for sensor outputs to go low
#define LAPCOUNTER     18
#define DIPSWITCH1     51
#define DIPSWITCH2     49
#define DIPSWITCH3     47
#define DIPSWITCH4     45
#define DIPSWITCH5     43
/* Funcion para arreglo de sensores pololu digitales */
QTRSensorsRC qtrrc((unsigned char[]) {13, 11, 10, 9, 8},NUM_SENSORS, TIMEOUT);
LiquidCrystal lcd(41, 39, 37, 35, 33, 31);                            // Pines de inicio de LCD 			
unsigned int sensorValues[NUM_SENSORS];													      
int count     = 0;                                                
int giro      = 0;                                                    // Variable de control de giro
int DIPvalue  = 0;          
int sensorPos = 0;                                                    // Variable de posicion del disco en base a sensors
int MotorON   = 0;                                                    // Variable para determinar el tiempo que dura encendido el motor

/*============  MAIN  ============*/
void setup()
{
  lcd.begin(16,2);                                                    // set up the LCD's number of columns and rows
  lcd.print("Dir:  #Vuelt:");                                         // Imprime direccion y vuelta al inicio
  lcd.setCursor(0,1);
  lcd.print("DS: ");                                                  // Imprime Dip switch al inicio 
  lcd.setCursor(6,1);											                		      
  lcd.print("PSen: ");                                                // Posicion de sensor al inicio
	//attachInterrupt(5, increment, RISING);											      // Interrupcion en el micro #5, para lectura de break B
  pinMode(RIGHT_MOTOR1, OUTPUT);														          
  pinMode(RIGHT_MOTOR2, OUTPUT);														           
  pinMode(RIGHT_MOTORPWM, OUTPUT);														       
  pinMode(ENABLE_MOTOR, OUTPUT);														          
  pinMode(BOTONDER, INPUT);																             
  pinMode(BOTONIZQ, INPUT);																             
  pinMode(LAPCOUNTER, INPUT);																           
  pinMode(DIPSWITCH1, INPUT);															            
  pinMode(DIPSWITCH2, INPUT);															            		
  pinMode(DIPSWITCH3, INPUT);															             
  pinMode(DIPSWITCH4, INPUT);															            
  pinMode(DIPSWITCH5, INPUT);															             
  //digitalWrite(3, HIGH);    											 
  motorCalibration();																		                
  for (int i = 0; i < 400; i++)                                       // make the calibration take about 10 seconds
  {
  	// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  	qtrrc.calibrate();       							
  }
  
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);																	              
  for (int i = 0; i < NUM_SENSORS; i++)                      
  {
  	Serial.print(qtrrc.calibratedMinimumOn[i]);                      // Imprime valores minimos
  	Serial.print(' ');                                               
  }

  Serial.println();                                                  // Imprime end line
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)												      
  {
  	Serial.print(qtrrc.calibratedMaximumOn[i]);                      // Imprime valores maximos
  	Serial.print(' ');																              
  }

  Serial.println();																	                
  Serial.println();																	               
  motorstop();																		                  
}

/*============  BUCLE  ============*/
void loop()
{
  MotorON = 0;                                                  
  if((digitalRead(BOTONDER) == 1)||(digitalRead(BOTONIZQ) == 1))             
  {
    if(digitalRead(BOTONDER) == 1)                                       
    {
      giro = 1;                                                  
      lcd.setCursor(4, 0);                                                  
      lcd.print("D");                                               // Imprime D de derecha en la pantalla 
    }

    else if (digitalRead(BOTONIZQ)==1)                              
    {
      giro = 2;
      lcd.setCursor(4, 0);                                     
      lcd.print("I");                                               // Imprime I de izquierda en la pantalla
    }

    else
    {
      giro = 0;                                                        
    }

    Serial.println(giro);           
    if (giro == 1)
    {   
      while(MotorON == 0)
      {   
        motorgder();                                                 // Funcion que inicia el motor hacia la izquierda
        if(digitalRead(LAPCOUNTER) > 0)                              // Solo entra si el laser se corta
        {
          count=count+1;  
          if(count < 10)
          {
            lcd.setCursor(13,0);
            lcd.print("0");
            lcd.setCursor(14,0);
            lcd.print(count);   
          }

          else
          {
            lcd.setCursor(13, 0);
            lcd.print(count);                         
          }
        }
        /**
        *
        *read calibrated sensor values and obtain a measure of the line position from 0 to 500
        unsigned int position = qtrrc.readLine(sensorValues);
        print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
        1000 means minimum reflectance, followed by the line positionntro
        *
        **/

        unsigned int position = qtrrc.readLine(sensorValues);

        for (unsigned char i = 0; i < NUM_SENSORS; i++)               
        {
          Serial.print(sensorValues[i]);                              
          Serial.print('\t');                                         
          if (sensorValues[i] < 500)
          {
            sensorValues[i] = 0;
          }

          else  
          {
            sensorValues[i] = 1;
          }
        }

        sensorPos = sensorValues[4]*16 + sensorValues[3]*8 + sensorValues[2]*4 + sensorValues[1]*2 + sensorValues[0];
        Serial.print("lala:");
        Serial.print(sensorPos);
        // Entrada de DIP a DEC
        DIPvalue     = digitalRead(51)*16 + digitalRead(49)*8 + digitalRead(47)*4 + digitalRead(45)*2 + digitalRead(43)*1;
        byte DIPbyte = byte(DIPvalue);
        // Condicion que detiene motor si han pasado diez vueltas y todos los sensores leen blanco
        if(count >= 10 && DIPvalue == sensorPos)
        {
          motorstop();                                                
          Serial.println("vueltas max");                             // Imprime que ha finalizado el task
          MotorON = 1;  
        } 

        if(DIPvalue < 10)
        {
          lcd.setCursor(3,1);
          lcd.print("0");
          lcd.setCursor(4,1);
          lcd.print(DIPvalue);
        }
        else
        {
          lcd.setCursor(3, 1);                                        
          lcd.print(DIPvalue);
        }
            
        byte sensorByte = byte(sensorPos);
        if(sensorPos < 10)
        {
          lcd.setCursor(11,1);
          lcd.print("0");
          lcd.setCursor(12,1);
          lcd.print(sensorPos);
        }
        else
        {
          lcd.setCursor(11, 1);                         
          lcd.print(sensorPos);
        }
        Serial.print("guide:");                                         // -----------------------------------  
        Serial.print(digitalRead(sensorValues[1]));                     // Imprime el cambio del sensor laser
        Serial.print("count:");                                         // ----------------------------------- 
        Serial.println(count);                                          // Imprime el numero de vuelta
      }

      count = 0;  
    }

    else if(giro == 2)
    {
      while(MotorON == 0)
      {
        motorgizq();                                                // Funcion que inicia el motor hacia la izquierda
        for (unsigned char i = 0; i < NUM_SENSORS; i++)            
        {
          Serial.print(sensorValues[i]);                            // Imprime los valores de los sensores
          Serial.print('\t');                                      
        }
        if(digitalRead(LAPCOUNTER) > 0)                             // Solo entra si el laser se corta
        {
          count=count+1;                                            // Variable que cuenta las vueltas aumenta
          if(count < 10)
          {
            lcd.setCursor(13,0);
            lcd.print("0");
            lcd.setCursor(14,0);
            lcd.print(count);   
          }

          else
          {
            lcd.setCursor(13, 0);
            lcd.print(count);                         
          }
        }
        // read calibrated sensor values and obtain a measure of the line position from 0 to 500
        // unsigned int position = qtrrc.readLine(sensorValues);
        // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
        // 1000 means minimum reflectance, followed by the line positionntro
        unsigned int position = qtrrc.readLine(sensorValues);
        for (unsigned char i = 0; i < NUM_SENSORS; i++)          
        {
          Serial.print(sensorValues[i]);                            
          Serial.print('\t');                                      
          if (sensorValues[i] < 500)
          {
            sensorValues[i] = 0;
          }

          else  
          {
            sensorValues[i] = 1;
          }   
        }

        sensorPos = sensorValues[4]*16 + sensorValues[3]*8 + sensorValues[2]*4 + sensorValues[1]*2 + sensorValues[0];
        Serial.print("lala:");  
        Serial.print(sensorPos);

        // Entrada de DIP a DEC
        DIPvalue = digitalRead(51)*16 + digitalRead(49)*8 + digitalRead(47)*4 + digitalRead(45)*2 + digitalRead(43)*1;
        byte DIPbyte = byte(DIPvalue);
        if(count >= 10 && DIPvalue == sensorPos)
        {
          motorstop();                                         
          Serial.println("vueltas max");                          
          MotorON = 1;
        }

        if(DIPvalue < 10)
        {
          lcd.setCursor(3,1);
          lcd.print("0");
          lcd.setCursor(4,1);
          lcd.print(DIPvalue);
        }

        else
        {
          lcd.setCursor(3, 1);                          
          lcd.print(DIPvalue);
        }

        byte sensorByte = byte(sensorPos);
        if(sensorPos < 10)
        {
          lcd.setCursor(11,1);
          lcd.print("0");
          lcd.setCursor(12,1);
          lcd.print(sensorPos);
        }

        else
        {
          lcd.setCursor(11, 1);                         
          lcd.print(sensorPos);
        } 
        Serial.print("guide:");                                      // ----------------------------------- 
        Serial.print(digitalRead(sensorPos));                        // Imprime el cambio del sensor laser
        Serial.print("count:");                                      // ----------------------------------- 
        Serial.println(count);                                       // Imprime el numero de vuelta
      }

      count = 0;
    }

    else
    {
      motorstop();
    }
  }

	else
  {
    motorstop();
  }
}

/*===============  FUNCIONES  ===============*/
void motorCalibration()                                             // Encendido de motor para calibracion
{
  digitalWrite(ENABLE_MOTOR, HIGH); 												
  digitalWrite(RIGHT_MOTOR1, LOW);                                  // Giro derecha apagado
  digitalWrite(RIGHT_MOTOR2, HIGH);                                 // Giro izquierda encendido
  analogWrite(RIGHT_MOTORPWM, PWMC);												
}

void motorgizq()                                                    // Encendido de motor a la izquierda
{
  digitalWrite(ENABLE_MOTOR, HIGH); 				
  digitalWrite(RIGHT_MOTOR1, LOW);                                  // Giro derecha apagado
  digitalWrite(RIGHT_MOTOR2, HIGH);                                 // Giro izquierda encendido
  analogWrite(RIGHT_MOTORPWM, PWMW);													
}

void motorgder()                                                    // Encendido de motor a la derecha
{
  digitalWrite(ENABLE_MOTOR, HIGH); 										
  digitalWrite(RIGHT_MOTOR1, HIGH);                                 // Giro derecha encendido
  digitalWrite(RIGHT_MOTOR2, LOW);                                  // Giro izquierda apagado
  analogWrite(RIGHT_MOTORPWM, PWMW);
}

void motorstop()                                                    // Parada del motor
{
  digitalWrite(ENABLE_MOTOR, LOW); 								
  digitalWrite(RIGHT_MOTOR1, LOW);                                  // Giro derecha apagado
  digitalWrite(RIGHT_MOTOR2, LOW);                                  // Giro izquierda apagado
  analogWrite(RIGHT_MOTORPWM, 0);											
}

void motorstop1()
{
  digitalWrite(ENABLE_MOTOR, HIGH); 											
  digitalWrite(RIGHT_MOTOR1, HIGH);                                 // Giro derecha encendido
  digitalWrite(RIGHT_MOTOR2, LOW);                                  // Giro izquierda apagado
  analogWrite(RIGHT_MOTORPWM, PWMW);                                 
  delay(20);
}

void motorstop2()
{
	digitalWrite(ENABLE_MOTOR, HIGH); 													  
  digitalWrite(RIGHT_MOTOR1, LOW);                                  // Giro derecha apagado
  digitalWrite(RIGHT_MOTOR2, HIGH);                                 // Giro izquierda encendido
  analogWrite(RIGHT_MOTORPWM, PWMW); 
  delay(20);
}

void increment()                                                    // Incrementa la cuenta de las vueltas
{
    count = count++;
}

