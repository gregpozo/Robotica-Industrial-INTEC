// Practica 2 robotica.ino
/*********************************************************************************************************************************************
Gregory Pozo 	  1045331 
Angel Berroa 	  1040705
Hector Moreta     1045643
Programa que utiliza practica 1 de encoder y un potenciometro para controlar la posicion
*********************************************************************************************************************************************/

/**********************************************************Declaracion de variables**********************************************************/
#include <QTRSensors.h>											// Libreria de sensores de pololu de reflectancia
                     
#define motorPower    2                     								// Stand by (enable de driver)					                      
#define rightMotor1   3										        // Motor de transmision derecho (pin de driver)                                   
#define rightMotor2   4											// Motor de transmision derecho (pin de driver)    
#define rightMotorPWM 5                        
#define PWMN          40	                   							// Valor de PWM de trabajo						                               
#define PWMNN         150                    								// Valor de PWM de calibracion					                            	
#define NUM_SENSORS   5                                        						// number of sensors used							                 
#define TIMEOUT       2500               							        // waits 2500 uS for sensor outputs to go low			               
#define	POT 		      0										// Pin 0 analogo de arduino
/*------------------------------------------------------------------------------------------------------------------------------------------*/

//Funcion para arreglo de sensores pololu digitales
QTRSensorsRC qtrrc((unsigned char[]) {13, 11, 10, 9, 8},NUM_SENSORS, TIMEOUT);				// Arreglo de 5 sensores, cantidad y tiempo de off			
unsigned int sensorValues[NUM_SENSORS];									// Variable positiva que lee los valores de cada S.
int Value     = 0;											// Variable que toma el valor del POT
int sensorPos = 0;                                                          				// Variable de posicion del disco en base a sensors
int force     = 0;

void setup() 
{
	pinMode(rightMotor1, OUTPUT);									// Salida 1 de motor
  	pinMode(rightMotor2, OUTPUT);								        // Salida 2 de motor
  	pinMode(rightMotorPWM, OUTPUT);									// Salida de PWM
  	pinMode(motorPower, OUTPUT);									// Salida a driver
  	pinMode(POT, INPUT);								                // Entrada del potenciometro
  	motorcali();										        // Funcion de calibracion de motor
  	for (int i = 0; i < 400; i++)  								        // make the calibration take about 10 seconds
  	{
  	// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    	qtrrc.calibrate();       							
  	}
  
  	// print the calibration minimum values measured when emitters were on
  	Serial.begin(9600);					
  	for (int i = 0; i < NUM_SENSORS; i++)									  		            
  	{
    	Serial.print(qtrrc.calibratedMinimumOn[i]);										       
    	Serial.print(' ');																                   
  	}

  	Serial.println();																	      
  	// print the calibration maximum values measured when emitters were on
  	for (int i = 0; i < NUM_SENSORS; i++)												            
  	{
    	Serial.print(qtrrc.calibratedMaximumOn[i]);										  	    
    	Serial.print(' ');																                  
  	}

  	Serial.println();																	                    
  	Serial.println();																	                      
  	motorstop();																		                        
}

void loop() 
{
  Value       = analogRead(POT);                                            // Lee valor analogo de POT
  int mapping = map(Value, 0, 1023, 0, 31);                                 // Mapeo de POT a ENCODER
  // read calibrated sensor values and obtain a measure of the line position from 0 to 500
  // unsigned int position = qtrrc.readLine(sensorValues);
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line positionntro
  unsigned int position = qtrrc.readLine(sensorValues);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)                           // Lee todos los sensores del array
  {
      //Serial.print(sensorValues[i]);                                      // Imprime los valores de los sensores
      //Serial.print('\t');                                                 // Imprime un tab   
      if (sensorValues[i] < 500)
      {
        sensorValues[i] = 0;
      }

      else  
      {
        sensorValues[i] = 1;
      }
  } 

  // Vuelve el valor de sensores en decimal
  sensorPos = sensorValues[4]*16 + sensorValues[3]*8 + sensorValues[2]*4 + sensorValues[1]*2 + sensorValues[0];
  while(mapping != sensorPos)
  {
    force = 0;
    motorgder();
    Value       = analogRead(POT);                                          // Lee valor analogo de POT
    int mapping = map(Value, 0, 1023, 0, 31);                               // Mapeo de POT a ENCODER
    // read calibrated sensor values and obtain a measure of the line position from 0 to 500
    // unsigned int position = qtrrc.readLine(sensorValues);
    // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
    // 1000 means minimum reflectance, followed by the line positionntro
    unsigned int position = qtrrc.readLine(sensorValues);
    for (unsigned char i = 0; i < NUM_SENSORS; i++)                         // Lee todos los sensores del array
    {
        //Serial.print(sensorValues[i]);                                    // Imprime los valores de los sensores
        //Serial.print('\t');                                               // Imprime un tab   
        if (sensorValues[i] < 500)
        {
          sensorValues[i] = 0;
        }

        else  
        {
          sensorValues[i] = 1;
        }
    } 

    // Vuelve el valor de sensores en decimal
    sensorPos = sensorValues[4]*16 + sensorValues[3]*8 + sensorValues[2]*4 + sensorValues[1]*2 + sensorValues[0];
    Serial.print(mapping);
    Serial.println(sensorPos);
    if(mapping == sensorPos)
    {
        motorgizq();
        delay(50);
        motorstop();
        delay(1000);
    }
  }
  
  motorstop();                                                             
}

/***************************************************************FUNCIONES********************************************************************/
void motorcali()																		                       
{
  digitalWrite(motorPower, HIGH); 														            
  digitalWrite(rightMotor1, LOW);															            
  digitalWrite(rightMotor2, HIGH);														            
  analogWrite(rightMotorPWM, PWMNN);														          
}

void motorgizq()																		                        
{
  digitalWrite(motorPower, HIGH); 														            
  digitalWrite(rightMotor1, LOW);															            
  digitalWrite(rightMotor2, HIGH);														            
  analogWrite(rightMotorPWM, PWMN);														            
}

void motorgder()																		                        
{
  digitalWrite(motorPower, HIGH); 														           
  digitalWrite(rightMotor1, HIGH);														            
  digitalWrite(rightMotor2, LOW);															            
  analogWrite(rightMotorPWM, PWMN);														           
}

void motorstop()																		                        
{
  digitalWrite(motorPower, LOW); 															            
  digitalWrite(rightMotor1, LOW);															            
  digitalWrite(rightMotor2, LOW);															            
  analogWrite(rightMotorPWM, 0);													  	            
}
