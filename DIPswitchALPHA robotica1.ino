// DipSwitch Robotica#1.ino
/*********************************************************************************************************************************************
Gregory Pozo 	  1045331 
Angel Berroa 	  1040705
Hector Moreta   1045643
Programa que utiliza disco giratorio y sensores para determinar velocidad y posicion de un motor (ENCODER)
*********************************************************************************************************************************************/

/**********************************************************Declaracion de variables**********************************************************/
#include <QTRSensors.h>																                // Libreria de sensores de pololu de reflectancia
#include <LiquidCrystal.h>														                // Libreria de pantalla LCD
/*------------------------------------------------------------------------------------------------------------------------------------------*/
#define bder 53																		   	                // Pin de boton giro derecho
#define bizq 52																			                  // Pin de boton giro izquierdo
#define motorPower 2 																	                // Stand by (enable de driver)
#define rightMotor1 3																	                // Motor de transmision derecho (pin de driver)
#define rightMotor2 4																	                // Motor de transmision derecho (pin de driver)
#define rightMotorPWM 5
#define PWMN 120 																		                  // Valor de PWM de trabajo
#define PWMNN 100 																	                 	// Valor de PWM de calibracion 
#define NUM_SENSORS 5   															                // number of sensors used
#define TIMEOUT  2500  																                // waits 2500 microseconds for sensor outputs to go low
#define vaina 	18 																		                // Pin de sensor de cuenta de vueltas
#define switch1 51 																		                // Entrada DIP 1
#define switch2 49																		                // Entrada DIP 2
#define switch3 47																		                // Entrada DIP 3
#define switch4 45																		                // Entrada DIP 4
#define switch5 43																		                // Entrada DIP 5
/*------------------------------------------------------------------------------------------------------------------------------------------*/
//Funcion para arreglo de sensores pololu digitales
QTRSensorsRC qtrrc((unsigned char[]) {13, 11, 10, 9, 8},NUM_SENSORS, TIMEOUT);			// Arreglo de 5 sensores, cantidad y tiempo de off
LiquidCrystal lcd(41, 39, 37, 35, 33, 31);												    // Pines de inicio de LCD 			
unsigned int sensorValues[NUM_SENSORS];													      // Variable positiva que lee los valores de cada S.
int count = 0;																			                  // Variable que cuenta
int giro = 0;																			                    // Variable de control de giro
int DIPvalue = 0;																		                  // Variable del DIP switch
int sensorPos = 0;																		                // Variable de posicion del disco en base a sensors
int force = 0;                                                        // Variable para determinar el tiempo que dura encendido el motor

/*********************************************************************MAIN*******************************************************************/
void setup()
{
	  lcd.begin(16, 2);																	                // set up the LCD's number of columns and rows
   	lcd.print("Dir:  #Vuelt:");	                                      // Imprime direccion y vuelta al inicio
   	lcd.setCursor(0,1);
   	lcd.print("DS: ");	                                              // Imprime Dip switch al inicio 
   	lcd.setCursor(6,1);											                		      // Print a message to the LCD.
   	lcd.print("PSen: ");                                              // Posicion de sensor al inicio
	   //attachInterrupt(5, increment, RISING);											      // Interrupcion en el micro #5, para lectura de break B
  	pinMode(rightMotor1, OUTPUT);														          // Salida 1 de motor
  	pinMode(rightMotor2, OUTPUT);														          // Salida 2 de motor
  	pinMode(rightMotorPWM, OUTPUT);														        // Salida de PWM
  	pinMode(motorPower, OUTPUT);														          // Salida a driver
  	pinMode(bder, INPUT);																              // Entrada boton derecho
  	pinMode(bizq, INPUT);																              // Entrada boton izquierdo
  	pinMode(vaina, INPUT);																            // Entrada sensor cuenta break beam
  	pinMode(switch1, INPUT);															            // -------------------------------
  	pinMode(switch2, INPUT);															            //		
  	pinMode(switch3, INPUT);															            // Se declaran como entradas
  	pinMode(switch4, INPUT);															            //
  	pinMode(switch5, INPUT);															            // -------------------------------
  	//digitalWrite(3, HIGH);    											 
  	motorcali();																		                  // Funcion de calibracion de motor
  	for (int i = 0; i < 400; i++)  														        // make the calibration take about 10 seconds
  	{
  		// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    	qtrrc.calibrate();       							
  	}
  
  	// print the calibration minimum values measured when emitters were on
  	Serial.begin(9600);																	              // Inicia puerto serial a 9600 baudios
  	for (int i = 0; i < NUM_SENSORS; i++)												      // Lee todos los sensores del array
  	{
    	Serial.print(qtrrc.calibratedMinimumOn[i]);										  // Imprime valores minimos
    	Serial.print(' ');																              // Imprime espacio
  	}

  	Serial.println();																	                // Imprime end line
  	// print the calibration maximum values measured when emitters were on
  	for (int i = 0; i < NUM_SENSORS; i++)												      // Lee todos los sensores del array
  	{
    	Serial.print(qtrrc.calibratedMaximumOn[i]);										  // Imprime valores maximos
    	Serial.print(' ');																              // Imprime espacio
  	}

  	Serial.println();																	                // Imprime end line
  	Serial.println();																	                // Imprime end line
  	motorstop();																		                  // Detiene el motor
}

/*******************************************************************BUCLE********************************************************************/
void loop()
{
	force = 0;                                                          // Se inicia en 0 cada vez que inicia el bucle
	if((digitalRead(bder) == 1)||(digitalRead(bizq) == 1))              // Se cumple si cualquier boton es presionado
	{
		if(digitalRead(bder) == 1)                                        // Si es el boton de la derecha
		{
      giro = 1;                                                  
      lcd.setCursor(4, 0);													                  // -------------------------------------------      
      lcd.print("D");                                                 // Imprime D de derecha en la pantalla 
    }

    else if (digitalRead(bizq)==1)                                    // Si es el boton de la izquierda
    {
      giro = 2;
      lcd.setCursor(4, 0);													                  // -------------------------------------------
      lcd.print("I");                                                 // Imprime I de izquierda en la pantalla
    }

    else
    {
      giro= 0;                                                        // Sino giro es 0
    }

    Serial.println(giro);           
    if (giro == 1)
    {   
      while(force == 0)
      {   
      	motorgder();															                    // Funcion que inicia el motor hacia la izquierda
      	if(digitalRead(vaina) > 0)												            // Solo entra si el laser se corta
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
       // read calibrated sensor values and obtain a measure of the line position from 0 to 500
    	// unsigned int position = qtrrc.readLine(sensorValues);
    	// print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
    	// 1000 means minimum reflectance, followed by the line positionntro
    	unsigned int position = qtrrc.readLine(sensorValues);

			for (unsigned char i = 0; i < NUM_SENSORS; i++)							   // Lee todos los sensores del array
   		{
       	Serial.print(sensorValues[i]);										           // Imprime los valores de los sensores
			  Serial.print('\t');													                 // Imprime un tab		
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
    	// Condicion que detiene motor si han pasado diez vueltas y todos los sensores leen blanco
	    if(count >= 10 && DIPvalue == sensorPos)
	    {
	      motorstop();														                    // Funcion de parada del motor
	      Serial.println("vueltas max"); 										          // Imprime que ha finalizado el task
	      force = 1;	
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
        lcd.setCursor(3, 1);											                  // Variable que cuenta las vueltas aumenta
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
			Serial.print("guide:");													              // -----------------------------------	
			Serial.print(digitalRead(sensorValues[1]));										// Imprime el cambio del sensor laser
			Serial.print("count:");													              // ----------------------------------- 
			Serial.println(count); 													              // Imprime el numero de vuelta

	    }

	    //motorstop2();
	    //motorstop();
	    count = 0;	
    }

    else if(giro == 2)
    {
      while(force == 0)
      {
        motorgizq();															                  // Funcion que inicia el motor hacia la izquierda
   			for (unsigned char i = 0; i < NUM_SENSORS; i++)							// Lee todos los sensores del array
			  {
			    Serial.print(sensorValues[i]);										        // Imprime los valores de los sensores
			    Serial.print('\t');													              // Imprime un tab		
			  }
	      if(digitalRead(vaina) > 0)												          // Solo entra si el laser se corta
	    	{
	        count=count+1;														                // Variable que cuenta las vueltas aumenta
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
			for (unsigned char i = 0; i < NUM_SENSORS; i++)							 // Lee todos los sensores del array
			{
			  Serial.print(sensorValues[i]);										         // Imprime los valores de los sensores
			  Serial.print('\t');													               // Imprime un tab
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
			  motorstop();														                 // Funcion de parada del motor
			  Serial.println("vueltas max"); 										       // Imprime que ha finalizado el task
			  force = 1;
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
			Serial.print("guide:");													         // -----------------------------------	
			Serial.print(digitalRead(sensorPos));										 // Imprime el cambio del sensor laser
			Serial.print("count:");													         // ----------------------------------- 
			Serial.println(count); 													         // Imprime el numero de vuelta
		  }

		  //motorstop1();
		  //motorstop();
			count = 0;
		}

		else
    {
      motorstop();
    }
  }
}

/***************************************************************FUNCIONES********************************************************************/
void motorcali()																		          // Encendido de motor para calibracion
{
	  digitalWrite(motorPower, HIGH); 													// Enable activo
  	digitalWrite(rightMotor1, LOW);														// Giro derecha apagado
  	digitalWrite(rightMotor2, HIGH);													// Giro izquierda encendido
  	analogWrite(rightMotorPWM, PWMNN);												// Ancho de pulso de motor para calibracion 
}

void motorgizq()																		          // Encendido de motor a la izquierda
{
  	digitalWrite(motorPower, HIGH); 													// Enable activo
  	digitalWrite(rightMotor1, LOW);														// Giro derecha apagado
  	digitalWrite(rightMotor2, HIGH);													// Giro izquierda encendido
  	analogWrite(rightMotorPWM, PWMN);													// Ancho de pulso de motor
}

void motorgder()																		          // Encendido de motor a la derecha
{
  	digitalWrite(motorPower, HIGH); 													// Enable activo
  	digitalWrite(rightMotor1, HIGH);													// Giro derecha encendido
  	digitalWrite(rightMotor2, LOW);														// Giro izquierda apagado
  	analogWrite(rightMotorPWM, PWMN);													// Ancho de pulso de motor
}

void motorstop()																		          // Parada del motor
{
  	digitalWrite(motorPower, LOW); 														// Enable desactivado
  	digitalWrite(rightMotor1, LOW);														// Giro derecha apagado
  	digitalWrite(rightMotor2, LOW);														// Giro izquierda apagado
  	analogWrite(rightMotorPWM, 0);													  // Ancho de pulso de motor
}

void motorstop1()
{
	  digitalWrite(motorPower, HIGH); 													// Enable activado
  	digitalWrite(rightMotor1, HIGH);													// Giro derecha encendido
  	digitalWrite(rightMotor2, LOW);														// Giro izquierda apagado
  	analogWrite(rightMotorPWM, PWMN);													// Ancho de pulso de motor
  	delay(20);
}

void motorstop2()
{
	digitalWrite(motorPower, HIGH); 													  // Enable activado
  digitalWrite(rightMotor1, LOW);														// Giro derecha apagado
  digitalWrite(rightMotor2, HIGH);													// Giro izquierda encendido
  analogWrite(rightMotorPWM, PWMN);													// Ancho de pulso de motor
  delay(20);
}

void increment() 																		          // Incrementa la cuenta de las vueltas
{
    count = count++;
}