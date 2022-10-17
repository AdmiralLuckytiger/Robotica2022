//Incluimos librerias 
#include <StringSplitter.h>
#include <Stepper.h>
#include <AutoPID.h>

//Definimos Pines

  //Pines Motor 1
    //PWM PINES
      
    //PINES ENCODERS
    
  //Pines Motor 2
  
    //PWM PINES

  //PINES DE INTERRUPCIÓN

//Definimos estructuras de datos
  //---Struct
    struct Motor{
      double Spos = 0; //Setpoint of position
      double Svel = 0; //Setpoint of velocity
      double Rpos = 0; //Real value of position
      double Rvel = 0; //Real value of velocity
      double pwm = 0; //Input motor
    };

    struct MotorPasoAPaso{
      int stepCount = 0;
      int Spos = 0;
      int Rpos = 1.8*stepCount;
      int stepsPerRevolution = 200;
      double pwm = 0;
    };
  
//Definimos variables globales
  
  //Variables del motor 1
    struct Motor Motor_1;
  
  //Variables del motor 2
    struct MotorPasoAPaso Motor_2;
    Stepper MotorPAP(Motor_2.stepsPerRevolution, 8, 9, 10, 11); 
  //Variables de estado del programa
    bool currentlyWorking = false;
  
  //Constantes de trabajo
    //pid settings and gains
      #define OUTPUT_MIN 0
      #define OUTPUT_MAX 255
      #define KP .12
      #define KI .0003
      #define KD 0
    //Constantes temporales
      #define TEMP_READ_DELAY 800; // Hay un error pendiente de corregir
  //PIDs
    AutoPID PID_1(&Motor_1.Rpos, &Motor_1.Spos, &Motor_1.pwm, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

  //Vaiebles temporables
    unsigned long LastUpdated = millis();
    
//-----------------------Definimos funciones de trabajo------------------------------//

  void recibirMensaje(void){
    if(!currentlyWorking){
      //Definimos output de la funcion
        String mensajes[2];
      //Leemos el buffer
        String mensage = Serial.readStringUntil(';');
      //Procesamos la informacion
        Serial.println(F("StringSplitter Library Test"));
        Serial.println("Test String: " + mensage);
        StringSplitter *splitter = new StringSplitter(mensage, ',', 2);
      
      for(int i = 0; i < splitter->getItemCount(); i++){
        String item = splitter->getItemAtIndex(i);
        Serial.println("Item @ index " + String(i) + ": " + String(item));
      }
    
      //MOTOR_1
        Motor_1.Spos = ((new StringSplitter(splitter->getItemAtIndex(0),':',2))->getItemAtIndex(1)).toInt();
        Serial.println(String(Motor_1.Spos));
      //MOTOR_2
        Motor_2.Spos = ((new StringSplitter(splitter->getItemAtIndex(1),':',2))->getItemAtIndex(1)).toInt();
        Serial.println(String(Motor_2.Spos));
    
      //FINALIZAMOS 
        Serial.println(("End of program."));
        currentlyWorking = true;
    }
  }

  //-----------------------------//
  void setMotor(){
    //Comprobamos si tenemos que trabajar
      //PID_1.run();
      if(currentlyWorking){
        //Operaciones Motor_1
          analogWrite(13,Motor_1.pwm);
        //Operaciones Motor_2
          if(Motor_2.pwm > 0){
            MotorPAP.setSpeed(Motor_2.pwm);
            MotorPAP.step(Motor_2.stepsPerRevolution/100);
          }
        if(PID_1.isStopped())
           currentlyWorking = false;
      }
  }
  //----------------------------//
  void updateMotor(){
    //Comprobamos si tenemos que enviar mensajes
    if(currentlyWorking){
      if( millis() - LastUpdated > 800 ){
        //Updated Encoders Motor_1

        //Updated Encoders Motor_2

        //Actualizamos tiempos
          LastUpdated = millis();
      }
    }
  }

//------------------Programa Principal---------------------------//
void setup() {
  //Configuramos pines

  //Configuramos comunicación
  Serial.begin(9600);
    //Esperamos hasta que se abra la comunicación
    while(!Serial){
      ;
    }
  //Definimos interupciones

  //Configuramos PID

  //Configuramos Motores
    
}

void loop() {
  //Recibo mensaje
  if(Serial.available() > 0){
    recibirMensaje();
  }
  //Operaciones con Motores
  setMotor(); //Posicionamos motores
  updateMotor(); //Actualizamos posicionamiento
}
