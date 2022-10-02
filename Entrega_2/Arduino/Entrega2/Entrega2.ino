//Declaramos librerias 
 
//-----------------------// 
 
//Definir Pines Respuesta 
 
#define PIN_LED 13 
 
//Definir Pines Motor 
#define PWM1 5 
#define PWM2 6 
 
//Definir Pines Encoder 
#define ENC_A 2 
#define ENC_B 3 
 
//Definir Pines Sensores 
#define PIN_FIN_DE_CARRERA 4 
#define ALI_FIN_DE_CARRERA 7 
#define DEAD_ZONE 4
 
//VARIABLE PARA EL ENVÍO 
long int LastEnviado = millis(); 
 
//SALIDA Y MEDIDAS DE POSICIÓN 
int Salida = 0; 
long int Pulsos = 0;    //pulsos 382 pulsos vuelta jamas debe sobre pasar 300 
float Pos_Vu = 0; 
 
//Variables utilizadas en el controlador PID. 
unsigned long lastTime = 0; 
double Input = 0, Setpoint = 0; 
double errSum = 0, lastErr = 0; 
double kp = 1, ki = 0.001, kd = 0.01, convGrados = 0.94241; 
int SampleTime = 10; // Seteamos el tiempo de muestreo en 1 segundo. 
int OutputDef = 0; 
int Output = 0; 
 
 
 
 
//Declaramos funciones de trabajo              ENCODER//////////////////// 
void Sensor1() 
{ 
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) Pulsos++; 
  else Pulsos--; 
} 
void Sensor2() 
{ 
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) Pulsos--; 
  else Pulsos++; 
} 
 
//Declaramos funciones de trabajo            fin  ENCODER//////////////////// 
 
 
void Compute() 
{ 
   
    unsigned long now = millis(); 
    int timeChange = (now - lastTime); 
    // Determina si hay que ejecutar el PID o retornar de la función. 
    if(timeChange>=SampleTime) 
    { 
      // Calcula todas las variables de error. 
      double error = Setpoint - Pulsos; 
      errSum += error; 
      double dErr = (error - lastErr); 
      // Calculamos la función de salida del PID. 
      Output = kp * error + ki * errSum + kd * dErr; 
      // Guardamos el valor de algunas variables para el próximo ciclo de cálculo. 
      lastErr = error; 
      lastTime = now; 
    } 
    //Establecemos límites 
  if (Output >= 255) Output = 255; 
  if (Output >DEAD_ZONE && Output<155) Output = 155; 
  if (Output <-DEAD_ZONE && Output>-155) Output = -155; 
  if (Output < -255) Output = -255; 
  if (Output <=DEAD_ZONE && Output>=-DEAD_ZONE) Output = 0;
 
  //AJUSTAMOS LA POLARIDAD DEL MOTOR 
  if (Output > 0) 
  { 
    OutputDef = round(Output); 
    analogWrite(PWM1, OutputDef); 
    analogWrite(PWM2, 0); 
  } 
  else 
  { 
    OutputDef = (-1) * round(Output); 
    analogWrite(PWM1, 0); 
    analogWrite(PWM2, OutputDef); 
  } 
 
} 
 
 
//////////////////////////////////////////////////////////// 
 
 
//// 
void SetTunings(double Kp, double Ki, double Kd) 
{ 
  double SampleTimeInSec = ((double)SampleTime)/1000; 
  kp = Kp; 
  ki = Ki * SampleTimeInSec; 
  kd = Kd / SampleTimeInSec; 
} 
 
void enviarPos() 
{ 
  //ENVIAMOS POSICIÓN REAL CADA X ms 
 
  //ACTUALIZAR ESTO 
  if (millis() - LastEnviado > 200) 
  { 
    Serial.print("Real: "); 
    Serial.print(Pulsos); 
    Serial.print("  Deseada: "); 
    Serial.print(Setpoint); 
    Serial.print("    "); 
 
    LastEnviado = millis(); 
  } 
} 
void parada(){ 
  analogWrite(PWM1,0); 
  analogWrite(PWM2,0);  
} 
void flash() 
{ 
  for(int i = 0; i < 5; i++) 
  { 
    digitalWrite(PIN_LED,HIGH); 
    delay(500); 
    digitalWrite(PIN_LED,LOW); 
    delay(500); 
  } 
} 
 
void enviarPosicion(int pos){ 
  Serial.println(String(pos)); 
} 
 
void ControlPosicion(){ 
  String Random = Serial.readStringUntil('\n'); 
  Setpoint = Random.toInt();
  
  //Codigo PID CARLOS Y DAVID
  for(int i=1;i<4;i++){
  while(Setpoint-Pulsos>DEAD_ZONE || Setpoint-Pulsos<-DEAD_ZONE){ 
  Compute();
  Serial.println(String(Pulsos)); 
  }
  analogWrite(PWM1, 0); 
  analogWrite(PWM2, 0);
  }
  //Serial.println("POSITIONING DONE");

        //CODIGO MANDAR POSICION 
        //enviarPos(); 
} 
 
void Saludo(){ 
  Serial.println("S"); 
  flash(); 
} 
 
void GoToCero(){ 
  while(digitalRead(PIN_FIN_DE_CARRERA)!=1){ 
    analogWrite(PWM2,150); 
    analogWrite(PWM1,0); 
    } 
    analogWrite(PWM2,150); 
    delay(200); 
    analogWrite(PWM2,0); 
    Pulsos=0; 
  } 
 
 
 
//---------------------// 
void setup() { 
  //Establecemos modo de los pines 
    //PIN EXTRA ALIMANTACIÓN 
    pinMode(ALI_FIN_DE_CARRERA,OUTPUT); 
    digitalWrite(ALI_FIN_DE_CARRERA,HIGH); 
    //Pines Motor 
    pinMode(PWM1, OUTPUT);   
    pinMode(PWM2, OUTPUT);   
    //Pines Sensores 
    pinMode(PIN_FIN_DE_CARRERA,INPUT); 
    //Pines Encoder 
    pinMode(ENC_A,INPUT_PULLUP); 
    pinMode(ENC_B,INPUT_PULLUP); 
    //Pines Respuesta 
    pinMode(PIN_LED,OUTPUT); 
    attachInterrupt( (digitalPinToInterrupt(ENC_A)),  Sensor1, RISING); 
    attachInterrupt( (digitalPinToInterrupt(ENC_B)),  Sensor2, RISING); 
    //attachInterrupt( (digitalPinToInterrupt(PIN_FIN_DE_CARRERA)), parada, RISING);
  //Posicionamos motor
    GoToCero();  
  //Establecemos comunicación serie 
    Serial.begin(9600);   
} 
////////////////////////////////////////////////// inicia 
 
 
void loop() { 
digitalWrite(ALI_FIN_DE_CARRERA,HIGH); 
   
  if(Serial.available() > 0){ 
    String codigo = Serial.readStringUntil('\n'); 
    switch(codigo[0]){ 
      case 'S': 
        digitalWrite(PIN_LED,HIGH);//Modo activo visual 
        Saludo(); 
        flash(); //Usamos el flash para visualizar si hemos entrado en un modo especifico 
        break; 
      case 'P':  
        digitalWrite(PIN_LED,HIGH);//Modo activo visual 
        ControlPosicion(); 
        Serial.println("ENDP"); 
        break; 
      default: 
        digitalWrite(PIN_LED,LOW);//Modo inactivo visual 
    } 
  }  
}
