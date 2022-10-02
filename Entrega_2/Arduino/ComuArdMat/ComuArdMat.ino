//Declaramos librerias

//Variables globales
String pos;
//-----------------------//
//Definir Pines Respuesta
#define PIN_LED 13
//Definir Pines Motor

//Definir Pines Encoder

//Definir Pines Sensores
#define PIN_FIN_DE_CARRERA 5
//----------------------//
//Declaramos funciones de trabajo
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

int recibirParametro(){
  while(Serial.available() == 0){}; //Esperamos a recibir parametro
  return Serial.readString().toInt(); //Leemos y adaptamos el valor a int
}

void ControlVelocidad(){
  int rpm = recibirParametro();

  //Codigo PID CARLOS
    //CODIGO CONTROL MOTOR
  
}

void ControlPosicion(){
  int pos = recibirParametro();

  //Codigo PID CARLOS
    //CODIGO CONTROL POSICION
    //CODIGO MANDAR POSICION
  
}

void Saludo(){
  Serial.println("S");
  flash();
}

//---------------------//
void setup() {
  //Establecemos modo de los pines
    //Pines Motor
    //Pines Sensores
    pinMode(PIN_FIN_DE_CARRERA,INPUT_PULLUP);
    //Pines Encoder
    //Pines Respuesta
    pinMode(PIN_LED,OUTPUT);
  //Establecemos comunicación serie
    Serial.begin(9600); //9600 baudios es una frecuencia adecuada para los interlocutores planteados.
}

void loop() {
  if(Serial.available() > 0){
    String codigo = Serial.readString();
    switch(codigo[0]){
      case 'S':
        digitalWrite(PIN_LED,HIGH);//Modo activo visual
        Saludo();
        flash(); //Usamos el flash para visualizar si hemos entrado en un modo especifico
        
        digitalWrite(PIN_LED,LOW);
        break;
      case 'P':
        digitalWrite(PIN_LED,HIGH);//Modo activo visual
        delay(5000);
        pos = Serial.readString();
        Serial.println(pos);
       
        digitalWrite(PIN_LED,LOW);
        break;
      case 'V':
        digitalWrite(PIN_LED,HIGH);//Modo activo visual
        for(int i = 0; i < 5; i++)
          Serial.println(String(i));
          
        digitalWrite(PIN_LED,LOW);
        break;
      default:
        digitalWrite(PIN_LED,LOW);//Modo inactivo visual
    }
  }

}
