#include <util/atomic.h>     //Codigo usado en el concurso InnovaTECNM 2024 en Colima para el concurso Maze Robot
//                             Diego Favela Flores      05/11/2024


// Pins
#define ENCA 2
#define ENCB 3
#define PWM 9
#define IN1 8
#define IN2 10

#define aENCA 18
#define aENCB 19
#define aPWM 13
#define aIN1 12
#define aIN2 11

//globals
long prevT = 0;
int posPrev = 0;
//globals2
long aprevT = 0;
int aposPrev = 0;

//direccion 
int dir = 1;
int adir = 1;


volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

double lastError=0;
double alastError=0;
double rateError = 0;
double arateError = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

volatile int apos_i = 0;
volatile float avelocity_i = 0;
volatile long aprevT_i = 0;


float av2Filt = 0;
float av2Prev = 0;

float aeintegral = 0;

                  //timers vuelta derecha
volatile long tiempo = 0; 
volatile long pulsostotales1 = 0;
volatile long pulsostotales2 = 0;

int pulsosahora;

                  //////sensor de presencia
   const int sensorPinI = 7; // Pin donde está conectado el sensor
   const int sensorPinD = 6; // Pin donde está conectado el sensor
   int sensorState = 0; // Variable para guardar el estado del sensor

//validar rutina 1
volatile int webOS = 2;
volatile long tiempoInicioSensor = 0;
volatile long tiempoquehapasado;


//////////////////////////////sensores de linea 
int lineaIzq = A7;
int lineaDer = A8;
int medicionIzq;
int medicionDer;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

 pinMode(aENCA,INPUT);
  pinMode(aENCB,INPUT);
  pinMode(aPWM,OUTPUT);
  pinMode(aIN1,OUTPUT);
  pinMode(aIN2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
   attachInterrupt(digitalPinToInterrupt(aENCB),areadEncoder,RISING);
   digitalWrite(PWM,LOW);
   digitalWrite(aPWM,LOW);
    pinMode(lineaIzq, INPUT);
    pinMode(lineaDer, INPUT);

   //sensor
    pinMode(sensorPinI, INPUT); // Configurar el pin D7 como entrada
   pinMode(sensorPinD, INPUT); // Configurar el pin D7 como entrada
  
}




void loop() {
     tiempo = millis();
     sensorState = digitalRead(sensorPinI); // Leer el estado del sensor de presencia
     medicionIzq = digitalRead(lineaIzq);
     medicionDer = digitalRead(lineaDer);
    
    if(webOS == 2){//variable para validar cambiar direccion (dir = -1) y contar pulsos 
       if (sensorState == LOW){ 
          vueltaDerecha();            ///variable para validar inicio de cambio de rutina y vuelta a la normalidad
       }
    }
     if(webOS == 3){ //validar que ya se haya dado un cambio de vuelta
              volverDerecho();
              }
  //read the position in an atomic block
  //to avoid possible misreads
  int pos = 0;
  float velocity2 = 0;
  int apos = 0;
  float avelocity2 = 0;
                           ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
                           pos = pos_i;
                              velocity2 = velocity_i;
                              apos = apos_i;
                              avelocity2 = avelocity_i; 
                              }
                      

  //compute the speed using method 1
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
 prevT = currT;
///---
  long acurrT = micros();
  float adeltaT = ((float)(acurrT - aprevT))/1.0e6;
  float avelocity1 = (apos - aposPrev)/adeltaT;
  aposPrev = apos;
 aprevT = acurrT;
  //convert counts to rpm
 
  float v2 = velocity2/600.0*60.0;

  float av2 = avelocity2/600.0*60.0;
  //low-pass filter
 
  v2Filt = 0.854*v2Filt + 0.0728*v2 +  0.0728*v2Prev;
  v2Prev = v2;

 av2Filt = 0.854*av2Filt + 0.0728*av2 +  0.0728*av2Prev;
  av2Prev = av2;
  
  
  
  //set target
  float vt =40;
  float avt = 40.7;
  //compute u             first motor
  float kp = 0.8;
  float ki = 3.1;
  float kd = 0.2;
  float e = vt - v2Filt;   
  rateError = e - lastError;
  eintegral = eintegral + e*deltaT ;
  float u = kp*e + ki*eintegral + kd*rateError;       // if(u<0){u=0;}
//                      second motor
  float akp =0.8;
  float aki = 3.1;
  float akd = 0.2;
  float ae = avt - av2Filt;    
  arateError = ae -alastError;
  aeintegral = aeintegral + ae*adeltaT ;
  float au = akp*ae + aki*aeintegral + kd*arateError ;   
  
  lastError = e;
  alastError = ae;
//set motor speed

    int pwr = (int) fabs(u);   //motor 1
         if (pwr >180){
                 pwr=180;
              }
             

    int apwr = (int) fabs(au);   //motor 2
         if (apwr >180){
                 apwr=180;
              }
              
  //reinicio de variables de pid
 //correccion en linea negra
 if(medicionIzq == HIGH){apwr = 0; u = 0; rateError = 0; e = 0; v2Filt =0; lastError = 0; eintegral = 0; }     //correccion al detectar linea           
 if(medicionDer == HIGH){pwr = 0; au = 0; arateError = 0; ae = 0; av2Filt = 0; alastError = 0; aeintegral = 0;}  
  
  if(medicionIzq == HIGH && medicionDer == HIGH){ 
       dir = -1;             //cambiar ambas direcciones
    adir = -1;
    apwr = 70;              //velocidad de motores
    pwr=70;
    setMotor(dir,pwr,PWM,IN1,IN2);
    asetMotor(adir,apwr,aPWM,aIN1,aIN2);
  
     webOS = 4;            //variable para que no detecte sensores ni haga cambio de direccion 
    delay(500); //tiempo para que el robot pueda salir de la linea negra
    u = 0; rateError = 0;   au = 0; arateError = 0; e = 0; ae = 0; //reiniciar variables
    rateError = 0; e = 0; v2Filt =0; lastError = 0; eintegral = 0;
    au = 0; arateError = 0; ae = 0; av2Filt = 0; alastError = 0; aeintegral = 0;
  }

setMotor(dir,pwr,PWM,IN1,IN2);         //motor 1 PID
asetMotor(adir,apwr,aPWM,aIN1,aIN2);   //motor 2 PID



  
    Serial.print(avt);  //setpoint 
    Serial.print(" ");
    Serial.print(av2Filt); //velicidad
    Serial.println();
    delay(1);
}


 void vueltaDerecha(){
          dir = -1;   //cambio direccion motor b
          pulsosahora = pulsostotales2;  //medir pulsos al momento de detectar sensor
          webOS = 3;   //variable para validar

 }
 
 void volverDerecho(){
ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
                int pulsosdevuelta = pulsostotales2 - pulsosahora ; //calcular cuantos pulsos han pasado desde que se detecto presencia
                 
                  if(pulsosdevuelta >= 700 && pulsosdevuelta <= 750){
                    dir = 1;    //vuelta a direccion normal motor b
                    }
                  }

 }
 void marchaAtras(){
  
 }







void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}
void asetMotor(int adir, int apwmVal, int apwm, int ain1, int ain2){
  analogWrite(apwm,apwmVal); // Motor speed
  if(adir == 1){ 
    // Turn one way
    digitalWrite(ain1,HIGH);
    digitalWrite(ain2,LOW);
  }
  else if(adir == -1){
    // Turn the other way
    digitalWrite(ain1,LOW);
    digitalWrite(ain2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(ain1,LOW);
    digitalWrite(ain2,LOW);    
  }
}




void readEncoder(){      //leer encoder 
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0)
  {
    increment = 1;
    }
    else{
      increment = 1;
      }
      pos_i = pos_i + increment;

  /////vueltaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  pulsostotales1 = pos_i;
      //compute velocity with method 2
      long currT = micros();
      float deltaT = ((float) (currT - prevT_i))/1.0e6;
      velocity_i = increment /deltaT;
   
      prevT_i = currT;
  }

void areadEncoder(){
  int ab = digitalRead(aENCB);
  int aincrement = 0;
  if(ab>0)
  {
    aincrement = 1;
    }
    else{
      aincrement = 1;
      }
      apos_i = apos_i + aincrement;
                                                        //DAR VUELTAS
                     pulsostotales2 = apos_i;
      //compute velocity with method 2
      long acurrT = micros();
      float adeltaT = ((float) (acurrT - aprevT_i))/1.0e6;
      avelocity_i = aincrement /adeltaT;
      aprevT_i = acurrT;
  }  
