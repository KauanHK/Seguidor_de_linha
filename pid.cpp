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
#define TRESHOLD 700                       // Valor de referencia para cor da linha branca
#define SPEED0 255                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 0 0) 
#define SPEED1 240                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 1 0) 

#define SPEED2 220                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 0 0) 
#define SPEED3 210                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 0)  
#define SPEED4 150                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 1) 

#define SPEED5 130                            // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 0) 
#define SPEED6 0                            // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 1) 
#define SPEED7 230                          // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 0 1) 

#define RUNTIME 40000                      // Valor para executar o percurso 

int sensors[6];



 //--- Definições das Variáveis utilizadas ---//
  int V=200;  // valor inicial de V.
  int ctn =120; // valor constante dos motores.
  int setPoint = 200; // valor inicial para obtenção do erro.
  int x;  // variavel do motor a
  int y;  // variavel do motor b
  //float kp = 1.0 , ki = 0.0005, kd = 5;  // constantes referentes ao controlador PID, sendo kp,ki e kd para P I e D respectivamente.
  float kp=2,ki=0.0015,kd=110;          // pista U 
  float kp = 2.5 , ki = 0.005, kd = 200 // pista H
  float kp=2,ki=0.0015,kd=110           // pista W
  int erro;
  int Va;  
  int t = 0, ta = 0, dt = 0;
  float P,I,D;

  
//--- inicialização das entradas utilizadas ---//

void setup() {

  


 pinMode(L, INPUT);
 pinMode(C, INPUT);
 pinMode(R, INPUT);
 pinMode(ENA, OUTPUT);
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(ENB, OUTPUT);

 digitalWrite(ENA, LOW);
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, LOW);
 digitalWrite(ENB, LOW);


 
 

 //Serial.begin(9600);
 //Serial.println("Setup feito.");

  
}
   


void loop() {
  ta = t;
  t = millis();
  dt = t - ta;
  int l = digitalRead(L); 
  int c = digitalRead(C);
  int r = digitalRead(R);
  
  Serial.println(l);
  Serial.println(c);
  Serial.println(r);
 

  if(l == 1){
    l = 0; 
  } else {          //
    l = 1;
  } 

  if(c == 1){
    c = 0;
  } else{           // inverção dos valores emitidos pelos sensores de linha para utilização no programa.
    c = 1;
  }

  if(r == 1){
    r = 0;
  } else {          //
    r = 1;
  } 

  Va = V;

  if((l+r+c)!=0){
    
   V = (300*l + 200*c + 100*r)/(l+c+r);  // calculo do valor de V
  }
    

  erro = setPoint - V;

  P = erro*kp; 
  I+= erro*dt*ki;
  D = (kd*(V - Va))/dt;

  x = ctn + (P + I + D); // adicionando valores de P I e D encontrados com a constante
  y = ctn - (P + I + D); // subtraindo  valores de P I e D encontrados com a constante 


  if(x > 255){
    x = 255;
  }
  if(y > 255){
    y = 255;
  }
  
  if(x < -180){
    x = -180;
  }
  if(y < -180){
    y = -180;
  }
  if (x>0 && y>0)
  frente(x,y);
  else if((x<0) && (y>0)){
    esquerda(-x,y);
  }else  if((x>0) && (y<0)){
    direita(x,-y);
  }



 
      
}

//--- Funções estabelecidas de movimento ---//

void esquerda(int a, int b){
  //a = (int)a*0.4;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, a+40);
  analogWrite(ENB, b);
}

void frente(int a, int b){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, a);
  analogWrite(ENB, b);
}

void tras(int a, int b){

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, a);
  analogWrite(ENB, b);
  
}

void direita(int a, int b){

 digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, a);
  analogWrite(ENB, b+40);
}

void parar(){
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
}