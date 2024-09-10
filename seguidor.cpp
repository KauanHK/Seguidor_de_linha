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

void setup() {
  Serial.begin(9600);
  pinMode(PININ1, OUTPUT);
  pinMode(PININ2, OUTPUT);
  pinMode(PININ3, OUTPUT);
  pinMode(PININ4, OUTPUT);
  pinMode(PINENA, OUTPUT);
  pinMode(PINENB, OUTPUT);
  
  pinMode(PINLEDR, OUTPUT);
  pinMode(PINLEDG, OUTPUT);
  pinMode(PINLEDB, OUTPUT);
  
  pinMode(13, OUTPUT);

  ledControl(true, 500);
  ledControl(false, 500);
  ledControl(true, 500);
  ledControl(false, 500);
}

void loop() {
  followLineMEF();
}

void motorControl(int speedLeft, int speedRight) {
  
  // Função para controle do driver de motor
  int leftDir = (speedLeft <= 0) ? HIGH : LOW;
  int rightDir = (speedRight < 0) ? HIGH : LOW;

  digitalWrite (PININ3, leftDir);
  digitalWrite (PININ4, !leftDir);
  digitalWrite (PININ1, !rightDir);
  digitalWrite (PININ2, rightDir);

  analogWrite (PINENA, abs(speedLeft));
  analogWrite (PINENB, abs(speedRight));
}

bool motorStop(long currentTime) {
  // Função de parada do robô
  return millis() <= (RUNTIME + currentTime);
}

void rgbControl(int red, int green, int blue, long rumtime) {
  // Função para controle do led rgb

  analogWrite(PINLEDR, red);
  analogWrite(PINLEDG, green);
  analogWrite(PINLEDB, blue);
  delay(rumtime);
}

void ledControl(bool status, long runtime) {
  // Função para controle do led
  int mode = (status) ? HIGH : LOW;
  digitalWrite(13, mode);
  delay(runtime);
}

void readSensors(int *sensors) {
  // Função para leitura dos sensores
    
  bool s1 = analogRead(S1) < TRESHOLD;
  bool s2 = analogRead(S2) < TRESHOLD;
  bool s3 = analogRead(S3) < TRESHOLD;
  bool s4 = analogRead(S4) < TRESHOLD;
  bool s5 = analogRead(S5) < TRESHOLD;
  bool s6 = analogRead(S6) < TRESHOLD;
  *sensors = (s1) << 5 | (s2) << 4 | (s3) << 3 | (s4) << 2 | (s5) << 1 | (s6);
    
}

void followLineMEF(void) {
  // Função para controle do seguidor de linha em modo de maquina de estado finita
  long currentTime = millis();
  int sensors;
  while (motorStop(currentTime)) {

    // Leitura sensores
    readSensors(&sensors);

    switch (sensors){
        case 0b111111:
        case 0b011110:
        case 0b001100:
            motorControl(-SPEED0, SPEED0);
            break;
        case 0b011100:
            motorControl(-SPEED0, SPEED1);
            break;
        case 0b001110:
            motorControl(-SPEED1, SPEED0);
            break;
        case 0b001000:
            motorControl(-SPEED0, SPEED2);
            break;
        case 0b000100:
            motorControl(-SPEED2, SPEED0);
            break;
        case 0b011000:
            motorControl(-SPEED0, SPEED3);
            break;
        case 0b000110:
            motorControl(-SPEED3, SPEED0);
            break;
        case 0b111000:
            motorControl(-SPEED0, SPEED4);
            break;
        case 0b000111:
            motorControl(-SPEED4, SPEED0);
            break;
        case 0b010000:
            motorControl(-SPEED0, SPEED5);
            break;
        case 0b000010:
            motorControl(-SPEED5, SPEED0);
            break;
        case 0b110000:
            motorControl(-SPEED0, SPEED6);
            break;
        case 0b000011:
            motorControl(-SPEED6, SPEED0);
            break;
        case 0b100000:
            motorControl(SPEED7, SPEED7);
            break;
        case 0b000001:
            motorControl(-SPEED7, -SPEED7);
            break;

        
    }
  }
	motorControl(0, 0);
    while (true) {
      ledControl(true, 250);
      ledControl(false, 250);
    }

}