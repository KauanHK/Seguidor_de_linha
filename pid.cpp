// Portas driver motor
#define PININ1 2
#define PININ2 4
#define PININ3 5
#define PININ4 7
#define PINENA 3
#define PINENB 6

// Portas led rgb
#define PINLEDR 11
#define PINLEDG 9
#define PINLEDB 10

// Portas sensor QTR
#define SENSOR1 A0
#define SENSOR2 A1
#define SENSOR3 A2
#define SENSOR4 A3
#define SENSOR5 A4
#define SENSOR6 A5

// Valores de ajustes para o seguidor de linha MIF
#define TRESHOLD 512
#define SPEED0 255  
#define SPEED1 210  

#define SPEED2 180
#define SPEED3 150
#define SPEED4 100

#define SPEED5 0  
#define SPEED6 0  
#define SPEED7 100

// Tempo total de percurso
#define RUNTIME 25000

// Constantes para definir a direção do motor
#define ESQUERDA '6'
#define DIREITA '4'
#define TRAS '2'
#define FRENTE '8'
#define PARAR '0'

// Dados para PID
float Kp = 10,
      Ki = 0,
      Kd = 0,
      P = 0,
      I = 0,
      D = 0,
      PID_value = 0,
      previous_error = 0,

int sensor[6] = {0, 0, 0, 0, 0, 0};
int initial_motor_speed = 100;

void setup(){
    Serial.begin(9600);
    rgbControl(255, 0, 0, 100);
    rgbControl(0, 255, 0, 100);
    rgbControl(0, 0, 255, 100);
    rgbControl(0, 0, 255, 1);

    // Definições das portas digitais
    pinMode(PININ1, OUTPUT);
    pinMode(PININ2, OUTPUT);
    pinMode(PININ3, OUTPUT);
    pinMode(PININ4, OUTPUT);
    pinMode(PINENA, OUTPUT);
    pinMode(PINENB, OUTPUT);
}

void loop(){

    long stopTime = millis() + RUNTIME;
    while (motorStop(stopTime)){

        if (read_sensor_values()){
            calculate_pid();
            motor_control_pid();
        }
    }
    motorOption('0', 0, 0);
}

void rgbControl(int red, int green, int blue, long rumtime){

    pinMode(PINLEDR, OUTPUT);
    pinMode(PINLEDG, OUTPUT);
    pinMode(PINLEDB, OUTPUT);

    digitalWrite(PINLEDR, HIGH);
    digitalWrite(PINLEDG, HIGH);
    digitalWrite(PINLEDB, HIGH);

    analogWrite(PINLEDR, red);
    analogWrite(PINLEDG, green);
    analogWrite(PINLEDB, blue);
    delay(rumtime);
}

void motorControl(int speedLeft, int speedRight){

    // Ajustes motor da esquerda
    if (speedLeft < 0){
        speedLeft = -speedLeft;
        digitalWrite(PININ3, HIGH);
        digitalWrite(PININ4, LOW);
    } else {
        digitalWrite(PININ3, LOW);
        digitalWrite(PININ4, HIGH);
    }

    // Ajustes motor da direita
    if (speedRight < 0){
        speedRight = -speedRight;
        digitalWrite(PININ1, LOW);
        digitalWrite(PININ2, HIGH);
    } else {
        digitalWrite(PININ1, HIGH);
        digitalWrite(PININ2, LOW);
    }

    analogWrite(PINENA, speedLeft);
    analogWrite(PINENB, speedRight);
}


void motorOption(char option, int speedLeft, int speedRight){
    // Função para controle de motor com pre definições
    switch (option){
        case ESQUERDA:
            motorControl(-speedLeft, speedRight);
            break;
        case DIREITA:
            motorControl(speedLeft, -speedRight);
            break;
        case TRAS:
            motorControl(-speedLeft, -speedRight);
            break;
        case FRENTE:
            motorControl(speedLeft, speedRight);
            break;
        case PARAR:
            motorControl(0, 0);
            break;
    }
}

bool motorStop(long stopTime){

    if (millis() >= (stopTime)){

        motorOption('0', 0, 0);
        int cont = 0;
        while (cont < 5)
        {
            rgbControl(255, 0, 0, 500);
            rgbControl(0, 0, 0, 500);
            cont++;
        }
        return false;
    }
    return true;
}

// Retorna false caso esteja perfeito na linha (leitura 001100), senão true
bool read_sensor_values(){

    bool s1 = analogRead(SENSOR1) <= TRESHOLD,
         s2 = analogRead(SENSOR2) <= TRESHOLD,
         s3 = analogRead(SENSOR3) <= TRESHOLD,
         s4 = analogRead(SENSOR4) <= TRESHOLD,
         s5 = analogRead(SENSOR5) <= TRESHOLD,
         s6 = analogRead(SENSOR6) <= TRESHOLD;

    int sensors = (s1) << 5 | (s2) << 4 | (s3) << 3 | (s4) << 2 | (s5) << 1 | (s6);

    switch (sensors){
        case 0b111111:
        case 0b000000:
        case 0b011110:
        case 0b001100:
            motorControl(SPEED0, -SPEED0);
            return false;
        case 0b001000:
            P = -1;
            break;
        case 0b000100:
            P = 1;
            break;
        case 0b011000:
            P = -2;
            break;
        case 0b000110:
            P = 2;
            break;
        case 0b010000:
            P = -3;
            break;
        case 0b000010:
            P = 3;
            break;
        case 0b110000:
            P = -4;
            break;
        case 0b000011:
            P = 4;
            break;
        case 0b100000:
            P = -5;
            break;
        case 0b000001: 
            P = 5;
            break;   
    }

    Serial.print("Erro: ");
    Serial.print(P);
    Serial.print(", ");
    return true;
}

void calculate_pid(){

    // Acumula o erro
    I += P;

    // Diferença entre o erro atual e o anterior
    D = P - previous_error;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    previous_error = P;
}


// controla os motores conforme valor do PID
void motor_control_pid(){
    
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;

    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);
	
    Serial.print("Motor esquerda: ");
    Serial.print(left_motor_speed);
    Serial.print(" ");
    Serial.print("Motor direita: ");
    Serial.println(right_motor_speed);
   
    motorControl(left_motor_speed, right_motor_speed);
}
