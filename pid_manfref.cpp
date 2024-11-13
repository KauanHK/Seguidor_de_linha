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
#define TRESHOLD 512 // Valor de referencia para cor da linha branca
#define SPEED0 255   // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 0 0)
#define SPEED1 210   // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 1 1 1 0)

#define SPEED2 180 // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 0 0)
#define SPEED3 150 // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 0)
#define SPEED4 100 // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 1)

#define SPEED5 0   // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 0)
#define SPEED6 0   // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 1 1)
#define SPEED7 100 // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 0 1)

#define RUNTIME 25000 // Valor para executar o percurso

#define ESQUERDA '6'
#define DIREITA '4'
#define TRAS '2'
#define FRENTE '8'
#define PARAR '0'

// Dados para PID
float Kp = 10,
      Ki = 0,
      Kd = 0,
      error = 0,
      P = 0,
      I = 0,
      D = 0,
      PID_value = 0,
      previous_error = 0,
      previous_I = 0;

int sensor[6] = {0, 0, 0, 0, 0, 0};
int initial_motor_speed = 100;
// ==========================

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
    char dado = Serial.read();
    if (dado == '8' || dado == '2' || dado == '4' || dado == '6' || dado == '0'){
        motorOption(dado, 10, 10);
    } else if (dado == 'z' || dado == 'x' || dado == 'y' || dado == 'a' || dado == 'b' || dado == 'c'){
        advancedOption(dado);
    }
}
// rgbControl(0, 255, 0, 1);

void advancedOption(char option){
    switch (option){
        case 'a':
            readSensors();
            break;
        case 'b':
            followLineMEF(); // Seguidor de linha com Máquina de Estados Finitos
            break;

        case 'c':
            followLinePid(); // Seguidor de linha com PID
            break;
    }
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


void motorOption(char option, int speedLeft, int speedRight)
{
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
    // Função de parada do robô
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

void rgbControl(int red, int green, int blue, long rumtime)
{
    // Função para controle do led rgb
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

void readSensors(void)
{
    // Função para leitura dos sensores
    Serial.print(analogRead(A0));
    Serial.print(' ');
    Serial.print(analogRead(A1));
    Serial.print(' ');
    Serial.print(analogRead(A2));
    Serial.print(' ');
    Serial.print(analogRead(A3));
    Serial.print(' ');
    Serial.print(analogRead(A4));
    Serial.print(' ');
    Serial.println(analogRead(A5));
    Serial.print(' ');
}

// Codigo do PID
void followLinePid(){

    long stopTime = millis() + RUNTIME;
    while (motorStop(stopTime)){

        read_sensor_values();
        calculate_pid();
        motor_control_pid();
    }
    motorOption('0', 0, 0);
}

void read_sensor_values(){

    bool s1 = analogRead(SENSOR1) <= TRESHOLD,
         s2 = analogRead(SENSOR2) <= TRESHOLD,
         s3 = analogRead(SENSOR3) <= TRESHOLD,
         s4 = analogRead(SENSOR4) <= TRESHOLD,
         s5 = analogRead(SENSOR5) <= TRESHOLD,
         s6 = analogRead(SENSOR6) <= TRESHOLD;

    int sensors = (s1) << 5 | (s2) << 4 | (s3) << 3 | (s4) << 2 | (s5) << 1 | (s6);

    switch (sensors){
        case 0b111111:
        case 0b011110:
        case 0b001100:
            error = 0;
            break;
        case 0b001000:
            error = -1;
            break;
        case 0b000100:
            error = 1;
            break;
        case 0b011000:
            error = -2;
            break;
        case 0b000110:
            error = 2;
            break;
        case 0b010000:
            error = -3;
            break;
        case 0b000010:
            error = 3;
            break;
        case 0b110000:
            error = -4;
            break;
        case 0b000011:
            error = 4;
            break;
        case 0b100000:
            error = -5;
            break;
        case 0b000001: 
            error = 5;
            break;   
    }

    Serial.print("Erro: ");
    Serial.print(error);
    Serial.print(", ");
}

// calcula PID conforme o error.
void calculate_pid(){
    P = error;
    I = I + previous_I;
    D = error - previous_error;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    previous_I = I;
    previous_error = error;
}

// controla os motores conforme valor do PID
void motor_control_pid()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;

    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed, 0, 255);
    constrain(right_motor_speed, 0, 255);
	
    Serial.print("Motor esquerda: ");
    Serial.print(left_motor_speed);
    Serial.print(" ");
    Serial.print("Motor direita: ");
    Serial.println(right_motor_speed);
   
    motorControl(left_motor_speed, right_motor_speed);
}

// Fim codigo PID

void followLineMEF(void)
{
    // Função para controle do seguidor de linha em modo de maquina de estado finita
    bool flag = true;
    long currentTime = millis();

    while (flag)
    {
        flag = motorStop(RUNTIME, currentTime);

        // leitura do sensor (1 1 1 1 1 1)
        if (analogRead(A0) <= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) <= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED0);
            // leitura do sensor (0 1 1 1 1 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED0);
            // leitura do sensor (0 0 1 1 0 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED0);

            // leitura do sensor (0 1 1 1 0 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED1);

            // leitura do sensor (0 0 1 1 1 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED1, SPEED0);

            // leitura do sensor (0 0 1 0 0 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED2);
            // leitura do sensor (0 0 0 1 0 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED2, SPEED0);

            // leitura do sensor (0 1 1 0 0 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED3);

            // leitura do sensor (0 0 0 1 1 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED3, SPEED0);

            // leitura do sensor (1 1 1 0 0 0)
        }
        else if (analogRead(A0) <= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) <= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED4);
            // leitura do sensor (0 0 0 1 1 1)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) <= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) <= TRESHOLD)
        {
            motorOption('8', SPEED4, SPEED0);

            // leitura do sensor (0 1 0 0 0 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED5);

            // leitura do sensor (0 0 0 0 1 0)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED5, SPEED0);

            // leitura do sensor (1 1 0 0 0 0)
        }
        else if (analogRead(A0) <= TRESHOLD && analogRead(A1) <= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('8', SPEED0, SPEED6);

            // leitura do sensor (0 0 0 0 1 1)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) <= TRESHOLD && analogRead(A5) <= TRESHOLD)
        {
            motorOption('8', SPEED6, SPEED0);

            // leitura do sensor (1 0 0 0 0 0)
        }
        else if (analogRead(A0) <= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) >= TRESHOLD)
        {
            motorOption('6', SPEED7, SPEED7);
            // leitura do sensor (0 0 0 0 0 1)
        }
        else if (analogRead(A0) >= TRESHOLD && analogRead(A1) >= TRESHOLD && analogRead(A2) >= TRESHOLD && analogRead(A3) >= TRESHOLD && analogRead(A4) >= TRESHOLD && analogRead(A5) <= TRESHOLD)
        {
            motorOption('4', SPEED7, SPEED7);
        }
    }
    motorOption('0', 0, 0);
}
