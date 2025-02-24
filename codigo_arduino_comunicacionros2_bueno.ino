
#include <Servo.h>

// Crear objetos Servo para los motores
Servo motor1, motor2, motor3, motor4;

// Pines de conexión para los motores
#define MOTOR_PIN1 9
#define MOTOR_PIN2 10
#define MOTOR_PIN3 11
#define MOTOR_PIN4 12

// Pines para los encoders
#define ENCODER1_PIN 2   // Encoder motor 1 (lado derecho, interrupción 0)
#define ENCODER2_PIN 3   // Encoder motor 2 (lado izquierdo, interrupción 1)
#define ENCODER3_PIN 21  // Encoder motor 3 (lado derecho, interrupción 2)
#define ENCODER4_PIN 20  // Encoder motor 4 (lado izquierdo, interrupción 3)

// Variables para los encoders
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
volatile int pulseCount3 = 0;
volatile int pulseCount4 = 0;

unsigned long lastTime = 0;
float rpm_right = 0, rpm_left = 0;

// Bluetooth configurado en Serial1
#define RX_BT 19
#define TX_BT 18

void setup() {
  // Configurar motores
  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);

  // Configurar comunicación
  Serial.begin(9600);       // Monitor serial
  Serial1.begin(9600);      // Bluetooth en Serial1

  // Configurar pines de los encoders con pull-up interno
  pinMode(ENCODER1_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_PIN, INPUT_PULLUP);
  pinMode(ENCODER4_PIN, INPUT_PULLUP);

  // Configurar interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN), countPulse2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PIN), countPulse3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_PIN), countPulse4, RISING);

  Serial.println("Sistema iniciado: Control de Motores y RPM para 4 encoders");
}

// Funciones de interrupción para contar pulsos
void countPulse1() { pulseCount1++; }  // Encoder motor derecho 1
void countPulse2() { pulseCount2++; }  // Encoder motor izquierdo 1
void countPulse3() { pulseCount3++; }  // Encoder motor derecho 2
void countPulse4() { pulseCount4++; }  // Encoder motor izquierdo 2

// Calcular RPM cada 2 segundos
void calculateRPM() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 2000) {  // Cada 2 segundos
    float rpm_motor1 = (pulseCount1 / 20.0) * 30.0; // Suponiendo 20 pulsos/rev
    float rpm_motor2 = (pulseCount2 / 20.0) * 30.0;
    float rpm_motor3 = (pulseCount3 / 20.0) * 30.0;
    float rpm_motor4 = (pulseCount4 / 20.0) * 30.0;

    // Calcular promedio de RPM por lado
    rpm_right = (rpm_motor1 + rpm_motor3) / 2.0;
    rpm_left = (rpm_motor2 + rpm_motor4) / 2.0;

    // Resetear contadores
    pulseCount1 = 0;
    pulseCount2 = 0;
    pulseCount3 = 0;
    pulseCount4 = 0;

    lastTime = currentTime;

    // Enviar datos al monitor serial
    //Serial.print("RPM Derecho: "); Serial.print(rpm_right);
    //Serial.print(", RPM Izquierdo: "); Serial.println(rpm_left);

    // Enviar datos a la Raspberry Pi
    Serial.print(rpm_right);
    Serial.print(",");
    Serial.println(rpm_left);
  }
}

void loop() {
  // Verificar si hay comandos desde el Bluetooth
  if (Serial1.available() > 0) {
    char command = Serial1.read();
    switch (command) {
      case 'w':  // Avanzar

        motor1.write(160);

        motor2.write(160);

        motor3.write(160);

        motor4.write(160);

        break;



      case 's':  // Retroceder

        motor1.write(20);

        motor2.write(20);

        motor3.write(20);

        motor4.write(20);

        break;



      case 'x':  // Detener

        motor1.write(90);

        motor2.write(90);

        motor3.write(90);

        motor4.write(90);

        break;



      case 'd':  // Girar a la derecha

        motor1.write(100);

        motor2.write(100);

        motor3.write(160);

        motor4.write(160);

        break;



      case 'a':  // Girar a la izquierda

        motor1.write(160);

        motor2.write(160);

        motor3.write(100);

        motor4.write(100);

        break;



      case 'c':  // Girar derecha en reversa

        motor1.write(80);  // Motores lado izquierdo hacia atrás

        motor2.write(80);

        motor3.write(20);  // Motores lado derecho hacia adelante

        motor4.write(20);

        break;



      case 'z':  // Girar izquierda en reversa

        motor1.write(20);  // Motores lado izquierdo hacia adelante

        motor2.write(20);

        motor3.write(80);  // Motores lado derecho hacia atrás

        motor4.write(80);

        break;



      case 'q':  // Pivot a la izquierda

        motor1.write(160);  // Motores izquierdo hacia atrás

        motor2.write(160);

        motor3.write(20);   // Motores derecho hacia adelante

        motor4.write(20);

        break;



      case 'e':  // Pivot a la derecha

        motor1.write(20);   // Motores izquierdo hacia adelante

        motor2.write(20);

        motor3.write(160);  // Motores derecho hacia atrás

        motor4.write(160);

        break;



      default:

        // Comando no reconocido

        break;

    }
  }

  // Calcular y mostrar las RPM
  calculateRPM();
}
