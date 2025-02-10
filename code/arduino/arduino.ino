#include <Servo.h>


// определение констант
#define MOTOR_PIN_A_1 2
#define MOTOR_PIN_A_2 3
#define MOTOR_PIN_B_1 4
#define MOTOR_PIN_B_2 5
#define MOTOR_PIN_C_1 6
#define MOTOR_PIN_C_2 7

#define SPEED_MAX 180
#define SPEED_MIN 20

#define SERVO_PIN_ROTATE_RIGHT 8
#define SERVO_PIN_ROTATE_LEFT 9
#define SERVO_PIN_PUSH 10

#define MOTOR_PIN_THROW_A 11
#define MOTOR_PIN_THROW_B 12

#define ANGLE_GRAB_DOWN 180
#define ANGLE_GRAB_WAIT 160
#define ANGLE_GRAB_UP 30
#define ANGLE_PUSH_BACKWARD 180
#define ANGLE_PUSH_WAIT 120
#define ANGLE_PUSH_FORWARD 40

#define ANGLE_THROW_LONG 92
#define ANGLE_THROW_MIDDLE 95
#define ANGLE_THROW_SHORT 102

#define SPEED_TRHOW_LONG 1500
#define SPEED_TRHOW_MIDDLE 1350
#define SPEED_TRHOW_SHORT 1220
#define SPEED_THROW_WAIT 1100


// класс для управления моторами
class Motor {
  public:
    int pin1, pin2;

    Motor(int mpin1, int mpin2) {
      set_pins(mpin1, mpin2);
    }

    void rotate(int speed) {
      (speed > 0) ? analogWrite(pin1, 2 * speed) : digitalWrite(pin1, 0); 
      (speed > 0) ? digitalWrite(pin2, 0) : analogWrite(pin2, 2 * abs(speed));
    }

    void set_pins(int motor1, int motor2) {
    pin1 = motor1;
    pin2 = motor2;
  }
};


// объявление всех актуаторов
Motor motor_base[3] = {
  Motor(MOTOR_PIN_A_1, MOTOR_PIN_A_2),
  Motor(MOTOR_PIN_B_1, MOTOR_PIN_B_2),
  Motor(MOTOR_PIN_C_1, MOTOR_PIN_C_2)
};

Servo servo_rotate_left, servo_rotate_right, servo_push;

Servo motor_throw_a, motor_throw_b, motor_throw_c;



// функция для калибровки бесколлекорных моторов
void test() {
  motor_throw_a.writeMicroseconds(1010);
  motor_throw_b.writeMicroseconds(1010);
  delay(300);
  motor_throw_a.writeMicroseconds(2000);
  motor_throw_b.writeMicroseconds(2000);
  delay(3000);
}

// флаг для работы функции запуска мяча
bool fl_shoot;
// переменная для тактирования функции запуска мяча
uint32_t timer_shoot;

// фунция для запуска мяча
void shoot(int angle, uint16_t speed, bool if_shoot = 1) {
  fl_shoot = if_shoot;
  timer_shoot = millis();
  servo_rotate_left.write(180 - angle);
  servo_rotate_right.write(angle);

  motor_throw_a.writeMicroseconds(speed);
  motor_throw_b.writeMicroseconds(speed);
  motor_throw_c.writeMicroseconds(speed);
}


// настройка контроллера
void setup() {

  pinMode(MOTOR_PIN_A_1, OUTPUT);
  pinMode(MOTOR_PIN_A_2, OUTPUT);
  pinMode(MOTOR_PIN_B_1, OUTPUT);
  pinMode(MOTOR_PIN_B_2, OUTPUT);
  pinMode(MOTOR_PIN_C_1, OUTPUT);
  pinMode(MOTOR_PIN_C_2, OUTPUT);

  servo_rotate_left.attach(SERVO_PIN_ROTATE_LEFT);
  servo_rotate_right.attach(SERVO_PIN_ROTATE_RIGHT);
  servo_push.attach(SERVO_PIN_PUSH);

  servo_rotate_left.write(90);
  servo_rotate_right.write(180 - 90);


  motor_throw_a.attach(MOTOR_PIN_THROW_A);
  motor_throw_b.attach(MOTOR_PIN_THROW_B);

  test();
  shoot(30, 1100);
  Serial.begin(9600);
}

// переменная для корректной ориентации работы моторов
int side_mod  = 2;

// функция движения робота
void move(int speed_x, int speed_y, int speed_rotate) {
  motor_base[0].rotate(speed_x + speed_y / 2 + speed_rotate - 250);
  motor_base[1].rotate(-speed_x + speed_y / 2 + speed_rotate - 50);
  motor_base[2].rotate(-speed_y + speed_rotate);
}

// флаги для работы захвата мяча
bool fl_grab, canon_pos;

// переменные для тактирования основного цикла и функции захвата 
uint32_t timer_loop, timer_grab;


// основной цикл
void loop() {
  if (millis() - timer_loop > 20) {  
    char datay;

    if (Serial.available() == 4) {
      byte speed_y = Serial.read(), speed_x = Serial.read(), speed_rotate = Serial.read(), act = Serial.read();
      
      if (act == 1) {
        shoot(70, 1300);
      } else if (act >> 1 == 1) {
        shoot(70, 1500);
      } else if (act >> 2 == 1) {
        shoot(50, 2000);
      } else if (act >> 3 == 1) {
        grab_ball();
      }

      motor_base[0].rotate(speed_x + speed_y / 2 + speed_rotate - 250);
      motor_base[1].rotate(-speed_x + speed_y / 2 + speed_rotate - 50);
      motor_base[2].rotate(-speed_y + speed_rotate);
    
    }
    timer_loop = millis();
  }

  if (fl_grab && millis() - timer_grab > 2000) {
    servo_push.write(ANGLE_PUSH_WAIT);
    fl_grab = 0;
  }
  if (fl_shoot && millis() - timer_shoot > 500) {
    servo_push.write(ANGLE_PUSH_FORWARD);
    fl_shoot = 0;
  }
}


void grab_ball() {
  if (canon_pos) {
    shoot(0, 1000, 0);
    servo_push.write(ANGLE_PUSH_BACKWARD);
    fl_grab = 1;
    timer_grab = millis();
    // servo_push.write(ANGLE_PUSH_WAIT);
  } else {
    shoot(180 - 30, 1000, 0);
    servo_push.write(ANGLE_PUSH_BACKWARD);
  }
  canon_pos = !canon_pos;
}
