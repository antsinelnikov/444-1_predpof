#include <Arduino.h>
#include <Servo.h>


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
#define MOTOR_PIN_THROW_C 13

#define ANGLE_GRAB_DOWN 180
#define ANGLE_GRAB_WAIT 90
#define ANGLE_GRAB_UP 60
#define ANGLE_PUSH_BACKWARD 0
#define ANGLE_PUSH_WAIT 60
#define ANGLE_PUSH_FORWARD 120

#define ANGLE_THROW_LONG 92
#define ANGLE_THROW_MIDDLE 95
#define ANGLE_THROW_SHORT 102

#define SPEED_TRHOW_LONG 1500
#define SPEED_TRHOW_MIDDLE 1350
#define SPEED_TRHOW_SHORT 1220
#define SPEED_THROW_WAIT 1100


class Motor {
  public:
    int pin1, pin2;

    Motor(int mpin1, int mpin2) {
      set_pins(mpin1, mpin2);
    }

    void rotate(int speed) {
      (speed > 0) ? analogWrite(pin1, speed) : digitalWrite(pin1, 0); 
      (speed > 0) ? digitalWrite(pin2, 0) : analogWrite(pin2, speed);
    }

    void set_pins(int motor1, int motor2) {
    pin1 = motor1;
    pin2 = motor2;
  }
};


Motor motor_base[3] = {
  Motor(MOTOR_PIN_A_1, MOTOR_PIN_A_2),
  Motor(MOTOR_PIN_B_1, MOTOR_PIN_B_2),
  Motor(MOTOR_PIN_C_1, MOTOR_PIN_C_2)
};

int side_mod = 0;

Servo servo_rotate_left, servo_rotate_right, servo_push;

Servo motor_throw_a, motor_throw_b, motor_throw_c;


void throw_ball(int, uint16_t);

void move(int, int, int);

void read_serial();

void grab_ball();


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

  motor_throw_a.attach(MOTOR_PIN_THROW_A);
  motor_throw_b.attach(MOTOR_PIN_THROW_B);
  motor_throw_c.attach(MOTOR_PIN_THROW_C);

  Serial.begin(9600);
}


uint32_t timer_read;
uint16_t tick_read = 50;

void loop() {
  
  if (millis() - timer_read > tick_read) {
    read_serial();
  }

}


void throw_ball(int angle, uint16_t speed) {
  servo_rotate_left.write(angle);
  servo_rotate_right.write(angle);
  
  motor_throw_a.writeMicroseconds(speed);
  motor_throw_b.writeMicroseconds(speed);
  motor_throw_c.writeMicroseconds(speed);

  delay(100);

  servo_push.write(ANGLE_PUSH_FORWARD);

  delay(500);

  servo_rotate_left.write(ANGLE_GRAB_DOWN);
  servo_rotate_right.write(ANGLE_GRAB_DOWN);
  servo_push.write(ANGLE_PUSH_BACKWARD);
}


void grab_ball() {
  servo_rotate_left.write(ANGLE_GRAB_WAIT);
  servo_rotate_right.write(ANGLE_GRAB_WAIT);

  delay(1000);

  servo_push.write(ANGLE_PUSH_WAIT);
}


void move(int speed_x, int speed_y, int speed_rotate) {
  
  if (abs(speed_x) < SPEED_MIN) speed_x = 0;
  if (abs(speed_y) < SPEED_MIN) speed_y = 0;
  if (abs(speed_rotate) < SPEED_MIN) speed_rotate = 0;

  int velocity_a = speed_x + speed_y / 2 + speed_rotate;
  int velocity_b = -speed_x + speed_y / 2 + speed_rotate;
  int velocity_c = 0 - speed_y + speed_rotate;


  int max_velocity;

  (velocity_a > velocity_b) ? max_velocity = velocity_a : max_velocity = velocity_b;
  (max_velocity > velocity_c) ? max_velocity = max_velocity : max_velocity = velocity_c;

  if (max_velocity > SPEED_MAX) {
    motor_base[0 - side_mod].rotate(map(velocity_a, -max_velocity, max_velocity, -SPEED_MAX, SPEED_MAX));
    motor_base[1 - side_mod].rotate(map(velocity_b, -max_velocity, max_velocity, -SPEED_MAX, SPEED_MAX));
    motor_base[2 - side_mod].rotate(map(velocity_c, -max_velocity, max_velocity, -SPEED_MAX, SPEED_MAX));
  } else {
    motor_base[0 - side_mod].rotate(velocity_a);
    motor_base[1 - side_mod].rotate(velocity_b);
    motor_base[2 - side_mod].rotate(velocity_c);
  }

}


void read_serial() {

  if (Serial.available() == 4) {
    byte speed_x = Serial.read();
    byte speed_y = Serial.read();
    byte speed_rotate = Serial.read();

    byte action = Serial.read();

    move((int) speed_x, (int) speed_y, (int) speed_rotate);

    if (action % 2) grab_ball();
    else if (action >> 1 % 2) throw_ball(ANGLE_THROW_SHORT, SPEED_TRHOW_SHORT);
    else if (action >> 2 % 2) throw_ball(ANGLE_THROW_MIDDLE, SPEED_TRHOW_MIDDLE);
    else if (action >> 3 % 2) throw_ball(ANGLE_THROW_LONG, SPEED_TRHOW_LONG);
  }

}

