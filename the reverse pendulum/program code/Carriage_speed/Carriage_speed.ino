#include "Servo.h"
#include "MPU6050.h"
#include "math.h"

#define servaPin 9

#define maxAngle 25
#define stillAngle 90

// #define Kp  40
// #define Kd  0.05
// #define Ki  40


//наш dt в сеундах
#define dt 0.01
//по факту це наш dt просто в миллисекундах(по длительнности управляющего импульса сервопривода)
#define sampleTime (dt * 1000)
//пол секунды dt для ввода-вывода
#define sampleBigTime 100
//если нам нужна наклонённая конструкция, в случае с роботом
#define targetAngle 0  //-2.5

//коэффиценты pid регулятора
float Kp = 30;
float Kd = 0.5;
float Ki = 2;

//коэффиценты для получения правильного угла currentAngle с помощью комлиментарного фильтра
#define range_time_for_filter 0.5
#define ratio_a (range_time_for_filter/(range_time_for_filter+dt))
#define ratio_b (1-ratio_a)


bool Start = false;

MPU6050 mpu;
Servo myservo;

volatile int PrevPos = 0;

int16_t accY, accZ, gyroX;
// int16_t accY, accX, gyroZ;
volatile int  gyroRate;
volatile float servo_speed, servo_angle, previous_servo_angle;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
// volatile byte count = 0;

//для ровного вывода на график
volatile float servo_speed_graphic;

//переменная для отсчёта времени квантавания
unsigned long Tnext;
//переменная для отсчёта времени ввода вывода
unsigned long TBigNext;

int count = 0;



void setup() {
  Serial.begin(115200); //задаём boud такой же как в консоле
  Serial.flush();  // ждём когда все старые данные пройдут, чтобы не портить названия
  Serial.println("серва, датчик");
  Serial.print(ratio_a);
  Serial.print(',');
  Serial.println(ratio_b);

  //задаём начально время
  Tnext = millis() + sampleTime;
  TBigNext = millis() + sampleBigTime;

  //присоединяем серву к пину и задаём начальное положение
  myservo.attach(servaPin);//присоединяем серву к пину
  previous_servo_angle = stillAngle;
  
  myservo.write(stillAngle);
  delay(1000);

  // initialize the MPU6050 and set offset values, которые получили при калибровке
  mpu.initialize();
  mpu.setYAccelOffset(594);
  mpu.setZAccelOffset(713);
  mpu.setXGyroOffset(-203);
}

void setMotors(int speed) {
  servo_angle = previous_servo_angle + dt * speed;
  servo_angle = constrain(servo_angle, 0, 180);
  previous_servo_angle = servo_angle;

  myservo.write(servo_angle);

  // if (fabs(grade - PrevPos) > 9) {
  //   myservo.write(grade);
  //   PrevPos = grade;
  // }
}

void loop() {
  //пробегаемся по выводу-вводу каждые пол секунды
  if (TBigNext <= millis()) {
    TBigNext += sampleBigTime;

    if (Serial.available() > 1) { //если в консоль что-то ввели
      char key = Serial.read(); //считываем первый буквенный символ
      float val = Serial.parseFloat(); //считываем последующее число
      switch (key) {
        case 'p': Kp = val; break; //меняем пропорциональную составляющую при вводе например p10
        case 'i': Ki = val; break; //меняем интегральную составляющую при вводе например z
        case 'd': Kd = val; break; //меняем дифференциальную составляющую при вводе например d
        case 's': Start = true; break; //старт 
        case 'b': Start = false; break; //стоп
      }
    }
    //выводим пару значений угол сервы и значение pid регулятора
    Serial.print(servo_angle);
    Serial.print(',');
    servo_speed_graphic = map(servo_speed, -1800, 1800, -180, 180);
    Serial.print(servo_speed_graphic);
    Serial.print(',');
    Serial.println(currentAngle);
  }

  //если наша очередь по времени дошла до нынешнего то выполняем
  if (Tnext <= millis()) {
    Tnext += sampleTime; //дополняем наш Tnext на ещё 20 миллисекунд
    if(count++==100){
      count = 0;
      // Serial.println("opa");
    }
    // if(Start == false){
    //   return;
    // }
    // read acceleration and gyroscope values
    accY = mpu.getAccelerationY();
    accZ = mpu.getAccelerationZ();
    gyroX = mpu.getRotationX();

    // calculate the angle of inclination
    accAngle = atan2(accY, accZ) * RAD_TO_DEG;
    gyroRate = map(gyroX, -32768, 32767, -250, 250);
    //умножаем на время опроса sampleTime/1000 тоесть 20 милисекунд
    gyroAngle = (float)gyroRate * dt;
    // ratio_a, ratio_b являются коэффициентами комплиментарного фильтра для интервала времени 0.75с. a = j/(j+dt)=0.75/(0.75+0.02)
    currentAngle = ratio_a * (prevAngle + gyroAngle) + ratio_b * (accAngle);

    error = currentAngle - targetAngle;
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -30, 30);
    //calculate output from P, I and D values
    servo_speed = (Kp * (error) + Ki * (errorSum)*sampleTime - Kd * (currentAngle - prevAngle) / sampleTime);

    prevAngle = currentAngle;

    // держим значения, подающиеся на серву в пределах 180 градусов
    // servo_speed = constrain(servo_speed, 0, 180);

    // if(Start){
    //   setMotors(servo_speed);
    // }
    setMotors(servo_speed);
  }
}
