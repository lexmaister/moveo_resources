
// Контролируем робот Moveo с помощью его 3D-модели и CAD-редактора


#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

//
// Steppers
//
// Поворотная платформа
#define J1_STEP_PIN         60
#define J1_DIR_PIN          61

// Плечевой сустав
#define J2_STEP_PIN         46
#define J2_DIR_PIN          48

// Локтевой сустав
#define J3_STEP_PIN         54
#define J3_DIR_PIN          55
#define J3_EN_PIN           38

// Запястье - поворот
#define J4_STEP_PIN         26
#define J4_DIR_PIN          28
#define J4_EN_PIN           24

// Запястье - наклон
#define J5_STEP_PIN         36
#define J5_DIR_PIN          34
#define J5_EN_PIN           30

AccelStepper stepper_J1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper stepper_J2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);
AccelStepper stepper_J3(AccelStepper::DRIVER, J3_STEP_PIN, J3_DIR_PIN);
AccelStepper stepper_J4(AccelStepper::DRIVER, J4_STEP_PIN, J4_DIR_PIN);
AccelStepper stepper_J5(AccelStepper::DRIVER, J5_STEP_PIN, J5_DIR_PIN);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

// Array of desired stepper positions
long positions[5] = {
  0,
  0,
  0,
  0,
  0 
};

// Массив максимальных перемещений в шагах ШД
long end_stop[5] = {
  4000,              // поворотная платформа
  2450,              // плечевой сустав
  3020,              // локтевой сустав
  200,               // поворот запястья (половина оборота)
  400,               // наклон запястья
};

// Коэффициенты перевода градусов в шаги
float angle2pos_coeff[5] = {
  44.44,
  24.44,
  25.09,
  1.11,
  5
};

// Поправки на направление при задании углов
int angle_dir_coef[5] = {
  1,
  -1,
  1,
  1,
  1
};

// Cкорость приводов
float stepper_speed[5];

// Установка скорости относительно минимальной
void SetSpeed(float new_speed) {
  if (new_speed >= 1 && new_speed <= 90) {
    for (int i=0; i<5; i++) 
      stepper_speed[i] = new_speed * angle2pos_coeff[i]; 
    
    stepper_J1.setMaxSpeed(stepper_speed[0]);
    stepper_J2.setMaxSpeed(stepper_speed[1]);
    stepper_J3.setMaxSpeed(stepper_speed[2]);
    stepper_J4.setMaxSpeed(stepper_speed[3]);
    stepper_J5.setMaxSpeed(stepper_speed[4]);
    
    Serial.print("Установлена скорость ");
    Serial.print(new_speed);
    Serial.println(" градусов в секунду.");
  }
  else {
    Serial.println("Ошибка установки скорости!");
    return;
  }
}

// Установка минимальной скорости - градус в секунду
void SetMinSpeed() {
  for (int i = 0; i < 5; i++)
    stepper_speed[i] = angle2pos_coeff[i];
  
  stepper_J1.setMaxSpeed(stepper_speed[0]);
  stepper_J2.setMaxSpeed(stepper_speed[1]);
  stepper_J3.setMaxSpeed(stepper_speed[2]);
  stepper_J4.setMaxSpeed(stepper_speed[3]);
  stepper_J5.setMaxSpeed(stepper_speed[4]);
  
  Serial.println("Установлена скорость градус в секунду");

}

// Установка в исходное положение - вертикальная стойка
void GoHome(void) {
  for(int i=0; i<sizeof(positions)/sizeof(long); i++) positions[i] = 0;
  SetSpeed(30);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}

// Установка в положение паркинга
void GoPark(void) {
  positions[0] = 0;
  positions[1] = end_stop[1];
  positions[2] = end_stop[2];
  positions[3] = 0;
  positions[4] = end_stop[4];
  SetSpeed(30);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}

// Вращение сустава на заданное ко-во шагов
void MoveJoint(int joint_number, int steps) {
  if (joint_number > 0 && joint_number <= 5) {
    if (abs(steps) <= end_stop[joint_number-1]) {
      positions[joint_number-1] = steps;
      Serial.print("Moving J");
      Serial.print(joint_number);
      Serial.print(" to ");
      Serial.println(steps);
      steppers.moveTo(positions);
      steppers.runSpeedToPosition(); 
      Serial.println("Done");
    }
    else Serial.println("Задано недопустимое перемещение!");
  }
}

// Вращение сустава на заданный угол
void MoveJointAngle(int joint_number, float angle) {
  if (joint_number > 0 && joint_number <= 5) {
    int steps = angle * angle2pos_coeff[joint_number-1];
    if (abs(steps) <= end_stop[joint_number-1]) {        
      
      positions[joint_number-1] = steps * angle_dir_coef[joint_number-1];
      
      Serial.print("Moving J");
      Serial.print(joint_number);
      Serial.print(" to ");
      Serial.println(angle);
      steppers.moveTo(positions);
      steppers.runSpeedToPosition(); 
      Serial.println("Done");
    }
    else Serial.println("Задано недопустимое перемещение!");
  }
}

// Поворот манипулятора в требуемое положение
void MoveAngles(float a0, float a1, float a2, float a3, float a4) {
  float angles[5] = {
    a0,
    a1,
    a2,
    a3,
    a4
  };
  
  for (int i=0; i<5; i++) 
    if (abs(angles[i] * angle2pos_coeff[i]) > end_stop[i]) {
      Serial.println("Выход за диапазон - angles");
      return; 
    }

  for (int i=0; i<5; i++) 
    positions[i] = angles[i] * angle2pos_coeff[i] * angle_dir_coef[i];
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}

//
// Servo
//

// Создание объекта для управления сервоприводом
Servo end_effector;

// Функция для освобождения захвата
void Release() {
  end_effector.write(60);          
}

// Функция для активации захвата
void Grab() {
  end_effector.write(140);       
}

//
// Functions other
//

// Получение float из команды c двухбуквенным преффиксом
float ParseFloat2(String in_str) {
  if (in_str.length() > 2) {
    String s = in_str.substring(2);
    float f = s.toFloat();
    return f;  
  } 
}

void setup() {

  Serial.begin(115200);

  // Выбор режима работы
  Serial.println("Управление манипулятором Moveo:");
  Serial.println("-> H - установка в стартовое вертикальное положение");
  Serial.println("-> P - установка в паркинг");
  Serial.println("-> G - захватить");
  Serial.println("-> R - от пустить");
  Serial.println("-> SS kk - установка скорости 1 <= kk <= 20");
  Serial.println("-> Jx yyyy -  поворот ШД звена x на yyyy шагов");
  /*
  Serial.println(F("допустимые диапазоны поворота для приводов:"));
  for (int i=0; i<5; i++) {
    Serial.print("J" + String(i+1) + " - ");
    Serial.println(end_stop[i]);
  }
  */
  Serial.println(F("-> Ax zzz -  поворот ШД звена x на zzz градусов"));
  Serial.println(F("допустимые диапазоны поворота для приводов:"));
  for (int i=0; i<5; i++) {
    Serial.print("J" + String(i+1) + " - ");
    Serial.println(int(end_stop[i]/angle2pos_coeff[i]));
  }
   
  // Configure each stepper
  pinMode(J3_EN_PIN, OUTPUT);
  digitalWrite(J3_EN_PIN, LOW);
  pinMode(J4_EN_PIN, OUTPUT);
  digitalWrite(J4_EN_PIN, LOW);
  pinMode(J5_EN_PIN, OUTPUT);
  digitalWrite(J5_EN_PIN, LOW);
  
  SetMinSpeed(); // по умолчанию скорость градус в секунду

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper_J1);
  steppers.addStepper(stepper_J2);
  steppers.addStepper(stepper_J3);
  steppers.addStepper(stepper_J4);
  steppers.addStepper(stepper_J5);

  // Инициализация сервопривода на пине D4 в позиции "отпустить"
  end_effector.attach(4); 
  Release();
  
}

void loop() {

  if (Serial.available()) {
    
    String in_str = Serial.readString();  
    // подготовка строки
    // убираю пробелы в начале и конце
    in_str.trim();
    // убираю символы переноса и пробелы
    in_str.replace("\n","");         
    in_str.replace("\r","");
    in_str.replace(" ","");

  //
  // Обработка команд
  //
  
  bool bad_command = true;
  
  if (in_str.startsWith("h") || in_str.startsWith("H")) {
    Serial.println("Going home...");
    GoHome();
    Serial.println("Done");
    bad_command = false;
  }
  
  if (in_str.startsWith("p") || in_str.startsWith("P")) {
    Serial.println("Going park...");
    GoPark();
    Serial.println("Done");
    bad_command = false;
  }
  
  if (in_str.startsWith("g") || in_str.startsWith("G")) {
    Serial.println("Grabbing...");
    Grab();
    bad_command = false;
  }

  if (in_str.startsWith("r") || in_str.startsWith("G")) {
    Serial.println("Release");
    Release();
    bad_command = false;
  }

  if (in_str.startsWith("ss") || in_str.startsWith("SS")) {
    float new_speed = ParseFloat2(in_str);
    SetSpeed(new_speed);
    bad_command = false;
  }
  
  for (int i=1; i<=5; i++) {
    if (in_str.startsWith("j"+String(i)) || in_str.startsWith("J"+String(i))) {
      int steps = int(ParseFloat2(in_str));
      MoveJoint(i, steps);
      bad_command = false;  
    }
  }

  for (int i=1; i<=5; i++) {
    if (in_str.startsWith("a"+String(i)) || in_str.startsWith("A"+String(i))) {
      float angle = ParseFloat2(in_str);
   //   Serial.println(int(angle * angle2pos_coeff[i-1]));
      MoveJointAngle(i, angle);
      bad_command = false;  
    }
  }

  if (in_str.startsWith("s3")) {
    Serial.println("Running script 3 - dance ...");

    SetSpeed(50);

    MoveAngles(-90,0,0,0,0);
    delay(1000);

    // выход на исходную движение пятым звеном

    SetSpeed(50);

    MoveAngles(-90,0,0,0,-60);
    delay(300);
    MoveAngles(-90,0,0,0,60);
    delay(300);
    MoveAngles(-90,0,0,0,0);
    delay(500);

    // змейка

    SetSpeed(50);

    MoveAngles(-90,-58,114.9,0,32.9);
    delay(300);
    MoveAngles(-90,58,-114.9,0,-32.9);
    delay(300);
    MoveAngles(-90,-58,114.9,0,32.9);
    delay(300);

    // сплайн

    SetSpeed(30);
    
    // 1-2 секунда
    MoveAngles(-90,-46,116.6,0,19.2);
    MoveAngles(-90,-34.2,119.8,0,4.2);
    MoveAngles(-90,-20.4,120.2,0,-10);
    MoveAngles(-90,-5,115.5,0,-20.6);
    MoveAngles(-90,9.7,103.5,0,-23.4);

    // 2-4 секунда
    MoveAngles(-90,19.9,85,0,-15.1);
    MoveAngles(-90,24.7,66.1,0,-1);
    MoveAngles(-90,26.4,49.9,0,13.6);
    MoveAngles(-90,24.2,39.9,0,25.8);
    MoveAngles(-90,15.1,43.2,0,31.5);

    // 4-6 секунда
    SetSpeed(35);

    MoveAngles(-90,4,51,0,35);
    MoveAngles(-90,-7,55.3,0,41.5);
    MoveAngles(-90,-16.7,56.7,0,50);
    MoveAngles(-90,-25.2,55.8,0,59.2);
    MoveAngles(-90,-25.2,55.8,0,59.2);

    // 6-8 секунда
    MoveAngles(-90,-40,50,0,80);
    MoveAngles(-90,-47.6,48.2,0,80);
    MoveAngles(-90,-57.8,51.3,0,80);
    MoveAngles(-90,-72.1,63,0,80);
    MoveAngles(-90,-88,83,0,70);

    // 8-11 секунда
    MoveAngles(-90,-100,104.3,0,60);
    MoveAngles(-90,-100,120,0,50);
    MoveAngles(-90,-86.6,120,0,40);
    MoveAngles(-90,-68.5,114,0,30);
    MoveAngles(-90,-50.6,94,0,20);
    MoveAngles(-90,-33,65.8,0,10);
    MoveAngles(-90,0,0,0,0);
    
    
    delay(500);
    GoHome();
    delay(2000);

    // поклон
    MoveAngles(0,30,30,0,60);
    delay(500);
    GoHome();
    delay(2000);
    GoPark();


    Serial.println("Done");
    bad_command = false;
  }

  if (in_str.startsWith("s4")) {
    Serial.println("Running script 3 - light ...");

    SetSpeed(20);
    GoPark();
    delay(3000);    

    // захват лампы

    SetSpeed(20);

    MoveAngles(-29.5,19.6,99.4,0,60.8);
    delay(1000);
    Release();
    delay(1000);
    MoveAngles(-29.5,36,105.3,0,38.6);
    delay(1000);
    Grab();
    delay(1000);

    // подготовка к установке

    MoveAngles(-29.5,19.6,99.4,0,60.8);
    delay(1000);
    MoveAngles(90,-5.7,95.6,0,0);
    delay(1000);
    
    SetSpeed(90);
    MoveJointAngle(4, 180);

    // установка лампы

    SetSpeed(10);

    MoveAngles(90,31.6,58.2,180,0);
    delay(1000);
    
    SetSpeed(60);
    MoveJointAngle(4, -180);
    delay(1000);
    Release();
    delay(1000);

    SetSpeed(90);
    MoveJointAngle(4, 90);
    Grab();
    delay(1000);

    SetSpeed(60);
    MoveJointAngle(4, -180);
    delay(1000);
    Release();
    delay(1000);

    SetSpeed(90);
    MoveJointAngle(4, 90);
    Grab();
    delay(1000);

    SetSpeed(60);
    MoveJointAngle(4, -40);
    delay(1000);
    Release();
    delay(1000);

    // откат

    SetSpeed(40);
    MoveAngles(90,-5.7,95.6,0,0);
    delay(1000);
    GoPark();


    Serial.println("Done");
    bad_command = false;
  }
  
  if (bad_command) Serial.println("Неверная команда!");
  }
}
