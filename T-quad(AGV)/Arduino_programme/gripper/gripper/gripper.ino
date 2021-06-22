#include "meArm.h"
#include <Servo.h>

// Définition des pins pour les axes du bras robotique
#define  pin_axis_0 9
#define  pin_axis_1 10
#define  pin_axis_2 11
#define  pin_axis_3 6
/*
typedef struct 
{
  uint8_t pin;
  int offset;
  int min_pos;
  int max_pos ;
} MeArmServo;

Servo axis[4];

MeArmServo mearmServo[AXIS]= {{9,0,0,180},{10,0,50,170},{11,0,20,160},{}}
*/
// Déclaration des axes du bras robotique
Servo axis_0 ; //base
Servo axis_1 ; //right
Servo axis_2 ; //left
Servo axis_3 ; //gripper
void setup() {
  attach_axis();
  init_position();
  delay(1000);

}

void loop() {
  gotoPosition(0,90,90);
  delay(1000);
  gotoPosition(180,160,25);
  delay(1000);
  init_position();
  delay(1000);
//  servo.write(100);
//  delay(1000);
//  servo.write(180);
//  delay(1000);

}
/*-------------------------------------------------------------*/
// Fonction d'attache des axes du bras robotique
void attach_axis() {
  axis_0.attach(pin_axis_0);
  axis_1.attach(pin_axis_1);
  axis_2.attach(pin_axis_2);
  axis_3.attach(pin_axis_3);
}

/*-------------------------------------------------------------*/
// Fonction d'initialisation des positions du bras robotique
void init_position() {
  axis_0.write(0);
  axis_1.write(65);
  axis_2.write(110); 
  open_gripper();
  close_gripper();
}
void gotoPosition(int x,int y,int z) {
  axis_0.write(x);
  axis_1.write(y);
  axis_2.write(z);
}

void open_gripper() {
  for(int i=180; i>=120; i--){
    axis_3.write(i);
    delay(30);
  }
}

void close_gripper() {
  for (int i = 120; i<=180; i++){
    axis_3.write(i);
    delay(30);
  }
}
