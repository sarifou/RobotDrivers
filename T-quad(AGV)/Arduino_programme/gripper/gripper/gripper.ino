#include "meArm.h"
#include <Servo.h>

// Définition des pins pour les axes du bras robotique
#define  pin_axis_0 9
#define  pin_axis_1 10
#define  pin_axis_2 11
#define  pin_axis_3 6

// Déclaration des axes du bras robotique
Servo axis_0 ;
Servo axis_1 ;
Servo axis_2 ;
Servo axis_3 ;
void setup() {
  attach_axis();
  init_position();
  open_gripper();
  close_gripper();
  delay(1000);

}

void loop() {
//  servo.write(100);
//  delay(1000);
//  servo.write(180);
//  delay(1000);

}
/*-------------------------------------------------------------*/
// Fonction d'attache des axes du bras robotique
void attachAxis() {
  axis_0.attach(pin_axis_0);
  axis_1.attach(pin_axis_1);
  axis_2.attach(pin_axis_2);
  axis_3.attach(pin_axis_3);
}

/*-------------------------------------------------------------*/
// Fonction d'initialisation des positions du bras robotique
void init_position() {
  axis_0.write(0);
  axis_1.write(50);
  axis_2.write(120);
  axis_3.write(180);
}

void open_gripper() {
  for(int i=180; i>=120; i--){
    axis_3.write(i);
    delay(50);
  }
}

void close_gripper() {
  for (int i = 120; i<=180; i++){
    axis_3.write(i);
    delay(50);
  }
}
