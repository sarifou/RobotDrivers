#include "meArm.h"
#include <Servo.h>

// Définition des pins pour les axes du bras robotique
#define  pin_axis_0 9
#define  pin_axis_1 10
#define  pin_axis_2 11
#define  pin_axis_3 6


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
  axis_0.write(180);
  axis_1.write(65);
  axis_2.write(110); 
  open_gripper();
  close_gripper();
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
