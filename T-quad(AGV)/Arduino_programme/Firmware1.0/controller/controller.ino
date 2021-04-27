//============ Insertions des bibliothèques ============================

#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

//============ Définitions et déclarations des codeurs =================
// Moteur arrière droit
#define codeurArriereDroitPinA 2
#define codeurArriereDroitPinB 8

//Moteur arrière gauche
#define codeurArriereGauchePinA 3
#define codeurArriereGauchePinB 9

//Moteur avant droit
#define codeurAvantGauchePinA 19
#define codeurAvantGauchePinB 49

//Moteur avant gauche
#define codeurAvantGauchePinA 18
#define codeurAvantGauchePinB 23

void velcallback( const geometry_msgs::Twist &vel) {
  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velcallback);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);

void setup() {
  initEncoder();

}

void loop() {
  // put your main code here, to run repeatedly:

}

void initEncoder() {
  pinMode(codeurArriereDroitPinA, INPUT_PULLUP);
  pinMode(codeurArriereDroitPinB, INPUT_PULLUP);
  
  pinMode(codeurArriereGauchePinA, INPUT_PULLUP);
  pinMode(codeurArriereGauchePinB, INPUT_PULLUP);
  
  pinMode(codeurAvantGauchePinA, INPUT_PULLUP);
  pinMode(codeurAvantGauchePinB, INPUT_PULLUP);
  
  pinMode(codeurAvantGauchePinA, INPUT_PULLUP);
  pinMode(codeurAvantGauchePinB, INPUT_PULLUP);
  
}
