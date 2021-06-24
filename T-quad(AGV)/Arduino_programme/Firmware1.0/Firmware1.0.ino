/*
 * Firmware arduino pour contrôler les robots T-quad avec ROS
 * @auteur : Mamadou Sarifou Diallo
 * @email : diallo.msdpro@gmail.com
 * @version : 1.0.0
 */

//============ Insertions des bibliothèques ===========================================
#include "Motor.h"
#include <NewPing.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <Servo.h>

//============ Définitions et déclarations des moteurs et des codeurs =================
//Moteur avant gauche
Motor moteurGaucheAvant(32,31,46);

//Moteur arrière gauche
Motor moteurGaucheArriere(7,34,6);

//Moteur avant droit
Motor moteurDroitAvant(33,30,44);

//Moteur arrière droit
Motor moteurDroitArriere(36,4,5);


//============ Définitions et déclarations du capteurs ultrasons ======================
// Déclarations du capteurs ultrason
NewPing sonar(11,10,200);
volatile float sonar_range = 0;


//============ Définitions et déclarations du capteurs de ligne ======================
// Pin des capteurs de ligne
int ligne_gauche=2;
int ligne_centre=3;
int ligne_droite=4;

//============ Définition du pin de la Batterie ======================
#define pin_bat 5

// message ros pour publier les valeurs des capteurs
std_msgs::Float64MultiArray pub_msgs;
char pub_label[]= "serial_pub_values";

// Callback pour le contrôle des moteurs
void subscriber_callback(const std_msgs::Int64MultiArray &value) {
  // Commande des moteurs
  moteurGaucheAvant.run(value.data[0]);
  moteurGaucheArriere.run(value.data[1]);
  moteurDroitAvant.run(value.data[2]);
  moteurDroitArriere.run(value.data[3]);
}

ros::NodeHandle nh;
// Subscriber pour le contrôle des moteurs du Tquad
ros::Subscriber<std_msgs::Int64MultiArray> serial_subscriber("tquad/serial_subscriber", subscriber_callback);
// Publisher pour les valeurs des capteurs du Tquad
ros::Publisher serial_publisher("tquad/serial_publisher", &pub_msgs);

void setup() {
  // Initialisations des moteurs
  init_motors();

  setPubArray();

  // Initialisation du handler ros
  nh.getHardware()->setBaud(115200); // Vitesse de communication fixée à 115200 bauds
  nh.initNode();
  
  // Initilisation du publisher
  nh.advertise(serial_publisher);
  // Initialisation du subscriber des moteurs
  nh.subscribe(serial_subscriber);
}

void loop() {
  publisher();
  nh.spinOnce();
  delay(1000);
}

float getSense() {
  sonar_range = sonar.ping_cm();
  return sonar_range ;
}
void init_motors() {
  moteurDroitAvant.init();
  moteurDroitArriere.init();
  moteurGaucheAvant.init();
  moteurGaucheArriere.init();
}
void publisher() {
  pub_msgs.data[0]= getSense();
  pub_msgs.data[1] = analogRead(ligne_centre);
  pub_msgs.data[2] = analogRead(ligne_gauche);
  pub_msgs.data[3] = analogRead(ligne_droite);
  pub_msgs.data[4] = getBatteryVoltage();
  serial_publisher.publish(&pub_msgs);
}

void setPubArray() {
  pub_msgs.layout.dim[0].label = pub_label;
  pub_msgs.layout.dim[0].size = 5;
  pub_msgs.layout.data_offset = 0;
  pub_msgs.data = (float*)malloc(sizeof(float) * 5);
  pub_msgs.data_length = 5;
}

uint16_t getBatteryVoltage() {
    return (int)(1000. * 3. * (5. * (float)analogRead(pin_bat) / 1024.));
}