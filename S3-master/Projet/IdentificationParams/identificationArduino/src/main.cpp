/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <libExample.h> // Vos propres librairies
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 1;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

enum Etats { SetupPosition, SetupSapin, Acceleration, Aller, Stabilisation, Drop };

enum Etats etat;

/*------------------------- Prototypes de fonctions -------------------------*/

float lirePotentiometre(uint8_t pin);
void centraleInertielle(float Axyz[3], float Gxyz[3]);
void avancer ();
void stop();
void reculer();
void electromagnet_on(uint8_t pin);
void electromagnet_off(uint8_t pin);

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();

// Fonctions pour le PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);                // initialisation de la communication serielle
  AX_.init();                        // initialisation de la carte ArduinoX 
  imu_.init();                       // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);             // initialisation de l'encodeur VEX
  pinMode(32, OUTPUT);               // Definition du IO
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  pinMode(POTPIN, INPUT);

  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID
  pid_.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001);
  pid_.setPeriod(200);

  etat = SetupPosition;
}

/* Boucle principale (infinie)*/
void loop() {

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }

  switch (etat)
  {
    case SetupPosition:
      // While (Micro switch pas appuyée)
        // Reculer avec le moteur
      
      etat = SetupSapin;
      break;
    case SetupSapin:
      // If électroaimant activé && bool Pince == true
        etat = Acceleration;
      break;
    case Etat3:
      break;
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  // mise à jour du PID
  pid_.run();

   // Deplacement du robot (moteur)
  avancer();
  delay(5000);
  stop();
  delay(1000);
  reculer();
  delay(5000);
  stop();
  delay(1000);

  // Activer/desactiver electro-aimant
  /* test */
  electromagnet_on(MAGPIN);
  delay(5000);
  electromagnet_off(MAGPIN);
  delay(5000);
}

/*---------------------------Definition de fonctions ------------------------*/

void avancer ()
{
  AX_.setMotorPWM(0, pulsePWM_);
}

void stop()
{
  AX_.setMotorPWM(0,0);
}

void reculer()
{
  AX_.setMotorPWM(0, -pulsePWM_);
}

void electromagnet_on(uint8_t pin)
{
digitalWrite(pin, HIGH); // Activation electroAimant

}

void electromagnet_off(uint8_t pin)
{
  digitalWrite(pin, LOW); // Desactivation electroAimant
}

float lirePotentiometre(uint8_t pin)
{
  float angle, angleDeg;

  angle = analogRead(pin);

  angleDeg = ((angle - 511.5) * 275) / 1023;

  return angleDeg;
}

void centraleInertielle(float Axyz[3], float Gxyz[3])
{
  Axyz[0] = imu_.getAccelX();
  Axyz[1] = imu_.getAccelY();
  Axyz[2] = imu_.getAccelZ();

  Gxyz[0] = imu_.getGyroX();
  Gxyz[1] = imu_.getGyroY();
  Gxyz[2] = imu_.getGyroZ();

}

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_.getGoal();
  doc["measurements"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal"] = pid_.isAtGoal();
  doc["actualTime"] = pid_.getActualDt();
  doc["degresPendule"] = lirePotentiometre(POTPIN);

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
  parse_msg = doc["setGoal"];
  if(!parse_msg.isNull()){
    pid_.disable();
    pid_.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_.setEpsilon(doc["setGoal"][3]);
    pid_.setGoal(doc["setGoal"][4]);
    pid_.enable();
  }
}


// Fonctions pour le PID
double PIDmeasurement(){
  // To do
  return 0;
}
void PIDcommand(double cmd){
  // To do
}
void PIDgoalReached(){
  // To do
}