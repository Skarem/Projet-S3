/*
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

// Comit

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <libExample.h> // Vos propres librairies
#include <ezButton.h>
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur
ezButton limitSwitch(7);  // create ezButton object that attach to pin 7;
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

uint16_t pulseTime_ = 1;            // temps dun pulse en ms
float pulsePWM_ = 1;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

enum Etats { Acceleration, Stabilisation };

enum Etats etat;

int const MOTEUR = 0;
double DIAMETRE_ROUE = 6.2;

/*------------------------- Prototypes de fonctions -------------------------*/

float lirePotentiometre(uint8_t pin);
void centraleInertielle(float Axyz[3], float Gxyz[3]);
void avancer ();
void stop();
void reculer();
void electromagnet_on(uint8_t pin);
void electromagnet_off(uint8_t pin);
void detectswitch();

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
  pid_.setGains(0.001993, 0.000446, 0.000728);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(1);
  pid_.setPeriod(200);

  etat = Acceleration;

  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
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

  electromagnet_on(MAGPIN);

  switch (etat)
  {
    case Acceleration:
      
      if (!pid_.isAtGoal())
      {

        // Serial.println("Acceleration");
        pid_.enable();
        pid_.setGoal(150);

        if (!pid_.isAtGoal())
        {
          // Serial.println("Not at goal");
          pid_.run();
        }
        else
        {
          etat = Stabilisation;
          AX_.setMotorPWM(0, 0);
        }

      }
      else
      {
        etat = Stabilisation;
        pid_.disable();
        AX_.setMotorPWM(0, 0);
      }

      break;

    case Stabilisation:

      Serial.println(PIDmeasurement());
      break;
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  // mise à jour du PID
  // pid_.run();

  //electromagnet_on(MAGPIN);
  //delay(5000);  
  //avancer();

 /* delay(1000);
  stop();
  electromagnet_off(MAGPIN);
  delay(3000);
  reculer();
  delay(1000);
  stop();
  delay(3000);*/
  
  
  
 // Serial.print();
  // Activer/desactiver electro-aimant
  /* test */
 /* electromagnet_on(MAGPIN);
  delay(5000);
  electromagnet_off(MAGPIN);
  delay(5000);*/
}

/*---------------------------Definition de fonctions ------------------------*/

void avancer ()
{
 // int a = digitalRead(LED_BUILTIN) ? 0 : 1 ;
 // digitalWrite(LED_BUILTIN, a);
  for(pulsePWM_ = 0; pulsePWM_<1;pulsePWM_+= 0.2)
  {
    AX_.setMotorPWM(0, pulsePWM_);
    delay(3000);
  }
  
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

void detectswitch()
{
  limitSwitch.loop(); // MUST call the loop() function first

  if(limitSwitch.isPressed()){
    // Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
  }

  if(limitSwitch.isReleased()){
    // Serial.println("The limit switch: TOUCHED -> UNTOUCHED");
  }
  int state = limitSwitch.getState();
  if(state == HIGH){
	  // Serial.println("The limit switch: UNTOUCHED");
  }
  else{
	  // Serial.println("The limit switch: TOUCHED");
  }
}

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
 // AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
 // AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

 /* doc["time"] = millis();
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
  doc["Encodeur"] = AX_.readEncoder(0);
  doc["PuissanceConsommee"] = AX_.getVoltage() * AX_.getCurrent();
  */

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
double PIDmeasurement() {
  // To do
  return -((AX_.readEncoder(MOTEUR) * DIAMETRE_ROUE * PI) / 1216);
}
void PIDcommand(double cmd) {
  // To do
  AX_.setMotorPWM(0, -cmd);
  Serial.println(cmd);
}
void PIDgoalReached() {
  // To do
}