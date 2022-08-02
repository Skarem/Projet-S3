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

#define PANIER          70.0        //Positon du panier en x
#define SWITCH_PIN      3           //Pin de la microswitch

ezButton limitSwitch(SWITCH_PIN);  // create ezButton object that attach to pin 7;
/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu

PID pidPosition_;                   // objets PID
PID pidPendule_;
PID pidLent_;


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

enum Etats { AccrocherSapin, Acceleration, Stabilisation, Drop, ReculerVite, ReculerLent };
enum Etats etat;

enum Direction { Avancer, Reculer };
enum Direction dir;


int const MOTEUR = 0;
double DIAMETRE_ROUE = 6.4;

bool activePID = false;
bool activePIDPendule = false;
bool activePIDLent = false;
bool flagAvancerInit = false;

float cmd_pos = 0;
float cmd_pen = 0;

bool switchPressed = false;

//Variable/constante_ish d'init du PID
float pidPosG1 = 0.025;
float pidPosG2 = 0.001;
float pidPosG3 = 0.0025;
float pidPosEpsilon = 5;
float pidPosGoal = PANIER;


/*------------------------- Prototypes de fonctions -------------------------*/

double lirePotentiometre( /*uint8_t pin*/ );
void electromagnet_on(uint8_t pin);
void electromagnet_off(uint8_t pin);
void detectswitch();

void avancer(PID *pid, double goal);
void reculer(PID *pid);

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
void PIDinit();

// Fonctions pour le PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();

double PIDmeasurementPendule();
void PIDcommandPendule(double cmd);

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

  etat = AccrocherSapin;

  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds

  Serial.println("Setup Done");
}

const unsigned long TIME_SAPIN = 3000;
unsigned long timer_ = 0;
bool timerFlag_ = false;
bool flag_PID_pos = false;

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

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  // mise à jour du PID
  
  switch(etat)
  {

    case AccrocherSapin:
      electromagnet_on(MAGPIN);
      AX_.resetEncoder(0);

      if (!timerFlag_)
      {
        //while(digitalRead(SWITCH_PIN));
        timer_ = millis();
        timerFlag_ = true;
      }
      else if (millis() - timer_ >= TIME_SAPIN)
      { 
        timerFlag_ = false;
        etat = Acceleration;
      }
      break;
    

    case Acceleration:
      if (!flagAvancerInit)
      {
        pidPosition_ = PID();
        flagAvancerInit = true;
        flag_PID_pos = true;
      }

      avancer(&pidPosition_, PANIER);

      if (pidPosition_.isAtGoal())
      {
        flagAvancerInit = false;
        activePID = false;
        AX_.setMotorPWM(0, 0);
        etat = Stabilisation;
        flag_PID_pos = false;
        pidPosition_.~PID();
      }

      break;
    

    case Stabilisation:
     
      if (!activePIDPendule)
      {
        pidPendule_ = PID();
          pidPendule_.setGains(0.01, 0.001, 0.001);
          pidPendule_.setMeasurementFunc(PIDmeasurementPendule);
          pidPendule_.setCommandFunc(PIDcommandPendule);
          pidPendule_.setEpsilon(4);
          pidPendule_.setPeriod(50);
          pidPendule_.setTimeGoal(500);

        pidPosition_ = PID();
          pidPosition_.setGains(pidPosG1, pidPosG2, pidPosG3);
          pidPosition_.setMeasurementFunc(PIDmeasurement);
          pidPosition_.setCommandFunc(PIDcommand);
          pidPosition_.setEpsilon(pidPosEpsilon);
          pidPosition_.setPeriod(50);
          pidPosition_.setTimeGoal(500);

        pidPendule_.enable();
        pidPosition_.enable();

        pidPendule_.setGoal(0);
        pidPosition_.setGoal(pidPosGoal);

        pidPosition_.disable();
        activePIDPendule = true;
      }

      if (!pidPendule_.isAtGoal() && !pidPosition_.isAtGoal())
      {
        pidPendule_.run();
        pidPosition_.run();

        Serial.print("pid pendule : ");
        Serial.println(pidPendule_.isAtGoal());
        Serial.print("pid Position : ");
        Serial.println(pidPosition_.isAtGoal());
        Serial.print("lirePotentiometre() : ");
        Serial.println(lirePotentiometre());
      }
      else
      {
        AX_.setMotorPWM(0, 0);
        pidPendule_.~PID();
        pidPosition_.~PID();
        activePIDPendule = false;
        etat = Drop;
      }
      break;
    

    case Drop:
      
      if (!timerFlag_)
      {
        electromagnet_off(MAGPIN);
        timer_ = millis();
        timerFlag_ = true;
      }
      else if (millis() - timer_ >= 250)
      { 
        timerFlag_ = false;
        etat = ReculerVite;
      }
      break;
    

    case ReculerVite:
    
      Serial.println("Reculer Vite");

      while((AX_.readEncoder(MOTEUR) * DIAMETRE_ROUE * PI) / 1216 < 15)
        AX_.setMotorPWM(0, 3);

      while((AX_.readEncoder(MOTEUR) * DIAMETRE_ROUE * PI) / 1216 < 7)
        AX_.setMotorPWM(0, 0.6);
      
      etat = ReculerLent;
      break;
    

    case ReculerLent:
    
      Serial.println("Reculer Lent");
      while (digitalRead(SWITCH_PIN))
      {
        AX_.setMotorPWM(0, 0.2);
      }
      AX_.setMotorPWM(0, 0);
      etat = AccrocherSapin;
      break;
    
  }
}

/*--------------------------- Definition de fonctions ------------------------*/

void avancer(PID *pid, double goal)
{
  if (!activePID)
  {
    pid->setGains(pidPosG1, pidPosG2, pidPosG3);
    pid->setMeasurementFunc(PIDmeasurement);
    pid->setCommandFunc(PIDcommand);
    pid->setEpsilon(pidPosEpsilon);
    pid->setPeriod(50);
    pid->setTimeGoal(0);

    pid->setGoal(goal);
    pid->enable();
    activePID = true;
    dir = Avancer;
  }
  else
  {
    pid->run();
  }
}

double posReculer;

void reculer(PID *pid)
{
  if (!activePID)
  {
    posReculer = (AX_.readEncoder(MOTEUR) * DIAMETRE_ROUE * PI) / 1216;
    pid->setGoal(0.5 * posReculer);
    pid->enable();
    activePID = true;
    dir = Reculer;
  }
  else
  {
    pid->run();
  }
}

void electromagnet_on(uint8_t pin)
{
digitalWrite(pin, HIGH); // Activation electroAimant

}

void electromagnet_off(uint8_t pin)
{
  digitalWrite(pin, LOW); // Desactivation electroAimant
}

double angle;

double lirePotentiometre(/*uint8_t pin*/)
{
  return (((analogRead(POTPIN) - 336) * 212.5) / 1023) + 1;
}

void detectswitch()
{
  limitSwitch.loop(); // MUST call the loop() function first

  if(limitSwitch.isPressed()){
    // Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
    switchPressed = true;
  }

  if(limitSwitch.isReleased()){
    // Serial.println("The limit switch: TOUCHED -> UNTOUCHED");
    switchPressed = false;
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
  doc["position"] = -((AX_.readEncoder(MOTEUR) * DIAMETRE_ROUE * PI) / 1216);
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
    pidPosG1 = doc["setGoal"][0];
    pidPosG2 = doc["setGoal"][1];
    pidPosG3 = doc["setGoal"][2];
    pidPosEpsilon = doc["setGoal"][3];
    pidPosGoal = doc["setGoal"][4];
  }
}

// Fonctions pour le PID

double PIDmeasurementPendule() 
{
  return (((analogRead(POTPIN) - 336) * 212.5) / 1023);
}

void PIDcommandPendule(double cmd) 
{
  // Serial.println("Moteur");
  cmd_pen = -cmd;
  AX_.setMotorPWM(0, 0.1*cmd_pos + 0.9*cmd_pen);
}

double PIDmeasurement() 
{
  if (dir == Avancer)
  {
    return -((AX_.readEncoder(MOTEUR) * DIAMETRE_ROUE * PI) / 1216);
  }
  else if (dir == Reculer)
  {
    return ((AX_.readEncoder(MOTEUR) * DIAMETRE_ROUE * PI) / 1216);
  }
}

void PIDcommand(double cmd) 
{
  if (dir == Avancer)
  {
    cmd_pos = -cmd;
    if (flag_PID_pos)
    {
      Serial.println("Commande envoye");
      AX_.setMotorPWM(0, -cmd);
    }
  }
  else if (dir == Reculer)
  {
    cmd_pos = cmd;
    if (flag_PID_pos)
    {
      Serial.println("Commande envoye");
      AX_.setMotorPWM(0, cmd);
    }
  }
  // Serial.println(cmd);
}

void PIDgoalReached() 
{
  // To do
}