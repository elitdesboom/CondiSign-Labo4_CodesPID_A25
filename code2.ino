//Code d'implémentations finale : 
//Par Éliott Desbordes-Boom et Nathan Levasseur
 
#include <PID_v1.h>

const int potCons = A1;    // Potentiomètre de consigne (Setpoint)
const int potPos = A0;    // Potentiomètre de position angulaire (Input)
const int pwmPin = 3;          // PWM pour vitesse (ENA sur L298N)
const int dir1Pin = 4;         // Direction 1 (IN1)
const int dir2Pin = 5;         // Direction 2 (IN2)

// Variables PID
double Setpoint = 0;
double Input = 0;
double Output = 0;

// Gains initiaux à 0
double Kp = 0.0;  
double Ki = 0.0;
double Kd = 0.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Buffer pour tuning via série
String buffer = "";
bool emergencyStop = false;

void setup() {
  Serial.begin(115200);
  
  // Configuration PID
  myPID.SetOutputLimits(-255, 255);  // Limites pour signe (direction) et valeur absolue (vitesse)
  myPID.SetMode(AUTOMATIC);          // Mode automatique
  myPID.SetSampleTime(2);            // Échantillonnage à 2 ms
  
  // Configuration pins moteur
  pinMode(pwmPin, OUTPUT);
  pinMode(dir1Pin, OUTPUT);
  pinMode(dir2Pin, OUTPUT);
  digitalWrite(dir1Pin, LOW);
  digitalWrite(dir2Pin, LOW);
  analogWrite(pwmPin, 0);
  stopMotor();
  
  Serial.println("PID initialise. Envoyez des commandes comme '1.5p' pour changer Kp.");
}

void loop() {
  // Lecture des potentiomètres
  Setpoint = analogRead(potCons);  // Mise à jour Setpoint liée au pot consigne
  Input = analogRead(potPos);     // Input pour PID
  
  // Calcul PID
  myPID.Compute();
  
  // Contrôle moteur basé sur Output
  int vitesse = abs((int)Output);  // Valeur absolue pour PWM
  if (Output > 0) 
  {
    // Sens 1 (horaire)
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
    analogWrite(pwmPin, vitesse);
  } 
  else if (Output < 0)
  {
    // Sens 2 (anti-horaire)
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    analogWrite(pwmPin, vitesse);
  } 
  else 
  {
    // Arrêt
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
  
  // Gestion du tuning via port série
  while (Serial.available() > 0) 
  {
    char c = Serial.read();

     // Arrêt d’urgence : espace seul (sans \n ni \r)
    if (c == ' ') 
    {
      emergencyStop = true;
      stopMotorEmergency();
      Serial.println("=== ARRET D'URGENCE ACTIVE ===");
      buffer = "";                 
      continue;                   
    }

    if (isdigit(c) || c == '.' || c == '-') {
      buffer += c;  // Accumule seulement chiffres, point et signe négatif
    } else if (c == 'p' || c == 'i' || c == 'd') {
      double val = atof(buffer.c_str());  // Conversion en double
      if (c == 'p') {
        Kp = val;
        Serial.print("Nouveau Kp: ");
        Serial.println(Kp, 2);  // Affichage avec 2 décimales
      } else if (c == 'i') {
        Ki = val;
        Serial.print("Nouveau Ki: ");
        Serial.println(Ki, 2);
      } else if (c == 'd') {
        Kd = val;
        Serial.print("Nouveau Kd: ");
        Serial.println(Kd, 2);
      }
      myPID.SetTunings(Kp, Ki, Kd);  // Mise à jour des gains
      buffer = "";  // Reset buffer
    }  
  }

  // --- Si arrêt d’urgence, on ne fait rien d’autre ---
  if (emergencyStop)
  {
    delay(2);
    return;
  }
  
  delay(2); 
}

// --- Fonctions utilitaires ---
void stopMotor() {
  digitalWrite(dir1Pin, LOW);
  digitalWrite(dir2Pin, LOW);
  analogWrite(pwmPin, 0);
}

void stopMotorEmergency() {
  myPID.SetMode(MANUAL);   // Désactive le calcul PID
  Output = 0;
  stopMotor();
}