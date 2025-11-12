//Code comprenant toutes les implémentations demandées dans le laboratoire 4 - Asservissement de position angulaire
//Par Éliott Desbordes-Boom et Nathan Levasseur 

#include <PID_v1.h>

// Pins 
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
  Setpoint = analogRead(potCons);
  Input = analogRead(potPos);

  // --- Gestion série (tuning + arrêt + reprise) ---
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Arrêt d’urgence : espace
    if (c == ' ') {
      emergencyStop = true;
      stopMotorEmergency();
      Serial.println("=== ARRET D'URGENCE ACTIVE ===");
      Serial.println("Appuyez sur ENTRÉE ou envoyez 'r' pour relancer.");
      buffer = "";
      continue;
    }

    // Relancer après arrêt d’urgence : 'r', 'R' ou '\n' (Entrée)
    if (emergencyStop && (c == 'r' || c == 'R' || c == '\n' || c == '\r')) {
      emergencyStop = false;
      myPID.SetMode(AUTOMATIC);  // Réactive le PID
      Output = 0;
      stopMotor();  // Assure l'arrêt propre avant reprise
      Serial.println("=== REPRISE DU SYSTEME ===");
      buffer = "";
      continue;
    }

    // Si arrêt d’urgence, ignorer les autres commandes sauf relance
    if (emergencyStop) {
      if (c == '\n' || c == '\r') buffer = "";  // Nettoie buffer sur nouvelle ligne
      continue;
    }

    // Tuning normal (Kp, Ki, Kd)
    if (isdigit(c) || c == '.' || c == '-') {
      buffer += c;
    } else if (c == 'p' || c == 'i' || c == 'd') {
      double val = atof(buffer.c_str());
      if (c == 'p') {
        Kp = val;
        Serial.print("Nouveau Kp: ");
        Serial.println(Kp, 2);
      } else if (c == 'i') {
        Ki = val;
        Serial.print("Nouveau Ki: ");
        Serial.println(Ki, 2);
      } else if (c == 'd') {
        Kd = val;
        Serial.print("Nouveau Kd: ");
        Serial.println(Kd, 2);
      }
      myPID.SetTunings(Kp, Ki, Kd);
      buffer = "";
    } else if (c == '\n' || c == '\r') {
      buffer = "";  // Nettoie buffer sur nouvelle ligne
    }
  }

  // --- Si arrêt d’urgence, on attend la relance ---
  if (emergencyStop) {
    stopMotor();  // Sécurité : on force l’arrêt à chaque tour
    delay(2);
    return;
  }

  // --- Calcul PID normal ---
  myPID.Compute();

  // Contrôle moteur
  int vitesse = abs((int)Output);
  if (Output > 0) {
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
    analogWrite(pwmPin, vitesse);
  } 
  else if (Output < 0) {
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    analogWrite(pwmPin, vitesse);
  } 
  else {
    stopMotor();
  }

  delay(2);
}

// --- Fonctions utilitaires  ---
void stopMotor() {
  digitalWrite(dir1Pin, LOW);
  digitalWrite(dir2Pin, LOW);
  analogWrite(pwmPin, 0);
}

void stopMotorEmergency() {
  myPID.SetMode(MANUAL);
  Output = 0;
  stopMotor();
}
