//Code d'implémentation préliminaire : 
//Par Éliott Desbordes-Boom et Nathan Levasseur
#include <PID_v1.h>

// Pins
const int potConsigne = A1;    	// Potentiomètre de consigne
const int potPos = A0;    	// Potentiomètre de position angulaire
const int pwmPin = 3;        	 // PWM pour vitesse (ENA sur L298N)
const int dir1Pin = 4;        	 // Direction 1 (IN1)
const int dir2Pin = 5;         	// Direction 2 (IN2)

// Variables PID
double Setpoint = 0;
double Input = 0;
double Output = 0;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;  // Gains à tuner 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  
  // Configuration pins moteur
  pinMode(pwmPin, OUTPUT);
  pinMode(dir1Pin, OUTPUT);
  pinMode(dir2Pin, OUTPUT);
  digitalWrite(dir1Pin, LOW);
  digitalWrite(dir2Pin, LOW);
  analogWrite(pwmPin, 0);
  
  Serial.println("Contrôle ouvert de vitesse/direction activé. Position affichée en continu.");
}

void loop() {
  // Lecture des potentiomètres (0-1023)
  int consigne = analogRead(potConsigne);
  int position = analogRead(potPos);
  
  // Affichage de la position angulaire sur le port série
  Serial.println(consigne);
  
  // Contrôle vitesse et direction basé sur consigne
  int vitesse = 0;
  if (consigne <= 512) {
    // Sens 1 : 0 = pleine vitesse (255), 512 = arrêté (0)
    vitesse = map(consigne, 0, 512, 255, 0);
    //vitesse = consigne >> 1;
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
  } 
  else if (consigne >= 513) {
    // Sens 2 : 513 = arrêté (0), 1023 = pleine vitesse (255)
    vitesse = map(consigne, 513, 1023, 0, 255);
    //vitesse = (consigne - 512) >> 1;
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
  } 
  else {
    vitesse = 0;
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, LOW);
  }
  
  // Application de la vitesse PWM
  analogWrite(pwmPin, vitesse);
  
  delay(50);  // Délai pour fluidité
}