#include <Arduino.h>

// --- Piny sterownika silników (Sprawdź ze swoimi!) ---
const int PWMA = 25; const int AIN1 = 27; const int AIN2 = 26;
const int PWMB = 13; const int BIN1 = 14; const int BIN2 = 12;

// --- Piny enkoderów ---
const int ENC_A_LEFT = 32;  
const int ENC_A_RIGHT = 34; 

volatile long countLeft = 0;
volatile long countRight = 0;

void IRAM_ATTR countLeftInterrupt() { countLeft++; }
void IRAM_ATTR countRightInterrupt() { countRight++; }

// --- Zmienne i Regulatory PI ---
float targetLeft = 0.0;  // Cel od ROS 2 (impulsy na 50ms)
float targetRight = 0.0; // Cel od ROS 2 (impulsy na 50ms)

float Kp = 1.5; // Do ewentualnej poprawki
float Ki = 0.5; // Do ewentualnej poprawki

float integralLeft = 0;
float integralRight = 0;

unsigned long poprzedniCzas = 0;
const int OKRES_KOREKCJI = 50; 

// --- Zmienne na całkowitą pozycję (dla ROS 2 Odometrii) ---
long totalPositionLeft = 0;
long totalPositionRight = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  pinMode(ENC_A_LEFT, INPUT_PULLUP);
  pinMode(ENC_A_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT), countLeftInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT), countRightInterrupt, CHANGE);
}

void loop() {
  unsigned long obecnyCzas = millis();

  // 1. NASŁUCHIWANIE ROZKAZÓW Z RASPBERRY PI (Kabel USB)
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    
    // Jeśli wiadomość zaczyna się od "T," (Target)
    if (msg.startsWith("T,")) {
      // Wyciągamy dwie liczby z tekstu "T,10.0,15.5"
      sscanf(msg.c_str(), "T,%f,%f", &targetLeft, &targetRight);
    }
  }

  // 2. REGULACJA SILNIKÓW (Co 50ms)
  if (obecnyCzas - poprzedniCzas >= OKRES_KOREKCJI) {
    poprzedniCzas = obecnyCzas;

    // Zapisujemy, ile przejechaliśmy w te 50ms
    long currentCountLeft = countLeft;
    long currentCountRight = countRight;
    countLeft = 0; 
    countRight = 0;

    // Aktualizujemy globalną pozycję dla ROS 2
    totalPositionLeft += currentCountLeft;
    totalPositionRight += currentCountRight;

    // --- Lewe Koło (Regulator PI) ---
    float errorLeft = targetLeft - currentCountLeft;
    integralLeft += errorLeft;
    integralLeft = constrain(integralLeft, -500, 500); // Anti-windup
    int pwmLeft = (Kp * errorLeft) + (Ki * integralLeft);

    // --- Prawe Koło (Regulator PI) ---
    float errorRight = targetRight - currentCountRight;
    integralRight += errorRight;
    integralRight = constrain(integralRight, -500, 500); // Anti-windup
    int pwmRight = (Kp * errorRight) + (Ki * integralRight);

    // --- Sterowanie Fizyczne (Kierunek i Moc) ---
    // Lewe koło
    if (pwmLeft >= 0) {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); // Do przodu
    } else {
      digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); // Do tyłu
    }
    analogWrite(PWMA, constrain(abs(pwmLeft), 0, 255));

    // Prawe koło (pamiętaj, że miałeś je odwrócone!)
    if (pwmRight >= 0) {
      digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); // Do przodu
    } else {
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); // Do tyłu
    }
    analogWrite(PWMB, constrain(abs(pwmRight), 0, 255));

    // 3. WYSYŁANIE STATUSU DO RASPBERRY PI
    // Format: S,PozycjaLewa,PozycjaPrawa
    Serial.print("S,");
    Serial.print(totalPositionLeft);
    Serial.print(",");
    Serial.println(totalPositionRight);
  }
}