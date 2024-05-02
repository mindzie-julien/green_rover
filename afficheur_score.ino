#include "SevSeg.h"    // La librairie 7 segments
SevSeg sept_segments;  // On appel la librairie

int score = 0;
int increment_state = 0;
int decrement_state = 0;
const int increment_pin = A0;
const int decrement_pin = A1;

void setup() {

  byte nombre_chiffre_7_segments = 4;  // Nombre de digit de votre 7 segments
  // Les broches de votre 7 segments
  byte Broche_pins[] = { 10, 11, 12, 13 };
  byte segmentPins[] = { 9, 2, 3, 5, 6, 8, 7, 4 };

  bool resistance_sur_Segments = true;  // On applique une résistance sur chaque segment
  bool pause_entre_mise_a_jour = true;
  // On fait une mini pause entre chaque segment
  byte configuration_materiel = COMMON_CATHODE;                                                                               // On fait un 7 segments a cathode commune
  sept_segments.begin(configuration_materiel, nombre_chiffre_7_segments, Broche_pins, segmentPins, resistance_sur_Segments);  // On initialize l'afficheur 7 segments
  sept_segments.setBrightness(1000);                                                                                          // On initialize la luminosité de l'afficheur


  pinMode(increment_pin, INPUT);
  pinMode(decrement_pin, INPUT);
}

int change_score_value() {

  return score;
}

void loop() {
  sept_segments.setNumber(2);  // On assigne le nombre 16 à l'afficheur 7 segments
  increment_state = digitalRead(increment_pin);
  decrement_state = digitalRead(decrement_pin);

  if (increment_state == HIGH) {
    score++;
  }

  if (decrement_state == HIGH) {
    score--;
  }

  char output[10];  // Array to hold the final output string

  if ()
  // Convert the integer to a string and append " P" to indicate "points"
  snprintf(output, sizeof(output), "%d P", score);  // Use snprintf to format the output string

  sept_segments.setChars(output);
  // sept_segments.
  sept_segments.refreshDisplay();  // On rafraichit l'affichage pour que le nouveau nombre soit pris en compte
  delay(1000);

}
