
void setup() {
  pinMode(20, OUTPUT);
  pinMode(19, OUTPUT);
}

void loop() {

  int light = analogRead(A4);

  if (light < 500) {

    digitalWrite(20, HIGH);
    digitalWrite(19, HIGH);
  } else {
  
    digitalWrite(20, LOW);
    digitalWrite(19, LOW);
  }

  delay(100);
}
