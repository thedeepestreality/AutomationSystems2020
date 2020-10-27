void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}
byte buf[] = {48, 50};
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '1')
      digitalWrite(13,HIGH);
    else if (c == '0')
      digitalWrite(13,LOW);
  }
  Serial.write(buf,2);
  delay(500);
}
