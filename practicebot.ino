//nrc file for practicing with the test bot
char D3 = 3;
char D4 = 4;
char D6 = 6;
char D7 = 7;

void setup() {
  // put your setup code here, to run once:
  //Setup Motor Controls as output
  
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Right now this program stops for a quarter second, then moves foward for a quarter second then repeats
  digitalWrite(D3, LOW);
  digitalWrite(D4, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  delay(250);
  digitalWrite(D3, HIGH);
  digitalWrite(D6, HIGH);
  delay(250);
}
