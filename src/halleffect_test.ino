#define HALL_PIN A3
#define THRESHOLD 200

int baseline = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Calibrating... (remove magnet)");
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(HALL_PIN);
    delay(20);
  }
  baseline = sum / 50;
  Serial.print("Baseline: ");
  Serial.println(baseline);
}

void loop() {
  int val = analogRead(HALL_PIN);
  int diff = val - baseline;
  int absDiff = abs(diff);  // 절대값만 사용
  
  Serial.print("Value: ");
  Serial.print(val);
  Serial.print(" / Diff: ");
  Serial.print(diff);
  Serial.print(" / |Diff|: ");
  Serial.print(absDiff);
  Serial.print(" / Status: ");
  
  if (absDiff > THRESHOLD) {
    Serial.println("PIECE DETECTED");
  } else {
    Serial.println("Empty");
  }
  
  delay(200);
}