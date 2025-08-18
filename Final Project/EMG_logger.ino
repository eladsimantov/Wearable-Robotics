void setup() {
  Serial1.begin(115200);
}
int emgSig[3] = {0, 0, 0};

void loop() {
    // Unpack Raw EMG Signals
    emgSig[0] = analogRead(A0);
    emgSig[1] = analogRead(A1);
    emgSig[2] = analogRead(A4);
    
    Serial1.print("rawA0:");
    Serial1.print(emgSig[0]);
    Serial1.print(",");
    Serial1.print("rawA1:");
    Serial1.print(emgSig[1]);
    Serial1.print(",");
    Serial1.print("rawA4:");
    Serial1.print(emgSig[2]);

    Serial1.print("\n"); // New line for next print
    delay(20);
}
