const int analogPin = A0; // Change to your desired analog input pin
const int numReadings = 500;

void setup() {
  Serial.begin(9600);
}
int i(0);
int average(0);
int total(0);
int reading[numReadings]={0};

void loop() {

    reading[i] = analogRead(analogPin);
    total += reading;
    i++;

  if(i==(numReadings-1)){
    i=0;
  }

  average = averageIntArray(reading,numReadings);
  Serial.print("Average reading: ");
  Serial.println(average-15);

}


float averageIntArray(int* array, int length) {
  if (length <= 0) return 0;

  long sum = 0;
  for (int i = 0; i < length; i++) {
    sum += array[i];
  }

  return static_cast<float>(sum) / length;
}
