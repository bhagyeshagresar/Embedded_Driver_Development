/*
 This script prints the data received on the rx line of the ESP32 board
*/


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  // Define the LED pin as Output
  pinMode (2, OUTPUT);
  
  Serial.println("Arduino UART Receiver");
  Serial.println("-----------------------------");
    
}


void loop() {

  digitalWrite(2, LOW); 
  //wait until something is received
  while(! Serial1.available());
  digitalWrite(2, HIGH); 
  //read the data
  Serial.print("Byte available: ");
  Serial.println(Serial1.available());
  char in_read=Serial1.read();
  //print the data
  Serial.print(in_read);

}