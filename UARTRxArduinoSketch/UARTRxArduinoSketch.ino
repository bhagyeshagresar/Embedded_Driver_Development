/*
 This script prints the data received on the rx line of the ESP32 board
*/


void setup() {
  Serial.begin(115200);
  
  // Define the LED pin as Output
  pinMode (2, OUTPUT);
  
  Serial.println("Arduino UART Receiver");
  Serial.println("-----------------------------");
    
}


void loop() {

  digitalWrite(2, LOW); 
  //wait until something is received
  while(! Serial.available());
  digitalWrite(2, HIGH); 
  //read the data
  char in_read=Serial.read();
  //print the data
  Serial.print(in_read);

}