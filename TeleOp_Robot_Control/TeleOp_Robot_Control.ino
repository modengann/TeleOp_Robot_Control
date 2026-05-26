#include "WebHelper.h" //Keep, do not MODIFY!

void setup() {
  Serial.begin(9600);
  setupOTA();
  setupWebServer();
  
}

void loop() {  // put your main code here, to run repeatedly:
  handleWebServer(); //Keep, do not MODIFY!

  if(keyboard.a && keyboard.s){
    Serial.println("a and s!");
  }

  if(keyboard.a){
    Serial.println("a!");
  }
  
  delay(10);

}
