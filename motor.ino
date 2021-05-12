unsigned int motorPin[] = { 6, 10, 9, 5 };  
// 모터 연결 번호 및 속도제어 코딩 마지막단계


void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial1.begin(115200);
}

uint8_t cnt_msg = 0;

void loop() {
  if(Serial1.available() > 0) {
    while(Serial1.available() > 0) {
      uint8_t msp_data = Serial1.read();
      if(msp_data == '$') cnt_msg = 0;
      else                cnt_msg++;
      
      if(cnt_msg == 8) {
        Serial.print(" | T = "); Serial.println(msp_data);
        for(int i = 0; i < 4; i++)
          analogWrite(motorPin[i], msp_data/2);
      }
      else;
      
    }
  }
}
