
// KeyaCANBUS
// Trying to get Keya to steer the tractor over CANBUS

#define lowByte(w) ((uint8_t)((w)&0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

#if isallnavy == 1
uint64_t KeyaPGN = 0x06c73001;
#else
uint64_t KeyaPGN = 0x06000001;
#endif




//Enable	0x23 0x0D 0x20 0x01 0x00 0x00 0x00 0x00
//Disable	0x23 0x0C 0x20 0x01 0x00 0x00 0x00 0x00
//Fast clockwise	0x23 0x00 0x20 0x01 0xFC 0x18 0xFF 0xFF (0xfc18 signed dec is - 1000
//Anti - clockwise	0x23 0x00 0x20 0x01 0x03 0xE8 0x00 0x00 (0x03e8 signed dec is 1000
//Slow clockwise	0x23 0x00 0x20 0x01 0xFE 0x0C 0xFF 0xFF (0xfe0c signed dec is - 500)
//Slow anti - clockwise	0x23 0x00 0x20 0x01 0x01 0xf4 0x00 0x00 (0x01f4 signed dec is 500)

uint8_t KeyaSteerPGN[] = { 0x23, 0x00, 0x20, 0x01, 0, 0, 0, 0 };  // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = {
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
};

// templates for matching responses of interest
uint8_t keyaCurrentResponse[] = { 0x60, 0x12, 0x21, 0x01 };
int keyasensorSample = 0;




const bool debugKeya = true;

void keyaSend(uint8_t data[]) {
  //TODO Use this optimisation function once we're happy things are moving the right way
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  memcpy(KeyaBusSendData.buf, data, sizeof(data));
  Keya_Bus.write(KeyaBusSendData);
}

void CAN_Setup() {
  Keya_Bus.begin();

  if (isallnavy) {
    Keya_Bus.setBaudRate(500000);
    Serial.println("Set Baud to 500000 for allynav");
     Serial.print("PGN is set to : ");
     Serial.println(KeyaPGN,HEX);

     CAN_message_t msg;
     
msg.flags.extended = 1;
msg.id = 0x06c73001;
msg.len = 8;
msg.buf[0] = 0x23;
msg.buf[1] = 0x0d;
msg.buf[2] = 0x20;
msg.buf[3] = 0x01;
msg.buf[4] = 0;
msg.buf[5] = 0;
msg.buf[6] = 0;
msg.buf[7] = 0;

 


setally(90,500);
setally(91,1600);
setally(92,500);
setally(93,1000);

setally(100,400);
setally(101,400);
setally(102,300);
setally(103,225);
setally(104,400);
setally(105,400);

setally(256,3000);
setally(252,180);


     //save all
    msg.flags.extended = 1;
    msg.id = 0x06c73001;
    msg.len = 8;
    msg.buf[0] = 0x24;
    msg.buf[1] = 0x01;
    msg.buf[2] = 0x32;
    msg.buf[3] = 0x02;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x02;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Keya_Bus.write(msg);
    delay(10);


   
   
   
  } else {
    Keya_Bus.setBaudRate(250000);

    Serial.println("Set Baud to 250000 for keya");
    Serial.println("PGN is set to : ");
     Serial.println(KeyaPGN,HEX);
   
    
  
  }

  // Dedicated bus, zero chat from others. No need for filters
  //	CAN_message_t msgV;
  //	msgV.id = KeyaPGN;
  //	msgV.flags.extended = true;
  //	msgV.len = 8;
  //	// claim an address. Don't think I need to do this tho
  //	// anyway, just pinched this from Claas address. TODO, looks like we can do without, ditch this
  //	msgV.buf[0] = 0x00;
  //	msgV.buf[1] = 0x00;
  //	msgV.buf[2] = 0xC0;
  //	msgV.buf[3] = 0x0C;
  //	msgV.buf[4] = 0x00;
  //	msgV.buf[5] = 0x17;
  //	msgV.buf[6] = 0x02;
  //	msgV.buf[7] = 0x20;
  //	Keya_Bus.write(msgV);
  delay(1000);
  if (debugKeya) Serial.println("Initialised Keya CANBUS");
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
  return memcmp(message.buf, pattern, patternSize) == 0;
}


void setally(int par, int value)
{
   CAN_message_t msg; 
  
    msg.flags.extended = 1;
    msg.id = 0x06c73001;
    msg.len = 8;
    msg.buf[0] = 0x24;
    msg.buf[1] = highByte(par);
    msg.buf[2] = lowByte(par);
    msg.buf[3] = 0x02;
    msg.buf[4] = highByte(value);
    msg.buf[5] = lowByte(value);
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Keya_Bus.write(msg);
    delay(10);

}

void disableKeyaSteer() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0c;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  //if (debugKeya) Serial.println("Disabled Keya motor");
}

void disableKeyaSteerTEST() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x03;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x11;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  //if (debugKeya) Serial.println("Disabled Keya motor");
}

void enableKeyaSteer() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  //if (debugKeya) Serial.println("Enabled Keya motor");
}

void SteerKeya(int steerSpeed) {

  
 // Wandelt den PWM-Wert in die Motor-Geschwindigkeit um
  int actualSpeed = map(steerSpeed, -255, 255, -900, 900);


  if (debugKeya) Serial.println("told to steer, with " + String(steerSpeed) + " so....");
  if (debugKeya) Serial.println("   I converted that to speed " + String(actualSpeed));


   // CAN-Nachricht vorbereiten
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x00;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;

  // Geschwindigkeits- und Richtungsinformationen eintragen
  if (actualSpeed < 0) {
    KeyaBusSendData.buf[4] = highByte(actualSpeed);
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0xff;
    KeyaBusSendData.buf[7] = 0xff;
  } else {
    KeyaBusSendData.buf[4] = highByte(actualSpeed);
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0x00;
    KeyaBusSendData.buf[7] = 0x00;
  }

  // Die Nachricht senden. Keine Aktivierungs-/Deaktivierungs-Logik mehr hier!
  Keya_Bus.write(KeyaBusSendData);
}


void KeyaBus_Receive() {
  CAN_message_t KeyaBusReceiveData;
  if (Keya_Bus.read(KeyaBusReceiveData)) {
    // parse the different message types
    // heartbeat 0x07000001
    // change heartbeat time in the software, default is 20ms
    if (KeyaBusReceiveData.id == 0x4854001 && isallnavy == true) {
      // 0-1 - Cumulative value of angle (360 def / circle)
      // 2-3 - Motor speed, signed int eg -500 or 500
      // 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
      //		is that accurate enough for us?
      // 6-7 - Control_Close (error code)
      // TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
      //KeyaCurrentSensorReading = abs((int16_t)((KeyaBusReceiveData.buf[5] << 8) | KeyaBusReceiveData.buf[4]));
      //if (KeyaCurrentSensorReading > 255) KeyaCurrentSensorReading -= 255;

      keyasensorSample = KeyaBusReceiveData.buf[0] * 256 + KeyaBusReceiveData.buf[1];

      KeyaCurrentSensorReading = KeyaCurrentSensorReading * 0.6 + keyasensorSample * 0.4;

    //  if (debugKeya) Serial.println("Heartbeat current is " + String(KeyaCurrentSensorReading));

      int16_t targets = KeyaBusReceiveData.buf[4] * 256 + KeyaBusReceiveData.buf[5];

      //if (debugKeya) Serial.println("Target speed current is " + String(targets));

      int16_t actuals = KeyaBusReceiveData.buf[2] * 256 + KeyaBusReceiveData.buf[3];

     // if (debugKeya) Serial.println("    actual speed current is " + String(actuals));
    }
    if (KeyaBusReceiveData.id == 0x07000001 && isallnavy == false) {
      // 0-1 - Cumulative value of angle (360 def / circle)
      // 2-3 - Motor speed, signed int eg -500 or 500
      // 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
      //		is that accurate enough for us?
      // 6-7 - Control_Close (error code)
      // TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
      //KeyaCurrentSensorReading = abs((int16_t)((KeyaBusReceiveData.buf[5] << 8) | KeyaBusReceiveData.buf[4]));
      //if (KeyaCurrentSensorReading > 255) KeyaCurrentSensorReading -= 255;
      if (KeyaBusReceiveData.buf[4] == 0xFF) {
        KeyaCurrentSensorReading = (256 - KeyaBusReceiveData.buf[5]) * 20;
      } else {
        KeyaCurrentSensorReading = KeyaBusReceiveData.buf[5] * 20;
      }
      //if (debugKeya) Serial.println("Heartbeat current is " + String(KeyaCurrentSensorReading));
    }

    // response from most commands 0x05800001
    // could have been separate PGNs, but oh no...

    //if (KeyaBusReceiveData.id == 0x05800001) {
    //	// response to current request (this is also in heartbeat)
    //	if (isPatternMatch(KeyaBusReceiveData, keyaCurrentResponse, sizeof(keyaCurrentResponse))) {
    //		// Current is unsigned float in [4]
    //		// set the motor current variable, when you find out what that is
    //		KeyaCurrentSensorReading = KeyaBusReceiveData.buf[4];
    //		if (debugKeya) Serial.println("Returned current is " + KeyaCurrentSensorReading);
    //	}
    //	else if (1 == 0) {
    //		// placeholder for more checks
    //	}
    //}
  }
}
