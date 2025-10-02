bool gnsspassThrough = false;
bool Relay_Type = 0;  // set to 1 if using mcp23017 port expander
bool xbee = 0;
bool isKeya = true;
#define isallnavy  1 // 0 for keya // 1 for allnav motor


HardwareSerial* SerialXbee = &Serial2;  //xbee modul

String nmeainput;

#include <ADC.h>
#include <ADC_util.h>
const int readPin = A0;      // ADC0
ADC* adcteensy = new ADC();  // adc object;

#include <Adafruit_MCP23X17.h>
Adafruit_MCP23X17 mcp;
uint8_t error;


int8_t KeyaCurrentSensorReading = 0;
#include <FlexCAN_T4.h>

// Seems to work for CAN2, not sure why it didn't for CAN1
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> Keya_Bus;



/************************* User Settings *************************/
// Serial Ports
#define SerialAOG Serial               //AgIO USB conection
#define SerialRTK Serial3              //RTK radio
HardwareSerial* SerialGPS = &Serial7;  //Main postion receiver (GGA)
HardwareSerial* SerialIMU = &Serial5;  //IMU BNO-085

constexpr int serial_buffer_size = 512;

#define SERIAL_TX_BUFFER_SIZE 512
#define SERIAL_RX_BUFFER_SIZE 512

const int32_t baudGPS = 115200;
const int32_t baudRTK = 115200;  // most are using Xbee radios with default of 115200

#define ImuWire Wire1  //SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

//Status LED's
#define GGAReceivedLED 13         //Teensy onboard LED
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 12              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 9   //Green

/*****************************************************************/

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO_RVC.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

//Roomba Vac mode for BNO085 and data
BNO_rvc rvc = BNO_rvc();
BNO_rvcData bnoData;
elapsedMillis bnoTimer;
bool bnoTrigger = false;
bool useBNO08xRVC = false;

struct ConfigIP {
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 5;
};
ConfigIP networkAddress;  //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0 };  //This is now set via AgIO
byte mac[] = { 0x00, 0x00, 0x56, 0x00, 0x00, 0x78 };

unsigned int portMy = 5120;                       // port of this module
unsigned int AOGNtripPort = 2233;                 // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;             // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;              // Port of AOG that listens
char Eth_NTRIP_packetBuffer[serial_buffer_size];  // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;      //Out port 5544
EthernetUDP Eth_udpNtrip;      //In port 2233
EthernetUDP Eth_udpAutoSteer;  //In & Out Port 8888

IPAddress Eth_ipDestination;

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;  // Velocity (MPH speed) PWM pin

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency);  // required prototype

bool useDual = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

elapsedMillis GGAReadyTime = 10000;
elapsedMillis ethernetLinkCheck = 1000;

//Dual
double headingcorr = 900;  //90deg heading correction (90deg*10)

double baseline = 0;
double rollDual = 0;
double relPosD = 0;
double heading = 0;

byte ackPacket[72] = { 0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t GPSrxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];  //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];  //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];   //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true;  //Auto set off in autosteer setup

float roll = 0;
float pitch = 0;
float yaw = 0;

// Setup procedure ------------------------
void setup() {
  delay(500);                //Small delay so serial can monitor start up
  set_arm_clock(450000000);  //Set CPU speed to 150mhz
  Serial.print("CPU speed set to: ");
  Serial.println(F_CPU_ACTUAL);

  pinMode(GGAReceivedLED, OUTPUT);
  pinMode(Power_on_LED, OUTPUT);
  pinMode(Ethernet_Active_LED, OUTPUT);
  pinMode(GPSRED_LED, OUTPUT);
  pinMode(GPSGREEN_LED, OUTPUT);
  pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
  pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);

  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);

  delay(10);
  Serial.println("Start setup");

  SerialXbee->begin(baudGPS);
  delay(10);

  SerialGPS->begin(baudGPS);

  delay(10);
  SerialRTK.begin(baudRTK);



  Serial.println("SerialAOG, SerialRTK, SerialGPS and SerialGPS2 initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();

  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();



  Serial.println("here");

  SerialIMU->begin(115200);
  rvc.begin(SerialIMU);

  static elapsedMillis rvcBnoTimer = 0;
  Serial.println("\r\nChecking for serial BNO08x");
  while (rvcBnoTimer < 1000) {
    //check if new bnoData
    if (rvc.read(&bnoData)) {
      useBNO08xRVC = true;
      Serial.println("Serial BNO08x Good To Go :-)");
      imuHandler();
      break;
    }
  }
  if (!useBNO08xRVC) Serial.println("No Serial BNO08x not Connected or Found");

Serial.print("use bno:");
 Serial.println(useBNO08xRVC);


  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");

  // use adc from teensy for was

  pinMode(readPin, INPUT);
  adcteensy->adc0->setAveraging(16);   // set number of averages
  adcteensy->adc0->setResolution(16);  // set bits of resolution

  // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // see the documentation for more information
  // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adcteensy->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adcteensy->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);  // change the sampling speed

  //mcp port expander

  Serial.println("MCP check now....");

  if (Relay_Type == 1) {

    error = (mcp.begin_I2C(0X20, &Wire1));
    Serial.print("MCP Status:");
    Serial.println(error);
    if (error == 0) {
      Relay_Type = false;
    }

    delay(100);

    mcp.pinMode(8, OUTPUT);
    mcp.pinMode(9, OUTPUT);
    mcp.pinMode(10, OUTPUT);
    mcp.pinMode(11, OUTPUT);
    mcp.pinMode(12, OUTPUT);
    mcp.pinMode(13, OUTPUT);
    mcp.pinMode(14, OUTPUT);
    mcp.pinMode(15, OUTPUT);

    delay(100);
    mcp.digitalWrite(8, LOW);
    mcp.digitalWrite(9, LOW);
    mcp.digitalWrite(10, LOW);
    mcp.digitalWrite(11, LOW);
    mcp.digitalWrite(12, LOW);
    mcp.digitalWrite(13, LOW);
    mcp.digitalWrite(14, LOW);
    mcp.digitalWrite(15, LOW);
  }

  Serial.print("MCP Status done:");
  Serial.println(Relay_Type);

  if (isKeya) {
    Serial.println("Right... time for some CANBUS! And, we're dedicated to SteeringwheelMotor here");
    CAN_Setup();
  }
}

void loop() {

 if (isKeya) {
  KeyaBus_Receive();
 }
  // Read incoming nmea from GPS
  if (SerialGPS->available()) {

    if (gnsspassThrough) {
      char c = SerialGPS->read();
      nmeainput += c;
      if (c == '\n') {

        Eth_udpPAOGI.beginPacket(Eth_ipDestination, 9999);
        Eth_udpPAOGI.print(nmeainput);
        Eth_udpPAOGI.endPacket();
        nmeainput = "";
      }

    } else {
      char c = SerialGPS->read();
      parser << c;

      if (xbee) {
        SerialXbee->write(c);
      }
    }
  }
  //checkk xbee for data
  if (SerialXbee->available()) {
    char c = SerialXbee->read();  // Zeichen von Xbee lesen
    SerialGPS->write(c);          // Zeichen an GPS senden (Serial2)
  }


  // Check for RTK via Radio
  if (SerialRTK.available()) {
    SerialGPS->write(SerialRTK.read());
  }

  // Check for RTK via UDP
  unsigned int packetLength = Eth_udpNtrip.parsePacket();

  if (packetLength > 0) {
    if (packetLength > serial_buffer_size) packetLength = serial_buffer_size;
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
    SerialGPS->write(Eth_NTRIP_packetBuffer, packetLength);
  }

  // If both dual messages are ready, send to AgOpen
  if (dualReadyGGA == true && dualReadyRelPos == true) {
    BuildNmea();
    dualReadyGGA = false;
    dualReadyRelPos = false;
  }



  // Check the message when the buffer is full
  if (relposnedByteCount > 71) {
    if (calcChecksum()) {
      //if(deBug) Serial.println("RelPos Message Recived");
      digitalWrite(GPSRED_LED, LOW);  //Turn red GPS LED OFF (we are now in dual mode so green LED)
      useDual = true;
      relPosDecode();
    }
    relposnedByteCount = 0;
  }

if(!gnsspassThrough)
{
 //RVC BNO08x
  if (rvc.read(&bnoData))  useBNO08xRVC = true;
}
 

  if (useBNO08xRVC && bnoTimer > 70 && bnoTrigger) {
    bnoTrigger = false;
    imuHandler();  //Get IMU data ready
  }

  if (Autosteer_running) autosteerLoop();
  else ReceiveUdp();

  //GGA timeout, turn off GPS LED's etc
  if (GGAReadyTime > 10000)  //GGA age over 10sec
  {
    digitalWrite(GPSRED_LED, LOW);
    digitalWrite(GPSGREEN_LED, LOW);
    useDual = false;
  }

  if (ethernetLinkCheck > 10000) {
    if (Ethernet.linkStatus() == LinkON) {
      ethernetLinkCheck = 0;
      digitalWrite(Power_on_LED, 0);
      digitalWrite(Ethernet_Active_LED, 1);
    } else {
      digitalWrite(Power_on_LED, 1);
      digitalWrite(Ethernet_Active_LED, 0);
    }
  }

}  //End Loop
//**************************************************************************

void adc0_isr() {
  adcteensy->adc0->readSingle();
}



bool calcChecksum() {
  CK_A = 0;
  CK_B = 0;

  for (int i = 2; i < 70; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}
