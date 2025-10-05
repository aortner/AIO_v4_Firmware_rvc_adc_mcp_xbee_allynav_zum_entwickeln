bool gnsspassThrough = false;  
bool useMCP23017 = true;       
bool isKeya = false;           
bool isallnavy = true;   

#define CRC32_POLYNOMIAL 0xEDB88320L

elapsedMillis imuDataTimer; // Timer zur Überwachung der IMU-Daten
const uint16_t IMU_TIMEOUT_MS = 1000; // 1 Sekunde Timeout

//HardwareSerial* SerialXbee = &Serial2;  //xbee modul

String nmeainput;

#include <ADC.h>
#include <ADC_util.h>
const int readPin = A0;      // ADC0
ADC* adcteensy = new ADC();  // adc object;

#include <Adafruit_MCP23X17.h>
Adafruit_MCP23X17 mcp;
uint8_t error;


uint8_t KeyaCurrentSensorReading = 0;
#include <FlexCAN_T4.h>

// Seems to work for CAN2, not sure why it didn't for CAN1
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> Keya_Bus;



/************************* User Settings *************************/
// Serial Ports
#define SerialAOG Serial               //AgIO USB conection
//#define SerialRTK Serial3              //RTK radio
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

#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 7   //Green
#define PCB_RELAY_1 8  //pcb relay
#define PCB_RELAY_2 9   // pcb relay




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


// ========== Am Anfang der Hauptdatei ==========

#define GNSS_BUFFER_SIZE 1024  // ✅ Verdoppelt auf 1024 für Sicherheit
char gnssPassThroughBuffer[GNSS_BUFFER_SIZE];
uint16_t gnssBufferIndex = 0;
bool gnssBufferOverflow = false;
bool gnssLastCharWasCR = false;  // ✅ NEU: CR-Tracking
elapsedMillis gnssBufferTimeout = 0;
#define GNSS_BUFFER_TIMEOUT_MS 500  // ✅ Kürzer: 500ms

uint32_t gnssPacketsSent = 0;
uint32_t gnssOverflowCount = 0;
uint32_t gnssTimeoutCount = 0;
uint32_t gnssCROnly = 0;      // ✅ NEU: Zähler für CR-only Enden
uint32_t gnssLFOnly = 0;      // ✅ NEU: Zähler für LF-only Enden
uint32_t gnssCRLF = 0;        // ✅ NEU: Zähler für CRLF Enden

inline void resetGnssBuffer() {
  gnssBufferIndex = 0;
  gnssBufferOverflow = false;
  gnssLastCharWasCR = false;
  gnssBufferTimeout = 0;
}

inline bool addToGnssBuffer(char c) {
  if (gnssBufferIndex >= GNSS_BUFFER_SIZE - 2) {
    gnssBufferOverflow = true;
    return false;
  }
  gnssPassThroughBuffer[gnssBufferIndex++] = c;
  return true;
}


// Minimale Debug-Ausgabe für Production
void sendGnssBuffer() {
  if (gnssBufferIndex > 0) {
    gnssPassThroughBuffer[gnssBufferIndex] = '\0';
    bool isDataValid = false;

    // Überprüfe, um welchen Satz-Typ es sich handelt
    if (strncmp(gnssPassThroughBuffer, "$KSXT", 5) == 0) {
      // Es ist ein KSXT-Satz, verwende die neue CRC32-Prüfung
      isDataValid = isKsxtCrc32Valid(gnssPassThroughBuffer);
    } else {
      // Andernfalls wird die Standard-NMEA-82-Prüfung verwendet
      isDataValid = isNmeaChecksumValid(gnssPassThroughBuffer); // Die Funktion aus der vorigen Antwort
    }

    // Sende die Daten nur, wenn sie gültig sind
    if (isDataValid) {
      Eth_udpPAOGI.beginPacket(Eth_ipDestination, 9999);
      Eth_udpPAOGI.write((uint8_t*)gnssPassThroughBuffer, gnssBufferIndex);
      Eth_udpPAOGI.endPacket();
      gnssPacketsSent++;
    } else {
      // Gib eine Warnung für ungültige Daten aus
      Serial.print("WARNUNG: Ungültige Prüfsumme! Verworfen: ");
      Serial.println(gnssPassThroughBuffer);
    }

    // Reduzierte Statistik nur alle 1000 Pakete
    if (gnssPacketsSent % 1000 == 0) {
      Serial.print("GNSS: ");
      Serial.print(gnssPacketsSent);
      Serial.println(" pkts OK");
      
      // Nur bei Problemen ausgeben
      if (gnssOverflowCount > 0 || gnssTimeoutCount > 0) {
        Serial.print("  ⚠️ Ovf:");
        Serial.print(gnssOverflowCount);
        Serial.print(", TO:");
        Serial.println(gnssTimeoutCount);
      }
    }
    
    resetGnssBuffer();
  }
}


// Setup procedure ------------------------
void setup() {
  delay(500);                //Small delay so serial can monitor start up
  set_arm_clock(450000000);  //Set CPU speed to 150mhz
  Serial.print("CPU speed set to: ");
  Serial.println(F_CPU_ACTUAL);

  
  pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
  pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);

  pinMode(PCB_RELAY_1, OUTPUT);
  pinMode(PCB_RELAY_2, OUTPUT);
  digitalWrite(PCB_RELAY_1,LOW);
  digitalWrite(PCB_RELAY_2,LOW);


  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);

  delay(10);
  Serial.println("Start setup");

  

  SerialGPS->begin(baudGPS);

  //delay(10);
  //SerialRTK.begin(baudRTK);



  Serial.println("SerialAOG, SerialRTK, SerialGPS and SerialGPS2 initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();

  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();



    Serial.println("here");

  SerialIMU->begin(115200);
  rvc.begin(SerialIMU);

  // ####################################################################
  // ## NEUER, NICHT-BLOCKIERENDER CHECK FÜR DAS SETUP
  // ####################################################################
  Serial.println("\r\nChecking for serial BNO08x (max 500ms)...");
  elapsedMillis bnoSetupTimer = 0;
  while (bnoSetupTimer < 500) { // Versuche es für eine halbe Sekunde
    if (rvc.read(&bnoData)) {
      // Erfolg! Sensor wurde schnell gefunden.
      useBNO08xRVC = true;
      Serial.println("Serial BNO08x Good To Go :-)");
      imuHandler(); // Verarbeite die erste Messung
      break;        // Verlasse die while-Schleife
    }
  }

  // Diese Meldung wird nur angezeigt, wenn der Sensor in der Zeit nicht gefunden wurde
  if (!useBNO08xRVC) {
    Serial.println("No Serial BNO08x found during startup check. Continuing to check in main loop...");
  }
  
  // Der Timeout-Timer für die Hauptschleife wird in jedem Fall gestartet
  imuDataTimer = 0;
  // ####################################################################
  // ## ENDE DES NEUEN BLOCKS
  // ####################################################################

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

  if (useMCP23017) {
    Serial.println("MCP23017 check now....");
    error = mcp.begin_I2C(0x20, &Wire1);
    Serial.print("MCP Status: ");
    Serial.println(error);
    
    if (error == 0) {
        Serial.println("MCP23017 not found - relay control disabled");
        useMCP23017 = false;
    } else {
        Serial.println("MCP23017 initialized successfully");

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

    Serial.print("MCP PINS Enabeld! ");

        // Pin-Konfiguration nur bei Erfolg
       
    }
    
    Serial.print("MCP Status done: ");
    Serial.println(useMCP23017 ? "ENABLED" : "DISABLED");


  }

  

  

  if (isKeya) {
    Serial.println("Right... time for some CANBUS! And, we're dedicated to SteeringwheelMotor here");
    CAN_Setup();
  }


  setupWebServer();
}

void loop() {

handleWebServer();



 if (isKeya) {
  KeyaBus_Receive();
 }

   // Read incoming nmea from GPS - ALLE verfügbaren Zeichen verarbeiten
  while (SerialGPS->available()) {
    char c = SerialGPS->read();
    
    if (gnsspassThrough) {
      gnssBufferTimeout = 0;
      
      // ✅ Erkenne verschiedene Zeilenenden
      bool isLineEnd = false;
      
      if (c == '\n') {
        // LF erkannt
        if (gnssLastCharWasCR) {
          // CRLF-Sequenz
          gnssCRLF++;
        } else {
          // Nur LF
          gnssLFOnly++;
        }
        isLineEnd = true;
        gnssLastCharWasCR = false;
      } 
      else if (c == '\r') {
        // CR erkannt - könnte CR-only oder CRLF sein
        // Füge zum Buffer hinzu und markiere
        gnssLastCharWasCR = true;
        
        // Füge CR zum Buffer hinzu
        if (!addToGnssBuffer(c)) {
          Serial.print("Overflow at CR, index: ");
          Serial.println(gnssBufferIndex);
          gnssOverflowCount++;
          
          // Zeige Buffer-Anfang
          Serial.print("Buffer start: ");
          for (int i = 0; i < min(60, (int)gnssBufferIndex); i++) {
            Serial.print(gnssPassThroughBuffer[i]);
          }
          Serial.println();
          
          resetGnssBuffer();
        }
        continue;  // Warte auf nächstes Zeichen
      }
      else if (gnssLastCharWasCR) {
        // Vorheriges Zeichen war CR, aber aktuelles ist kein LF
        // => CR-only Zeilenende
        gnssCROnly++;
        isLineEnd = true;
        gnssLastCharWasCR = false;
        
        // Aktuelles Zeichen NICHT vergessen - gehört zur nächsten Zeile
        // Also erst senden, dann Zeichen verarbeiten
        sendGnssBuffer();
        
        // Jetzt aktuelles Zeichen zur neuen Zeile hinzufügen
        if (!addToGnssBuffer(c)) {
          gnssOverflowCount++;
          resetGnssBuffer();
        }
        continue;
      }
      else {
        // Normales Zeichen
        gnssLastCharWasCR = false;
      }
      
      // Zeichen zum Buffer hinzufügen
      if (!isLineEnd) {
        if (!addToGnssBuffer(c)) {
          Serial.print("GNSS overflow at index: ");
          Serial.print(gnssBufferIndex);
          Serial.print(", char: 0x");
          Serial.println(c, HEX);
          
          Serial.print("Last 80 chars: ");
          uint16_t start = (gnssBufferIndex > 80) ? gnssBufferIndex - 80 : 0;
          for (uint16_t i = start; i < gnssBufferIndex; i++) {
            char ch = gnssPassThroughBuffer[i];
            if (ch >= 32 && ch < 127) Serial.print(ch);
            else Serial.print('.');
          }
          Serial.println();
          
          gnssOverflowCount++;
          
          // Verwerfe bis zum nächsten Zeilenende
          while (SerialGPS->available()) {
            char discard = SerialGPS->read();
            if (discard == '\n' || discard == '\r') break;
          }
          resetGnssBuffer();
        }
      } else {
        // Zeilenende erkannt - sende Buffer
        sendGnssBuffer();
      }
      
    } else {
      // Normaler Parsing-Modus
      parser << c;
    }
  }
  
  // Timeout-Prüfung
  if (gnsspassThrough && gnssBufferIndex > 0 && gnssBufferTimeout > GNSS_BUFFER_TIMEOUT_MS) {
    Serial.print("GNSS timeout (");
    Serial.print(gnssBufferIndex);
    Serial.print(" bytes): ");
    
    // Zeige ersten Teil des Buffers
    for (int i = 0; i < min(80, (int)gnssBufferIndex); i++) {
      char ch = gnssPassThroughBuffer[i];
      if (ch >= 32 && ch < 127) Serial.print(ch);
      else {
        Serial.print("[0x");
        Serial.print(ch, HEX);
        Serial.print("]");
      }
    }
    Serial.println();
    
    gnssTimeoutCount++;
    sendGnssBuffer();
  }
 


  // Check for RTK via Radio
  //if (SerialRTK.available()) {
 //   SerialGPS->write(SerialRTK.read());
 // }

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
     
      useDual = true;
      relPosDecode();
    }
    relposnedByteCount = 0;
  }

  // ###############################################################
  // ## BNO08x IMU LOGIK - NUR WENN gnsspassThrough DEAKTIVIERT IST
  // ###############################################################
  if (!gnsspassThrough) {
    // Prüfe, ob genügend Daten für ein komplettes Paket im Puffer sind
    if (SerialIMU->available() >= 19) {
        if (rvc.read(&bnoData)) {
         // Serial.println("Erfolgreich ein komplettes Datenpaket empfangen");
            // Erfolgreich ein komplettes Datenpaket empfangen
            if (!useBNO08xRVC) {
                Serial.println("Serial BNO08x Good To Go :-) Ich bin wieder da!");
            }
            useBNO08xRVC = true;
            imuDataTimer = 0;
            imuHandler();
        }
    }

    // Prüfen, ob der IMU aufgehört hat zu senden (Timeout)
    if (useBNO08xRVC && imuDataTimer > IMU_TIMEOUT_MS) {
        Serial.println("!!! FEHLER: BNO08x IMU Timeout - Deaktiviere IMU. !!!");
        useBNO08xRVC = false;
    }
  }
  // ###############################################################

  if (Autosteer_running) autosteerLoop();
  else ReceiveUdp();

  //GGA timeout, turn off GPS LED's etc
  if (GGAReadyTime > 10000)  //GGA age over 10sec
  {
    
    useDual = false;
  }

  if (ethernetLinkCheck > 10000) {
    if (Ethernet.linkStatus() == LinkON) {
      ethernetLinkCheck = 0;
      
    } else {
      
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




// Ihre Original-Funktion zur CRC-Berechnung für ein einzelnes Byte
unsigned long CalcCRC32Value(int value) {
  unsigned long ulCRC = value;
  for (int i = 8; i > 0; --i) {
    if (ulCRC & 1)
      ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
    else
      ulCRC >>= 1;
  }
  return ulCRC;
}

// Ihre Original-Funktion zur CRC-Berechnung für einen Datenblock
unsigned long CalcBlockCRC32(unsigned long ulCount, unsigned char* ucBuff) {
  unsigned long ulCRC = 0;
  while (ulCount-- != 0) {
    unsigned long ulTmp1 = (ulCRC >> 8) & 0x00FFFFFFL;
    unsigned long ulTmp2 = CalcCRC32Value(((int)ulCRC ^ *ucBuff++) & 0xFF);
    ulCRC = ulTmp1 ^ ulTmp2;
  }
  return ulCRC;
}

/**
 * Überprüft die CRC32-Prüfsumme eines proprietären $KSXT-Satzes.
 * @param sentence Der zu überprüfende Datensatz (z.B. "$KSXT,...*7B2DC8F2")
 * @return true, wenn die Prüfsumme korrekt ist, ansonsten false.
 */
bool isKsxtCrc32Valid(const char* sentence) {
  // Finde das Sternchen '*' am Ende des Datenteils
  const char* star = strrchr(sentence, '*');
  if (star == nullptr || strlen(star) < 9) {
    // Kein Sternchen oder zu kurze Prüfsumme
    return false;
  }

  // Extrahiere die 8-stellige CRC32-Prüfsumme aus dem Satz (z.B. "7B2DC8F2")
  // und konvertiere sie von Hexadezimal in eine Zahl.
  unsigned long receivedCrc = strtoul(star + 1, nullptr, 16);

  // Finde den Anfang des Datenteils (nach dem '$')
  const char* dataStart = sentence;
  if (*dataStart == '$') {
    dataStart++;
  }

  // Berechne die Länge der Daten, für die die CRC berechnet werden muss
  unsigned long dataLength = star - dataStart;

  // Berechne die CRC32 für den relevanten Datenteil
  unsigned long calculatedCrc = CalcBlockCRC32(dataLength, (unsigned char*)dataStart);

  // Vergleiche die berechnete mit der empfangenen Prüfsumme
  return receivedCrc == calculatedCrc;
}



bool isNmeaChecksumValid(const char* sentence) {
  // Finde das Sternchen '*' am Ende des Satzes
  const char* star = strrchr(sentence, '*');
  if (star == nullptr) {
    // Kein Sternchen gefunden, also keine Prüfsumme vorhanden
    return false;
  }

  // Extrahiere die zweistellige Prüfsumme aus dem Satz (z.B. "47")
  // und konvertiere sie von Hexadezimal in eine Zahl.
  uint8_t receivedChecksum = (uint8_t)strtol(star + 1, nullptr, 16);

  // Berechne die Prüfsumme aus den ankommenden Daten
  uint8_t calculatedChecksum = 0;
  // Starte die Berechnung nach dem '$' (falls vorhanden)
  const char* startChar = sentence;
  if (*startChar == '$') {
    startChar++;
  }
  
  // XOR-Verknüpfung aller Zeichen zwischen '$' und '*'
  while (startChar < star) {
    calculatedChecksum ^= *startChar;
    startChar++;
  }

  // Vergleiche die berechnete mit der empfangenen Prüfsumme
  return receivedChecksum == calculatedChecksum;
}