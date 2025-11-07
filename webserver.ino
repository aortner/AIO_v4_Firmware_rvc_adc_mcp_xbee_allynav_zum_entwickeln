// ========== KORRIGIERTE VERSION: webserver.ino mit GPS Stream Display ==========

#include <NativeEthernet.h>
#include <EEPROM.h>

// Externe Variablen aus anderen Dateien
extern char imuHeading[6];
extern char imuRoll[6];
extern char imuPitch[6];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char ageDGPS[10];
extern float gpsSpeed;
extern bool Autosteer_running;
extern uint8_t watchdogTimer;
extern const uint16_t WATCHDOG_THRESHOLD;
extern float steerAngleActual;
extern int16_t pwmDisplay;
extern bool useBNO08xRVC;
extern bool adsOk;
extern float steerAngleSetPoint;
extern float steerAngleError;
extern struct Storage steerSettings;
extern float sensorReading;
extern struct Setup steerConfig;
extern HardwareSerial* SerialGPS;

// Deklarieren, dass sysConfig und die zugehörigen Flags woanders definiert sind
extern struct SystemConfig sysConfig;
extern bool gnsspassThrough;
extern bool useMCP23017;
extern bool isKeya;
extern bool isallnavy;

// ✅ WICHTIG: Zugriff auf den GNSS Buffer
extern char gnssPassThroughBuffer[];
extern uint16_t gnssBufferIndex;

// Webserver auf Port 80
EthernetServer webServer(80);

// State Machine für non-blocking Webserver
enum WebserverState {
  WS_IDLE,
  WS_READING_REQUEST,
  WS_SENDING_RESPONSE,
  WS_CLOSING
};
WebserverState webState = WS_IDLE;
EthernetClient webClient;
unsigned long webStateTimer = 0;
String webRequest = "";

// GPS Stream Logging - Ring-Buffer für die letzten NMEA-Sätze
#define GPS_LOG_SIZE 30
String gpsStreamLog[GPS_LOG_SIZE];
uint8_t gpsLogWriteIndex = 0;
bool gpsStreamActive = false;
elapsedMillis gpsStreamTimer = 0;
uint32_t gpsLogCounter = 0;

// ✅ KORRIGIERTE Funktion zum Hinzufügen eines NMEA-Satzes zum Log
void addToGpsLog(const char* sentence) {


 


  if (!gpsStreamActive) return;
  
  // ✅ WICHTIG: Der Buffer ist NICHT null-terminiert!
  // Wir müssen die Länge aus gnssBufferIndex verwenden
  if (gnssBufferIndex == 0) return;
  
  // Prüfe ob es mit $ beginnt
  if (sentence[0] != '$') return;
  
  // ✅ Erstelle einen String mit der korrekten Länge
  String cleanSentence;
  cleanSentence.reserve(gnssBufferIndex + 1);
  
  for (uint16_t i = 0; i < gnssBufferIndex; i++) {
    char c = sentence[i];
    // Filtere Control-Characters raus
    if (c >= 32 && c < 127) {
      cleanSentence += c;
    }
  }
  
  // Nur wenn der Satz nicht leer ist
  if (cleanSentence.length() < 5) return;
  
  // Füge zum Ring-Buffer hinzu
  gpsStreamLog[gpsLogWriteIndex] = cleanSentence;
  gpsLogWriteIndex = (gpsLogWriteIndex + 1) % GPS_LOG_SIZE;
  gpsLogCounter++;
  
 

  // Debug: Alle 50 Sätze eine Meldung
  if (gpsLogCounter % 50 == 0) {
    Serial.print("GPS Log: ");
    Serial.print(gpsLogCounter);
    Serial.println(" Sätze erfolgreich geloggt");
  }
}

// Funktion zum Speichern der Einstellungen und Neustarten
void saveAndReset() {
  EEPROM.put(100, sysConfig);
  delay(100);
  SCB_AIRCR = 0x05FA0004; // Teensy Reset
}

void handleSetConfig(String req) {
  if (req.indexOf("gnsspass=") != -1) {
    int startPos = req.indexOf("gnsspass=") + 9;
    int endPos = req.indexOf(" ", startPos);
    if (endPos == -1) endPos = req.indexOf("&", startPos);
    if (endPos == -1) endPos = req.length();
    
    String value = req.substring(startPos, endPos);
    sysConfig.gnsspassThrough = (value == "1" || value == "true") ? 1 : 0;
    gnsspassThrough = (sysConfig.gnsspassThrough != 0);
    
    Serial.print("GNSS Pass-Through gesetzt auf: ");
    Serial.println(sysConfig.gnsspassThrough);
    
  } else if (req.indexOf("mcp=") != -1) {
    int startPos = req.indexOf("mcp=") + 4;
    int endPos = req.indexOf(" ", startPos);
    if (endPos == -1) endPos = req.indexOf("&", startPos);
    if (endPos == -1) endPos = req.length();
    
    String value = req.substring(startPos, endPos);
    sysConfig.useMCP23017 = (value == "1" || value == "true") ? 1 : 0;
    useMCP23017 = (sysConfig.useMCP23017 != 0);
    
    Serial.print("MCP23017 gesetzt auf: ");
    Serial.println(sysConfig.useMCP23017);
    
  } else if (req.indexOf("motor=") != -1) {
    int startPos = req.indexOf("motor=") + 6;
    int endPos = req.indexOf(" ", startPos);
    if (endPos == -1) endPos = req.indexOf("&", startPos);
    if (endPos == -1) endPos = req.length();
    
    int motorVal = req.substring(startPos, endPos).toInt();
    
    switch(motorVal) {
        case 0: 
          sysConfig.isKeya = 0; 
          sysConfig.isallnavy = 0; 
          break;
        case 1: 
          sysConfig.isKeya = 1; 
          sysConfig.isallnavy = 0; 
          break;
        case 2: 
          sysConfig.isKeya = 1; 
          sysConfig.isallnavy = 1; 
          break;
    }
    isKeya = (sysConfig.isKeya != 0);
    isallnavy = (sysConfig.isallnavy != 0);
    
    Serial.print("Motor Type gesetzt auf: ");
    Serial.println(motorVal);
    
  } else if (req.indexOf("relay1=") != -1) {
    int startPos = req.indexOf("relay1=") + 7;
    int endPos = req.indexOf(" ", startPos);
    if (endPos == -1) endPos = req.indexOf("&", startPos);
    if (endPos == -1) endPos = req.length();
    
    int val = req.substring(startPos, endPos).toInt();
    if (val >= 1 && val <= 7) {
      sysConfig.pcbRelay1_Mode = val;
      Serial.print("Relay 1 Mode gesetzt auf: ");
      Serial.println(val);
    }
    
  } else if (req.indexOf("relay2=") != -1) {
    int startPos = req.indexOf("relay2=") + 7;
    int endPos = req.indexOf(" ", startPos);
    if (endPos == -1) endPos = req.indexOf("&", startPos);
    if (endPos == -1) endPos = req.length();
    
    int val = req.substring(startPos, endPos).toInt();
    if (val >= 1 && val <= 7) {
      sysConfig.pcbRelay2_Mode = val;
      Serial.print("Relay 2 Mode gesetzt auf: ");
      Serial.println(val);
    }
  } else if (req.indexOf("gpsstream=") != -1) {
    // GPS Stream aktivieren/deaktivieren
    int startPos = req.indexOf("gpsstream=") + 10;
    int endPos = req.indexOf(" ", startPos);
    if (endPos == -1) endPos = req.indexOf("&", startPos);
    if (endPos == -1) endPos = req.length();
    
    String value = req.substring(startPos, endPos);
    gpsStreamActive = (value == "1" || value == "true");
    
    if (gpsStreamActive) {
      // Log leeren beim Aktivieren
      for (int i = 0; i < GPS_LOG_SIZE; i++) {
        gpsStreamLog[i] = "";
      }
      gpsLogWriteIndex = 0;
      gpsLogCounter = 0;
      gpsStreamTimer = 0;
      Serial.println("\r\n======================================");
      Serial.println("=== GPS Stream Logging AKTIVIERT ===");
      Serial.println("======================================");
      Serial.print("GNSS Passthrough Modus: ");
      Serial.println(gnsspassThrough ? "JA" : "NEIN");
      Serial.println("Warte auf GPS-Daten...\r\n");
    } else {
      Serial.println("\r\n======================================");
      Serial.println("=== GPS Stream Logging DEAKTIVIERT ===");
      Serial.print("Insgesamt geloggt: ");
      Serial.print(gpsLogCounter);
      Serial.println(" Sätze");
      Serial.println("======================================\r\n");
    }
    
    webClient.print("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK");
    return; // Kein Reset für Stream-Toggle
  }
  
  Serial.println("\r\n=== Config wird gespeichert ===");
  Serial.print("gnsspassThrough: "); Serial.println(sysConfig.gnsspassThrough);
  Serial.print("useMCP23017: "); Serial.println(sysConfig.useMCP23017);
  Serial.print("isKeya: "); Serial.println(sysConfig.isKeya);
  Serial.print("isallnavy: "); Serial.println(sysConfig.isallnavy);
  Serial.print("Relay1 Mode: "); Serial.println(sysConfig.pcbRelay1_Mode);
  Serial.print("Relay2 Mode: "); Serial.println(sysConfig.pcbRelay2_Mode);
  Serial.println("===============================\r\n");
  
  webClient.print("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK");
  saveAndReset();
}

void webserverSetup() {
  webServer.begin();
  Serial.print("Webserver started on: http://");
  Serial.print(Ethernet.localIP());
  Serial.println("/");
  
  gpsStreamActive = false;
}

// HTML-Seite mit GPS Stream Display
void sendWebPageFast(EthernetClient &client) {
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
  client.print("<!DOCTYPE html><html><head><meta charset='UTF-8'><title>AIO AgOpenGps Controller</title>");
  client.print("<style>body{font-family:Arial,sans-serif;margin:20px;background:#f0f2f5;color:#333}h1,h2{color:#1a2a45;text-align:center}");
  client.print(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin-bottom:25px}");
  client.print(".card{background:white;padding:15px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);border-left:5px solid #ccc}");
  client.print(".label{color:#666;font-size:14px;margin-bottom:5px}.value{font-size:22px;font-weight:bold;color:#1a2a45}");
  client.print(".btn-group button{width:33.3%;padding:10px 0;border:1px solid #ccc;background-color:#f0f0f0;cursor:pointer;float:left}");
  client.print(".btn-group button.active{background-color:#4CAF50;color:white;border-color:#4CAF50}");
  client.print(".toggle-btn{display:block;width:100%;padding:12px;font-size:16px;font-weight:bold;border:none;border-radius:5px;cursor:pointer;transition:background-color 0.3s}");
  client.print(".toggle-btn.on{background-color:#4CAF50;color:white}.toggle-btn.off{background-color:#757575;color:white}");
  client.print("#restarting{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.7);color:white;font-size:2em;text-align:center;padding-top:40vh}");
  client.print(".status-ok{border-left-color:#4CAF50}.status-warn{border-left-color:#FFC107}.status-error{border-left-color:#F44336}");
  client.print("select{width:100%;padding:10px;font-size:16px;border-radius:5px;border:1px solid #ccc}");
  
  // GPS Stream Modal Styles
  client.print("#gps-stream-container{display:none;position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);width:90%;max-width:900px;max-height:85vh;background:white;border-radius:8px;box-shadow:0 8px 32px rgba(0,0,0,0.3);z-index:1000}");
  client.print("#gps-stream-header{background:#1a2a45;color:white;padding:15px 20px;border-radius:8px 8px 0 0;display:flex;justify-content:space-between;align-items:center}");
  client.print("#gps-stream-content{padding:15px;max-height:65vh;overflow-y:auto;font-family:'Courier New',monospace;font-size:12px;background:#1e1e1e;color:#00ff00;line-height:1.4}");
  client.print(".gps-line{padding:6px 10px;border-bottom:1px solid #333;word-break:break-all;white-space:pre-wrap}");
  client.print(".gps-line:hover{background:#2a2a2a;cursor:pointer}");
  client.print(".gps-line:nth-child(odd){background:#252525}");
  client.print(".close-btn{background:#f44336;color:white;border:none;padding:10px 20px;border-radius:5px;cursor:pointer;font-weight:bold;font-size:14px}");
  client.print(".close-btn:hover{background:#d32f2f}");
  client.print(".overlay{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.7);z-index:999}");
  client.print(".stream-info{padding:10px;background:#2a2a2a;color:#ffaa00;font-size:11px;text-align:center;border-bottom:1px solid #444}");
  client.print("</style></head><body>");
  
  client.print("<h1>🚜 www.autosteer.cc AgOpenGPS AIO Controller</h1><div id='data-grid' class='grid'></div>");
  client.print("<h2>System Configuration</h2><div id='settings-grid' class='grid'></div>");
  
  // GPS Stream Button (auffälliger)
  client.print("<div class='card' style='text-align:center;grid-column:1/-1;margin-top:20px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);border:none'>");
  client.print("<button onclick='toggleGpsStream()' style='padding:15px 40px;font-size:18px;background:#fff;color:#667eea;border:none;border-radius:8px;cursor:pointer;font-weight:bold;box-shadow:0 4px 15px rgba(0,0,0,0.2)'>📡 Show Live GPS Stream</button>");
  client.print("</div>");
  
  // GPS Stream Modal
  client.print("<div class='overlay' id='overlay' onclick='closeGpsStream()'></div>");
  client.print("<div id='gps-stream-container'>");
  client.print("<div id='gps-stream-header'><span style='font-weight:bold;font-size:16px'>📡 GPS NMEA Stream (Live)</span><button class='close-btn' onclick='closeGpsStream()'>✕ Close</button></div>");
  client.print("<div class='stream-info' id='stream-info'>Initializing...</div>");
  client.print("<div id='gps-stream-content'><div style='color:#666;text-align:center;padding:40px;font-size:14px'>⏳ Waiting for GPS data...<br><small>Make sure GNSS Passthrough is enabled</small></div></div>");
  client.print("</div>");
  
  client.print("<div id='restarting'>⚙️ Restarting... Please wait.</div>");
  
  // JavaScript
  client.print("<script>let gpsStreamInterval=null;let lastCount=0;");
  client.print("function r(v){return parseFloat(v).toFixed(2)}function i(v){return parseInt(v)}");
  client.print("function createCard(l,v,c=''){return`<div class='card ${c}'><div class=label>${l}</div><div class=value>${v}</div></div>`}");
  client.print("function createToggleBtn(l,p,s){return`<div class=card><div class=label>${l}</div><button onclick='setParam(\"${p}\",${!s})' class='toggle-btn ${s?'on':'off'}'>${s?'ON':'OFF'}</button></div>`}");
  client.print("function createMotorSelector(m){let b='';const types=['PWM','Keya','AllyNav'];for(let i=0;i<3;i++){b+=`<button onclick='setParam(\"motor\",${i})' class='${m==i?\"active\":''}'>${types[i]}</button>`}return`<div class=card><div class=label>Motor Controller</div><div class='btn-group'>${b}</div></div>`}");
  client.print("function createRelaySelector(l,p,m){let o='';const t=['None','Autosteer','Tramline','Hitch Up','Hitch Down','Section 1','Section 2'];for(let i=0;i<t.length;i++){o+=`<option value='${i+1}' ${m==i+1?'selected':''}>${t[i]}</option>`}return`<div class=card><div class=label>${l}</div><select onchange='setParam(\"${p}\",this.value)'>${o}</select></div>`}");
  client.print("function setParam(p,v){document.getElementById('restarting').style.display='block';fetch(`/set?${p}=${v}`).then(()=>{setTimeout(checkRestart,3000)})}");
  client.print("function checkRestart(){fetch('/json').then(()=>{location.reload()}).catch(()=>{setTimeout(checkRestart,1000)})}");
  
  // GPS Stream Functions
  client.print("function toggleGpsStream(){document.getElementById('gps-stream-container').style.display='block';document.getElementById('overlay').style.display='block';");
  client.print("fetch('/set?gpsstream=1');lastCount=0;gpsStreamInterval=setInterval(updateGpsStream,800);setTimeout(updateGpsStream,100);}");
  client.print("function closeGpsStream(){document.getElementById('gps-stream-container').style.display='none';document.getElementById('overlay').style.display='none';");
  client.print("clearInterval(gpsStreamInterval);fetch('/set?gpsstream=0');}");
  client.print("function updateGpsStream(){fetch('/gpsstream').then(r=>r.json()).then(d=>{");
  client.print("let info='';if(d.count>0){info=`📊 ${d.count} sentences logged | ⏱️ ${Math.floor(d.time/1000)}s runtime`;}else{info='Waiting for data...';}");
  client.print("document.getElementById('stream-info').innerHTML=info;");
  client.print("let h='';if(d.lines&&d.lines.length>0){d.lines.forEach((line,idx)=>{if(line)h+=`<div class='gps-line'>${line}</div>`;});}");
  client.print("else{h='<div style=\"color:#888;text-align:center;padding:30px;font-size:14px\">📭 No GPS data received yet<br><small>Check that GPS is connected and transmitting</small></div>';}");
  client.print("document.getElementById('gps-stream-content').innerHTML=h;");
  client.print("if(d.count>lastCount){document.getElementById('gps-stream-content').scrollTop=document.getElementById('gps-stream-content').scrollHeight;}lastCount=d.count;");
  client.print("}).catch(e=>{console.log('GPS stream error:',e);document.getElementById('stream-info').innerHTML='⚠️ Connection error';});}");

  // Main Data Load Function
  client.print("function load(){fetch('/json').then(r=>r.json()).then(d=>{let h='';let s='';");
  client.print("h+=createCard('Uptime',i(d.up)+' s','status-ok');h+=createCard('GPS Fix',i(d.fix),'status-'+(i(d.fix)>=4?'ok':'warn'));h+=createCard('Satellites',i(d.sats),'status-'+(i(d.sats)>8?'ok':'warn'));h+=createCard('Speed',r(d.speed)+' km/h');");
  client.print("if(d.roll!=null){h+=createCard('IMU Heading',r(d.heading/10)+' °');h+=createCard('IMU Roll',r(d.roll/10)+' °');}");
  client.print("if(d.steer!=null){h+=createCard('Steering',d.steer==1?'ACTIVE':'OFF','status-'+(d.steer==1?'ok':'warn'));h+=createCard('Set Angle',r(d.setAngle)+' °');h+=createCard('Angle Error',r(d.angleErr)+' °');h+=createCard('PWM',i(d.pwm));}");
  client.print("if(d.sensor_type){h+=createCard(d.sensor_type+' Sensor',parseFloat(d.sensor_reading).toFixed(1));}");
  client.print("h+=createCard('IMU Status',d.imuOk?'OK':'OFF','status-'+(d.imuOk?'ok':'error'));h+=createCard('WAS Status',d.wasOk?'OK':'ERROR','status-'+(d.wasOk?'ok':'error'));");
  client.print("s+=createToggleBtn('GNSS Passthrough','gnsspass',d.gnsspass);s+=createToggleBtn('Use MCP23017','mcp',d.mcp);s+=createMotorSelector(d.motor_type);");
  client.print("s+=createRelaySelector('PCB Relay 1 Function','relay1',d.relay1_mode);s+=createRelaySelector('PCB Relay 2 Function','relay2',d.relay2_mode);");
  client.print("document.getElementById('data-grid').innerHTML=h;document.getElementById('settings-grid').innerHTML=s;");
  client.print("}).catch(e=>{console.log(e);})}load();setInterval(load,2000)</script></body></html>");
}

// JSON Status Data
void sendJsonDataFast(EthernetClient &client) {
  client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n{");
  client.print("\"up\":"); client.print(millis() / 1000);
  client.print(",\"fix\":\""); client.print(fixQuality);
  client.print("\",\"sats\":\""); client.print(numSats);
  client.print("\",\"speed\":"); client.print(gpsSpeed, 2);
  if (useBNO08xRVC){
    client.print(",\"roll\":\""); client.print(imuRoll);
    client.print("\",\"pitch\":\""); client.print(imuPitch);
    client.print("\",\"heading\":\""); client.print(imuHeading);
    client.print("\"");
  }
  if (Autosteer_running) {
    client.print(",\"steer\":"); client.print(watchdogTimer < WATCHDOG_THRESHOLD ? "1" : "0");
    client.print(",\"setAngle\":"); client.print(steerAngleSetPoint, 2);
    client.print(",\"angleErr\":"); client.print(steerAngleError, 2);
    client.print(",\"pwm\":"); client.print(pwmDisplay);
  }
  if (steerConfig.CurrentSensor) {
    client.print(",\"sensor_type\":\"Current\"");
    client.print(",\"sensor_reading\":"); client.print(sensorReading, 1);
  } else if (steerConfig.PressureSensor) {
    client.print(",\"sensor_type\":\"Pressure\"");
    client.print(",\"sensor_reading\":"); client.print(sensorReading, 1);
  }
  client.print(",\"imuOk\":"); client.print(useBNO08xRVC);
  client.print(",\"wasOk\":"); client.print(adsOk);
  client.print(",\"gnsspass\":"); client.print(gnsspassThrough);
  client.print(",\"mcp\":"); client.print(useMCP23017);
  int motorType = 0; 
  if (isKeya && isallnavy) { motorType = 2; } else if (isKeya) { motorType = 1; }
  client.print(",\"motor_type\":"); client.print(motorType);
  client.print(",\"relay1_mode\":"); client.print(sysConfig.pcbRelay1_Mode);
  client.print(",\"relay2_mode\":"); client.print(sysConfig.pcbRelay2_Mode);
  client.println("}");
}

// GPS Stream JSON Response
void sendGpsStreamJson(EthernetClient &client) {
  client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n{");
  client.print("\"lines\":[");
  
  bool first = true;
  // Durchlaufe den Ring-Buffer in der richtigen Reihenfolge (älteste zuerst)
  for (int i = 0; i < GPS_LOG_SIZE; i++) {
    int idx = (gpsLogWriteIndex + i) % GPS_LOG_SIZE;
    if (gpsStreamLog[idx].length() > 0) {
      if (!first) client.print(",");
      client.print("\"");
      // Escape Anführungszeichen und Backslashes
      String escaped = gpsStreamLog[idx];
      escaped.replace("\\", "\\\\");
      escaped.replace("\"", "\\\"");
      client.print(escaped);
      client.print("\"");
      first = false;
    }
  }
  
  client.print("],\"active\":");
  client.print(gpsStreamActive ? "true" : "false");
  client.print(",\"time\":");
  client.print(gpsStreamTimer);
  client.print(",\"count\":");
  client.print(gpsLogCounter);
  client.println("}");
}

// Non-blocking Webserver Loop
void webserverLoop() {
  switch (webState) {
    case WS_IDLE: {
      EthernetClient newClient = webServer.available();
      if (newClient) {
        webClient = newClient;
        webState = WS_READING_REQUEST;
        webStateTimer = millis();
        webRequest = "";
        webRequest.reserve(128);
      }
      break;
    }
    case WS_READING_REQUEST: {
      if (millis() - webStateTimer > 300) {
        webClient.stop();
        webState = WS_IDLE;
        break;
      }
      while (webClient.available()) {
        char c = webClient.read();
        if (c == '\n') {
          webState = WS_SENDING_RESPONSE;
          break;
        }
        if (webRequest.length() < 128) webRequest += c;
      }
      break;
    }
    case WS_SENDING_RESPONSE: {
      if (webRequest.indexOf("GET /set?") >= 0) {
        handleSetConfig(webRequest);
      } else if (webRequest.indexOf("GET /gpsstream") >= 0) {
        sendGpsStreamJson(webClient);
      } else if (webRequest.indexOf("GET /json") >= 0) {
        sendJsonDataFast(webClient);
      } else {
        sendWebPageFast(webClient);
      }
      webState = WS_CLOSING;
      break;
    }
    case WS_CLOSING: {
      webClient.stop();
      webState = WS_IDLE;
      break;
    }
  }
}
