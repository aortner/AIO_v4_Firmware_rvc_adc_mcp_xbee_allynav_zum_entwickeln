// webserver.ino - Erstelle diese als neue Datei im gleichen Ordner

#include <NativeEthernet.h>

// Webserver auf Port 80
EthernetServer webServer(80);

// HTML-Seite im Flash-Speicher (PROGMEM) - komprimiert
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AgOpen</title><style>*{box-sizing:border-box}body{margin:0;padding:10px;background:#111;color:#fff;font-family:Arial,sans-serif;font-size:14px}.header{display:flex;justify-content:space-between;margin-bottom:15px}h1{margin:0;font-size:20px}.status{padding:12px;border-radius:6px;margin-bottom:15px;border:2px solid #666}.active{background:#0a4;border-color:#0f6}.standby{background:#333}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px}.card{background:#222;padding:12px;border-radius:6px;border:1px solid #444}.card h2{margin:0 0 8px;font-size:16px}.row{display:flex;justify-content:space-between;padding:4px 0;font-size:13px}.label{color:#999}.value{font-family:monospace}.good{color:#0f6}.warn{color:#fa0}.bad{color:#f33}.compass{position:relative;width:100px;height:100px;border:3px solid #666;border-radius:50%;margin:8px auto}.needle{position:absolute;top:50%;left:50%;width:2px;height:40px;background:#f33;transform-origin:bottom;transform:translate(-50%,-100%)}.north{position:absolute;top:3px;left:50%;transform:translateX(-50%);font-size:11px}.relay-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:8px}.relay{padding:8px;border-radius:4px;text-align:center;font-size:11px}.relay-on{background:#0a4}.relay-off{background:#333}</style></head><body><div class="header"><h1>AgOpen Monitor</h1><span id="time">--:--:--</span></div><div id="status" class="status standby"><div style="display:flex;justify-content:space-between"><span id="statusText">STANDBY</span><span id="speed">0.0 km/h</span></div></div><div class="grid"><div class="card"><h2>GPS</h2><div class="row"><span class="label">Fix:</span><span id="fix" class="value">-</span></div><div class="row"><span class="label">Sats:</span><span id="sats" class="value">0</span></div><div class="row"><span class="label">HDOP:</span><span id="hdop" class="value">0.0</span></div><div class="row"><span class="label">Lat:</span><span id="lat" class="value">-</span></div><div class="row"><span class="label">Lon:</span><span id="lon" class="value">-</span></div><div class="row"><span class="label">Alt:</span><span id="alt" class="value">0m</span></div></div><div class="card"><h2>IMU</h2><div class="row"><span class="label">Heading:</span><span id="heading" class="value">0°</span></div><div class="row"><span class="label">Roll:</span><span id="roll" class="value">0°</span></div><div class="row"><span class="label">Pitch:</span><span id="pitch" class="value">0°</span></div><div class="compass"><div class="north">N</div><div id="needle" class="needle"></div></div></div><div class="card"><h2>Steering</h2><div class="row"><span class="label">Actual:</span><span id="steerActual" class="value">0°</span></div><div class="row"><span class="label">Setpoint:</span><span id="steerSet" class="value">0°</span></div><div class="row"><span class="label">Error:</span><span id="steerErr" class="value">0°</span></div><div class="row"><span class="label">PWM:</span><span id="pwm" class="value">0</span></div></div><div class="card"><h2>Relays</h2><div class="relay-grid" id="relays"></div></div><div class="card"><h2>Sensors</h2><div class="row"><span class="label">Current:</span><span id="current" class="value">0A</span></div></div><div class="card"><h2>GNSS</h2><div class="row"><span class="label">Packets:</span><span id="gnssP" class="value">0</span></div><div class="row"><span class="label">Ovf:</span><span id="gnssO" class="value">0</span></div><div class="row"><span class="label">TO:</span><span id="gnssT" class="value">0</span></div></div></div><script>function update(){fetch('/data').then(r=>r.json()).then(d=>{document.getElementById('time').textContent=new Date().toLocaleTimeString();document.getElementById('statusText').textContent=d.a?'ACTIVE':'STANDBY';document.getElementById('status').className='status '+(d.a?'active':'standby');document.getElementById('speed').textContent=d.s.toFixed(1)+' km/h';var fx=['Invalid','GPS','DGPS','PPS','RTK','Float','DR'];document.getElementById('fix').textContent=fx[d.fq]||'-';document.getElementById('fix').className='value '+(d.fq>=4?'good':d.fq>=2?'warn':'bad');document.getElementById('sats').textContent=d.ns;document.getElementById('hdop').textContent=d.hd.toFixed(1);document.getElementById('lat').textContent=d.lt.toFixed(6);document.getElementById('lon').textContent=d.ln.toFixed(6);document.getElementById('alt').textContent=d.al.toFixed(1)+'m';document.getElementById('heading').textContent=d.h.toFixed(1)+'°';document.getElementById('roll').textContent=d.r.toFixed(2)+'°';document.getElementById('pitch').textContent=d.p.toFixed(2)+'°';document.getElementById('needle').style.transform='translate(-50%,-100%) rotate('+d.h+'deg)';document.getElementById('steerActual').textContent=d.sa.toFixed(2)+'°';document.getElementById('steerSet').textContent=d.ss.toFixed(2)+'°';document.getElementById('steerErr').textContent=d.se.toFixed(2)+'°';document.getElementById('steerErr').className='value '+(Math.abs(d.se)>2?'warn':'good');document.getElementById('pwm').textContent=d.pw;document.getElementById('current').textContent=d.cu.toFixed(1)+'A';document.getElementById('gnssP').textContent=d.gp;document.getElementById('gnssO').textContent=d.go;document.getElementById('gnssT').textContent=d.gt;var rh='';for(var i=0;i<8;i++)rh+='<div class="relay relay-'+(d.rl[i]?'on':'off')+'">R'+(i+1)+'</div>';document.getElementById('relays').innerHTML=rh;}).catch(e=>console.error(e));}setInterval(update,500);update();</script></body></html>
)rawliteral";

// Setup-Funktion
void setupWebServer() {
  webServer.begin();
  Serial.println("Webserver started on port 80");
  Serial.print("Access at: http://");
  Serial.println(Ethernet.localIP());
}

// Loop-Funktion
void handleWebServer() {
  EthernetClient client = webServer.available();
  
  if (client) {
    bool currentLineIsBlank = true;
    String request = "";
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        
        if (request.length() < 100) { // Verhindere Overflow
          request += c;
        }
        
        if (c == '\n' && currentLineIsBlank) {
          // Request komplett empfangen
          if (request.indexOf("GET /data") >= 0) {
            sendJsonData(client);
          } else {
            sendHtmlPage(client);
          }
          break;
        }
        
        if (c == '\n') {
          currentLineIsBlank = true;
        } else if (c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    
    delay(1);
    client.stop();
  }
}

void sendHtmlPage(EthernetClient &client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();
  
  // HTML aus PROGMEM senden
  size_t len = strlen_P(HTML_PAGE);
  for (size_t i = 0; i < len; i++) {
    client.write(pgm_read_byte(&HTML_PAGE[i]));
  }
}

// Hilfsfunktion für Koordinaten-Parsing
double parseCoord(const char* coord, const char* dir) {
  if (strlen(coord) == 0) return 0.0;
  
  double val = atof(coord);
  int deg = (int)(val / 100);
  double min = val - (deg * 100);
  double dec = deg + (min / 60.0);
  
  if (dir[0] == 'S' || dir[0] == 'W') {
    dec = -dec;
  }
  
  return dec;
}

void sendJsonData(EthernetClient &client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Connection: close"));
  client.println();
  
  // JSON kompakt (mit Kurznamen um Bytes zu sparen)
  client.print(F("{\"a\":"));
  client.print(watchdogTimer < WATCHDOG_THRESHOLD ? "true" : "false");
  
  client.print(F(",\"s\":"));
  client.print(gpsSpeed, 1);
  
  // GPS
  
  
  // IMU
  if (useBNO08xRVC) {
    client.print(F(",\"h\":"));
    client.print(bnoData.yawX10 / 10.0, 1);
    
    client.print(F(",\"r\":"));
    client.print(bnoData.rollX10 / 10.0, 2);
    
    client.print(F(",\"p\":"));
    client.print(bnoData.pitchX10 / 10.0, 2);
  } else {
    client.print(F(",\"h\":0,\"r\":0,\"p\":0"));
  }
  
  // Steering
  if (Autosteer_running) {
    client.print(F(",\"sa\":"));
    client.print(steerAngleActual, 2);
    
    client.print(F(",\"ss\":"));
    client.print(steerAngleSetPoint, 2);
    
    client.print(F(",\"se\":"));
    client.print(steerAngleError, 2);
    
    client.print(F(",\"pw\":"));
    client.print(pwmDisplay);
  } else {
    client.print(F(",\"sa\":0,\"ss\":0,\"se\":0,\"pw\":0"));
  }
  
  // Current
  client.print(F(",\"cu\":"));
  if (isKeya) {
    client.print(KeyaCurrentSensorReading / 20.0, 1);
  } else {
    client.print(sensorReading / 5.0, 1);
  }
  
  // Relays
  client.print(F(",\"rl\":["));
  for (int i = 0; i < 8; i++) {
    if (i > 0) client.print(',');
    client.print(bitRead(relay, i) ? "true" : "false");
  }
  client.print(']');
  
  // GNSS Stats
  client.print(F(",\"gp\":"));
  client.print(gnssPacketsSent);
  
  client.print(F(",\"go\":"));
  client.print(gnssOverflowCount);
  
  client.print(F(",\"gt\":"));
  client.print(gnssTimeoutCount);
  
  client.println('}');
}