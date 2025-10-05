// ========== NEUE DATEI: zWebserver.ino ==========

#include <NativeEthernet.h>

// Externe Variablen aus anderen Dateien
extern char imuHeading[6];
extern char imuRoll[6];
extern char imuPitch[6];
extern char imuYawRate[6];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];
extern float gpsSpeed;
extern bool Autosteer_running;
extern uint8_t watchdogTimer;
extern const uint16_t WATCHDOG_THRESHOLD;
extern float steerAngleActual;
extern int16_t pwmDisplay;


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
int webSendPosition = 0;

void webserverSetup() {
  webServer.begin();
  Serial.print("Webserver started on: http://");
  Serial.print(Ethernet.localIP());
  Serial.println("/");
}

// Separate HTML-Seite die JSON lädt (für Browser)
void sendWebPageFast(EthernetClient &client) {
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
  client.print("<!DOCTYPE html><html><head><meta charset='UTF-8'><title>AIO v4</title>");
  client.print("<style>body{font-family:Arial;margin:20px;background:#f0f0f0}");
  client.print(".card{background:white;padding:15px;margin:10px 0;border-radius:5px;border-left:4px solid #4CAF50}");
  client.print(".label{color:#666;font-size:13px}.value{font-size:20px;color:#333;margin-top:5px}</style></head><body>");
  client.print("<h1>AIO v4 Status</h1><div id='data'>Loading...</div>");
  client.print("<script>function load(){fetch('/json').then(r=>r.json()).then(d=>{");
  client.print("document.getElementById('data').innerHTML=");
  client.print("'<div class=card><div class=label>GPS Fix</div><div class=value>'+d.fix+'</div></div>'+");
  client.print("'<div class=card><div class=label>Satellites</div><div class=value>'+d.sats+'</div></div>'+");
  client.print("'<div class=card><div class=label>Speed</div><div class=value>'+d.speed+' km/h</div></div>'+");
  client.print("(d.steer!=null?'<div class=card><div class=label>Steering</div><div class=value>'+(d.steer==1?'ACTIVE':'OFF')+'</div></div>':'')+");
  client.print("(d.angle!=null?'<div class=card><div class=label>Angle</div><div class=value>'+d.angle+'&deg;</div></div>':'')+");
  client.print("'<div class=card><div class=label>Uptime</div><div class=value>'+d.up+' s</div></div>'");
  client.print("}).catch(e=>console.log(e))}load();setInterval(load,3000)</script></body></html>");
}

// Minimale JSON-Antwort (schnell!)
void sendJsonDataFast(EthernetClient &client) {
  client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n{");

  client.print("\"fix\":\"");
  client.print(fixQuality);
  client.print("\",\"sats\":\"");
  client.print(numSats);
  client.print("\",\"speed\":");
  client.print(gpsSpeed, 1);

  if (Autosteer_running) {
    client.print(",\"steer\":");
    client.print(watchdogTimer < WATCHDOG_THRESHOLD ? "1" : "0");
    client.print(",\"angle\":");
    client.print(steerAngleActual, 1);
    client.print(",\"pwm\":");
    client.print(pwmDisplay);
  }

  client.print(",\"up\":");
  client.print(millis() / 1000);

  client.println("}");
}


// Non-blocking Webserver Loop - MAX 5ms pro Aufruf!
void webserverLoop() {
  switch (webState) {

    case WS_IDLE: {
      // Prüfe auf neuen Client (sehr schnell)
      EthernetClient newClient = webServer.available();
      if (newClient) {
        webClient = newClient;
        webState = WS_READING_REQUEST;
        webStateTimer = millis();
        webRequest = "";
        webRequest.reserve(100);
        //Serial.println("Web: New client");
      }
      break;
    }

    case WS_READING_REQUEST: {
      // Timeout nach 200ms
      if (millis() - webStateTimer > 200) {
        //Serial.println("Web: Request timeout");
        webClient.stop();
        webState = WS_IDLE;
        break;
      }

      // Lese nur wenige Bytes pro Loop
      int bytesRead = 0;
      while (webClient.available() && bytesRead < 30) {
        char c = webClient.read();
        if (webRequest.length() < 100) {
          webRequest += c;
        }

        // Ende der ersten Zeile?
        if (c == '\n') {
          // Request komplett - starte Antwort
          webState = WS_SENDING_RESPONSE;
          webSendPosition = 0;
          //Serial.print("Web: Request ");
          //Serial.println(webRequest);
          break;
        }
        bytesRead++;
      }
      break;
    }

    case WS_SENDING_RESPONSE: {
       // Prüfen, ob der Request "/json" war, ansonsten die HTML-Seite senden
      if (webSendPosition == 0) {
        if (webRequest.indexOf("GET /json") >= 0) {
          sendJsonDataFast(webClient);
        } else {
          sendWebPageFast(webClient);
        }
        webSendPosition = 1;
      }

      // Sofort schließen
      webState = WS_CLOSING;
      webStateTimer = millis();
      break;
    }

    case WS_CLOSING: {
      // Warte kurz, dann schließen
      if (millis() - webStateTimer > 10) {
        webClient.stop();
        webState = WS_IDLE;
        //Serial.println("Web: Closed");
      }
      break;
    }
  }
}