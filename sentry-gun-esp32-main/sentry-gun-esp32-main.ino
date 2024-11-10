#include <ESP32Servo.h>        // Bibliothek für Servo-Steuerung auf ESP32
#include <Ultrasonic.h>        // Bibliothek für Ultraschallsensor
#include <WiFi.h>              // WiFi für Video- und Steuerungsübertragung
#include <WebServer.h>         // Webserver-Bibliothek
#include <WebSocketsServer.h>  // Websocket-Bibliothek
#include <ESPmDNS.h>           // DNS Service für Hostnamen
#include <WiFiManager.h>       // WiFiManager Bibliothek




// Pin-Definitionen
const int servoYawPin = 18;     // Servo für Yaw (Drehung)
const int servoPitchPin = 19;   // Servo für Pitch (Höheneinstellung)
const int servoReloadPin = 21;  // Servo fürs Nachladen
const int escMotor1Pin = 22;    // ESC für Motor 1
const int escMotor2Pin = 23;    // ESC für Motor 2

const int laserPin = 27;  // Pin für Ziellaser

const int ultrasonicTriggerPin = 5;
const int ultrasonicEchoPin = 4;

#define RXD2 16  // RX zur ESP32-CAM
#define TXD2 17  // TX zur ESP32-CAM

// Hostnamen fürs Netzwerk
const char* HOSTNAME = "sentrygun";

// Flag zum Steuern des Lasers
bool laserEnabled = false;



// Servo- und ESC-Objekte erstellen
Servo servoYaw;
Servo servoPitch;
Servo escMotor1;
Servo escMotor2;
Servo servoReload;

// Ultraschallsensor-Objekt
Ultrasonic ultrasonic(ultrasonicTriggerPin, ultrasonicEchoPin);

// Webserver auf Port 80
WebServer server(80);
// WebSocketserver auf Port 81
WebSocketsServer webSocket = WebSocketsServer(81);


// Serielle Verbindung zur ESP32-CAM
HardwareSerial SerialCam(2);

// Variablen zur Steuerung
// int targetDistance = 100; // Beispielwert für Zielentfernung in cm

// Steuerungsvariablen mit initialen winkel
// 0 - Rotation gegen den UZ
// 90 - Stillstand
// 180 - Rotatin mit UZ
int yawAngle = 90;    // Yaw (Drehwinkel)
int pitchAngle = 90;  // Pitch (Neigungswinkel)
int reloadAngle = 90;

void setup() {

  // Setzt den Laser-Pin als Ausgang
  pinMode(laserPin, OUTPUT);

  // Serielle Kommunikation für Debugging
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Serielle Verbindung zur ESP32-CAM

  delay(2000);

  // WiFiManager Initialisierung
  WiFiManager wifiManager;

  // Statische IP für den AP konfigurieren
  wifiManager.setAPStaticIPConfig(IPAddress(10, 0, 1, 1), IPAddress(10, 0, 1, 1), IPAddress(255, 255, 255, 0));

  // WiFi-Konfigurationsportal öffnen, wenn keine gespeicherten Netzwerke vorhanden sind
  if (!wifiManager.autoConnect("ESP32-ConfigAP")) {
    Serial.println("WLAN-Verbindung fehlgeschlagen! Starte neu...");
    delay(3000);
    ESP.restart();
  }
  Serial.println("WLAN verbunden!");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.localIP());

  // Servos initialisieren
  servoYaw.attach(servoYawPin);
  servoPitch.attach(servoPitchPin);
  servoReload.attach(servoReloadPin);

  // ESC initialisieren
  escMotor1.attach(escMotor1Pin, 1000, 2000);  // PWM für ESCs (1000-2000 us)
  escMotor2.attach(escMotor2Pin, 1000, 2000);

  // Webserver-Endpunkte einrichten
  server.on("/", HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, streamVideo);
  server.on("/control", HTTP_GET, handleControl);
  server.begin();

  // Websocket starten
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // mDNS-Dienst starten
  if (!MDNS.begin(HOSTNAME)) {
    Serial.println("Fehler beim Starten von mDNS");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS gestartet. Erreichbar unter " + String(HOSTNAME) + ".local");
}

void loop() {
  //  Abstand messen und Geschützsteuerung
  //  int distance = ultrasonic.read();
  //  Serial.print("Abstand: ");
  //  Serial.println(distance);

  //  Schießvorgang auslösen, wenn das Ziel nahe genug ist
  //  if (distance <= targetDistance) {
  //    fireNerfGun();
  //  }

  //  Beispielhafter Aufruf für das Ausrichten des Turms
  //  controlTurret(90, 45); // Beispielhaft: 90° Yaw, 45° Pitch

  // Webserver-Client-Handling
  server.handleClient();
  webSocket.loop();
}

//  Funktion zur Steuerung der Dreh- und Höheneinstellung des Geschützes
//  void controlTurret(int yawAngle, int pitchAngle) {
//    servoYaw.write(yawAngle);
//    servoPitch.write(pitchAngle);
//  }

// Steuerungsbefehle verarbeiten
void handleControl() {
  String command = server.arg("command");

  if (command == "left") {
    yawAngle = max(0, yawAngle - 10);
    servoYaw.write(yawAngle);
  } else if (command == "right") {
    yawAngle = min(180, yawAngle + 10);
    servoYaw.write(yawAngle);
  } else if (command == "up") {
    pitchAngle = max(0, pitchAngle - 10);
    servoPitch.write(pitchAngle);
  } else if (command == "down") {
    pitchAngle = min(180, pitchAngle + 10);
    servoPitch.write(pitchAngle);
  } else if (command == "fire") {
    fireNerfGun();
  }

  server.send(200, "text/plain", "Command received");
}

// Laser an oder aus togglen
void toggleLaser() {
  laserEnabled = !laserEnabled;

  if (laserEnabled) {
    digitalWrite(laserPin, HIGH);
  } else {
    digitalWrite(laserPin, LOW);
  }

  Serial.println("toggle.laser = " + String(laserEnabled));
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("WebSocket %u disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("WebSocket %u connected from %s\n", num, ip.toString().c_str());
        // Sende eine Begrüßungsnachricht
        webSocket.sendTXT(num, "Welcome to ESP32 WebSocket!");
        break;
      }
    case WStype_TEXT:
      Serial.printf("WebSocket %u received text: %s\n", num, (char*)payload);
      webSocketEventToControll(num, type, payload, length);
      break;
    case WStype_ERROR:
      Serial.printf("WebSocket error: %s\n", (char*)payload);
      break;
    default:
      Serial.printf("Unknown WebSocket event type: %d\n", type);
      break;
  }
}



// Websocket Steuerung
void webSocketEventToControll(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {

  Serial.printf("WebSocket event: Type=%d, Payload Length=%d\n", type, length);

  const int servoSpeedMovinPos = 60;
  const int servoSpeedMovinNeg = 120;
  const int servoSpeedMovinNeutral = 90;

  String command = String((char*)payload);
  if (command == "left") {
    yawAngle = servoSpeedMovinPos;
    servoYaw.write(yawAngle);
  } else if (command == "right") {
    yawAngle = servoSpeedMovinNeg;
    servoYaw.write(yawAngle);
  } else if (command == "up") {
    pitchAngle = servoSpeedMovinPos;
    servoPitch.write(pitchAngle);
  } else if (command == "down") {
    pitchAngle = servoSpeedMovinNeg;
    servoPitch.write(pitchAngle);
  } else if (command == "stop") {
    yawAngle = servoSpeedMovinNeutral;
    pitchAngle = servoSpeedMovinNeutral;
    servoYaw.write(yawAngle);
    servoPitch.write(pitchAngle);
  } else if (command == "fire") {
    fireNerfGun();
  }  else if (command == "laser") {
    toggleLaser();
  }
  // Sende den aktuellen Winkel zurück an den Client
  String response = "Yaw: " + String(yawAngle) + ", Pitch: " + String(pitchAngle) + ", Laser: " + String(laserEnabled);
  webSocket.sendTXT(num, response);
}



// Funktion zum Feuern der Nerf-Darts
void fireNerfGun() {
  // Motoren starten
  escMotor1.write(90);
  escMotor2.write(90);
  delay(200);

  // Simple ladebewegung
  servoReload.write(0);
  delay(200);
  servoReload.write(180);
  delay(200);
  servoReload.write(90);

  // Feuer
  delay(500);
  escMotor1.write(0);
  escMotor2.write(0);
  Serial.println("Feuer!");
}



// Funktion zum Streamen des Videos als MJPEG
void streamVideo() {
  WiFiClient client = server.client();

  // MJPEG-Header senden
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {
    if (SerialCam.available() >= sizeof(uint32_t)) {
      // Bildgröße empfangen
      uint32_t imgSize;
      SerialCam.readBytes((uint8_t*)&imgSize, sizeof(imgSize));

      // Bilddaten empfangen
      uint8_t imgBuf[imgSize];
      SerialCam.readBytes(imgBuf, imgSize);

      // MJPEG-Frame senden
      client.println("--frame");
      client.println("Content-Type: image/jpeg");
      client.print("Content-Length: ");
      client.println(imgSize);
      client.println();
      client.write(imgBuf, imgSize);
      client.println();
    }
    delay(30);  // Pause zur Bildstabilisierung
  }
}

void handleRoot() {
  String html = "<html><body><h1>Nerf Gun Turret Control</h1>";
  html += "<button onmousedown=\"sendCommand('left')\" onmouseup=\"stopCommand()\">Drehen Links</button>";
  html += "<button onmousedown=\"sendCommand('right')\" onmouseup=\"stopCommand()\">Drehen Rechts</button>";
  html += "<button onmousedown=\"sendCommand('up')\" onmouseup=\"stopCommand()\">Neigen Hoch</button>";
  html += "<button onmousedown=\"sendCommand('down')\" onmouseup=\"stopCommand()\">Neigen Runter</button>";
  html += "<button onclick=\"sendCommand('laser')\">Laser</button>";
  html += "<button onclick=\"sendCommand('fire')\">Feuer</button>";
  html += "<div id=\"angles\">Yaw: 90, Pitch: 90</div>";

  html += "<script>"
          "var websocket;"
          
          "function initWebSocket() {"
          "   websocket = new WebSocket('ws://' + location.hostname + ':81');"
          "   websocket.onopen = function(event) { console.log('WebSocket connection established'); };"
          "   websocket.onclose = function(event) { console.log('WebSocket connection closed'); reconnectWebSocket(); };"
          "   websocket.onerror = function(event) { console.error('WebSocket error:', event); };"
          "   websocket.onmessage = function(event) {"
          "       document.getElementById('angles').innerText = event.data;"
          "   };"
          "}"

          "function reconnectWebSocket() {"
          "   setTimeout(function() {"
          "       console.log('Reconnecting WebSocket...');"
          "       initWebSocket();"
          "   }, 2000);"
          "}"
          
          "function sendCommand(command) {"
          "   if (websocket.readyState === WebSocket.OPEN) {"
          "       websocket.send(command);"
          "       console.log('Command sent:', command);"
          "   } else {"
          "       console.warn('WebSocket connection not ready, command not sent:', command);"
          "   }"
          "}"
          
          "function stopCommand() {"
          "   if (websocket.readyState === WebSocket.OPEN) {"
          "       websocket.send('stop');"
          "       console.log('Stop command sent');"
          "   } else {"
          "       console.warn('WebSocket connection not ready, stop command not sent');"
          "   }"
          "}"
          
          "window.onload = initWebSocket;"
          "</script>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}
