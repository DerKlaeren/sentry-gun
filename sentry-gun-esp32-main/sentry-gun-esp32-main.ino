#include <ESP32Servo.h>  // Bibliothek für Servo-Steuerung auf ESP32
#include <Ultrasonic.h>  // Bibliothek für Ultraschallsensor
#include <WiFi.h>        // WiFi für Video- und Steuerungsübertragung
#include <WebServer.h>   // Webserver-Bibliothek
#include <WebSocketsServer.h> // Websocket-Bibliothek



// Pin-Definitionen
const int servoYawPin = 18;    // Servo für Yaw (Drehung)
const int servoPitchPin = 19;  // Servo für Pitch (Höheneinstellung)
const int escMotor1Pin = 22;   // ESC für Motor 1
const int escMotor2Pin = 23;   // ESC für Motor 2
const int ultrasonicTriggerPin = 5;
const int ultrasonicEchoPin = 4;
#define RXD2 16  // RX zur ESP32-CAM
#define TXD2 17  // TX zur ESP32-CAM

// WiFi-Daten
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Servo- und ESC-Objekte erstellen
Servo servoYaw;
Servo servoPitch;
Servo escMotor1;
Servo escMotor2;

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

// Steuerungsvariablen
int yawAngle = 90;    // Yaw (Drehwinkel)
int pitchAngle = 90;  // Pitch (Neigungswinkel)


void setup() {
  // Serielle Kommunikation für Debugging
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Serielle Verbindung zur ESP32-CAM

  // WiFi verbinden
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi verbunden!");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.localIP());

  // Servos initialisieren
  servoYaw.attach(servoYawPin);
  servoPitch.attach(servoPitchPin);

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

// Websocket Steuerung
void webSocketEvent(uint8_t *payload, size_t length) {
    String command = String((char*)payload);
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
    } else if (commane == "stop") {
      
    }
}

// Funktion zum Feuern der Nerf-Darts
void fireNerfGun() {
  escMotor1.write(180);
  escMotor2.write(180);
  delay(500);
  escMotor1.write(90);
  escMotor2.write(90);
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

// HTML-Seite mit Steuerungsbuttons und initialisierung des Websockets
void handleRoot() {
    String html = "<html><body><h1>Nerf Gun Turret Control</h1>";
    html += "<button onmousedown=\"sendCommand('left')\" onmouseup=\"stopCommand()\">Drehen Links</button>";
    html += "<button onmousedown=\"sendCommand('right')\" onmouseup=\"stopCommand()\">Drehen Rechts</button>";
    html += "<button onmousedown=\"sendCommand('up')\" onmouseup=\"stopCommand()\">Neigen Hoch</button>";
    html += "<button onmousedown=\"sendCommand('down')\" onmouseup=\"stopCommand()\">Neigen Runter</button>";
    html += "<button onclick=\"sendCommand('fire')\">Feuer</button>";
    html += "<br><br><img src='/stream' width='400' height='300'>";
    html += "<script>var websocket = new WebSocket('ws://' + location.hostname + ':81');"
            "function sendCommand(command) { websocket.send(command); }"
            "function stopCommand() { websocket.send('stop'); }</script>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}
