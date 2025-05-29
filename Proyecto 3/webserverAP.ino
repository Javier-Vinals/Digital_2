#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#define I2CSlaveAddress1 0x55
#define I2CSlaveAddress2 0x45

#define I2C_SDA 21
#define I2C_SCL 22

const char* ssid = "ESP3232";
const char* password = "12345678";

IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

String received1 = "0000";
String received2 = "0000";

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/actualizar", handle_actualizar);
  server.onNotFound(handle_NotFound);

  server.on("/", []() {
  server.sendHeader("Location", "/actualizar");
  server.send(303); // Redirección temporal
});


  server.begin();
  Serial.println("HTTP server started");
}

void loop() {

  Serial.print(received1);
  Serial.print(received2);


  server.handleClient();

  // Leer datos del esclavo 1
  Wire.requestFrom(I2CSlaveAddress1, 4);
  received1 = "";
  while (Wire.available()) {
    received1 += (char)Wire.read();
  }

  // Leer datos del esclavo 2
  Wire.requestFrom(I2CSlaveAddress2, 4);
  received2 = "";
  while (Wire.available()) {
    received2 += (char)Wire.read();
  }



 uint8_t estadoParqueos = 0;

// Convertir cada nibble de string a entero binario
uint8_t nibble1 = 0;
uint8_t nibble2 = 0;

// Parsear received1: 4 bits más significativos
for (int i = 0; i < 4 && i < received1.length(); i++) {
  if (received1[i] == '1') {
    nibble1 |= (1 << (3 - i));  // Mantenemos el orden del string
  }
}

// Parsear received2: 4 bits menos significativos
for (int i = 0; i < 4 && i < received2.length(); i++) {
  if (received2[i] == '1') {
    nibble2 |= (1 << (3 - i));  // Mantenemos el orden del string
  }
}

estadoParqueos = (nibble1 << 4) | nibble2;


  Wire.beginTransmission(I2CSlaveAddress1);
  Wire.write(estadoParqueos);
  Wire.endTransmission();

  Wire.beginTransmission(I2CSlaveAddress2);
  Wire.print(estadoParqueos);
  Wire.endTransmission();

}


void handle_actualizar() {
  bool parqueos[8];
  for (int i = 0; i < 4 && i < received1.length(); i++) {
    parqueos[i] = (received1[i] == '1');  // parqueos 1–4
  }
  for (int i = 0; i < 4 && i < received2.length(); i++) {
    parqueos[i + 4] = (received2[i] == '1');  // parqueos 5–8
  }
  server.send(200, "text/html", SendHTML2(parqueos));
}



void handle_NotFound() {
  server.send(404, "text/plain", "Not found");
}

String SendHTML2(bool parqueos[8]) {
  String html = "<!DOCTYPE html>\n";
  html += "<html lang='es'>\n<head>\n";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>\n";
  html += "<meta charset='UTF-8'>\n";
  html += "<title>Estado Parqueomatic 🚗</title>\n";
  html += "<meta http-equiv='refresh' content='5'>\n";

  html += "<style>";
  html += "@import url('https://fonts.googleapis.com/css2?family=Press+Start+2P&display=swap');";
  html += "body { margin: 0; font-family: 'Press Start 2P', cursive; background: #5c94fc; color: white; text-align: center; overflow-x: hidden; }";
  html += "h1 { font-size: 16px; margin: 30px 0 10px; text-shadow: 2px 2px #000; }";
  html += ".button { display: inline-block; background: #3498db; padding: 10px 20px; color: white; font-size: 12px;";
  html += "text-decoration: none; border-radius: 5px; border: 3px solid #1c5aa6; margin-bottom: 20px; box-shadow: 4px 4px #000; }";
  html += "table { margin: 0 auto; border-collapse: collapse; background: rgba(255, 255, 255, 0.9); color: black;";
  html += "border: 4px solid #333; font-size: 12px; }";
  html += "th, td { border: 2px solid #444; padding: 12px 20px; }";
  html += ".ocupado { background: #ff6666; }";
  html += ".disponible { background: #90ee90; }";

  // Nubes animadas
  html += ".nube { position: absolute; top: 40px; width: 100px; height: 60px;";
  html += "background: white; border-radius: 50%; box-shadow: 30px 0 white, 60px 0 white;";
  html += "animation: moverNube 60s linear infinite; opacity: 0.6; }";
  html += ".nube2 { top: 80px; left: 200px; animation-delay: 10s; }";
  html += ".nube3 { top: 120px; left: 400px; animation-delay: 20s; }";
  html += "@keyframes moverNube { from { left: -150px; } to { left: 100%; } }";

  // Ladrillos reales (3 filas intercaladas)
  html += ".suelo { position: fixed; bottom: 0; left: 0; width: 100%; z-index: -1; }";
  html += ".fila-ladrillos { display: flex; height: 40px; }";
  html += ".fila-ladrillos.offset .ladrillo:first-child { margin-left: 20px; }";
  html += ".ladrillo { width: 40px; height: 40px; background: #b22222; border: 2px solid #8b0000; box-sizing: border-box; }";

  html += "</style>\n</head>\n<body>\n";

  // Nubes
  html += "<div class='nube'></div>\n";
  html += "<div class='nube nube2'></div>\n";
  html += "<div class='nube nube3'></div>\n";

  html += "<h1>Estado Parqueomatic 🚗</h1>\n";
  html += "<a href='/actualizar' class='button'>Actualizar</a>\n";

  html += "<table>\n<thead>\n<tr><th># Parqueo</th><th>Estado</th></tr>\n</thead>\n<tbody>\n";

  for (int i = 0; i < 8; i++) {
    html += "<tr class='";
    html += parqueos[i] ? "ocupado" : "disponible";
    html += "'><td>" + String(i + 1) + "</td><td>";
    html += parqueos[i] ? "Ocupado 🚫" : "Disponible 🚗";
    html += "</td></tr>\n";
  }

  html += "</tbody>\n</table>\n";

  // Suelo de ladrillos (3 filas intercaladas)
  html += "<div class='suelo'>\n";
  for (int fila = 0; fila < 3; fila++) {
    html += "<div class='fila-ladrillos" + String((fila % 2 == 1) ? " offset" : "") + "'>\n";
    for (int i = 0; i < 50; i++) {
      html += "<div class='ladrillo'></div>\n";
    }
    html += "</div>\n";
  }
  html += "</div>\n";

  html += "</body>\n</html>\n";
  return html;
}
