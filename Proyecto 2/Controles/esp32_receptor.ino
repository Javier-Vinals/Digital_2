/*
  Program to control LED (ON/OFF) from ESP32 using Serial Bluetooth
  by Daniel Carrasco -> https://www.electrosoftcloud.com/
*/
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define LED 2

BluetoothSerial BT; // Objeto Bluetooth

void setup() {
  Serial.begin(115200); // Inicialización de la conexión en serie para la depuración
  Serial2.begin(115200);
  BT.begin("ESP32-1"); // Nombre de su dispositivo Bluetooth y en modo esclavo
  Serial.println("El dispositivo Bluetooth está listo para emparejarse");
  pinMode (LED, OUTPUT); // Cambie el pin LED a OUTPUT
}
void loop() {
  if (BT.available()) // Compruebe si recibimos algo de Bluetooth
  {
    int incoming = BT.read(); // Lee lo que recibimos
   // Serial.print("Recibido: ");
  //  Serial.println(incoming);


    if (incoming == 48){ // 0 en ASCII
      digitalWrite(LED, HIGH); // LED Encendido
      //BT.println("LED encendido"); // Envía el mensaje de texto a través de BT Serial
    //  Serial2.write("0");
      Serial2.print("0");
      Serial.print("0");
    }

    if (incoming == 49){ // 1 en ASCII
      digitalWrite(LED, LOW); // LED Apagado
    //  BT.println("LED apagado"); // Envía el mensaje de texto a través de BT Serial
     // Serial2.write("1");
      Serial2.print("1");
      Serial.print("1");
    }

    if (incoming == 50){ // 2 en ASCII
    //  Serial.write("2");
      Serial2.print("2");
      Serial.print("2");
    }

    if (incoming == 51){ // 3 en ASCII
      Serial2.print("3");
      Serial.print("3");
    }

    if (incoming == 52){ // 4 en ASCII
      Serial2.print("4");
      Serial.print("4");
    }

    if (incoming == 53){ // 5 en ASCII
      Serial2.print("5");
      Serial.print("5");
    }

    if (incoming == 54){ // 6 en ASCII
      Serial2.print("6");
      Serial.print("6");
    }

    if (incoming == 55){ // 7 en ASCII
      Serial2.print("7");
      Serial.print("7");
    }

    if (incoming == 56){ // 8 en ASCII
      Serial2.print("8");
      Serial.print("8");
    }

    if (incoming == 57){ // 9 en ASCII
      Serial2.print("9");
      Serial.print("9");
    }

    if (incoming == 97){ // a en ASCII
      Serial2.print("a");
      Serial.print("a");
    }

  }
  delay(20);
}