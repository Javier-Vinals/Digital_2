/*
  Program to control LED (ON/OFF) from ESP32 using Serial Bluetooth
  by Daniel Carrasco -> https://www.electrosoftcloud.com/
*/
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial BT; // Objeto Bluetooth

String clientName = "ESP32-1";
bool connected;


//control 1
const int buttonPin32 = 32;
int buttonState32 = 0;
const int buttonPin33 = 33;
int buttonState33 = 0;
const int buttonPin25 = 25;
int buttonState25 = 0;
const int buttonPin26 = 26;
int buttonState26 = 0;
const int buttonPin27 = 27;
int buttonState27 = 0;

//control 2
const int buttonPin14 = 14;
int buttonState14 = 0;
const int buttonPin13 = 13;
int buttonState13 = 0;
const int buttonPin21 = 21;
int buttonState21 = 0;
const int buttonPin19 = 19;
int buttonState19 = 0;
const int buttonPin18 = 18;
int buttonState18 = 0;

//inicio
const int buttonPin23 = 22;
int buttonState23 = 0;



void setup() {
  //control 1
  pinMode(buttonPin32, INPUT_PULLUP);
  pinMode(buttonPin33, INPUT_PULLUP);
  pinMode(buttonPin25, INPUT_PULLUP);
  pinMode(buttonPin26, INPUT_PULLUP);
  pinMode(buttonPin27, INPUT_PULLUP);
  
  //control 2
  pinMode(buttonPin14, INPUT_PULLUP);
  pinMode(buttonPin13, INPUT_PULLUP);
  pinMode(buttonPin21, INPUT_PULLUP);
  pinMode(buttonPin19, INPUT_PULLUP);
  pinMode(buttonPin18, INPUT_PULLUP);

  //inicio
  pinMode(buttonPin23, INPUT_PULLUP);

  Serial.begin(115200); // Inicialización de la conexión en serie para la depuración
  BT.begin("ESP32-2", true); // Nombre de su dispositivo Bluetooth y en modo maestro
  //Serial.println("El dispositivo Bluetooth está en modo maestro. Conectando con el anfitrión ...");
  connected = BT.connect(clientName);
  if(connected) {
    Serial.println("¡Conectado exitosamente!");
  } else {
    while(!BT.connected(10000)) {
 //     Serial.println("No se pudo conectar. Asegúrese de que el dispositivo remoto esté disponible y dentro del alcance, luego reinicie la aplicación."); 
    }
  }
}
void loop() {

  //control 1
  buttonState32 = digitalRead(buttonPin32);
  buttonState33 = digitalRead(buttonPin33);
  buttonState25 = digitalRead(buttonPin25);
  buttonState26 = digitalRead(buttonPin26);
  buttonState27 = digitalRead(buttonPin27);
  //control 2
  buttonState14 = digitalRead(buttonPin14);
  buttonState13 = digitalRead(buttonPin13);
  buttonState21 = digitalRead(buttonPin21);
  buttonState19 = digitalRead(buttonPin19);
  buttonState18 = digitalRead(buttonPin18);
  //inicio
  buttonState23 = digitalRead(buttonPin23);


  //control 1
  if (buttonState32 == LOW) {
   // Serial.print("0");
    BT.write(49);
    delay(40);
  } if (buttonState33 == LOW){
  //  Serial.print("1");
    BT.write(48);
    delay(40);
  } if (buttonState25 == LOW){
  //  Serial.print("2");
    BT.write(50);
    delay(40);
  } if (buttonState26 == LOW){
  //  Serial.print("3");
    BT.write(51);
    delay(40);
  } if (buttonState27 == LOW){
   // Serial.println(4);
    BT.write(52);
    delay(40);

    //control 2
  } if (buttonState14 == LOW){
  //  Serial.println("5");
    BT.write(53);
    delay(40);
  } if (buttonState13 == LOW){
  //  Serial.println(6);
    BT.write(54);
    delay(40);
  } if (buttonState21 == LOW){
  //  Serial.println(7);
    BT.write(55);
    delay(40);
  } if (buttonState19 == LOW){
    BT.write(56);
    delay(40);
  } if (buttonState18== LOW){
  //  Serial.println(7);
    BT.write(57);
    delay(40);

    //inicio
  } if (buttonState23== LOW){
  //  Serial.println(7);
    BT.write(97); //a
    delay(40);

  }
}

