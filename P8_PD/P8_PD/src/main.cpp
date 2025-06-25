//1
#include <Arduino.h>

void setup() {
  Serial.begin(115200);         // UART0 (Terminal)
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // UART2 (RX: GPIO16, TX: GPIO17)

  Serial.println("Inicio del bucle UART");
}

void loop() {
  // Si llegan datos del terminal (UART0), se envían a UART2
  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
  }

  // Si llegan datos de UART2, se envían al terminal (UART0)
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }
}
//2
#include <TinyGPS++.h>

TinyGPSPlus gps;

HardwareSerial SerialGPS(2); // UART2

void setup() {
  Serial.begin(115200); // Monitor serie
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // RX2: GPIO16, TX2: GPIO17
  Serial.println("Esperando datos GPS...");
}

void loop() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitud: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitud: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Satélites: ");
    Serial.println(gps.satellites.value());
    Serial.print("Precisión HDOP: ");
    Serial.println(gps.hdop.hdop());
    Serial.println("------------");
  }
}
//3



#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define SerialMon Serial
#define SerialAT Serial1
#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

const char apn[] = "internet.movistar.es"; // Cambia según tu operador
const char gprsUser[] = "";
const char gprsPass[] = "";

const char* broker = "XXX.XXX.XXX.XXX";
const char* mqttUser = "usuario";
const char* mqttPass = "clave";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

void setup() {
  SerialMon.begin(115200);
  delay(10);

  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  digitalWrite(MODEM_PWRKEY, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  digitalWrite(MODEM_RST, HIGH);
  delay(1000);

  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  SerialMon.println("Inicializando módem...");
  modem.restart();
  modem.gprsConnect(apn, gprsUser, gprsPass);

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Mensaje recibido [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  for (int i = 0; i < len; i++) {
    SerialMon.print((char)payload[i]);
  }
  SerialMon.println();
}

void reconnect() {
  while (!mqtt.connected()) {
    SerialMon.print("Intentando conectar a MQTT...");
    if (mqtt.connect("espClient", mqttUser, mqttPass)) {
      SerialMon.println("Conectado");
      mqtt.subscribe("esp/output1");
    } else {
      SerialMon.print("Error, rc=");
      SerialMon.print(mqtt.state());
      delay(5000);
    }
  }
}

void loop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();
}
