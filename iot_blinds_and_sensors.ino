#include <Ticker.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

void sensarBMP180();
void sensarDHT11();
void sensarLuz();
void sensarRSSI();
void persiana(int);

// Uso knolleary library para MQTT

// Problemas y mejoras:

// proximos pasos:

// usar gpio15, con internal pullup, como entrada de switch1

// Definir estados iniciales(!), cuando bootea o se resetea.

// definir 2 pines de salida para el combo de reles para la persiana.

// en cuanto a la persiana a efectos del 8266 defino 3 estados. SUBE, BAJA, PARA. Esto replica comportamiento via switch de hardware.
// estimo posicion basado en el tiempo que lleva moviendose, y la informo via MQTT
// me falta poder recibir un comando con la posicion y mover hasta ahi. tambien se puede hacer un lazo de control en openhab
// porque le envio feedback


// Sensor PIR

// Ver cuantos pines quedan disponibles como entradas/salidas.

// hay que definir los pines como entrada? por ahora funcionan sin definirlos pero puede fallar?

// agregar 2 o 3 alternativas de AP para que se conecte a alguno (mayor senial?)
// alternativas: WifiMulti o WiFImanager

// en el futuro pensar en que envie un solo json con todas las lecturas?
// es mas eficiente que mandar todo por separado? vale la pena siendo 220v powered?

// Instancio ticker para timer de n segundos de sensores
Ticker sensar;

// Para que mida la entrada ADC. Sino aparentemente mide Vin. Util para casos a bateria.
ADC_MODE(ADC_TOUT);

// instancio bmp180 presion y temperatura
SFE_BMP180 pressure;
#define ALTITUDE 30.0 // Altitude of Buenos Aires, meters.

// Declaro variables globales
const char *ssid            = "libertad";		         // cannot be longer than 32 characters!
const char *pass            = "satanylola";		         //
char* tempTopic             = "openhab/sensor1/temperature";     // topic to publish temperatures (DHT11) readings to
char* temp2Topic            = "openhab/sensor1/temperature2";    // topic to publish temperatures (bmp180) readings to
char* pressureTopic         = "openhab/sensor1/pressure";        // topic for barometric pressure
char* humidityTopic         = "openhab/sensor1/humidity";        // publish humidity readings
char* lightTopic            = "openhab/sensor1/light";           // Luz (LDR Analogico)
char* signalTopic           = "openhab/sensor1/rssi";            // nivel de senial wifi
char* switch1Topic          = "openhab/sensor1/switch/1";
char* switch1statusTopic    = "openhab/sensor1/switch/status/1";
char* estadopersianaTopic   = "openhab/sensor1/persiana/status";  // para que openhab sepa si la persiana esta subiendo o bajando a traves
char* posicionpersianaTopic = "openhab/sensor1/persiana/posicion"; // 0 a 100, posicion estimada persiana
char* mqtt_server           = "192.168.1.4";
char message_buff[100];
bool sensarahora;
bool cambiopin;
String pubString;
bool firstrun = true;

bool cambioenpersiana = false;
int comandopersiana;
float recorridopersiana = 10000.0; // Calculo 10 segundos de recorrido total de persiana. Esto hay que ajustarlo y calibrarlo para cada persiana.
int moviendo;
unsigned long iniciomovimiento;
int posicionpersiana = 100; // inicializo como 100, que es totalmente abajo, porque en setup() la bajo para arrancar
int posicionanterior = 100;
int posicionreporte = 100;

// instancio DHT
#define DHTTYPE DHT11 // DHT11 or DHT22
#define DHTPIN  12
DHT dht(DHTPIN, DHTTYPE, 11);
// fin DHT

// callback MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String sTopic;
  // no deberia tener serial println en este callback por un tema de blocking
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  //for (int i = 0; i < length; i++) {
  //  Serial.print((char)payload[i]);
  //}
  //Serial.println();

  // Creo un string con payload
  // create character buffer with ending null terminator (string)
  int i = 0;
  for (i = 0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  String msgString = String(message_buff);

  sTopic = String(topic);

  if (sTopic.equals("openhab/sensor1/persiana")) {
    // Recibi mensaje persiana
    Serial.println("persiana");
    // Los comandos de persiana son SUBE, BAJA y PARA
    if (msgString.equals("UP")) {
      Serial.println("MQTT UP");
      persiana(-1);
    } else if (msgString.equals("DOWN")) {
      Serial.println("MQTT DOWN");
      persiana(1);
    } else if (msgString.equals("STOP")) {
      Serial.println("MQTT STOP");
      persiana(0);
    } else
    {
      Serial.println("MQTT Persiana comando desconocido");
    }
    // Fin persiana
  }  // else if  (sTopic.equals("openhab/sensor1/persiana/posicion"))  { }

  else if (sTopic.equals("openhab/sensor1/switch/1")) {   // cambiar por variable switch1Topic (verificar!)
    // Recibi mensaje switch
    //Serial.println("switch1");
    // switch puede ser 0 o 1
    if ((char)payload[0] == '1') {
      //Serial.println("Switch 1 ON");
      switch1(1);
    } else { // Cualquier otra cosa, apago. Sirve por las dudas. A menos que sea "1" explicitamente, apago.
      //Serial.println("Switch 1 OFF");
      switch1(0);
    }
    // fin switch
  }
}

WiFiClient wclient;
PubSubClient mqttclient(wclient);

void sensarahorafunc()
{
  sensarahora = true;
}

void sensores() {
  sensarBMP180();
  sensarDHT11();
  sensarLuz();
  sensarRSSI();
}

void setup() {
  Serial.begin(115200);  // Setup console
  Serial.println("Consola start.");
  // Sensor presion bmp180
  Wire.begin(4, 5);
  Serial.println("Wire beginned, pines 4 y 5");

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail\n\n");
    while (1); // Pause forever.
  }
  // sensor presion fin

  dht.begin();
  delay(10);
  Serial.println();

  // Lectura LUZ
  pinMode(A0, INPUT);

  // arranco el ticker
  // Cada 60 segundos llamo a la funcion sensar
  sensar.attach(60, sensarahorafunc);

  // Seteo pines entrada persiana
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  // IRQ en pines de entradas. para leer interruptores
  attachInterrupt(14, pinPersianaChanged, CHANGE); // funciona, subir persiana
  attachInterrupt(13, pinPersianaChanged, CHANGE); // funciona, bajar persiana


   // esto funciona perfecto, pero estoy usando pin 15 para controlar la persiana
  //pinMode(15,INPUT_PULLUP);
  //attachInterrupt(15,  pinChanged, CHANGE); // probar

  cambiopin = false;

  // mqtt version knolleary
  mqttclient.setServer(mqtt_server, 1883);

  // uso el led interno como display
  pinMode(2, OUTPUT);
  
  // prueba pines salida persiana. No puedo usar gpio09 y gpio10 porque los usa la FLASH
  pinMode(16, OUTPUT);
  pinMode(15,OUTPUT);
  
  
  // Como parte de la inicializacion, bajo la persiana por completo y seteo el estado inicial en 0%
  // Informo este 0% via MQTT
  // una ventaja adicional de esto es que en caso de corte de luz, al volver todas las persianas se bajan.
  persiana(1);
  delay(recorridopersiana * 1.2); // Le doy cierto margen para asegurar que esta totalmente baja
  persiana(0);

} // Fin SETUP

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, pass);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
    Serial.println(WiFi.localIP());
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttclient.connected()) {
      if (mqttclient.connect("sensor1", "sensores", "picad0r3s")) {
        Serial.println("MQTT Client connected");
        mqttclient.setCallback(callback);
        mqttclient.subscribe("openhab/sensor1/persiana");
        mqttclient.subscribe("openhab/sensor1/persiana/posicion");
        mqttclient.subscribe("openhab/sensor1/switch/1");
      }
    }
    if (mqttclient.connected())
      mqttclient.loop();
  }

  if (sensarahora) {
    //Serial.println("Ahora deberia sensar");
    sensores();
    sensarahora = false;
  }

  // Solo primera corrida para que no tarde en informar cosas cuando se corta la luz
  if (firstrun) {
    delay(5000);  // puse esto porque a veces MQTT tarda unos segundos en conectar
    sensores();
    // Informo estado de persiana como 100% (totalmente abajo)
    mqttclient.publish(posicionpersianaTopic, "100");
    firstrun = false;
  }

  ///// persiana. detecto en ISR que cambiaron los pines y obro en consecuencia
  if (cambioenpersiana) {
    // Determino la configuracion actual de los dos pines para saber que tengo que hacer
    // 00 Stop
    // 10 Subir
    // 01 Bajar
    // 11 Ambos. Error. Lo tengo que tomar como STOP, por las dudas

    delay(50); // debouncing
    // Leo estado de las entradas
    int pinSubir = digitalRead(13);
    int pinBajar = digitalRead(14);

    if (pinSubir == HIGH && pinBajar == LOW) {
      persiana(1);
    } else if (pinBajar == HIGH && pinSubir == LOW) {
      persiana(-1);
    } else
    {
      persiana(0);   // Si no es subir ni bajar, es parar.
    }
    cambioenpersiana = false;
  }
  //// fin persiana

  // primer intento de estimar posicion persiana
  if (moviendo == 1 || moviendo == -1) {

    float a = ((millis() - iniciomovimiento) / recorridopersiana) * 100;
    posicionpersiana = posicionanterior + (moviendo * a);

    if (posicionpersiana > 100) {
      posicionpersiana = 100;  // revisar estos topes
    }
    if (posicionpersiana < 0)   {
      posicionpersiana = 0;
    }

    if (abs(posicionpersiana - posicionreporte) >= 1) {
      // cambio 2 o mas %, informo
      posicionreporte = posicionpersiana; // actualizo referencia para volver a informar cuando me aleje 2%
      pubString = String(posicionpersiana);
      pubString.toCharArray(message_buff, pubString.length() + 1);
      mqttclient.publish(posicionpersianaTopic, message_buff);
    }
  } // estime posicion persiana


  // Switch digital
  if (cambiopin) {
      delay(50); // debounce
      int pinNueve = digitalRead(15);
      switch1(pinNueve);
      cambiopin=false;
  }

  delay(0); // para que el OS pueda correr sus cosas, como wifi
} // Fin Loop.

void sensarBMP180() {
  /// Temperatura y Presion (BMP180)
  char status;
  double T, P, p0;

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T, 2);
      Serial.println(" deg C, ");

      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P, 2);
          Serial.print(" mb, ");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0, 2);
          Serial.print(" mb, ");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          //a = pressure.altitude(P,p0);
          //Serial.print("computed altitude: ");
          //Serial.print(a,0);
          //Serial.print(" meters, ");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  // publico a MQTT temperatura y presion de BMP180
  pubString = String(T);
  pubString.toCharArray(message_buff, pubString.length() + 1);
  mqttclient.publish(temp2Topic, message_buff);

  pubString = String(P);
  pubString.toCharArray(message_buff, pubString.length() + 1);
  mqttclient.publish(pressureTopic, message_buff);
}

void sensarDHT11() {
  // DHT
  float h, t;

  Serial.println("checking DHT");
  h = dht.readHumidity();
  t = dht.readTemperature();

  // h = h*1.23;
  // t = t*1.1;

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {  // Ojo que esto puede ocultar errores de lectura

    Serial.print("Temp: ");
    Serial.print(t);
    Serial.print(" HR: ");
    Serial.println(h);

    //char* tString = f2s(t,0);
    //char* hString = f2s(h,0);

    pubString = String(t);
    pubString.toCharArray(message_buff, pubString.length() + 1);
    mqttclient.publish(tempTopic, message_buff);

    pubString = String(h);
    pubString.toCharArray(message_buff, pubString.length() + 1);
    mqttclient.publish(humidityTopic, message_buff);
    // fin DHT
  }
}

void sensarLuz() {
  int lightsensor;
  //Serial.println("Leyendo valor Luz");
  lightsensor = analogRead(A0);
  pubString = String(lightsensor);
  pubString.toCharArray(message_buff, pubString.length() + 1);
  mqttclient.publish(lightTopic, message_buff);
  Serial.print("Valor Luz: ");
  Serial.println(lightsensor);
}

void sensarRSSI() {
  int32_t rssi = WiFi.RSSI();
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.println("dBm");
  pubString = String(rssi);
  pubString.toCharArray(message_buff, pubString.length() + 1);
  mqttclient.publish(signalTopic, message_buff);
}

void pinChanged()  // Callback IRQ para otros inputs que no sean los 2 de persiana
{
  cambiopin = true;
}

// Hay una especie de FSM de la persiana, con 3 estados. Subiendo, Bajando, STOP.
// Se cambia de un estado a otro tanto por comando fisico (2 pines) como logico (MQTT)
void pinPersianaChanged() // Rutina de callback de IRQ de los dos pines de entrada de persiana
{
  cambioenpersiana = true; // solo notifico a loop()
}

// Esta funcion activa los pines de salida para que la persiana suba/baje/pare
void persiana(int mov) {
  if (mov == 0) {
    // modifico pines de salida para que la persiana pare
    Serial.println("Para.");
    mqttclient.publish(estadopersianaTopic, "PARADA");
    // codigo para acomodar los pines
    /////////digitalWrite(15,HIGH); irrelevante
    digitalWrite(15,LOW); // energia apagada
    //
    moviendo = 0;
  }
  if (mov == -1) {
    // modifico pines de salida para que la persiana suba
    Serial.println("Sube.");
    mqttclient.publish(estadopersianaTopic, "SUBIENDO");
    // codigo para acomodar los pines
    digitalWrite(16,LOW);
    digitalWrite(15,HIGH); // energia prendida
    //
    moviendo = -1;
    iniciomovimiento = millis();
    posicionanterior = posicionpersiana;
    posicionreporte = posicionpersiana;

  }
  if (mov == 1) {
    // modifico pines de salida para que la persiana baje
    Serial.println("Baja.");
    mqttclient.publish(estadopersianaTopic, "BAJANDO");
    // codigo para acomodar los pines

     digitalWrite(16,HIGH);
    digitalWrite(15,HIGH); // energia prendida
    //
    moviendo = 1;
    iniciomovimiento = millis();
    posicionanterior = posicionpersiana;
    posicionreporte = posicionpersiana; // una referencia para ir reportando via MQTT
  }
}

// la llamo desde el callback de MQTT y tambien deberia llamarla desde el callback del IRQ del pin que corresponda
void switch1(int orden) {
  if (orden) {
    //Serial.println("Enciendo el switch.");
    mqttclient.publish(switch1statusTopic, "ON");  // si es el mismo topic lo leo yo mismo, asi que aviso en otro
    digitalWrite(2, LOW);   // turn the LED on (HIGH is the voltage level)
  } else // si no es ON, es OFF
  {
    //Serial.println("Apago el switch.");
    mqttclient.publish(switch1statusTopic, "OFF");
    digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
}

