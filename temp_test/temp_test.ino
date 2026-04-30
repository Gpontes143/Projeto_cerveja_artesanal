#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ─── WiFi ────────────────────────────────────────────────────────
const char* WIFI_SSID     = "";
const char* WIFI_PASSWORD = "";

// ─── MQTT ────────────────────────────────────────────────────────
const char* MQTT_BROKER    = "";
const int   MQTT_PORT      = 0000;
const char* MQTT_USER      = "";
const char* MQTT_PASSWORD  = "";
const char* MQTT_CLIENT_ID = "";
const char* MQTT_TOPIC     = "";

// ─── Sensor DS18B20 ──────────────────────────────────────────────
const short PINODEDADOS = 4;
unsigned long delaytemp = 0;
unsigned int  contagem_de_erro = 0;

// ─── PID / SSR ───────────────────────────────────────────────────
short temperatura_alvo = 25;
const short PINO_SSR = 5;
short ssr_state = 0;
double Setpoint, Input, Output;

double Kp = 1500.0, Ki = 10.0, Kd = 50.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

// ─── Botões ──────────────────────────────────────────────────────
unsigned long tempoUltimoClique = 0;
const unsigned long intervaloDebounce = 200;
const short PINO_BOTAO_AUMENTAR_TEMPERATURA = 26;
const short PINO_BOTAO_DIMINUIR_TEMPERATURA = 27;

// ─── I2C / LCD ───────────────────────────────────────────────────
const short sda = 21;
const short scl = 22;
LiquidCrystal_I2C lcd(0x27, 16, 2);

float lastTempDisplay = -999;
short lastAlvoDisplay = -999;
short lastSsrDisplay  = -1;

// ─── Constantes de energia ───────────────────────────────────────
const float TENSAO_ESP   = 3.3;
const float CORRENTE_ESP = 0.240;
const float POTENCIA_ESP = TENSAO_ESP * CORRENTE_ESP; // 0.792W

// ─── Fila FreeRTOS ───────────────────────────────────────────────
struct DadosJson {
  float temperatura;
  short temperatura_alvo;
  short ssr_state;
  unsigned long tempoAtivo;
};

QueueHandle_t filaJSON;

// ─── Clientes WiFi/MQTT ──────────────────────────────────────────
// ✅ Mesmo padrão do código de teste que funcionou
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ─────────────────────────────────────────────────────────────────
// Reconecta MQTT — chamado no loop(), igual ao código de teste
// SEM FreeRTOS task, SEM mutex — exatamente como funcionou no teste
// ─────────────────────────────────────────────────────────────────
void reconectarMQTT() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("[MQTT] Conectando ao broker " + String(MQTT_BROKER) + "...");
  bool ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
  int  rc  = mqttClient.state();

  if (ok) {
    Serial.println("[MQTT] Conectado!");
  } else {
    Serial.println("[MQTT] Falhou rc=" + String(rc));
  }
}

// ─────────────────────────────────────────────────────────────────
// Publica JSON com os campos solicitados:
// timestamp | temperatura | temperatura_alvo | ssr_state
// potencia_esp_w | energia_esp_wh
// ─────────────────────────────────────────────────────────────────
void publicarMQTT(const DadosJson &dados) {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Nao conectado — publicacao ignorada");
    return;
  }

  float tempoHoras = dados.tempoAtivo / 3600.0;
  float energiaWh  = POTENCIA_ESP * tempoHoras;

  StaticJsonDocument<256> doc;
  doc["timestamp"]        = dados.tempoAtivo;
  doc["temperatura"]      = dados.temperatura;
  doc["temperatura_alvo"] = dados.temperatura_alvo;
  doc["ssr_state"]        = dados.ssr_state;
  doc["potencia_esp_w"]   = POTENCIA_ESP;
  doc["energia_esp_wh"]   = energiaWh;

  String output;
  output.reserve(256);
  serializeJson(doc, output);

  Serial.println(output);

  bool ok = mqttClient.publish(MQTT_TOPIC, output.c_str(), false);
  Serial.println(ok ? "[MQTT] Publicado!" : "[MQTT] Publicacao falhou");
}

// ─── Display LCD ─────────────────────────────────────────────────
void atualizarDisplay(float temp, int temp_alvo) {
  bool mudou = (temp != lastTempDisplay || temp_alvo != lastAlvoDisplay || ssr_state != lastSsrDisplay);
  if (!mudou) return;

  lastTempDisplay = temp;
  lastAlvoDisplay = temp_alvo;
  lastSsrDisplay  = ssr_state;

  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  if (temp <= -127) {
    lcd.print("ERRO ");
  } else {
    char buf[8];
    dtostrf(temp, 5, 1, buf);
    lcd.print(buf);
    lcd.print("C");
  }

  lcd.setCursor(0, 1);
  lcd.print("Alvo:");
  lcd.print(temp_alvo);
  lcd.print(ssr_state == 1 ? "C ssr:on " : "C ssr:off");
}

// ─────────────────────────────────────────────────────────────────
OneWire oneWire(PINODEDADOS);
DallasTemperature sensors(&oneWire);

bool executarACada(unsigned long intervalo, unsigned long *ultimoTempo) {
  unsigned long tempoAtual = millis();
  if (tempoAtual - *ultimoTempo >= intervalo) {
    *ultimoTempo = tempoAtual;
    return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Brewery IoT — ESP32 iniciando ===");

  // ✅ WiFi — mesmo padrão do código de teste
  Serial.println("[WiFi] Conectando a: " + String(WIFI_SSID));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Conectado!");
    Serial.println("[WiFi] IP     : " + WiFi.localIP().toString());
    Serial.println("[WiFi] Gateway: " + WiFi.gatewayIP().toString());
    Serial.println("[WiFi] RSSI   : " + String(WiFi.RSSI()) + " dBm");
  } else {
    Serial.println("[WiFi] FALHOU — verifique SSID e senha");
  }

  // ✅ MQTT — mesmo padrão do código de teste
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setSocketTimeout(5);
  mqttClient.setBufferSize(512);

  Serial.println("[MQTT] Broker  : " + String(MQTT_BROKER) + ":" + String(MQTT_PORT));
  Serial.println("[MQTT] Usuario : " + String(MQTT_USER));
  Serial.println("[MQTT] ClientID: " + String(MQTT_CLIENT_ID));

  reconectarMQTT();

  // ─── Hardware ────────────────────────────────────────────────
  pinMode(PINO_BOTAO_AUMENTAR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_BOTAO_DIMINUIR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_SSR, OUTPUT);

  Wire.begin(sda, scl);
  lcd.init();
  lcd.backlight();

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delay(750);

  float temp_inicial = sensors.getTempCByIndex(0);
  Input    = temp_inicial;
  Setpoint = temperatura_alvo;

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetSampleTime(800);

  windowStartTime = millis();
  atualizarDisplay(temp_inicial, temperatura_alvo);
  sensors.requestTemperatures();

  Serial.println("=== Setup concluido ===");
}

// ─────────────────────────────────────────────────────────────────
void valor_de_temperatura() {
  if (executarACada(800, &delaytemp)) {
    float temp = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    if (temp != DEVICE_DISCONNECTED_C && temp > -50) {
      Input = temp;

      DadosJson dados;
      dados.temperatura      = temp;
      dados.temperatura_alvo = temperatura_alvo;
      dados.ssr_state        = ssr_state;
      dados.tempoAtivo       = millis() / 1000;

      publicarMQTT(dados);
      atualizarDisplay(temp, temperatura_alvo);
    } else {
      contagem_de_erro++;
      Serial.print("[ERRO] Sensor desconectado! Total erros: ");
      Serial.println(contagem_de_erro);
      atualizarDisplay(-500, temperatura_alvo);
      digitalWrite(PINO_SSR, LOW);
      // Nao publica leitura invalida
    }
  }
}

void lerBotoes() {
  if (millis() - tempoUltimoClique > intervaloDebounce) {
    bool mudou = false;
    if (digitalRead(PINO_BOTAO_AUMENTAR_TEMPERATURA) == LOW) { temperatura_alvo++; mudou = true; }
    if (digitalRead(PINO_BOTAO_DIMINUIR_TEMPERATURA) == LOW) { temperatura_alvo--; mudou = true; }
    if (mudou) {
      Setpoint = temperatura_alvo;
      tempoUltimoClique = millis();
      atualizarDisplay(Input, temperatura_alvo);
    }
  }
}

void controleSSR() {
  myPID.Compute();
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) windowStartTime += WindowSize;
  if (Output > (now - windowStartTime)) {
    digitalWrite(PINO_SSR, HIGH);
    ssr_state = 1;
  } else {
    ssr_state = 0;
    digitalWrite(PINO_SSR, LOW);
  }
}

// ─────────────────────────────────────────────────────────────────
// loop() — sem FreeRTOS, sem mutex, sem tasks de rede
// ✅ Mesmo padrão do código de teste que funcionou
// ─────────────────────────────────────────────────────────────────
void loop() {
  // Reconecta WiFi se cair
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Desconectado — reconectando...");
    WiFi.reconnect();
    delay(5000);
    return;
  }

  // Reconecta MQTT se cair — nao bloqueia o loop
  if (!mqttClient.connected()) {
    reconectarMQTT();
  }

  // Keepalive MQTT
  mqttClient.loop();

  lerBotoes();
  valor_de_temperatura();
  controleSSR();
}
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ─── WiFi ────────────────────────────────────────────────────────
const char* WIFI_SSID     = "ZENAIDE";
const char* WIFI_PASSWORD = "zenaide15031967";

// ─── MQTT ────────────────────────────────────────────────────────
const char* MQTT_BROKER    = "192.168.18.7";
const int   MQTT_PORT      = 1883;
const char* MQTT_USER      = "esp32";
const char* MQTT_PASSWORD  = "belezafatal";
const char* MQTT_CLIENT_ID = "esp32-brewery";
const char* MQTT_TOPIC     = "brewery/sensors/temperature";

// ─── Sensor DS18B20 ──────────────────────────────────────────────
const short PINODEDADOS = 4;
unsigned long delaytemp = 0;
unsigned int  contagem_de_erro = 0;

// ─── PID / SSR ───────────────────────────────────────────────────
short temperatura_alvo = 25;
const short PINO_SSR   = 5;
volatile short ssr_state = 0;
double Setpoint, Input, Output;

double Kp = 1500.0, Ki = 10.0, Kd = 50.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

// ─── Botões ──────────────────────────────────────────────────────
unsigned long tempoUltimoClique = 0;
const unsigned long intervaloDebounce = 200;
const short PINO_BOTAO_AUMENTAR_TEMPERATURA = 26;
const short PINO_BOTAO_DIMINUIR_TEMPERATURA = 27;

// ─── I2C / LCD ───────────────────────────────────────────────────
const short sda = 21;
const short scl = 22;
LiquidCrystal_I2C lcd(0x27, 16, 2);

float lastTempDisplay = -999;
short lastAlvoDisplay = -999;
short lastSsrDisplay  = -1;

// ─── Constantes de energia ───────────────────────────────────────
const float TENSAO_ESP   = 3.3;
const float CORRENTE_ESP = 0.240;
const float POTENCIA_ESP = TENSAO_ESP * CORRENTE_ESP; // 0.792W

// ─── Fila FreeRTOS ───────────────────────────────────────────────
struct DadosJson {
  float temperatura;
  short temperatura_alvo;
  short ssr_state;
  unsigned long tempoAtivo;
};

QueueHandle_t filaJSON;

// ─── Clientes WiFi/MQTT ──────────────────────────────────────────
// ✅ Mesmo padrão do código de teste que funcionou
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ─────────────────────────────────────────────────────────────────
// Reconecta MQTT — chamado no loop(), igual ao código de teste
// SEM FreeRTOS task, SEM mutex — exatamente como funcionou no teste
// ─────────────────────────────────────────────────────────────────
void reconectarMQTT() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("[MQTT] Conectando ao broker " + String(MQTT_BROKER) + "...");
  bool ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
  int  rc  = mqttClient.state();

  if (ok) {
    Serial.println("[MQTT] Conectado!");
  } else {
    Serial.println("[MQTT] Falhou rc=" + String(rc));
  }
}

// ─────────────────────────────────────────────────────────────────
// Publica JSON com os campos solicitados:
// timestamp | temperatura | temperatura_alvo | ssr_state
// potencia_esp_w | energia_esp_wh
// ─────────────────────────────────────────────────────────────────
void publicarMQTT(const DadosJson &dados) {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Nao conectado — publicacao ignorada");
    return;
  }

  float tempoHoras = dados.tempoAtivo / 3600.0;
  float energiaWh  = POTENCIA_ESP * tempoHoras;

  StaticJsonDocument<256> doc;
  doc["timestamp"]        = dados.tempoAtivo;
  doc["temperatura"]      = dados.temperatura;
  doc["temperatura_alvo"] = dados.temperatura_alvo;
  doc["ssr_state"]        = dados.ssr_state;
  doc["potencia_esp_w"]   = POTENCIA_ESP;
  doc["energia_esp_wh"]   = energiaWh;

  String output;
  output.reserve(256);
  serializeJson(doc, output);

  Serial.println(output);

  bool ok = mqttClient.publish(MQTT_TOPIC, output.c_str(), false);
  Serial.println(ok ? "[MQTT] Publicado!" : "[MQTT] Publicacao falhou");
}

// ─── Display LCD ─────────────────────────────────────────────────
void atualizarDisplay(float temp, int temp_alvo) {
  bool mudou = (temp != lastTempDisplay || temp_alvo != lastAlvoDisplay || ssr_state != lastSsrDisplay);
  if (!mudou) return;

  lastTempDisplay = temp;
  lastAlvoDisplay = temp_alvo;
  lastSsrDisplay  = ssr_state;

  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  if (temp <= -127) {
    lcd.print("ERRO ");
  } else {
    char buf[8];
    dtostrf(temp, 5, 1, buf);
    lcd.print(buf);
    lcd.print("C");
  }

  lcd.setCursor(0, 1);
  lcd.print("Alvo:");
  lcd.print(temp_alvo);
  lcd.print(ssr_state == 1 ? "C ssr:on " : "C ssr:off");
}

// ─────────────────────────────────────────────────────────────────
OneWire oneWire(PINODEDADOS);
DallasTemperature sensors(&oneWire);

bool executarACada(unsigned long intervalo, unsigned long *ultimoTempo) {
  unsigned long tempoAtual = millis();
  if (tempoAtual - *ultimoTempo >= intervalo) {
    *ultimoTempo = tempoAtual;
    return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Brewery IoT — ESP32 iniciando ===");

  // ✅ WiFi — mesmo padrão do código de teste
  Serial.println("[WiFi] Conectando a: " + String(WIFI_SSID));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Conectado!");
    Serial.println("[WiFi] IP     : " + WiFi.localIP().toString());
    Serial.println("[WiFi] Gateway: " + WiFi.gatewayIP().toString());
    Serial.println("[WiFi] RSSI   : " + String(WiFi.RSSI()) + " dBm");
  } else {
    Serial.println("[WiFi] FALHOU — verifique SSID e senha");
  }

  // ✅ MQTT — mesmo padrão do código de teste
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setSocketTimeout(5);
  mqttClient.setBufferSize(512);

  Serial.println("[MQTT] Broker  : " + String(MQTT_BROKER) + ":" + String(MQTT_PORT));
  Serial.println("[MQTT] Usuario : " + String(MQTT_USER));
  Serial.println("[MQTT] ClientID: " + String(MQTT_CLIENT_ID));

  reconectarMQTT();

  // ─── Hardware ────────────────────────────────────────────────
  pinMode(PINO_BOTAO_AUMENTAR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_BOTAO_DIMINUIR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_SSR, OUTPUT);

  Wire.begin(sda, scl);
  lcd.init();
  lcd.backlight();

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delay(750);

  float temp_inicial = sensors.getTempCByIndex(0);
  Input    = temp_inicial;
  Setpoint = temperatura_alvo;

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetSampleTime(800);

  windowStartTime = millis();
  atualizarDisplay(temp_inicial, temperatura_alvo);
  sensors.requestTemperatures();

  Serial.println("=== Setup concluido ===");
}

// ─────────────────────────────────────────────────────────────────
void valor_de_temperatura() {
  if (executarACada(800, &delaytemp)) {
    float temp = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    if (temp != DEVICE_DISCONNECTED_C && temp > -50) {
      Input = temp;

      DadosJson dados;
      dados.temperatura      = temp;
      dados.temperatura_alvo = temperatura_alvo;
      dados.ssr_state        = ssr_state;
      dados.tempoAtivo       = millis() / 1000;

      publicarMQTT(dados);
      atualizarDisplay(temp, temperatura_alvo);
    } else {
      contagem_de_erro++;
      Serial.print("[ERRO] Sensor desconectado! Total erros: ");
      Serial.println(contagem_de_erro);
      atualizarDisplay(-500, temperatura_alvo);
      digitalWrite(PINO_SSR, LOW);
      // Nao publica leitura invalida
    }
  }
}

void lerBotoes() {
  if (millis() - tempoUltimoClique > intervaloDebounce) {
    bool mudou = false;
    if (digitalRead(PINO_BOTAO_AUMENTAR_TEMPERATURA) == LOW) { temperatura_alvo++; mudou = true; }
    if (digitalRead(PINO_BOTAO_DIMINUIR_TEMPERATURA) == LOW) { temperatura_alvo--; mudou = true; }
    if (mudou) {
      Setpoint = temperatura_alvo;
      tempoUltimoClique = millis();
      atualizarDisplay(Input, temperatura_alvo);
    }
  }
}

void controleSSR() {
  myPID.Compute();
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) windowStartTime += WindowSize;
  if (Output > (now - windowStartTime)) {
    digitalWrite(PINO_SSR, HIGH);
    ssr_state = 1;
  } else {
    ssr_state = 0;
    digitalWrite(PINO_SSR, LOW);
  }
}

// ─────────────────────────────────────────────────────────────────
// loop() — sem FreeRTOS, sem mutex, sem tasks de rede
// ✅ Mesmo padrão do código de teste que funcionou
// ─────────────────────────────────────────────────────────────────
void loop() {
  // Reconecta WiFi se cair
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Desconectado — reconectando...");
    WiFi.reconnect();
    delay(5000);
    return;
  }

  // Reconecta MQTT se cair — nao bloqueia o loop
  if (!mqttClient.connected()) {
    reconectarMQTT();
  }

  // Keepalive MQTT
  mqttClient.loop();

  lerBotoes();
  valor_de_temperatura();
  controleSSR();
}
