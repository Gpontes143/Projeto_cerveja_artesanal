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
const int   MQTT_PORT      = 1883;
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
const float TENSAO_ESP  = 3.3;
const float CORRENTE_ESP = 0.240;
const float POTENCIA_ESP = TENSAO_ESP * CORRENTE_ESP; // 0.792W

// ─── Fila FreeRTOS ───────────────────────────────────────────────
struct DadosJson {
  float temperatura;
  short temperatura_alvo;
  short ssr_state;
  unsigned long tempoAtivo;
};

QueueHandle_t filaMQTT;

// ─── Clientes WiFi/MQTT ──────────────────────────────────────────
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ─── SEU MÉTODO ──────────────────────────────────────────────────
bool executarACada(unsigned long intervalo, unsigned long *ultimoTempo) {
  unsigned long tempoAtual = millis();
  if (tempoAtual - *ultimoTempo >= intervalo) {
    *ultimoTempo = tempoAtual;
    return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────
// TASK DUAL CORE (NÚCLEO 0) - GERENCIA APENAS WIFI E MQTT
// ─────────────────────────────────────────────────────────────────
void taskRede(void *pvParameters) {
  unsigned long timerWiFi = 0;
  
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (executarACada(5000, &timerWiFi)) {
        Serial.println("[WiFi] Reconectando...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }
    } else {
      if (!mqttClient.connected()) {
        if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
          Serial.println("[MQTT] Conectado!");
        }
      }
      mqttClient.loop();
    }

    DadosJson dadosRecebidos;
    if (xQueueReceive(filaMQTT, &dadosRecebidos, 0) == pdTRUE) {
      if (mqttClient.connected()) {
        float tempoHoras = dadosRecebidos.tempoAtivo / 3600.0;
        float energiaWh  = POTENCIA_ESP * tempoHoras;

        StaticJsonDocument<256> doc;
        doc["timestamp"]        = dadosRecebidos.tempoAtivo;
        doc["temperatura"]      = dadosRecebidos.temperatura;
        doc["temperatura_alvo"] = dadosRecebidos.temperatura_alvo;
        doc["ssr_state"]        = dadosRecebidos.ssr_state;
        doc["potencia_esp_w"]   = POTENCIA_ESP;
        doc["energia_esp_wh"]   = energiaWh;

        String output;
        output.reserve(256);
        serializeJson(doc, output);

        Serial.println(output);

        bool ok = mqttClient.publish(MQTT_TOPIC, output.c_str(), false);
        Serial.println(ok ? "[MQTT] Publicado!" : "[MQTT] Publicacao falhou");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
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

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Brewery IoT — ESP32 iniciando ===");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setSocketTimeout(5);
  mqttClient.setBufferSize(512);

  // Criar fila e Task no Core 0
  filaMQTT = xQueueCreate(10, sizeof(DadosJson));
  xTaskCreatePinnedToCore(taskRede, "TaskRede", 8192, NULL, 1, NULL, 0);

  // Hardware
  pinMode(PINO_BOTAO_AUMENTAR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_BOTAO_DIMINUIR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_SSR, OUTPUT);

  Wire.begin(sda, scl);
  lcd.init();
  lcd.backlight();

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  unsigned long timerWait = millis();
  while (!executarACada(750, &timerWait)) { yield(); }

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

      // Envia para a fila da Task de Rede (Core 0)
      xQueueSend(filaMQTT, &dados, 0);
      atualizarDisplay(temp, temperatura_alvo);
    } else {
      contagem_de_erro++;
      Serial.print("[ERRO] Sensor desconectado! Total erros: ");
      Serial.println(contagem_de_erro);
      atualizarDisplay(-500, temperatura_alvo);
      digitalWrite(PINO_SSR, LOW);
    }
  }
}

void lerBotoes() {
  if (executarACada(intervaloDebounce, &tempoUltimoClique)) {
    bool mudou = false;
    if (digitalRead(PINO_BOTAO_AUMENTAR_TEMPERATURA) == LOW) { temperatura_alvo++; mudou = true; }
    if (digitalRead(PINO_BOTAO_DIMINUIR_TEMPERATURA) == LOW) { temperatura_alvo--; mudou = true; }
    if (mudou) {
      Setpoint = temperatura_alvo;
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
    digitalWrite(PINO_SSR, LOW);
    ssr_state = 0;
  }
}

// ─────────────────────────────────────────────────────────────────
void loop() {
  lerBotoes();
  valor_de_temperatura();
  controleSSR();
}
