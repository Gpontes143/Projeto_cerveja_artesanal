#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <HTTPClient.h>

const char* WIFI_SSID = "NOME REDE WIFI";
const char* WIFI_PASSWORD = "SENHA WIFI";
const char* API_URL = "http://IP:1880/api/temperatura";

const short PINODEDADOS = 4;
unsigned long delaytemp = 0;
unsigned int contagem_de_erro = 0;

short temperatura_alvo = 25;
const short PINO_SSR = 5;
short ssr_state = 0;
double Setpoint, Input, Output;

double Kp = 1500.0, Ki = 10.0, Kd = 50.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

unsigned long tempoUltimoClique = 0;
const unsigned long intervaloDebounce = 200;
const short PINO_BOTAO_AUMENTAR_TEMPERATURA = 26;
const short PINO_BOTAO_DIMINUIR_TEMPERATURA = 27;

const short sda = 21;
const short scl = 22;

// Constantes de consumo do ESP32
const float TENSAO_ESP = 3.3;          // Volts
const float CORRENTE_ESP_MA = 240.0;   // mA médio com WiFi ativo
const float POTENCIA_ESP_W = (TENSAO_ESP * CORRENTE_ESP_MA) / 1000.0; // ~0.792W

// Na função enviarParaJSON(), adiciona no doc:
float tempoHoras = (millis() / 1000.0) / 3600.0;
float energiaEspWh = POTENCIA_ESP_W * tempoHoras;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Guarda último valor exibido para só atualizar quando mudar
float lastTempDisplay = -999;
short lastAlvoDisplay = -999;
short lastSsrDisplay = -1;

struct DadosJson {
  float temperatura;
  short temperatura_alvo;
  unsigned int erros;
  double saida_pid;
  short ssr_state;
  unsigned long tempoAtivo;
};

QueueHandle_t filaJSON;

void conectarWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi conectado");
  Serial.println(WiFi.localIP());
}

void enviarParaAPI(const String &payload) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(API_URL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(3000);
    int httpResponseCode = http.POST(payload);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    http.end();
  } else {
    Serial.println("WiFi desconectado");
  }
}

// Display só atualiza quando o valor muda — sem lcd.clear()
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

void taskProcessaDados(void *pvParameters) {
  const float TENSAO_ESP    = 3.3;
  const float CORRENTE_ESP  = 0.240;  // Amperes
  const float POTENCIA_ESP  = TENSAO_ESP * CORRENTE_ESP; // 0.792W

  DadosJson dados;
  for (;;) {
    if (xQueueReceive(filaJSON, &dados, portMAX_DELAY)) {
      float tempoHoras = dados.tempoAtivo / 3600.0;
      float energiaWh  = POTENCIA_ESP * tempoHoras;

      StaticJsonDocument<200> doc;
      doc["sensor"]           = "temperatura";
      doc["tempoAtivo"]       = dados.tempoAtivo;
      doc["temperatura"]      = dados.temperatura;
      doc["temperatura_alvo"] = dados.temperatura_alvo;
      doc["erros_de_conexao"] = dados.erros;
      doc["saida_pid"]        = dados.saida_pid;
      doc["ssr_state"]        = dados.ssr_state;
      doc["potencia_esp_w"]   = POTENCIA_ESP;          // 0.792W fixo
      doc["energia_esp_wh"]   = energiaWh;             // acumulado

      String output;
      output.reserve(200);
      serializeJson(doc, output);
      Serial.println(output);
      enviarParaAPI(output);
    }
  }
}

void enviarParaJSON(float temp) {
  DadosJson dados;
  dados.temperatura     = temp;
  dados.temperatura_alvo = temperatura_alvo;
  dados.erros           = contagem_de_erro;
  dados.saida_pid       = Output;
  dados.ssr_state       = ssr_state;
  dados.tempoAtivo      = millis() / 1000;
  xQueueSend(filaJSON, &dados, 0);
}

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

void setup() {
  Serial.begin(115200);
  conectarWiFi();

  filaJSON = xQueueCreate(10, sizeof(DadosJson));
  xTaskCreatePinnedToCore(taskProcessaDados, "TaskJSON", 4096, NULL, 1, NULL, 0);

  pinMode(PINO_BOTAO_AUMENTAR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_BOTAO_DIMINUIR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_SSR, OUTPUT);

  lcd.init();
  lcd.backlight();

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delay(750);
  float temp_inicial = sensors.getTempCByIndex(0);
  Input = temp_inicial;

  Setpoint = temperatura_alvo;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetSampleTime(800);

  windowStartTime = millis();
  atualizarDisplay(temp_inicial, temperatura_alvo);
  sensors.requestTemperatures();
}

void valor_de_temperatura() {
  if (executarACada(800, &delaytemp)) {
    float Temperaturaemcelsius = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    if (Temperaturaemcelsius != DEVICE_DISCONNECTED_C && Temperaturaemcelsius > -50) {
      Input = Temperaturaemcelsius;
      enviarParaJSON(Temperaturaemcelsius);
      atualizarDisplay(Temperaturaemcelsius, temperatura_alvo);
    } else {
      contagem_de_erro++;
      enviarParaJSON(-500);
      atualizarDisplay(-500, temperatura_alvo);
      digitalWrite(PINO_SSR, LOW);
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

void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWiFi();
  lerBotoes();
  valor_de_temperatura();
  controleSSR();
}