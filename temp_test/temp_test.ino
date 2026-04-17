#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h> // Biblioteca do LCD 16x2
#include <PID_v1.h>            // Biblioteca do PID

// Configuracao do sensor de temperatura
const short PINODEDADOS = 4;
unsigned long delaytemp = 0;
unsigned int contagem_de_erro = 0;

// PID E SSR CONTROL
short temperatura_alvo = 25; 
const short PINO_SSR = 5; // Pino de sinal para o Relé de Estado Sólido
short ssr_state = 0;
double Setpoint, Input, Output;
// Parâmetros do PID (Ajuste Kp, Ki e Kd conforme a inércia térmica do seu sistema)
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Janela de tempo para o SSR (ex: 5000ms = 5 segundos de ciclo)
int WindowSize = 5000;
unsigned long windowStartTime;

// Controle da temperatura botao
unsigned long tempoUltimoClique = 0;
const unsigned long intervaloDebounce = 200; 
const short PINO_BOTAO_AUMENTAR_TEMPERATURA = 26;
const short PINO_BOTAO_DIMINUIR_TEMPERATURA = 27;

// Pinos I2C
const short sda = 21;
const short scl = 22;

// Configuração do LCD 16x2 (Endereço I2C geralmente é 0x27 ou 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void atualizarDisplay(float Temperaturaemcelsius, int temp_alvo) {
  // ---- Atualiza LCD 16x2 ----
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  if(Temperaturaemcelsius <= -127) lcd.print("ERRO");
  else lcd.print(Temperaturaemcelsius, 1);
  lcd.print(" C");
  
  lcd.setCursor(0, 1);
  lcd.print("Alvo:");
  lcd.print(temp_alvo);
  lcd.print(" C");
  if (ssr_state == 0){
    lcd.print(" ssr: on");
  }
  else if (ssr_state == 1){
    lcd.print(" ssr: of");
  }
}

// Definicao do Json criacao
#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0

void criacao_de_json(float Temperaturaemcelsius){
  StaticJsonDocument<256> doc;

  doc["sensor"] = "temperatura";
  doc["tempoAtivo"] = millis() / 1000;
  doc["temperatura"] = Temperaturaemcelsius;
  doc["temperatura_alvo"] = temperatura_alvo;
  doc["erros_de_conexao"] = contagem_de_erro;
  // Adicionando a saída atual do PID ao JSON para monitoramento
  doc["saida_pid"] = Output; 
  
  doc.shrinkToFit();  
  String output; 
  serializeJson(doc, output);
  Serial.println(output);
}

// Instancias do Sensor
OneWire oneWire(PINODEDADOS);
DallasTemperature sensors(&oneWire);

// Substituir o delay
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
  Serial.println("Demonstração Termostato PID");

  // Configuração dos Pinos
  pinMode(PINO_BOTAO_AUMENTAR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_BOTAO_DIMINUIR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_SSR, OUTPUT);

  // Inicializa LCD 16x2
  lcd.init();
  lcd.backlight();

  // Inicia a biblioteca do sensor
  sensors.begin();
  
  // Setup do PID
  Setpoint = temperatura_alvo;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, WindowSize); // A saída varia de 0 até o tamanho da janela de tempo
  windowStartTime = millis();
  
  // Leitura inicial
  sensors.requestTemperatures();
  float temp_inicial = sensors.getTempCByIndex(0);
  Input = temp_inicial;
  atualizarDisplay(temp_inicial, temperatura_alvo);
}

void valor_de_temperatura(){
  // Aguarda 2 segundos antes da próxima leitura do sensor
  if (executarACada(2000, &delaytemp)){  
    sensors.requestTemperatures(); 
    float Temperaturaemcelsius = sensors.getTempCByIndex(0);

    if(Temperaturaemcelsius != DEVICE_DISCONNECTED_C) {
      Input = Temperaturaemcelsius;
      criacao_de_json(Temperaturaemcelsius);
      atualizarDisplay(Temperaturaemcelsius, temperatura_alvo);
    } else {
      Temperaturaemcelsius = -500;
      contagem_de_erro++;
      criacao_de_json(Temperaturaemcelsius);
      atualizarDisplay(Temperaturaemcelsius, temperatura_alvo);
      // Opcional: desligar o SSR por segurança 
      digitalWrite(PINO_SSR, LOW);

    }
  }
}

void lerBotoes() {
  if (millis() - tempoUltimoClique > intervaloDebounce) {
    bool mudou = false;

    // Lógica para Aumentar
    if (digitalRead(PINO_BOTAO_AUMENTAR_TEMPERATURA) == LOW) {
      temperatura_alvo++;
      mudou = true;
    }

    // Lógica para Diminuir
    if (digitalRead(PINO_BOTAO_DIMINUIR_TEMPERATURA) == LOW) {
      temperatura_alvo--;
      mudou = true;
    }

    // Se houve mudança, atualiza display, PID e Serial
    if (mudou) {
      Setpoint = temperatura_alvo; // Atualiza a meta do PID
      Serial.printf("Valor alterado para: %d\n", temperatura_alvo);
      tempoUltimoClique = millis();
      
      // Atualiza visualmente imediatamente
      atualizarDisplay(Input, temperatura_alvo); 
    }
  }
}

void controleSSR() {
  // Calcula a saída do PID
  myPID.Compute();

  // Controle de Janela de Tempo (Slow PWM) para o SSR
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) { 
    // Muda a janela
    windowStartTime += WindowSize;
  }
  
  // Se a saída do PID for maior que o tempo passado nesta janela, liga. Senão, desliga.
  if (Output > (now - windowStartTime)) {
    digitalWrite(PINO_SSR, HIGH);
  } else {
    digitalWrite(PINO_SSR, LOW);
  }
}

void loop() {
  lerBotoes(); 
  valor_de_temperatura();
  controleSSR(); 
}
