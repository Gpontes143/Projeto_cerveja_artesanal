#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h> //LCD OLED
#include <Adafruit_SSD1306.h> //LCD OLED


// Configuracao do sensor de temperatura
const short PINODEDADOS = 4;
unsigned long delaytemp = 0;
unsigned int contagem_de_erro = 0;

//PID CONTROL
short temperatura_alvo = 25; // Inicializado com um valor padrão

//Controle da temperatura botao
unsigned long tempoUltimoClique = 0; // Renomeado para consistência
const unsigned long intervaloDebounce = 200; 
const short PINO_BOTAO_AUMENTAR_TEMPERATURA = 26;
const short PINO_BOTAO_DIMINUIR_TEMPERATURA = 27;

//Display
const short sda = 21;
const short scl = 22;


// Configurações do OLED
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void atualizarDisplay(float Temperaturaemcelsius, int temperatura_alvo) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  
  // Título
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("TERMOSTATO ESP32");
  display.drawFastHLine(0, 10, 128, WHITE);

  // Temperatura Real
  display.setCursor(0, 20);
  display.print("Real: ");
  display.setTextSize(2);
  if(Temperaturaemcelsius <= -127) display.print("ERR");
  else display.print(Temperaturaemcelsius, 1);
  display.print(" C");

  // Setpoint (Valor Ajustável)
  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print("Ajuste (Set): ");
  display.setTextSize(2);
  display.print(temperatura_alvo);
  display.print(" C");

  display.display();
}

// Definicao do Json criacao
#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0


void criacao_de_json(float Temperaturaemcelsius){
  StaticJsonDocument<256> doc; //criando o arquivo para guardar dados do json

  doc["sensor"] = "temperatura";
  doc["tempoAtivo"] = millis() / 1000;
  doc["temperatura"] = Temperaturaemcelsius;
  doc["temperatura_alvo"] = temperatura_alvo;
  doc["erros_de_conexao"] = contagem_de_erro;
  doc.shrinkToFit();  // optional
  String output; 
  serializeJson(doc, output);
  Serial.println(output);
  
  }
// Instancia um objeto oneWire para comunicar com qualquer dispositivo OneWire
OneWire oneWire(PINODEDADOS);

// Passa a referência do oneWire para o sensor Dallas Temperature
DallasTemperature sensors(&oneWire);


//substituir o delay
bool executarACada(unsigned long intervalo, unsigned long *ultimoTempo) {
  unsigned long tempoAtual = millis();
  if (tempoAtual - *ultimoTempo >= intervalo) {
    *ultimoTempo = tempoAtual;
    return true;
  }
  return false;
}


void setup() {
  
  // Inicia a serial com 115200 baud
  Serial.begin(115200);
  Serial.println("Demonstração do Sensor DS18B20");

// Inicializa OLED (Endereço comum 0x3C)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("Falha ao iniciar OLED");
  }
  pinMode(PINO_BOTAO_AUMENTAR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_BOTAO_DIMINUIR_TEMPERATURA, INPUT_PULLUP);
  //pinMode(pinoRele, OUTPUT);
  // Inicia a biblioteca do sensor
  sensors.begin();
  
  // Leitura inicial para o display não iniciar vazio
  sensors.requestTemperatures();
  atualizarDisplay(sensors.getTempCByIndex(0), temperatura_alvo);
}

void valor_de_temperatura(){
  // Aguarda 2 segundos antes da próxima leitura
  if (executarACada(2000, &delaytemp)){  
    // Solicita que o sensor realize a leitura da temperatura
    sensors.requestTemperatures(); 

    // Obtém a temperatura em Celsius do primeiro sensor (índice 0)
    float Temperaturaemcelsius = sensors.getTempCByIndex(0);

    // Verifica se a leitura é válida (evita erros se o sensor desconectar)
    if(Temperaturaemcelsius != DEVICE_DISCONNECTED_C) {
    criacao_de_json(Temperaturaemcelsius);
     atualizarDisplay(Temperaturaemcelsius, temperatura_alvo);
      
    } else {
      Temperaturaemcelsius = -500;
      contagem_de_erro++;
      criacao_de_json(Temperaturaemcelsius);
      atualizarDisplay(Temperaturaemcelsius, temperatura_alvo);
    }
 }

}

void lerBotoes() {
  // Verifica o tempo para evitar o "repique" (bounce) do botão
  if (millis() - tempoUltimoClique > intervaloDebounce) {
    
    // Lógica para Aumentar
    if (digitalRead(PINO_BOTAO_AUMENTAR_TEMPERATURA) == LOW) {
      temperatura_alvo++;
      Serial.printf("Valor alterado para: %d\n", temperatura_alvo);
      tempoUltimoClique = millis();
      sensors.requestTemperatures();
      atualizarDisplay(sensors.getTempCByIndex(0), temperatura_alvo);
    }

    // Lógica para Diminuir
    if (digitalRead(PINO_BOTAO_DIMINUIR_TEMPERATURA) == LOW) {
      temperatura_alvo--;
      Serial.printf("Valor alterado para: %d\n", temperatura_alvo);
      tempoUltimoClique = millis();
      sensors.requestTemperatures();
      atualizarDisplay(sensors.getTempCByIndex(0), temperatura_alvo);
    }
  }
}

void loop() {
  lerBotoes(); 
  valor_de_temperatura();
}