#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h> 

// Pinos
#define PINODEDADOS 4
#define PINO_BOTAO_AUMENTAR_TEMPERATURA 26
#define PINO_BOTAO_DIMINUIR_TEMPERATURA 27
#define PINO_RELE 5
#define SDA_PIN 21
#define SCL_PIN 22

// Variáveis Globais de Controle
unsigned long delaytemp = 0;
unsigned int contagem_de_erro = 0;
unsigned long tempoUltimoClique = 0; 
const unsigned long intervaloDebounce = 200; 

// Variáveis de Temperatura
float temperaturaAtual = 0.0; 
double Setpoint, Input, Output; // Variáveis exigidas pela biblioteca PID
short temperatura_alvo = 68;  

// --- CONFIGURAÇÕES DO PID ---
// Estes valores (Kp, Ki, Kd) precisam ser ajustados 
// Valores iniciais genéricos para aquecimento lento:
double Kp = 50.0, Ki = 0.5, Kd = 2.0; 
PID meuPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Janela de tempo do "PWM Lento" do Relé (5000ms = 5 segundos)
int WindowSize = 5000;
unsigned long windowStartTime;

// Configurações do OLED
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Instanciação do Sensor
OneWire oneWire(PINODEDADOS);
DallasTemperature sensors(&oneWire);

// Função para substituir o delay
bool executarACada(unsigned long intervalo, unsigned long *ultimoTempo) {
  unsigned long tempoAtual = millis();
  if (tempoAtual - *ultimoTempo >= intervalo) {
    *ultimoTempo = tempoAtual;
    return true;
  }
  return false;
}

void atualizarDisplay(float tempReal, int tempAlvo) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  
  // Título
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("PANELA PID");
  display.drawFastHLine(0, 10, 128, WHITE);

  // Temperatura Real
  display.setCursor(0, 20);
  display.print("Real: ");
  display.setTextSize(2);
  if(tempReal <= -127) {
    display.print("ERR");
  } else {
    display.print(tempReal, 1);
  }
  display.print(" C");

  // Setpoint (Valor Ajustável)
  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print("Alvo: ");
  display.setTextSize(2);
  display.print(tempAlvo);
  display.print(" C");

  display.display();
}

void criacao_de_json(float tempReal){
  StaticJsonDocument<256> doc; 

  doc["sensor"] = "temperatura_panela";
  doc["tempoAtivo"] = millis() / 1000;
  doc["temperatura"] = tempReal;
  doc["temperatura_alvo"] = temperatura_alvo;
  doc["erros_de_conexao"] = contagem_de_erro;
  doc["resistencia_ligada"] = digitalRead(PINO_RELE) == HIGH ? true : false; 
  doc["potencia_pid"] = Output; // Mostra o valor calculado pelo PID
  
  String outputStr; 
  serializeJson(doc, outputStr);
  Serial.println(outputStr); 
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Sistema com PID...");

  // Configuração dos pinos
  pinMode(PINO_BOTAO_AUMENTAR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_BOTAO_DIMINUIR_TEMPERATURA, INPUT_PULLUP);
  pinMode(PINO_RELE, OUTPUT);
  digitalWrite(PINO_RELE, LOW); 

  // Inicializa I2C e OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("Falha ao iniciar OLED");
  }

  // Inicia o sensor
  sensors.begin();
  sensors.requestTemperatures();
  temperaturaAtual = sensors.getTempCByIndex(0);
  
  // Configurações iniciais do PID
  Input = temperaturaAtual;
  Setpoint = temperatura_alvo;
  windowStartTime = millis();
  meuPID.SetOutputLimits(0, WindowSize); // O PID vai cuspir um valor entre 0 e 5000
  meuPID.SetMode(AUTOMATIC);             // Liga o PID

  atualizarDisplay(temperaturaAtual, temperatura_alvo);
}

void controle_da_panela() {
  unsigned long now = millis();
  
  // 1. A CADA 2 SEGUNDOS: Leitura do Sensor
  if (executarACada(2000, &delaytemp)) {  
    sensors.requestTemperatures(); 
    temperaturaAtual = sensors.getTempCByIndex(0);

    if(temperaturaAtual != DEVICE_DISCONNECTED_C) {
      Input = temperaturaAtual; // Alimenta o PID com a temperatura nova
      Setpoint = temperatura_alvo;
      
      criacao_de_json(temperaturaAtual);
      atualizarDisplay(temperaturaAtual, temperatura_alvo);
    } else {
      // Falha no sensor! Desliga tudo por segurança
      digitalWrite(PINO_RELE, LOW); 
      meuPID.SetMode(MANUAL); // Pausa o PID
      Output = 0;
      
      temperaturaAtual = -500;
      contagem_de_erro++;
      criacao_de_json(temperaturaAtual);
      atualizarDisplay(temperaturaAtual, temperatura_alvo);
    }
  }

  // 2. CONTROLE CONTÍNUO DO RELÉ (PWM Lento)
  // Só roda se o sensor estiver OK
  if (temperaturaAtual != -500) {
    meuPID.SetMode(AUTOMATIC);
    meuPID.Compute(); /

    // Lógica do "PWM Lento" da janela de 5 segundos
    if (now - windowStartTime > WindowSize) { 
      windowStartTime += WindowSize; 
    }
    
    // Se o Output do PID for maior que o tempo passado na janela, liga. Senão, desliga.
    if (Output > (now - windowStartTime)) {
      digitalWrite(PINO_RELE, HIGH);
    } else {
      digitalWrite(PINO_RELE, LOW);
    }
  }
}

void lerBotoes() {
  if (millis() - tempoUltimoClique > intervaloDebounce) {
    if (digitalRead(PINO_BOTAO_AUMENTAR_TEMPERATURA) == LOW) {
      temperatura_alvo++;
      tempoUltimoClique = millis();
      atualizarDisplay(temperaturaAtual, temperatura_alvo);
    }
    if (digitalRead(PINO_BOTAO_DIMINUIR_TEMPERATURA) == LOW) {
      temperatura_alvo--;
      tempoUltimoClique = millis();
      atualizarDisplay(temperaturaAtual, temperatura_alvo);
    }
  }
}

void loop() {
  lerBotoes(); 
  controle_da_panela();
}