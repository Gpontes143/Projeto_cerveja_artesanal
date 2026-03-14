#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

// Definicao do Json criacao
#define ARDUINOJSON_SLOT_ID_SIZE 1
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0


// O pino de dados está conectado no GPIO 4 do ESP32
const short pinoDados = 4;
long unsigned int delaytemp = 0;
unsigned int contagem_de_erro = 0;

void criacao_de_json(float TempC){
  StaticJsonDocument<256> doc; //criando o arquivo para guardar dados do json

  doc["sensor"] = "temperatura";
  doc["tempoAtivo"] = millis() / 1000;
  doc["temperatura"] = TempC;
  doc["erros_de_conexao"] = contagem_de_erro;
  doc.shrinkToFit();  // optional
  String output; 
  serializeJson(doc, output);
  Serial.println(output);
  }
// Instancia um objeto oneWire para comunicar com qualquer dispositivo OneWire
OneWire oneWire(pinoDados);

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

  // Inicia a biblioteca do sensor
  sensors.begin();
}
void valor_de_temperatura(){
  // Aguarda 2 segundos antes da próxima leitura
  if (executarACada(2000, &delaytemp)){  
    // Solicita que o sensor realize a leitura da temperatura
    sensors.requestTemperatures(); 

    // Obtém a temperatura em Celsius do primeiro sensor (índice 0)
    float TempC = sensors.getTempCByIndex(0);

    // Verifica se a leitura é válida (evita erros se o sensor desconectar)
    if(TempC != DEVICE_DISCONNECTED_C) {
    criacao_de_json(TempC);
     
      
    } else {
      TempC = -500;
      contagem_de_erro++;
      criacao_de_json(TempC);
    }
 }

}
void loop() {

  valor_de_temperatura();

  
}