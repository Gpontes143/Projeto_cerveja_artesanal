#include <OneWire.h>
#include <DallasTemperature.h>

// O pino de dados está conectado no GPIO 4 do ESP32
const int pinoDados = 4;

// Instancia um objeto oneWire para comunicar com qualquer dispositivo OneWire
OneWire oneWire(pinoDados);

// Passa a referência do oneWire para o sensor Dallas Temperature
DallasTemperature sensors(&oneWire);

void setup() {
  // Inicia a serial com 115200 baud
  Serial.begin(115200);
  Serial.println("Demonstração do Sensor DS18B20");

  // Inicia a biblioteca do sensor
  sensors.begin();
}

void loop() {
  // Solicita que o sensor realize a leitura da temperatura
  sensors.requestTemperatures(); 

  // Obtém a temperatura em Celsius do primeiro sensor (índice 0)
  float tempC = sensors.getTempCByIndex(0);

  // Verifica se a leitura é válida (evita erros se o sensor desconectar)
  if(tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperatura: ");
    Serial.print(tempC);
    Serial.println("°C");
  } else {
    Serial.println("Erro: Sensor não encontrado!");
  }

  // Aguarda 2 segundos antes da próxima leitura
  delay(2000);
}