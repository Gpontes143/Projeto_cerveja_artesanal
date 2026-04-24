# Documentação do Sistema de Controle Térmico IoT

## 1. Visão Geral

O sistema consiste em um termostato inteligente baseado no **ESP32**, que utiliza um sensor **DS18B20** para leitura de temperatura e um algoritmo **PID** (Proporcional, Integral e Derivativo) para acionar um Relé de Estado Sólido (**SSR**). Os dados são exibidos em um display LCD I2C e enviados via JSON para uma API externa (Node-RED/Dashboard).

---

## 2. Arquitetura de Software

O código utiliza o **FreeRTOS** para gerenciar tarefas em paralelo, garantindo que a latência da rede WiFi não interfira na precisão do controle de temperatura.

### Distribuição de Núcleos (Cores)

- **Core 0 (Comunicação):** Gerencia a conexão WiFi e o processamento/envio de pacotes JSON para a API.
- **Core 1 (Controle):** Executa o loop principal, leitura de botões, sensor de temperatura e o cálculo do PID.

---

## 3. Componentes e Pinagem

| Componente          | Pino ESP32               | Descrição                             |
| :------------------ | :----------------------- | :------------------------------------ |
| **Sensor DS18B20**  | GPIO 4                   | Sensor de temperatura (OneWire)       |
| **Relé SSR**        | GPIO 5                   | Saída de controle de carga            |
| **Botão (+)**       | GPIO 26                  | Aumenta o Setpoint (Temperatura Alvo) |
| **Botão (-)**       | GPIO 27                  | Diminui o Setpoint (Temperatura Alvo) |
| **Display LCD I2C** | GPIO 21 (SDA) / 22 (SCL) | Monitoramento local                   |

---

## 4. Lógica de Controle (PID)

O controle é feito via **Modulação por Largura de Pulso em Tempo Real (PWM via Software)** com uma janela de tempo de 5 segundos (`WindowSize = 5000ms`).

- **Setpoint:** Definido pelo usuário via botões.
- **Input:** Temperatura lida pelo sensor.
- **Output:** Tempo (em ms) que o SSR permanecerá ligado dentro da janela de 5s.
- **Constantes Atuais:** $K_p = 1500$, $K_i = 10$, $K_d = 50$.

---

## 5. Estrutura de Dados e API

O sistema envia um objeto JSON para a URL configurada (`API_URL`) com a seguinte estrutura:

```json
{
  "sensor": "temperatura",
  "tempoAtivo": 120,
  "temperatura": 25.5,
  "temperatura_alvo": 26,
  "erros_de_conexao": 0,
  "saida_pid": 1500.0,
  "ssr_state": 1,
  "potencia_esp_w": 0.792,
  "energia_esp_wh": 0.0264
}
```

### Cálculo de Consumo Energético

O código realiza uma estimativa interna do consumo do próprio ESP32:

- **Potência Fixa:** $0.792W$ (calculado com base em $3.3V$ e $240mA$).
- **Energia ($Wh$):** $Potência \times (Tempo\ Ativo\ em\ horas)$.

---

## 6. Descrição das Funções Principais

### `taskManterWiFi()`

Roda no Núcleo 0. Monitora a conexão persistentemente. Se a conexão cair, ela tenta reconectar sem travar o processamento do sensor.

### `valor_de_temperatura()`

Realiza leituras a cada 800ms (tempo de conversão do DS18B20 em 12 bits). Inclui uma trava de segurança: se o sensor falhar (desconectado), o SSR é desligado imediatamente para evitar superaquecimento.

### `atualizarDisplay()`

Otimizada para evitar o "flicker" (tremulação) do LCD. Ela só escreve no display se houver mudança real nos valores de temperatura, alvo ou estado do SSR.

### `controleSSR()`

Aplica o algoritmo PID. Se o valor de `Output` calculado for maior que o tempo transcorrido na janela atual, liga o SSR; caso contrário, desliga.

---

## 7. Configurações de Segurança

- **Anti-travamento:** Uso de `vTaskDelay` e `executarACada` em vez de `delay()` para manter a responsividade.
- **Debounce:** Proteção de 200ms nos botões para evitar leituras falsas de cliques.
- **Fail-safe:** Se o sensor retornar `-127°C` (erro comum de hardware), o sistema entra em modo de erro e corta a energia da carga.

---

> **Nota:** Para calibrar o controle térmico, ajuste as variáveis `Kp`, `Ki` e `Kd` de acordo com a inércia térmica do seu sistema
