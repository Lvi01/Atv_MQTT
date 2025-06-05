# Sistema de Monitoramento de Planta com Raspberry Pi Pico W e MQTT 🌿💧🌡️

Este projeto implementa um sistema de monitoramento ambiental para plantas utilizando uma Raspberry Pi Pico W. Ele mede temperatura e umidade, fornece feedback visual e sonoro local, e se comunica remotamente via protocolo MQTT.

## 📜 Descrição

O sistema é projetado para monitorar as condições ambientais de uma planta, alertando o usuário sobre situações críticas (temperatura ou umidade fora das faixas ideais) através de LEDs, um display OLED, um buzzer e uma matriz de LEDs. Os dados também são publicados em um broker MQTT, permitindo monitoramento e controle remotos. O usuário pode alternar entre um modo de leitura automática (usando o sensor de temperatura interno do Pico) e um modo manual (usando um joystick para simular leituras de temperatura e umidade).

## ✨ Funcionalidades

* **Conectividade Wi-Fi:** Conecta-se a uma rede Wi-Fi local.
* **Comunicação MQTT:** Publica dados de sensores e status, e recebe comandos de um broker MQTT.
* **Leitura de Sensores:**
    * Temperatura: Sensor interno do RP2040 (modo automático) ou ADC via joystick (modo manual).
    * Umidade: ADC via joystick.
* **Feedback Visual:**
    * **Display OLED (I²C):** Exibe temperatura, umidade, modo de operação e status de alarme.
    * **LED RGB (PWM):** Indica o estado geral do ambiente (Verde: Normal, Laranja/Amarelo: Alerta, Vermelho: Crítico/Alarme).
    * **Matriz de LEDs (PIO):** Animação visual representando a "saúde" da planta com base na temperatura.
* **Feedback Sonoro:**
    * **Buzzer (PWM):** Ativa um alarme sonoro em condições críticas persistentes.
* **Interação do Usuário:**
    * **Botão Físico:** Alterna entre modo manual/automático e desativa o alarme.
* **Controle Remoto (via MQTT):**
    * Ligar/desligar LED onboard.
    * Imprimir mensagens no console do Pico.
    * Solicitar uptime.
    * Alterar modo de operação (manual/automático).
    * Desligar o cliente MQTT.
* **Operação Assíncrona:** Utiliza timers e workers para tarefas periódicas sem bloquear o loop principal.

## 🛠️ Hardware Utilizado

* Raspberry Pi Pico W
* Joystick analógico (para simulação de umidade e temperatura no modo manual)
* Display OLED SSD1306 (conectado via I²C)
* LED RGB (cátodo comum ou ânodo comum, ajuste o código PWM conforme necessário)
* Buzzer passivo
* Matriz de LEDs WS2812B (ou similar, controlada por PIO) - `NUM_LEDS` definido como 25
* Botão (Push-button)

### Pinos Utilizados (conforme `mqtt_client.c`):

* `ADC_UMIDADE_PIN`: GPIO 26 (ADC0)
* `ADC_TEMPERATURA_PIN`: GPIO 27 (ADC1)
* `BOTAO_MODO`: GPIO 5
* `LED_VERMELHO`: GPIO 13 (PWM)
* `LED_VERDE`: GPIO 11 (PWM)
* `LED_AZUL`: GPIO 12 (PWM)
* `BUZZER`: GPIO 21 (PWM)
* `LED_MATRIX`: GPIO 7 (PIO)
* `I2C_SDA`: GPIO 14 (I2C1 SDA)
* `I2C_SCL`: GPIO 15 (I2C1 SCL)

## ⚙️ Software e Bibliotecas

* [Pico SDK](https://github.com/raspberrypi/pico-sdk)
* lwIP (Lightweight IP stack)
* MQTT client (integrado com lwIP)
* Biblioteca `lib/ssd1306.h` para o display OLED
* Biblioteca `lib/font.h` para o display OLED
* Programa PIO `final.pio.h` (para a matriz de LEDs)

## 🚀 Configuração e Instalação

1.  **Clone o repositório:**
    ```bash
    git clone [URL_DO_SEU_REPOSITORIO]
    cd [NOME_DO_SEU_REPOSITORIO]
    ```
2.  **Configure as Credenciais:**
    Abra o arquivo `mqtt_client.c` e modifique as seguintes macros `#define` com suas configurações:
    * `WIFI_SSID`: Nome da sua rede Wi-Fi.
    * `WIFI_PASSWORD`: Senha da sua rede Wi-Fi.
    * `MQTT_SERVER`: Endereço IP ou hostname do seu broker MQTT.
    * `MQTT_USERNAME`: Usuário para autenticação no broker MQTT (se houver).
    * `MQTT_PASSWORD`: Senha para autenticação no broker MQTT (se houver).
    * Verifique e ajuste os pinos GPIO (`#define`) se seu hardware estiver conectado de forma diferente.

3.  **Configure o Ambiente de Build do Pico SDK:**
    Certifique-se de que você tem o Pico SDK configurado corretamente em seu ambiente.

4.  **Compile o Projeto:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

5.  **Flashe na Pico W:**
    Copie o arquivo `pico_plant_monitor.uf2` gerado no diretório `build` para a Pico W quando ela estiver no modo BOOTSEL.

## 💡 Como Funciona

1.  **Inicialização:** O sistema inicializa os periféricos (ADC, I2C para OLED, PWM para LEDs e buzzer, PIO para a matriz de LEDs, GPIO para o botão).
2.  **Conexão:** Tenta se conectar à rede Wi-Fi e, em seguida, ao broker MQTT especificado.
3.  **Loop Principal:**
    * Lê os sensores: temperatura (interna ou joystick) e umidade (joystick).
    * Atualiza o feedback visual: display OLED, LED RGB e matriz de LEDs com base nos valores lidos.
    * Verifica se há condições críticas para ativar o alarme (buzzer e LED vermelho).
    * Processa eventos de rede Wi-Fi e MQTT.
    * Verifica o estado do alarme e, se ativo, toca o buzzer.
4.  **Interrupção do Botão:** Uma interrupção é acionada ao pressionar o botão, permitindo alternar o modo de operação ou desativar o alarme (com debounce).
5.  **Workers Assíncronos:**
    * Um worker periódico publica os dados de temperatura, umidade, modo, estado do alarme e uma mensagem sobre o estado da planta para o broker MQTT.
    * Um timer periódico imprime um relatório de status no console serial.
6.  **Comandos MQTT:** O dispositivo se inscreve em tópicos para receber comandos remotos, como ligar/desligar um LED, imprimir mensagens, etc.

## 📡 Tópicos MQTT

O `client_id` do MQTT é formado por `MQTT_DEVICE_NAME` (padrão "pico") + os últimos 4 caracteres do ID único da placa Pico. Se `MQTT_UNIQUE_TOPIC` for `1` (não definido no código, mas uma construção comum), os tópicos abaixo seriam prefixados com `/CLIENT_ID`. No código fornecido, `MQTT_UNIQUE_TOPIC` não é usado dessa forma, então os tópicos são os base.

### Publicações (Dispositivo -> Broker)

* `CLIENT_ID/online` (Will Topic): Publica "1" quando conectado, "0" (configurado em `MQTT_WILL_MSG`) se desconectado abruptamente.
* `/temperature`: Valor da temperatura atual (ex: "25.34").
* `/umidade`: Valor da umidade atual (ex: "60.12").
* `/modo`: Modo de operação atual ("manual" ou "automatico").
* `/alarme`: Estado do alarme ("ligado" ou "desligado").
* `/estado_planta`: Mensagem descritiva do estado da planta (ex: "Sua planta está feliz!").
* `/led/state`: Confirmação do estado do LED onboard ("On" ou "Off") após comando.
* `/uptime`: Tempo de atividade do dispositivo em segundos, em resposta a um ping.

### Subscrições (Broker -> Dispositivo)

* `/led`: Controla o LED onboard da Pico W. Mensagens: "On", "1" (para ligar) ou "Off", "0" (para desligar).
* `/print`: Imprime a mensagem recebida no console serial do Pico.
* `/ping`: Ao receber qualquer mensagem neste tópico, o Pico responde com seu uptime no tópico `/uptime`.
* `/exit`: Faz com que o cliente MQTT se desconecte do broker.
* `/modo`: Altera o modo de operação. Mensagens: "manual" ou "automatico".

## 🔧 Configurações Adicionais (Defines)

O comportamento do sistema pode ser ajustado através de macros `#define` no início de `mqtt_client.c`:

* `TEMP_WORKER_TIME_S`: Intervalo (em segundos) para publicação de dados MQTT.
* `MQTT_KEEP_ALIVE_S`: Tempo de keep-alive para a conexão MQTT.
* `ADC_UMIDADE_PIN`, `ADC_TEMPERATURA_PIN`, etc.: Mapeamento dos pinos GPIO.
* `NUM_LEDS`: Número de LEDs na matriz WS2812B.
* Limites de temperatura e umidade para alertas e alarme (definidos implicitamente nas funções `atualizar_feedback_visual` e `alarme_periodico_callback`).
