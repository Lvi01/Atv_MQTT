// -------------------- INCLUDES --------------------
// Bibliotecas padrão Pico, periféricos, display OLED, MQTT e rede
// Bibliotecas padrão do Pico SDK
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"

// Bibliotecas de hardware e periféricos
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

// Bibliotecas de utilidades e funções auxiliares
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "final.pio.h"

// Bibliotecas de rede e MQTT
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"

// -------------------- DEFINES ---------------------
// Definições de pinos, tópicos MQTT, credenciais e parâmetros do sistema
#define WIFI_SSID "Casa1"
#define WIFI_PASSWORD "40302010"
#define MQTT_SERVER "192.168.1.7"
#define MQTT_USERNAME "levi"
#define MQTT_PASSWORD "levi21"

#define ADC_UMIDADE_PIN     26          // Pino ADC para umidade (joystick)
#define ADC_TEMPERATURA_PIN 27          // Pino ADC para temperatura (joystick)
#define BOTAO_MODO          5           // Pino do botão de modo (A)
#define LED_VERMELHO        13          // Pino do LED vermelho
#define LED_VERDE           11          // Pino do LED verde
#define LED_AZUL            12          // Pino do LED azul
#define BUZZER              21          // Pino do buzzer
#define LED_MATRIX          7           // Pino do LED matrix (PIO)
#define NUM_LEDS            25          // Número de LEDs na matriz
#define I2C_SDA             14          // Pino SDA do I2C
#define I2C_SCL             15          // Pino SCL do I2C
#define I2C_PORT            i2c1        // Porta I2C usada
#define I2C_ENDERECO        0x3C        // Endereço do display OLED

#define TEMP_WORKER_TIME_S 1            // Intervalo do worker de temperatura (1 segundo)
#define MQTT_KEEP_ALIVE_S 60            // Tempo de keep-alive do MQTT (60 segundos)
#define MQTT_SUBSCRIBE_QOS 1            // QoS para assinaturas MQTT
#define MQTT_PUBLISH_QOS 1              // QoS para publicações MQTT
#define MQTT_PUBLISH_RETAIN 0           // Retenção de mensagens MQTT
#define MQTT_WILL_TOPIC "/online"       // Tópico de mensagem de "online" MQTT
#define MQTT_WILL_MSG "0"               // Mensagem de "online" MQTT
#define MQTT_WILL_QOS 1                 // QoS para mensagem de "online" MQTT
#define MQTT_DEVICE_NAME "pico"         // Nome do dispositivo MQTT
#define MQTT_TOPIC_LEN 100              // Tamanho máximo do tópico MQTT


// -------------------- STRUCTS ---------------------
// Estrutura para armazenar dados do cliente MQTT e estado da conexão
typedef struct {
    mqtt_client_t* mqtt_client_inst;                // Instância do cliente MQTT
    struct mqtt_connect_client_info_t mqtt_client_info; // Informações de conexão MQTT
    char data[MQTT_OUTPUT_RINGBUF_SIZE];            // Buffer para dados recebidos
    char topic[MQTT_TOPIC_LEN];                     // Buffer para tópico recebido
    uint32_t len;                                   // Tamanho dos dados recebidos
    ip_addr_t mqtt_server_address;                  // Endereço IP do broker MQTT
    bool connect_done;                              // Flag de conexão estabelecida
    int subscribe_count;                            // Contador de tópicos assinados
    bool stop_client;                               // Flag para parar o cliente
} MQTT_CLIENT_DATA_T;

// -------------------- VARIÁVEIS GLOBAIS ---------------------
// Variáveis globais para controle de modo, alarme, display e matriz de LEDs
volatile bool modo_manual = false;                  // Indica se está no modo manual
volatile uint64_t ultima_troca_modo = 0;           // Timestamp da última troca de modo
static volatile uint64_t debounce_antes = 0;        // Auxiliar para debounce do botão
static volatile bool alarme_ativo = false;          // Indica se o alarme está ativo
static volatile bool exibir_temp = false;           // Flag para exibir temperatura (não usado)
static volatile bool exibir_umidade = false;        // Flag para exibir umidade (não usado)

ssd1306_t display;                                  // Estrutura do display OLED
PIO pio = pio0;                                     // PIO usado para matriz de LEDs
uint sm;                                            // State machine do PIO
uint32_t valor_led;                                 // Valor RGB para matriz de LEDs
uint8_t planta_matriz[NUM_LEDS] = {                 // Mapa da matriz de LEDs (formato da planta)
    2, 2, 2, 2, 2,
    0, 0, 1, 0, 0,
    0, 0, 1, 1, 0,
    1, 1, 1, 1, 0,
    1, 1, 1, 0, 0
};

// -------------------- PROTÓTIPOS ---------------------
// Prototipação das funções do sistema e MQTT

// Inicializa todos os periféricos do sistema (ADC, I2C, PWM, Display, PIO)
bool inicializar_perifericos();

// Configura um pino para uso com PWM
void configurar_pwm(uint gpio);

// Callback do botão de modo: alterna entre modo manual/automático ou desativa alarme
void botao_modo_callback(uint gpio, uint32_t eventos);

// Lê a temperatura do sensor interno do RP2040
float ler_temperatura_interna();

// Lê a temperatura simulada pelo joystick (ADC)
float ler_temperatura_joystick();

// Lê a umidade simulada pelo joystick (ADC)
float ler_umidade_joystick();

// Atualiza o display OLED com dados de temperatura, umidade e modo
void atualizar_display_dados(float temperatura, float umidade);

// Atualiza os LEDs de feedback visual conforme temperatura e umidade
void atualizar_feedback_leds(float temperatura, float umidade);

// Atualiza a animação da matriz de LEDs conforme a temperatura
void animacao_matriz_leds(float temperatura);

// Alterna o estado do alarme (ativa/desativa)
void acionar_alarme();

// Callback periódico para relatório de status no terminal
bool alarme_periodico_callback(repeating_timer_t *t);

// Converte valores RGB para formato da matriz de LEDs
uint32_t matrix_rgb(double b, double r, double g);

// Gera uma mensagem de estado da planta baseada em temperatura e umidade
void interpretar_estado_planta(float temp, float umid, char *msg, size_t tamanho);

// ==================== MQTT ====================

// Callback de publicação MQTT (verifica sucesso/erro)
static void pub_request_cb(__unused void *arg, err_t err);

// Monta o tópico MQTT completo (com client_id se necessário)
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Liga/desliga o LED onboard via MQTT
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

// Publica a temperatura atual no MQTT
static void publish_temperature(MQTT_CLIENT_DATA_T *state);

// Publica a umidade atual no MQTT
static void publish_umidade(MQTT_CLIENT_DATA_T *state);

// Publica o modo atual (manual/automático) no MQTT
static void publish_modo(MQTT_CLIENT_DATA_T *state);

// Publica o estado do alarme (ligado/desligado) no MQTT
static void publish_alarme(MQTT_CLIENT_DATA_T *state);

// Publica o estado da planta (mensagem) no MQTT
static void publish_estado_planta(MQTT_CLIENT_DATA_T *state);

// Callback de subscribe MQTT
static void sub_request_cb(void *arg, err_t err);

// Callback de unsubscribe MQTT
static void unsub_request_cb(void *arg, err_t err);

// Assina ou remove assinatura dos tópicos MQTT de controle
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

// Callback de dados recebidos via MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Callback de publicação recebida via MQTT
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Worker periódico para publicar dados no MQTT
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);

// Worker de tempo para o MQTT (estrutura global)
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };

// Callback de conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicia o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Callback de resolução DNS para o broker MQTT
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

// -------------------- MAIN ---------------------
int main(void) {
    stdio_init_all(); // Inicializa stdio para printf/scanf

    // Inicializa todos os periféricos do sistema
    if (!inicializar_perifericos()) {
        printf("Erro na inicialização dos periféricos.\n");
        return 1;
    }

    // Configura o botão de troca de modo com interrupção
    gpio_init(BOTAO_MODO);
    gpio_set_dir(BOTAO_MODO, GPIO_IN);
    gpio_pull_up(BOTAO_MODO);
    gpio_set_irq_enabled_with_callback(BOTAO_MODO, GPIO_IRQ_EDGE_FALL, true, botao_modo_callback);

    // --- Inicialização do MQTT ---
    static MQTT_CLIENT_DATA_T state;
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) unique_id_buf[i] = tolower(unique_id_buf[i]);
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

    // Inicializa Wi-Fi e conecta à rede
    if (cyw43_arch_init()) panic("Failed to initialize CYW43");
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) panic("Failed to connect to Wi-Fi");

    // Resolve o endereço do broker MQTT e inicia o cliente
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();
    if (err == ERR_OK) start_client(&state);
    else if (err != ERR_INPROGRESS) panic("dns request failed");

    // Inicia timer periódico para relatório no terminal
    static repeating_timer_t timer;
    add_repeating_timer_ms(5000, alarme_periodico_callback, NULL, &timer);

    // Loop principal do sistema
    while (true) {
        // Leitura dos sensores (temperatura e umidade)
        float temperatura = modo_manual ? ler_temperatura_joystick() : ler_temperatura_interna();
        float umidade = ler_umidade_joystick();

        // Atualiza feedback visual e display
        atualizar_feedback_leds(temperatura, umidade);
        animacao_matriz_leds(temperatura);
        atualizar_display_dados(temperatura, umidade);

        // Mantém a conexão Wi-Fi ativa
        cyw43_arch_poll();
        sleep_ms(200);

        // Controle do buzzer em caso de alarme
        const uint16_t frequencias[] = {1000, 1500};
        const uint16_t duracao = 300;
        uint8_t i = 0;
        while (alarme_ativo) {
            pwm_set_clkdiv(pwm_gpio_to_slice_num(BUZZER), 125.0f);
            pwm_set_wrap(pwm_gpio_to_slice_num(BUZZER), 125000 / frequencias[i]);
            pwm_set_chan_level(pwm_gpio_to_slice_num(BUZZER), pwm_gpio_to_channel(BUZZER), 125000 / frequencias[i] / 2);
            sleep_ms(duracao);
            i = !i;
        }
        pwm_set_chan_level(pwm_gpio_to_slice_num(BUZZER), pwm_gpio_to_channel(BUZZER), 0);

        // Mantém o loop MQTT ativo (timeout de 10s)
        if (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
            cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
        }
    }
    cyw43_arch_deinit();
    return 0;
}

// -------------------- FUNÇÕES DE HARDWARE ---------------------
bool inicializar_perifericos() {
    adc_init();
    adc_gpio_init(ADC_UMIDADE_PIN);
    adc_gpio_init(ADC_TEMPERATURA_PIN);
    adc_set_temp_sensor_enabled(true);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&display, WIDTH, HEIGHT, false, I2C_ENDERECO, I2C_PORT);
    ssd1306_config(&display);
    ssd1306_fill(&display, false);
    ssd1306_send_data(&display);

    configurar_pwm(LED_VERDE);
    configurar_pwm(LED_AZUL);
    configurar_pwm(LED_VERMELHO);
    configurar_pwm(BUZZER);

    uint offset = pio_add_program(pio, &final_program);
    sm = pio_claim_unused_sm(pio, true);
    final_program_init(pio, sm, offset, LED_MATRIX);

    return true;
}

void configurar_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 4095);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(gpio), 0);
    pwm_set_enabled(slice, true);
}

float ler_temperatura_joystick() {
    adc_select_input(1);
    uint16_t raw = adc_read();
    return (raw / 4095.0f) * 60.0f;
}

float ler_umidade_joystick() {
    adc_select_input(0);
    uint16_t raw = adc_read();
    return (raw / 4095.0f) * 100.0f;
}

float ler_temperatura_interna() {
    adc_select_input(4);
    uint16_t raw = adc_read();
    const float fator_conv = 3.3f / (1 << 12);
    return 27.0f - ((raw * fator_conv) - 0.706f) / 0.001721f;
}

void animacao_matriz_leds(float temperatura) {
    double r = 0.0, g = 1.0, b = 0.0;
    if (temperatura > 50.0) { r = 1.0; g = 0.0; b = 0.0; }
    else if (temperatura < 10.0) { r = 0.0; g = 0.0; b = 1.0; }
    for (int i = 0; i < NUM_LEDS; i++) {
        if (planta_matriz[i] == 1) valor_led = matrix_rgb(b, r, g);
        else if (planta_matriz[i] == 2) valor_led = matrix_rgb(0.0, 1.0, 1.0);
        else valor_led = matrix_rgb(0.0, 0.0, 0.0);
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}

uint32_t matrix_rgb(double b, double r, double g) {
    unsigned char R = r * 255, G = g * 255, B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

void acionar_alarme() {
    alarme_ativo = !alarme_ativo;
    if (alarme_ativo) {
        pwm_set_gpio_level(LED_VERDE, 0);
        pwm_set_gpio_level(LED_AZUL, 0);
        pwm_set_gpio_level(LED_VERMELHO, 4095);
        printf("Alarme ativado!\n");
    } else {
        pwm_set_gpio_level(BUZZER, 0);
        printf("Alarme desativado!\n");
    }
}

void atualizar_feedback_visual(float temperatura, float umidade) {
    static uint64_t tempo_inicio_alerta = 0;
    uint64_t agora = to_us_since_boot(get_absolute_time());
    bool temp_critica = temperatura < 20.0f || temperatura > 40.0f;
    bool umid_critica = umidade < 20.0f || umidade > 80.0f;
    bool cond_critica = temp_critica && umid_critica;
    if (cond_critica) {
        if (tempo_inicio_alerta == 0) tempo_inicio_alerta = agora;
        if ((agora - tempo_inicio_alerta) > 5000000) {
            printf("Condição crítica detectada.\n");
            acionar_alarme();
        }
        pwm_set_gpio_level(LED_VERDE, 0);
        pwm_set_gpio_level(LED_AZUL, 0);
        pwm_set_gpio_level(LED_VERMELHO, 4095);
    } else {
        tempo_inicio_alerta = 0;
        if (temp_critica || umid_critica) {
            pwm_set_gpio_level(LED_VERDE, 4095);
            pwm_set_gpio_level(LED_AZUL, 0);
            pwm_set_gpio_level(LED_VERMELHO, 4095);
        } else {
            pwm_set_gpio_level(LED_VERDE, 4095);
            pwm_set_gpio_level(LED_AZUL, 0);
            pwm_set_gpio_level(LED_VERMELHO, 0);
        }
    }
}

void atualizar_feedback_leds(float t, float u) {
    atualizar_feedback_visual(t, u);
}

void atualizar_display_dados(float temperatura, float umidade) {
    char temp_str[20], umidade_str[20], modo_str[20];
    snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", temperatura);
    snprintf(umidade_str, sizeof(umidade_str), "Umid: %.1f %%", umidade);
    snprintf(modo_str, sizeof(modo_str), "Modo: %s", modo_manual ? "Manual" : "Auto");
    ssd1306_fill(&display, false);
    if (!alarme_ativo) {
        ssd1306_draw_string(&display, temp_str, 3, 10);
        ssd1306_draw_string(&display, umidade_str, 3, 25);
        ssd1306_draw_string(&display, modo_str, 3, 40);
    } else {
        ssd1306_draw_string(&display, "ALARME", 10, 10);
        ssd1306_draw_string(&display, "DISPARADO", 10, 20);
        ssd1306_draw_string(&display, "Aperte o", 10, 30);
        ssd1306_draw_string(&display, "botao A para/", 10, 40);
        ssd1306_draw_string(&display, "desativar", 10, 50);
    }
    ssd1306_send_data(&display);
}

bool alarme_periodico_callback(repeating_timer_t *t) {
    float temperatura = modo_manual ? ler_temperatura_joystick() : ler_temperatura_interna();
    float umidade = ler_umidade_joystick();
    printf("[Relatório] Temp: %.2f °C | Umid: %.2f %%\n", temperatura, umidade);
    if (alarme_ativo) {
        printf("Alarme ligado, aperte o botao A na placa para desativar\n");
        return true;
    }
    bool temp_ok = temperatura >= 20.0f && temperatura <= 40.0f;
    bool umid_ok = umidade >= 20.0f && umidade <= 80.0f;
    if (temp_ok && umid_ok) printf("Sua planta esta feliz!\n");
    else if (!temp_ok && !umid_ok) printf("Sua planta esta em perigo!\n");
    else if (!temp_ok) printf(temperatura < 20.0f ? "Sua planta esta com frio!\n" : "Sua planta esta com calor!\n");
    else if (!umid_ok) printf(umidade < 20.0f ? "Sua planta esta com sede!\n" : "Excesso de agua detectado!\n");
    return true;
}

void interpretar_estado_planta(float temp, float umid, char *msg, size_t tamanho) {
    if (alarme_ativo) {
        strncpy(msg, "Alarme ligado, aperte o botao A na placa para desativar", tamanho);
        msg[tamanho - 1] = '\0';
        return;
    }
    bool temp_ok = temp >= 20.0 && temp <= 40.0;
    bool umid_ok = umid >= 20.0 && umid <= 80.0;
    if (temp_ok && umid_ok) strncpy(msg, "Sua planta está feliz!", tamanho);
    else if (!temp_ok && !umid_ok) strncpy(msg, "Sua planta está em perigo!", tamanho);
    else if (!temp_ok) strncpy(msg, temp < 20.0 ? "Sua planta está com frio!" : "Sua planta está com calor!", tamanho);
    else if (!umid_ok) strncpy(msg, umid < 20.0 ? "Sua planta está com sede!" : "Excesso de água detectado!", tamanho);
    msg[tamanho - 1] = '\0';
}

// -------------------- FUNÇÕES DE BOTÃO ---------------------
void botao_modo_callback(uint gpio, uint32_t eventos) {
    uint64_t agora = to_us_since_boot(get_absolute_time());
    if ((agora - ultima_troca_modo) > 300000) {
        if (alarme_ativo) acionar_alarme();
        else {
            modo_manual = !modo_manual;
            printf("Modo %s ativado\n", modo_manual ? "MANUAL" : "AUTOMÁTICO");
        }
        ultima_troca_modo = agora;
    }
}

// -------------------- FUNÇÕES MQTT ---------------------
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) printf("pub_request_cb failed %d", err);
}

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = modo_manual ? ler_temperatura_joystick() : ler_temperatura_interna();
    if (temperature != old_temperature) {
        old_temperature = temperature;
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void publish_umidade(MQTT_CLIENT_DATA_T *state) {
    static float old_umidade = -1.0f;
    const char *umidade_key = full_topic(state, "/umidade");
    float umidade = ler_umidade_joystick();
    if (umidade != old_umidade) {
        old_umidade = umidade;
        char umid_str[16];
        snprintf(umid_str, sizeof(umid_str), "%.2f", umidade);
        mqtt_publish(state->mqtt_client_inst, umidade_key, umid_str, strlen(umid_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void publish_modo(MQTT_CLIENT_DATA_T *state) {
    static bool old_modo = false;
    const char *modo_key = full_topic(state, "/modo");
    bool modo = modo_manual;
    if (modo != old_modo) {
        old_modo = modo;
        const char *modo_str = modo ? "manual" : "automatico";
        mqtt_publish(state->mqtt_client_inst, modo_key, modo_str, strlen(modo_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void publish_alarme(MQTT_CLIENT_DATA_T *state) {
    static bool old_alarme = false;
    const char *alarme_key = full_topic(state, "/alarme");
    bool alarme = alarme_ativo;
    if (alarme != old_alarme) {
        old_alarme = alarme;
        const char *alarme_str = alarme ? "ligado" : "desligado";
        mqtt_publish(state->mqtt_client_inst, alarme_key, alarme_str, strlen(alarme_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void publish_estado_planta(MQTT_CLIENT_DATA_T *state) {
    static char old_msg[64] = "";
    char msg[64];
    float temperatura = modo_manual ? ler_temperatura_joystick() : ler_temperatura_interna();
    float umidade = ler_umidade_joystick();
    interpretar_estado_planta(temperatura, umidade, msg, sizeof(msg));
    if (strcmp(msg, old_msg) != 0) {
        strncpy(old_msg, msg, sizeof(old_msg));
        const char *estado_key = full_topic(state, "/estado_planta");
        mqtt_publish(state->mqtt_client_inst, estado_key, msg, strlen(msg), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) panic("subscribe request failed %d", err);
    state->subscribe_count++;
}

static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) panic("unsubscribe request failed %d", err);
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);
    if (state->subscribe_count <= 0 && state->stop_client) mqtt_disconnect(state->mqtt_client_inst);
}

static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/modo"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    if (strcmp(basic_topic, "/led") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/print") == 0) {
        printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true;
        sub_unsub_topics(state, false);
    }
    else if (strcmp(basic_topic, "/modo") == 0) {
        if (lwip_stricmp(state->data, "manual") == 0) {
            modo_manual = true;
        } else if (lwip_stricmp(state->data, "automatico") == 0) {
            modo_manual = false;
        }
        // Atualiza o display imediatamente após mudança remota
        float temperatura = modo_manual ? ler_temperatura_joystick() : ler_temperatura_interna();
        float umidade = ler_umidade_joystick();
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    publish_temperature(state);
    publish_umidade(state);
    publish_modo(state);
    publish_alarme(state);
    publish_estado_planta(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true);
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) panic("Failed to connect to mqtt server");
    } else {
        panic("Unexpected status");
    }
}

static void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
#else
    const int port = MQTT_PORT;
#endif
    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) panic("MQTT client instance creation error");
    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}