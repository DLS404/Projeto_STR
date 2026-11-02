
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

/* =========================================================
   PINOS
   ========================================================= */

#define PIN_MQ2          GPIO_NUM_34
#define PIN_DHT22        GPIO_NUM_4
#define PIN_BOTAO        GPIO_NUM_33

#define PIN_LED_VERDE    GPIO_NUM_25
#define PIN_LED_AMARELO  GPIO_NUM_26
#define PIN_LED_VERMELHO GPIO_NUM_27

#define PIN_BUZZER       GPIO_NUM_14
#define PIN_SERVO        GPIO_NUM_13

/* =========================================================
   CONFIGURACOES
   ========================================================= */

#define LIMITE_ALERTA    3000
#define LIMITE_CRITICO   4000

#define PERIODO_MQ2_MS   100
#define PERIODO_DHT_MS   2000

/* 1 = T4 ligada | 0 = T4 desligada */
#define HABILITAR_T4     0

#define BUZZER_FREQ      2000

/* Limites temporais */
#define RT01_LIMITE_US   ((int64_t)PERIODO_MQ2_MS * 1000LL)
#define RT02_LIMITE_US   50000LL
#define RT03_LIMITE_US   100000LL

/* Relatorio a cada 10 segundos */
#define PERIODO_STATS_US 10000000LL

static const char *TAG = "MONITOR_GAS";

/* =========================================================
   ESTADOS E ESTRUTURAS
   ========================================================= */

typedef enum {
    ESTADO_NORMAL = 0,
    ESTADO_ALERTA,
    ESTADO_CRITICO
} estado_t;

typedef struct {
    int valor_mq2;
    int64_t instante_us;
} leitura_t;

typedef struct {
    estado_t estado;
    int valor_mq2;
    int64_t instante_origem_us;
} comando_t;

typedef struct {
    uint64_t leituras_mq2;

    uint64_t perdas_fila_leituras;
    uint64_t perdas_fila_comandos;

    uint64_t violacoes_rt01;
    uint64_t violacoes_rt02;
    uint64_t violacoes_rt03;

    uint64_t classificacoes;
    uint64_t atuacoes;

    uint64_t leituras_dht_ok;
    uint64_t leituras_dht_erro;

    uint64_t ciclos_t4;
    uint64_t operacoes_t4;

    uint32_t ocupacao_max_fila_leituras;
    uint32_t ocupacao_max_fila_comandos;

    int64_t intervalo_max_mq2_us;
    int64_t intervalo_max_dht_us;

    int64_t execucao_max_t2_us;
    int64_t latencia_max_rt02_us;
    int64_t latencia_max_rt03_us;
    int64_t execucao_max_t4_us;

} estatisticas_t;

/* =========================================================
   VARIAVEIS GLOBAIS
   ========================================================= */

static adc_oneshot_unit_handle_t adc_handle;

static QueueHandle_t fila_leituras;
static QueueHandle_t fila_comandos;

static TaskHandle_t tarefa_atuacao_handle;

static volatile bool buzzer_silenciado = false;
static volatile estado_t estado_atual = ESTADO_NORMAL;

/* Protecao do protocolo DHT22 */
static portMUX_TYPE dht_mux =
    portMUX_INITIALIZER_UNLOCKED;

/* Protecao das estatisticas */
static portMUX_TYPE stats_mux =
    portMUX_INITIALIZER_UNLOCKED;

static estatisticas_t stats = {0};

/* =========================================================
   ESTATISTICAS - FUNCOES AUXILIARES
   ========================================================= */

static void atualizar_maximo(
    int64_t *destino,
    int64_t valor
)
{
    if (valor > *destino) {
        *destino = valor;
    }
}

static void registrar_ocupacao(
    QueueHandle_t fila,
    bool fila_leituras_flag
)
{
    uint32_t ocupacao =
        (uint32_t)uxQueueMessagesWaiting(fila);

    taskENTER_CRITICAL(&stats_mux);

    if (fila_leituras_flag) {

        if (ocupacao > stats.ocupacao_max_fila_leituras) {
            stats.ocupacao_max_fila_leituras = ocupacao;
        }

    } else {

        if (ocupacao > stats.ocupacao_max_fila_comandos) {
            stats.ocupacao_max_fila_comandos = ocupacao;
        }
    }

    taskEXIT_CRITICAL(&stats_mux);
}

/* =========================================================
   IMPRESSAO DAS ESTATISTICAS
   ========================================================= */

static void imprimir_estatisticas(void)
{
    estatisticas_t copia;

    taskENTER_CRITICAL(&stats_mux);
    copia = stats;
    taskEXIT_CRITICAL(&stats_mux);

    ESP_LOGI(TAG, "========== ESTATISTICAS ==========");

    ESP_LOGI(
        TAG,
        "MQ2: leituras=%" PRIu64
        " | intervalo_max=%" PRId64 " us",
        copia.leituras_mq2,
        copia.intervalo_max_mq2_us
    );

    ESP_LOGI(
        TAG,
        "RT01: limite=%" PRId64 " us"
        " | violacoes=%" PRIu64,
        RT01_LIMITE_US,
        copia.violacoes_rt01
    );

    ESP_LOGI(
        TAG,
        "T2: classificacoes=%" PRIu64
        " | execucao_max=%" PRId64 " us",
        copia.classificacoes,
        copia.execucao_max_t2_us
    );

    ESP_LOGI(
        TAG,
        "RT02: limite=%" PRId64 " us"
        " | latencia_max=%" PRId64 " us"
        " | violacoes=%" PRIu64,
        RT02_LIMITE_US,
        copia.latencia_max_rt02_us,
        copia.violacoes_rt02
    );

    ESP_LOGI(
        TAG,
        "RT03: limite=%" PRId64 " us"
        " | latencia_max=%" PRId64 " us"
        " | violacoes=%" PRIu64,
        RT03_LIMITE_US,
        copia.latencia_max_rt03_us,
        copia.violacoes_rt03
    );

    ESP_LOGI(
        TAG,
        "Filas: perdas_T1_T2=%" PRIu64
        " | perdas_T2_T3=%" PRIu64,
        copia.perdas_fila_leituras,
        copia.perdas_fila_comandos
    );

    ESP_LOGI(
        TAG,
        "Filas: ocupacao_max_T1_T2=%" PRIu32 "/10"
        " | T2_T3=%" PRIu32 "/10",
        copia.ocupacao_max_fila_leituras,
        copia.ocupacao_max_fila_comandos
    );

    ESP_LOGI(
        TAG,
        "DHT22: sucesso=%" PRIu64
        " | erros=%" PRIu64
        " | intervalo_max=%" PRId64 " us",
        copia.leituras_dht_ok,
        copia.leituras_dht_erro,
        copia.intervalo_max_dht_us
    );

    ESP_LOGI(
        TAG,
        "T4: ciclos=%" PRIu64
        " | operacoes=%" PRIu64
        " | execucao_max=%" PRId64 " us",
        copia.ciclos_t4,
        copia.operacoes_t4,
        copia.execucao_max_t4_us
    );

    ESP_LOGI(TAG, "==================================");
}

/* =========================================================
   GPIO
   ========================================================= */

static void configurar_gpio(void)
{
    gpio_config_t saidas = {
        .pin_bit_mask =
            (1ULL << PIN_LED_VERDE) |
            (1ULL << PIN_LED_AMARELO) |
            (1ULL << PIN_LED_VERMELHO),

        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&saidas));

    gpio_config_t botao = {
        .pin_bit_mask = (1ULL << PIN_BOTAO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&botao));

    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_AMARELO, 0);
    gpio_set_level(PIN_LED_VERMELHO, 0);
}

/* =========================================================
   ADC - MQ2
   ========================================================= */

static void configurar_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&init_cfg, &adc_handle)
    );

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            ADC_CHANNEL_6,
            &chan_cfg
        )
    );
}

static int ler_mq2(void)
{
    int soma = 0;

    const int quantidade_amostras = 8;

    for (int i = 0; i < quantidade_amostras; i++) {

        int valor = 0;

        esp_err_t erro = adc_oneshot_read(
            adc_handle,
            ADC_CHANNEL_6,
            &valor
        );

        if (erro != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Falha na leitura do ADC: %s",
                esp_err_to_name(erro)
            );

            return 0;
        }

        soma += valor;
    }

    return soma / quantidade_amostras;
}

/* =========================================================
   SERVO - LEDC TIMER 0 / CHANNEL 0
   ========================================================= */

static void configurar_servo(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t canal = {
        .gpio_num = PIN_SERVO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&canal));
}

static void servo_definir_angulo(int angulo)
{
    if (angulo < 0) angulo = 0;
    if (angulo > 180) angulo = 180;

    const uint32_t pulso_min = 500;
    const uint32_t pulso_max = 2400;

    uint32_t pulso_us =
        pulso_min +
        ((pulso_max - pulso_min) * (uint32_t)angulo) / 180U;

    uint32_t duty =
        (pulso_us * 65535U) / 20000U;

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0
        )
    );
}

/* =========================================================
   BUZZER - LEDC TIMER 1 / CHANNEL 1
   ========================================================= */

static void configurar_buzzer(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = BUZZER_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t canal = {
        .gpio_num = PIN_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&canal));
}

static void atualizar_buzzer(void)
{
    bool ativar =
        (estado_atual != ESTADO_NORMAL) &&
        (!buzzer_silenciado);

    uint32_t duty = ativar ? 512 : 0;

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_1,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_1
        )
    );
}

/* =========================================================
   DHT22
   ========================================================= */

static bool dht_esperar_nivel(int nivel)
{
    for (int i = 0; i < 100; i++) {

        if (gpio_get_level(PIN_DHT22) == nivel) {
            return true;
        }

        esp_rom_delay_us(1);
    }

    return false;
}

/*
 * Retornos:
 *  0 = sucesso
 * -1 = timeout
 * -2 = checksum invalido
 */

static int dht22_ler(float *temperatura, float *umidade)
{
    uint8_t dados[5] = {0};

    ESP_ERROR_CHECK(
        gpio_set_direction(
            PIN_DHT22,
            GPIO_MODE_OUTPUT_OD
        )
    );

    ESP_ERROR_CHECK(
        gpio_set_pull_mode(
            PIN_DHT22,
            GPIO_PULLUP_ONLY
        )
    );

    gpio_set_level(PIN_DHT22, 0);

    esp_rom_delay_us(1100);

    gpio_set_level(PIN_DHT22, 1);

    esp_rom_delay_us(30);

    gpio_set_direction(
        PIN_DHT22,
        GPIO_MODE_INPUT
    );

    bool sucesso = true;

    taskENTER_CRITICAL(&dht_mux);

    /* Resposta do sensor */

    if (!dht_esperar_nivel(0)) {
        sucesso = false;
    }

    if (sucesso && !dht_esperar_nivel(1)) {
        sucesso = false;
    }

    if (sucesso && !dht_esperar_nivel(0)) {
        sucesso = false;
    }

    /* Leitura dos 40 bits */

    for (int bit = 0; bit < 40 && sucesso; bit++) {

        if (!dht_esperar_nivel(1)) {
            sucesso = false;
            break;
        }

        esp_rom_delay_us(40);

        int valor = gpio_get_level(PIN_DHT22);

        dados[bit / 8] <<= 1;

        if (valor == 1) {
            dados[bit / 8] |= 1;
        }

        if (!dht_esperar_nivel(0)) {
            sucesso = false;
        }
    }

    taskEXIT_CRITICAL(&dht_mux);

    if (!sucesso) {
        return -1;
    }

    /* Verificacao do checksum */

    uint8_t checksum =
        (uint8_t)(
            dados[0] +
            dados[1] +
            dados[2] +
            dados[3]
        );

    if (checksum != dados[4]) {
        return -2;
    }

    /* Conversao dos valores */

    uint16_t umidade_raw =
        ((uint16_t)dados[0] << 8) | dados[1];

    uint16_t temperatura_raw =
        ((uint16_t)(dados[2] & 0x7F) << 8) |
        dados[3];

    *umidade = umidade_raw / 10.0f;

    *temperatura = temperatura_raw / 10.0f;

    if (dados[2] & 0x80) {
        *temperatura = -*temperatura;
    }

    return 0;
}

/* =========================================================
   CLASSIFICACAO
   ========================================================= */

static estado_t classificar(int valor)
{
    if (valor >= LIMITE_CRITICO) {
        return ESTADO_CRITICO;
    }

    if (valor >= LIMITE_ALERTA) {
        return ESTADO_ALERTA;
    }

    return ESTADO_NORMAL;
}

/* =========================================================
   T1 - AQUISICAO MQ2
   ========================================================= */

static void tarefa_aquisicao(void *arg)
{
    TickType_t ultimo = xTaskGetTickCount();

    const TickType_t periodo =
        pdMS_TO_TICKS(PERIODO_MQ2_MS);

    int64_t inicio_anterior = 0;

    int64_t ultimo_relatorio =
        esp_timer_get_time();

    while (1) {

        int64_t inicio = esp_timer_get_time();

        /* Medicao do intervalo entre aquisicoes */

        if (inicio_anterior != 0) {

            int64_t intervalo =
                inicio - inicio_anterior;

            taskENTER_CRITICAL(&stats_mux);

            atualizar_maximo(
                &stats.intervalo_max_mq2_us,
                intervalo
            );

            if (intervalo > RT01_LIMITE_US) {
                stats.violacoes_rt01++;
            }

            taskEXIT_CRITICAL(&stats_mux);
        }

        inicio_anterior = inicio;

        /* Leitura do sensor */

        leitura_t leitura = {
            .valor_mq2 = ler_mq2(),
            .instante_us = esp_timer_get_time()
        };

        /* Envio para T2 */

        if (xQueueSend(
                fila_leituras,
                &leitura,
                0
            ) != pdPASS) {

            taskENTER_CRITICAL(&stats_mux);
            stats.perdas_fila_leituras++;
            taskEXIT_CRITICAL(&stats_mux);

            ESP_LOGW(
                TAG,
                "Fila T1->T2 cheia; leitura descartada"
            );

        } else {

            registrar_ocupacao(fila_leituras, true);
        }

        taskENTER_CRITICAL(&stats_mux);
        stats.leituras_mq2++;
        taskEXIT_CRITICAL(&stats_mux);

        /* Relatorio a cada 10 segundos */

        int64_t agora = esp_timer_get_time();

        if (agora - ultimo_relatorio >= PERIODO_STATS_US) {

            imprimir_estatisticas();

            ultimo_relatorio = agora;
        }

        vTaskDelayUntil(&ultimo, periodo);
    }
}

/* =========================================================
   T2 - CLASSIFICACAO
   ========================================================= */

static void tarefa_classificacao(void *arg)
{
    leitura_t leitura;

    while (1) {

        if (xQueueReceive(
                fila_leituras,
                &leitura,
                portMAX_DELAY
            ) == pdPASS) {

            int64_t inicio = esp_timer_get_time();

            estado_t estado =
                classificar(leitura.valor_mq2);

            int64_t fim = esp_timer_get_time();

            /* Tempo de execucao isolado da classificacao */

            int64_t execucao_us = fim - inicio;

            /* Latencia da aquisicao ate o fim da classificacao */

            int64_t latencia_us =
                fim - leitura.instante_us;

            taskENTER_CRITICAL(&stats_mux);

            stats.classificacoes++;

            atualizar_maximo(
                &stats.execucao_max_t2_us,
                execucao_us
            );

            atualizar_maximo(
                &stats.latencia_max_rt02_us,
                latencia_us
            );

            if (latencia_us > RT02_LIMITE_US) {
                stats.violacoes_rt02++;
            }

            taskEXIT_CRITICAL(&stats_mux);

            /* Prepara comando para T3 */

            comando_t comando = {
                .estado = estado,
                .valor_mq2 = leitura.valor_mq2,
                .instante_origem_us = leitura.instante_us
            };

            if (xQueueSend(
                    fila_comandos,
                    &comando,
                    0
                ) != pdPASS) {

                taskENTER_CRITICAL(&stats_mux);
                stats.perdas_fila_comandos++;
                taskEXIT_CRITICAL(&stats_mux);

                ESP_LOGW(
                    TAG,
                    "Fila T2->T3 cheia; comando descartado"
                );

            } else {

                registrar_ocupacao(fila_comandos, false);

                if (tarefa_atuacao_handle != NULL) {
                    xTaskNotifyGive(tarefa_atuacao_handle);
                }
            }
        }
    }
}

/* =========================================================
   T3 - ATUACAO
   ========================================================= */

static void aplicar_estado(const comando_t *cmd)
{
    estado_atual = cmd->estado;

    /* LEDs */

    gpio_set_level(
        PIN_LED_VERDE,
        cmd->estado == ESTADO_NORMAL
    );

    gpio_set_level(
        PIN_LED_AMARELO,
        cmd->estado == ESTADO_ALERTA
    );

    gpio_set_level(
        PIN_LED_VERMELHO,
        cmd->estado == ESTADO_CRITICO
    );

    /* Servo */

    if (cmd->estado == ESTADO_NORMAL) {

        servo_definir_angulo(0);

    } else if (cmd->estado == ESTADO_ALERTA) {

        servo_definir_angulo(45);

    } else {

        servo_definir_angulo(90);
    }

    /* Buzzer */

    atualizar_buzzer();

    ESP_LOGI(
        TAG,
        "MQ2=%d | estado=%s | buzzer=%s",
        cmd->valor_mq2,

        cmd->estado == ESTADO_NORMAL ? "NORMAL" :
        cmd->estado == ESTADO_ALERTA ? "ALERTA" :
        "CRITICO",

        buzzer_silenciado ? "SILENCIADO" : "HABILITADO"
    );
}

static void tarefa_atuacao(void *arg)
{
    comando_t comando;

    while (1) {

        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        while (xQueueReceive(
                   fila_comandos,
                   &comando,
                   0
               ) == pdPASS) {

            aplicar_estado(&comando);

            /* RT03: latencia ponta a ponta */

            int64_t fim = esp_timer_get_time();

            int64_t latencia_us =
                fim - comando.instante_origem_us;

            taskENTER_CRITICAL(&stats_mux);

            stats.atuacoes++;

            atualizar_maximo(
                &stats.latencia_max_rt03_us,
                latencia_us
            );

            if (latencia_us > RT03_LIMITE_US) {
                stats.violacoes_rt03++;
            }

            taskEXIT_CRITICAL(&stats_mux);
        }
    }
}

/* =========================================================
   T5 - DHT22
   ========================================================= */

static void tarefa_dht22(void *arg)
{
    float temperatura = 0.0f;
    float umidade = 0.0f;

    int64_t inicio_anterior = 0;

    while (1) {

        int64_t inicio = esp_timer_get_time();

        /* Intervalo entre leituras */

        if (inicio_anterior != 0) {

            int64_t intervalo =
                inicio - inicio_anterior;

            taskENTER_CRITICAL(&stats_mux);

            atualizar_maximo(
                &stats.intervalo_max_dht_us,
                intervalo
            );

            taskEXIT_CRITICAL(&stats_mux);
        }

        inicio_anterior = inicio;

        int resultado =
            dht22_ler(&temperatura, &umidade);

        taskENTER_CRITICAL(&stats_mux);

        if (resultado == 0) {
            stats.leituras_dht_ok++;
        } else {
            stats.leituras_dht_erro++;
        }

        taskEXIT_CRITICAL(&stats_mux);

        if (resultado == 0) {

            ESP_LOGI(
                TAG,
                "DHT22 | Temperatura: %.1f C | Umidade: %.1f%%",
                temperatura,
                umidade
            );

        } else if (resultado == -1) {

            ESP_LOGW(
                TAG,
                "DHT22: timeout na comunicacao"
            );

        } else {

            ESP_LOGW(
                TAG,
                "DHT22: checksum invalido"
            );
        }

        vTaskDelay(
            pdMS_TO_TICKS(PERIODO_DHT_MS)
        );
    }
}

/* =========================================================
   T6 - BOTAO
   ========================================================= */

static void tarefa_botao(void *arg)
{
    int nivel_anterior = 1;

    while (1) {

        int nivel_atual =
            gpio_get_level(PIN_BOTAO);

        /* Borda de descida: botao pressionado */

        if (nivel_anterior == 1 &&
            nivel_atual == 0) {

            vTaskDelay(pdMS_TO_TICKS(35));

            if (gpio_get_level(PIN_BOTAO) == 0) {

                buzzer_silenciado =
                    !buzzer_silenciado;

                atualizar_buzzer();

                ESP_LOGI(
                    TAG,
                    "Buzzer %s",
                    buzzer_silenciado ?
                    "silenciado" : "reativado"
                );

                /* Aguarda liberacao do botao */

                while (gpio_get_level(PIN_BOTAO) == 0) {

                    vTaskDelay(
                        pdMS_TO_TICKS(10)
                    );
                }
            }
        }

        nivel_anterior =
            gpio_get_level(PIN_BOTAO);

        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }
}

/* =========================================================
   T4 - CARGA COMPUTACIONAL
   ========================================================= */

static void tarefa_carga(void *arg)
{
#if HABILITAR_T4

    volatile double resultado = 0.0;

    while (1) {

        int64_t inicio = esp_timer_get_time();

        for (int i = 1; i <= 10000; i++) {

            resultado +=
                sqrt((double)i) *
                sin((double)i) *
                cos((double)i);
        }

        int64_t fim = esp_timer_get_time();

        int64_t duracao = fim - inicio;

        taskENTER_CRITICAL(&stats_mux);

        stats.ciclos_t4++;

        stats.operacoes_t4 += 10000;

        atualizar_maximo(
            &stats.execucao_max_t4_us,
            duracao
        );

        taskEXIT_CRITICAL(&stats_mux);

        ESP_LOGI(
            TAG,
            "T4 carga ativa | operacoes=10000"
            " | resultado=%.4f"
            " | duracao=%" PRId64 " us",
            (double)resultado,
            duracao
        );

        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }

#else

    ESP_LOGI(TAG, "T4 desativada");

    vTaskDelete(NULL);

#endif
}

/* =========================================================
   APP MAIN
   ========================================================= */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Inicializando monitoramento - ESP-IDF 6.1.0"
    );

    /* Configuracoes iniciais */

    configurar_gpio();

    configurar_adc();

    configurar_servo();

    configurar_buzzer();

    servo_definir_angulo(0);

    /* Criacao das filas */

    fila_leituras =
        xQueueCreate(10, sizeof(leitura_t));

    fila_comandos =
        xQueueCreate(10, sizeof(comando_t));

    if (fila_leituras == NULL ||
        fila_comandos == NULL) {

        ESP_LOGE(
            TAG,
            "Falha ao criar filas"
        );

        abort();
    }

    BaseType_t resultado_tarefa;

    /* T3 - Atuacao */

    resultado_tarefa = xTaskCreate(
        tarefa_atuacao,
        "T3_Atuacao",
        4096,
        NULL,
        3,
        &tarefa_atuacao_handle
    );

    configASSERT(resultado_tarefa == pdPASS);

    /* T2 - Classificacao */

    resultado_tarefa = xTaskCreate(
        tarefa_classificacao,
        "T2_Classificacao",
        4096,
        NULL,
        3,
        NULL
    );

    configASSERT(resultado_tarefa == pdPASS);

    /* T1 - Aquisicao */

    resultado_tarefa = xTaskCreate(
        tarefa_aquisicao,
        "T1_Aquisicao",
        4096,
        NULL,
        2,
        NULL
    );

    configASSERT(resultado_tarefa == pdPASS);

    /* T5 - DHT22 */

    resultado_tarefa = xTaskCreate(
        tarefa_dht22,
        "T5_DHT22",
        3072,
        NULL,
        1,
        NULL
    );

    configASSERT(resultado_tarefa == pdPASS);

    /* T6 - Botao */

    resultado_tarefa = xTaskCreate(
        tarefa_botao,
        "T6_Botao",
        2048,
        NULL,
        2,
        NULL
    );

    configASSERT(resultado_tarefa == pdPASS);

    /* T4 - Carga computacional */

    resultado_tarefa = xTaskCreate(
        tarefa_carga,
        "T4_Carga",
        4096,
        NULL,
        1,
        NULL
    );

    configASSERT(resultado_tarefa == pdPASS);

    ESP_LOGI(
        TAG,
        "Todas as seis tarefas foram inicializadas"
    );
}
