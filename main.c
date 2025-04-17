#include "pico/bootrom.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "ws2812.pio.h"

// Definições de pinos para a BitDogLab
#define I2C_PORT i2c1
#define I2C_SDA 14          // Pino SDA para display SSD1306
#define I2C_SCL 15          // Pino SCL para display SSD1306
#define endereco 0x3C       // Endereço I2C do display SSD1306
#define JOYSTICK_X_PIN 26   // Pino ADC para eixo X do joystick
#define JOYSTICK_Y_PIN 27   // Pino ADC para eixo Y do joystick
#define JOYSTICK_PB 22      // Botão do joystick
#define Botao_A 5           // Botão A (liga buzzer)
#define BOTAO_B 6           // Botão B (desliga buzzer)
#define LED_R 13            // LED vermelho (PWM)
#define LED_G 11            // LED verde (on/off)
#define LED_B 12            // LED azul (PWM)
#define BUZZER_PIN 10       // Pino PWM para buzzer
#define WS2812_PIN 7        // Pino para matriz de LEDs WS2812
#define NUM_PIXELS 25       // Número de LEDs na matriz 5x5
#define IS_RGBW false       // Tipo de LEDs WS2812 (RGB, não RGBW)
#define UART_ID uart0
#define UART_TX_PIN 0       // Pino TX para UART
#define UART_RX_PIN 1       // Pino RX para UART

// Constantes do projeto
#define PWM_WRAP 4095       // Valor de wrap para PWM (resolução)
#define JOY_CENTER 2048     // Valor central do joystick (12 bits, 4096/2)
#define JOY_THRESHOLD 100   // Limiar para evitar ruído no joystick
#define SQUARE_SIZE 8       // Tamanho do quadrado no display (8x8 pixels)
#define JOY_MAX 4095        // Valor máximo do ADC (12 bits)
#define DEBOUNCE_TIME_US 400000 // Tempo de debounce para botões (400 ms)

// Variáveis globais
ssd1306_t ssd;                     // Estrutura para o display SSD1306
bool cor = true;                   // Cor do quadrado (true = branco, false = preto)
uint16_t red_intensity;            // Intensidade do LED vermelho
uint16_t blue_intensity;           // Intensidade do LED azul
bool led_g_state = false;          // Estado do LED verde (on/off)
bool pwm_enabled = true;           // Habilita PWM dos LEDs RGB
bool borda = false;                // Modo de borda no display (true = retângulo, false = linhas)
int16_t square_x = (WIDTH - SQUARE_SIZE) / 2; // Posição X inicial do quadrado (centro)
int16_t square_y = (HEIGHT - SQUARE_SIZE) / 2; // Posição Y inicial do quadrado (centro)
uint32_t ultimo_estado_pb = 0;     // Timestamp do último evento do Joystick PB
uint32_t ultimo_estado_a = 0;      // Timestamp do último evento do Botão A
uint32_t ultimo_estado_b = 0;      // Timestamp do último evento do Botão B
bool buzzer_active = false;        // Estado do buzzer (ligado/desligado)

// Função para mapear valores do joystick para coordenadas do display ou matriz
int32_t map_value(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Inicializa PWM para LEDs RGB (vermelho e azul)
void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // Configura pino como PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_WRAP); // Define resolução do PWM
    pwm_init(slice_num, &config, true);     // Inicializa PWM
}

// Define o ciclo de trabalho do PWM (intensidade do LED)
void set_pwm_duty(uint gpio, uint16_t duty) {
    pwm_set_gpio_level(gpio, duty);
}

// Inicializa a matriz de LEDs WS2812 usando PIO
void init_ws2812() {
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
}

// Envia cor para um pixel WS2812
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// Converte valores RGB para formato WS2812
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

// Atualiza a matriz de LEDs WS2812 com base na posição do quadrado
void update_led_matrix(int square_x, int square_y) {
    // Mapeia a posição do quadrado (0-120, 0-56) para a grade 5x5 da matriz
    int row = square_y * 5 / (HEIGHT - SQUARE_SIZE); // Y controla linhas
    row = 4 - row; // Inverte para alinhar norte (cima) com linha 4
    int col = square_x * 5 / (WIDTH - SQUARE_SIZE); // X controla colunas
    col = 4 - col; // Inverte para alinhar leste (direita) com coluna 0
    int pixel_index = row * 5 + col; // Calcula índice do LED (0 a 24)

    // Define cor roxa para o LED ativo
    uint32_t color = urgb_u32(120, 0, 120);

    // Atualiza a matriz: acende apenas o LED correspondente
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(i == pixel_index ? color : 0);
    }
}

// Inicializa o buzzer para saída PWM
void init_buzzer() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0); // Desligado inicialmente
}

// Controla o buzzer: liga (500 Hz) ou desliga
void control_buzzer(bool active) {
    if (active) {
        uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
        pwm_set_clkdiv(slice_num, 125.0f / (500 * PWM_WRAP / 1000.0f)); // Frequência de 500 Hz
        pwm_set_gpio_level(BUZZER_PIN, PWM_WRAP / 2); // 50% duty cycle
    } else {
        pwm_set_gpio_level(BUZZER_PIN, 0); // Desliga
    }
}

// Inicializa a interface UART para depuração
void init_uart() {
    uart_init(UART_ID, 115200); // Configura UART com 115200 baud
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Envia dados de depuração via UART (joystick, posição do quadrado, botão)
void log_status(uint16_t adc_x, uint16_t adc_y, int16_t square_x, int16_t square_y, uint gpio) {
    printf("Joystick X: %u, Y: %u | Square X: %d, Y: %d | Button: %u\n", 
           adc_x, adc_y, square_x, square_y, gpio);
}

// Inicializa botões adicionais (apenas Botão B) com pull-up e interrupções
void init_buttons() {
    gpio_init(BOTAO_B); gpio_set_dir(BOTAO_B, GPIO_IN); gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled(BOTAO_B, 0x4, true); // Substituído GPIO_IRQ_EDGE_FALL por 0x4
}

// Manipulador de interrupções para botões
void irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Joystick PB: Alterna borda no display e LED verde
    if (gpio == JOYSTICK_PB && current_time - ultimo_estado_pb > DEBOUNCE_TIME_US) {
        ultimo_estado_pb = current_time;
        borda = !borda;
        led_g_state = !led_g_state;
        gpio_put(LED_G, led_g_state);
        log_status(0, 0, square_x, square_y, gpio);
        while (!gpio_get(JOYSTICK_PB)); // Aguarda soltura
    }
    // Botão A: Liga o buzzer
    if (gpio == Botao_A && current_time - ultimo_estado_a > DEBOUNCE_TIME_US) {
        ultimo_estado_a = current_time;
        buzzer_active = true;
        control_buzzer(buzzer_active);
        log_status(0, 0, square_x, square_y, gpio);
        while (!gpio_get(Botao_A));
    }
    // Botão B: Desliga o buzzer
    if (gpio == BOTAO_B && current_time - ultimo_estado_b > DEBOUNCE_TIME_US) {
        ultimo_estado_b = current_time;
        buzzer_active = false;
        control_buzzer(buzzer_active);
        log_status(0, 0, square_x, square_y, gpio);
        while (!gpio_get(BOTAO_B));
    }
}

int main() {
    // Inicializa comunicação serial e periféricos
    stdio_init_all();
    init_uart();
    init_buzzer();
    init_ws2812();

    // Configura botões como entradas com pull-up
    gpio_init(JOYSTICK_PB); gpio_set_dir(JOYSTICK_PB, GPIO_IN); gpio_pull_up(JOYSTICK_PB);
    gpio_init(Botao_A); gpio_set_dir(Botao_A, GPIO_IN); gpio_pull_up(Botao_A);
    init_buttons(); // Inicializa Botão B

    // Configura interrupções para botões
    gpio_set_irq_enabled_with_callback(JOYSTICK_PB, 0x4, true, &irq_handler); // Substituído GPIO_IRQ_EDGE_FALL por 0x4
    gpio_set_irq_enabled(Botao_A, 0x4, true); // Substituído GPIO_IRQ_EDGE_FALL por 0x4

    // Inicializa LED verde como saída
    gpio_init(LED_G); gpio_set_dir(LED_G, GPIO_OUT);

    // Inicializa I2C e display SSD1306
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa ADC para leitura do joystick
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Inicializa PWM para LEDs RGB
    init_pwm(LED_R);
    init_pwm(LED_B);

    while (true) {
        // Lê valores do joystick (eixos X e Y)
        adc_select_input(0); // Eixo X
        uint16_t adc_value_x = adc_read();
        adc_select_input(1); // Eixo Y
        uint16_t adc_value_y = adc_read();

        // Mapeia valores do joystick para a posição do quadrado no display
        square_x = map_value(adc_value_y, 0, JOY_MAX, 0, WIDTH - SQUARE_SIZE); // Y controla horizontal
        square_y = map_value(adc_value_x, JOY_MAX, 0, 0, HEIGHT - SQUARE_SIZE); // X controla vertical

        // Atualiza a matriz de LEDs com a posição do quadrado
        update_led_matrix(square_x, square_y);

        // Controla a intensidade dos LEDs RGB com base no joystick
        if (pwm_enabled) {
            int16_t delta_x = adc_value_x - JOY_CENTER;
            int16_t delta_y = adc_value_y - JOY_CENTER;
            red_intensity = (abs(delta_x) > JOY_THRESHOLD) ? abs(delta_x) * PWM_WRAP / JOY_CENTER : 0;
            blue_intensity = (abs(delta_y) > JOY_THRESHOLD) ? abs(delta_y) * PWM_WRAP / JOY_CENTER : 0;
            set_pwm_duty(LED_R, red_intensity);
            set_pwm_duty(LED_B, blue_intensity);
        } else {
            set_pwm_duty(LED_R, 0);
            set_pwm_duty(LED_B, 0);
        }

        // Atualiza o display SSD1306
        ssd1306_fill(&ssd, false); // Limpa o display
        if (borda) {
            // Desenha retângulo completo como borda
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
        } else {
            // Desenha linhas parciais como borda
            ssd1306_line(&ssd, 3, 3, 122, 3, cor);
            ssd1306_line(&ssd, 3, 60, 122, 60, cor);
            ssd1306_line(&ssd, 3, 8, 3, 55, cor);
            ssd1306_line(&ssd, 122, 8, 122, 55, cor);
        }
        // Desenha o quadrado 8x8 na posição atual
        ssd1306_rect(&ssd, square_y, square_x, SQUARE_SIZE, SQUARE_SIZE, cor, !cor);
        ssd1306_send_data(&ssd); // Envia dados ao display

        // Envia logs periódicos via UART para depuração
        static uint32_t last_log_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_log_time > 500) {
            log_status(adc_value_x, adc_value_y, square_x, square_y, 0);
            last_log_time = current_time;
        }

        sleep_ms(20); // Atualiza a cada 20 ms para suavidade
    }
}