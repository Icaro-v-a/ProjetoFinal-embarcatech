#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <string.h>
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "includes/funcoesMatriz.h"
#include "includes/convertePixels.h"
#include "animacoes/contagem.h"

// Definições para comunicação I2C e hardware
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define OLED_ADDRESS 0x3C

// Definições de pinos para botões e LEDs
#define BTN_A 5      // Pino GPIO para o botão A
#define BTN_B 6      // Pino GPIO para o botão B
#define LED_GREEN 11 // Pino GPIO para o LED verde
#define LED_BLUE 12  // Pino GPIO para o LED azul
#define LED_RED 13   // Pino GPIO para o LED vermelho
#define BUZZER 21    // Pino GPIO para o buzzer

#define DEBOUNCE_TIME_MS 200 // Tempo de debounce em milissegundos
#define DEADZONE 210         // Zona morta para evitar brilho residual nos LEDs
#define VRX_CENTER 1878      // Valor central do eixo X do joystick
#define VRY_CENTER 2042      // Valor central do eixo Y do joystick
#define FILTER_SIZE 5        // Tamanho do filtro de média móvel

// Definições para simulação de sensores
#define TEMP_MIN 10  // Temperatura mínima (simulada)
#define TEMP_MAX 40  // Temperatura máxima (simulada)
#define HUMID_MIN 20 // Umidade mínima (simulada)
#define HUMID_MAX 80 // Umidade máxima (simulada)

// Definições para a matriz de LEDs
#define MATRIX_ROWS 5  // Número de linhas da matriz de LEDs
#define MATRIX_COLS 5  // Número de colunas da matriz de LEDs
#define MATRIX_DEPTH 3 // Profundidade da matriz de LEDs (RGB)

// Definição dos pinos GPIO utilizados para o controle do Joystick
const int VRX = 27;          // Pino GPIO para o eixo X do joystick
const int VRY = 26;          // Pino GPIO para o eixo Y do joystick
const int SW = 22;           // Pino GPIO para o botão do joystick
const int ADC_CHANNEL_0 = 0; // Canal ADC para o eixo Y do joystick
const int ADC_CHANNEL_1 = 1; // Canal ADC para o eixo X do joystick

// Variáveis globais
absolute_time_t last_time_a = 0;  // Último tempo de pressionamento do botão A
absolute_time_t last_time_b = 0;  // Último tempo de pressionamento do botão B
absolute_time_t last_time_sw = 0; // Último tempo de pressionamento do botão SW
ssd1306_t ssd;                    // Estrutura para controlar o display OLED
bool display_mode = false;        // Modo de exibição (dados/gráficos)
float temperature = 25.0;         // Temperatura simulada
float humidity = 50.0;            // Umidade simulada

// Buffers para filtro de média móvel
uint16_t vrx_buffer[FILTER_SIZE] = {0}; // Buffer para leituras do eixo X do joystick
uint16_t vry_buffer[FILTER_SIZE] = {0}; // Buffer para leituras do eixo Y do joystick
uint8_t buffer_index = 0;               // Índice atual do buffer para o filtro de média móvel

// Configuração do PWM
const uint16_t PERIOD = 4096;            // Período do PWM
const float DIVIDER_PWM = 16.0;          // Divisor de clock do PWM
uint16_t led_b_level, led_r_level = 100; // Níveis de brilho inicial para os LEDs azul e vermelho
uint slice_led_b, slice_led_r;           // Slice do PWM para os LEDs azul e vermelho

// Função de interrupção para lidar com o pressionamento dos botões
void gpio_irq_handler(uint gpio, uint32_t events)
{
    absolute_time_t current_time = get_absolute_time();

    if (gpio == BTN_A && absolute_time_diff_us(last_time_a, current_time) > DEBOUNCE_TIME_MS * 1000)
    {
        last_time_a = current_time;
    }

    if (gpio == BTN_B && absolute_time_diff_us(last_time_b, current_time) > DEBOUNCE_TIME_MS * 1000)
    {
        last_time_b = current_time;
    }

    // Botão SW: Alterna entre exibição de dados e gráficos
    if (gpio == SW && absolute_time_diff_us(last_time_sw, current_time) > DEBOUNCE_TIME_MS * 1000)
    {
        display_mode = !display_mode;
        last_time_sw = current_time;
    }
}

// Função para configurar o PWM no pino do LED
void setup_pwm(uint led, uint *slice, uint16_t level)
{
    gpio_set_function(led, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(led);
    pwm_set_clkdiv(*slice, DIVIDER_PWM);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_gpio_level(led, level);
    pwm_set_enabled(*slice, true);
}

// Função para configurar o joystick
void setup_joystick()
{
    adc_init();
    adc_gpio_init(VRX);
    adc_gpio_init(VRY);

    gpio_init(SW);
    gpio_set_dir(SW, GPIO_IN);
    gpio_pull_up(SW);
    gpio_set_irq_enabled_with_callback(SW, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
}

// Função para configurar o display OLED
void setup_display()
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, OLED_ADDRESS, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

// Função para configurar os LEDs e os botões A e B
void setup_leds_and_buttons()
{
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_put(LED_GREEN, 0);

    gpio_init(LED_BLUE);
    gpio_set_dir(LED_BLUE, GPIO_OUT);
    gpio_put(LED_BLUE, 0);

    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_put(LED_RED, 0);

    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT);

    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(BTN_B);
    gpio_set_dir(BTN_B, GPIO_IN);
    gpio_pull_up(BTN_B);
    gpio_set_irq_enabled_with_callback(BTN_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
}

// Função para ler os valores dos eixos X e Y do joystick
void joystick_read_axis(uint16_t *vrx_value, uint16_t *vry_value)
{
    adc_select_input(ADC_CHANNEL_1);
    sleep_us(2);
    *vrx_value = adc_read();

    adc_select_input(ADC_CHANNEL_0);
    sleep_us(2);
    *vry_value = adc_read();
}

// Função para aplicar um filtro de média móvel nas leituras do joystick
uint16_t get_filtered_value(uint16_t *buffer, uint16_t new_value)
{
    buffer[buffer_index] = new_value;
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
    {
        sum += buffer[i];
    }
    return sum / FILTER_SIZE;
}

void mudarValor(float temperature, float humidity, npLED_t leds[], int rgb_matrix[MATRIX_ROWS][MATRIX_COLS][MATRIX_DEPTH])
{
    uint32_t *temp_symbol, *hum_symbol;

    if (temperature < 16)
    {
        temp_symbol = temperatura[0];
    }
    else if (temperature <= 22)
    {
        temp_symbol = temperatura[1];
    }
    else if (temperature <= 28)
    {
        temp_symbol = temperatura[2];
    }
    else if (temperature <= 34)
    {
        temp_symbol = temperatura[3];
    }
    else
    {
        temp_symbol = temperatura[4];
    }

    if (humidity < 32)
    {
        hum_symbol = umidade[0];
    }
    else if (humidity <= 44)
    {
        hum_symbol = umidade[1];
    }
    else if (humidity <= 56)
    {
        hum_symbol = umidade[2];
    }
    else if (humidity <= 68)
    {
        hum_symbol = umidade[3];
    }
    else
    {
        hum_symbol = umidade[4];
    }

    // Limpa a matriz antes de desenhar os novos ícones
    memset(rgb_matrix, 0, sizeof(int) * MATRIX_ROWS * MATRIX_COLS * MATRIX_DEPTH);

    // **Desenha a temperatura nas duas primeiras linhas**
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            int idx = i * 5 + j;
            rgb_matrix[i][j][0] = (temp_symbol[idx] >> 16) & 0xFF;
            rgb_matrix[i][j][1] = (temp_symbol[idx] >> 8) & 0xFF;
            rgb_matrix[i][j][2] = temp_symbol[idx] & 0xFF;
        }
    }

    // **Desenha a umidade nas duas últimas linhas**
    for (int i = 3; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            int idx = (i - 3) * 5 + j;
            rgb_matrix[i][j][0] = (hum_symbol[idx] >> 16) & 0xFF;
            rgb_matrix[i][j][1] = (hum_symbol[idx] >> 8) & 0xFF;
            rgb_matrix[i][j][2] = hum_symbol[idx] & 0xFF;
        }
    }

    // Atualiza a matriz de LEDs
    spriteWirite(rgb_matrix, leds);
    matrizWrite(leds);
}

void buzzer_active(uint32_t buzzer_frequency, uint32_t duration_buzzer_on)
{
    uint32_t half_period_us = (1000000 / buzzer_frequency) / 2; // Define por quanto tempo o pino conectado ao buzzer deve ficar em nível alto/baixo

    // gera uma onda quadrada
    for (uint32_t i = 0; i < duration_buzzer_on * 1000; i += half_period_us * 2)
    {
        gpio_put(BUZZER, 1);
        sleep_us(half_period_us);
        gpio_put(BUZZER, 0);
        sleep_us(half_period_us);
    }
}

// Função principal
int main()
{
    stdio_init_all();
    setup_display();
    setup_leds_and_buttons();
    setup_joystick();
    setup_pwm(LED_BLUE, &slice_led_b, led_b_level);
    setup_pwm(LED_RED, &slice_led_r, led_r_level);
    // Inicializa a matriz de LEDs
    npLED_t leds[LED_COUNT];
    int rgb_matrix[MATRIX_ROWS][MATRIX_COLS][MATRIX_DEPTH];
    matrizInit(LED_PIN, leds); // Inicializa os LEDs

    uint16_t vrx_value, vry_value;

    while (1)
    {
        joystick_read_axis(&vrx_value, &vry_value);

        // Aplica o filtro de média móvel para suavizar as leituras
        vrx_value = get_filtered_value(vrx_buffer, vrx_value);
        vry_value = get_filtered_value(vry_buffer, vry_value);
        buffer_index = (buffer_index + 1) % FILTER_SIZE; // Atualiza o índice do buffer

        // Simulação de sensores
        temperature = TEMP_MIN + (vrx_value / 4095.0) * (TEMP_MAX - TEMP_MIN);
        humidity = HUMID_MIN + (vry_value / 4095.0) * (HUMID_MAX - HUMID_MIN);

        // Aquecedor (LED Vermelho)
        if (temperature < 20)
            pwm_set_gpio_level(LED_RED, PERIOD);
        else
            pwm_set_gpio_level(LED_RED, 0);

        // Umidificador (LED Azul)
        if (humidity < 40)
            pwm_set_gpio_level(LED_BLUE, PERIOD);
        else
            pwm_set_gpio_level(LED_BLUE, 0);

        // Buzzer (alerta)
        if (temperature > 35 || humidity > 70)
            buzzer_active(900, 100);
        else
            buzzer_active(900, 0);

        // Exibição no display
        ssd1306_fill(&ssd, false);
        if (display_mode)
        {
            // Atualiza a matriz de LEDs com base nos valores dos sensores
            mudarValor(temperature, humidity, leds, rgb_matrix);
        }
        else
        {
            turnOffLEDs(leds);
            // Exibe dados
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "Temp: %.1fC", temperature);
            ssd1306_draw_string(&ssd, buffer, 0, 0);

            snprintf(buffer, sizeof(buffer), "Umidade: %.1f%%", humidity);
            ssd1306_draw_string(&ssd, buffer, 0, 10);
        }
        ssd1306_send_data(&ssd);

        sleep_ms(100);
    }
}