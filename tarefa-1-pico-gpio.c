#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <string.h>
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/adc.h"

// Definições para comunicação I2C e hardware
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Definições de pinos para botões e LEDs
#define BTN_A 5      // Pino GPIO para o botão A
#define BTN_B 6      // Pino GPIO para o botão B
#define LED_GREEN 11 // Pino GPIO para o LED verde
#define LED_BLUE 12  // Pino GPIO para o LED azul
#define LED_RED 13   // Pino GPIO para o LED vermelho

#define DEBOUNCE_TIME_MS 200 // Tempo de debounce em milissegundos (evita múltiplos registros de pressionamento do botão)
#define DEADZONE 210         // Define uma zona morta para evitar brilho residual nos LEDs
#define VRX_CENTER 1878      // Valor central do eixo X do joystick (ajuste dinâmico)
#define VRY_CENTER 2042      // Valor central do eixo Y do joystick (ajuste dinâmico)
#define FILTER_SIZE 5        // Tamanho do filtro de média móvel para suavizar as leituras do joystick

uint16_t vrx_buffer[FILTER_SIZE] = {0}; // Buffer para armazenar leituras do eixo X do joystick
uint16_t vry_buffer[FILTER_SIZE] = {0}; // Buffer para armazenar leituras do eixo Y do joystick
uint8_t buffer_index = 0;               // Índice atual do buffer para o filtro de média móvel

// Definição dos pinos GPIO utilizados para o controle do Joystick
const int VRX = 27; // Pino GPIO para o eixo X do joystick
const int VRY = 26; // Pino GPIO para o eixo Y do joystick
const int SW = 22;  // Pino GPIO para o botão do joystick
const int ADC_CHANNEL_0 = 0; // Canal ADC para o eixo Y do joystick
const int ADC_CHANNEL_1 = 1; // Canal ADC para o eixo X do joystick

// Variáveis globais
absolute_time_t last_time_a = 0;  // Armazena o último tempo de pressionamento do botão A
absolute_time_t last_time_b = 0;  // Armazena o último tempo de pressionamento do botão B
absolute_time_t last_time_sw = 0; // Armazena o último tempo de pressionamento do botão SW
ssd1306_t ssd;                    // Estrutura para controlar o display OLED
bool wrap_around = false;         // Controla se o quadrado atravessa as bordas do display (true) ou não (false)
bool pwm_enabled = true;          // Controla se os LEDs PWM estão habilitados (true) ou não (false)
bool border_style = false;        // Controla o estilo da borda do display (true = borda dupla, false = borda simples)

// Configuração do PWM
const uint16_t PERIOD = 4096;   // Período do PWM
const float DIVIDER_PWM = 16.0; // Divisor de clock do PWM para ajustar a frequência
uint16_t led_b_level, led_r_level = 100; // Níveis de brilho inicial para os LEDs azul e vermelho
uint slice_led_b, slice_led_r;           // Slice do PWM para os LEDs azul e vermelho

// Função de interrupção para lidar com o pressionamento dos botões
void gpio_irq_handler(uint gpio, uint32_t events)
{
    absolute_time_t current_time = get_absolute_time(); // Obtém o tempo atual

    // Verifica se o botão A foi pressionado e se o debounce foi respeitado
    if (gpio == BTN_A && absolute_time_diff_us(last_time_a, current_time) > DEBOUNCE_TIME_MS * 1000)
    {
        pwm_enabled = !pwm_enabled; // Alterna o estado dos LEDs PWM
        if (pwm_enabled)
        {
            pwm_set_enabled(slice_led_b, true); // Liga o PWM do LED azul
            pwm_set_enabled(slice_led_r, true); // Liga o PWM do LED vermelho
            printf("LEDs PWM ligados!\n");
        }
        else
        {
            pwm_set_enabled(slice_led_b, false); // Desliga o PWM do LED azul
            pwm_set_enabled(slice_led_r, false); // Desliga o PWM do LED vermelho
            printf("LEDs PWM desligados!\n");
        }
        last_time_a = current_time; // Atualiza o último tempo de pressionamento do botão A
    }

    // Verifica se o botão do joystick foi pressionado e se o debounce foi respeitado
    if (gpio == SW && absolute_time_diff_us(last_time_sw, current_time) > DEBOUNCE_TIME_MS * 1000)
    {
        // Alterna o estado do LED Verde
        gpio_put(LED_GREEN, !gpio_get(LED_GREEN));

        // Alterna o estilo da borda do display
        border_style = !border_style;

        printf("Botão do joystick pressionado! Borda alternada.\n");
        last_time_sw = current_time; // Atualiza o último tempo de pressionamento do botão SW
    }

    // Verifica se o botão B foi pressionado e se o debounce foi respeitado
    if (gpio == BTN_B && absolute_time_diff_us(last_time_b, current_time) > DEBOUNCE_TIME_MS * 1000)
    {
        wrap_around = !wrap_around; // Alterna o comportamento do quadrado (atravessar bordas ou não)
        printf("Botão B pressionado! Comportamento do quadrado alternado: %s\n", wrap_around ? "Atravessar bordas" : "Limitar às bordas");

        last_time_b = current_time; // Atualiza o último tempo de pressionamento do botão B
    }
}

// Função para configurar o PWM no pino do LED
void setup_pwm(uint led, uint *slice, uint16_t level)
{
    gpio_set_function(led, GPIO_FUNC_PWM); // Configura o pino como saída PWM
    *slice = pwm_gpio_to_slice_num(led);   // Obtém o slice do PWM associado ao pino
    pwm_set_clkdiv(*slice, DIVIDER_PWM);   // Define o divisor de clock para ajustar a frequência
    pwm_set_wrap(*slice, PERIOD);          // Define o valor máximo do contador do PWM
    pwm_set_gpio_level(led, level);        // Define o nível inicial do PWM
    pwm_set_enabled(*slice, true);         // Habilita o PWM
}

// Função para configurar o joystick
void setup_joystick()
{
    adc_init();
    adc_gpio_init(VRX); // Configura o pino do eixo X do joystick como entrada analógica
    adc_gpio_init(VRY); // Configura o pino do eixo Y do joystick como entrada analógica

    // Configuração do botão SW do joystick
    gpio_init(SW);                                                                       // Inicializa o pino do botão SW
    gpio_set_dir(SW, GPIO_IN);                                                           // Configura o pino como entrada
    gpio_pull_up(SW);                                                                    // Ativa o resistor pull-up para o botão
    gpio_set_irq_enabled_with_callback(SW, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler); // Configura a interrupção para o botão SW
}

// Função para configurar o display OLED
void setup_display()
{
    // Inicialização do barramento I2C
    i2c_init(I2C_PORT, 400 * 1000);            // Inicializa o I2C com frequência de 400 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura o pino SDA como função I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura o pino SCL como função I2C
    gpio_pull_up(I2C_SDA);                     // Ativa o resistor pull-up no pino SDA
    gpio_pull_up(I2C_SCL);                     // Ativa o resistor pull-up no pino SCL

    // Inicialização do display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display OLED
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    ssd1306_fill(&ssd, false);                                    // Limpa o display (todos os pixels apagados)
    ssd1306_send_data(&ssd);                                      // Atualiza o display
}

// Função para configurar os LEDs e os botões A e B
void setup_leds_and_buttons()
{
    // Configuração dos LEDs
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_put(LED_GREEN, 0);

    gpio_init(LED_BLUE);
    gpio_set_dir(LED_BLUE, GPIO_OUT);
    gpio_put(LED_BLUE, 0);

    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_put(LED_RED, 0);

    // Configuração do botão A
    gpio_init(BTN_A);                                                                       // Inicializa o pino do botão A
    gpio_set_dir(BTN_A, GPIO_IN);                                                           // Configura o pino como entrada
    gpio_pull_up(BTN_A);                                                                    // Ativa o resistor pull-up para o botão
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler); // Configura a interrupção para o botão A

    // Configuração do botão B
    gpio_init(BTN_B);                                                                       // Inicializa o pino do botão B
    gpio_set_dir(BTN_B, GPIO_IN);                                                           // Configura o pino como entrada
    gpio_pull_up(BTN_B);                                                                    // Ativa o resistor pull-up para o botão
    gpio_set_irq_enabled_with_callback(BTN_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler); // Configura a interrupção para o botão B
}

// Função para ler os valores dos eixos X e Y do joystick
void joystick_read_axis(uint16_t *vrx_value, uint16_t *vry_value)
{
    adc_select_input(ADC_CHANNEL_1); // Seleciona o canal ADC para o eixo X
    sleep_us(2);
    *vrx_value = adc_read(); // Lê o valor do eixo X

    adc_select_input(ADC_CHANNEL_0); // Seleciona o canal ADC para o eixo Y
    sleep_us(2);
    *vry_value = adc_read(); // Lê o valor do eixo Y
}

// Função para aplicar um filtro de média móvel nas leituras do joystick
uint16_t get_filtered_value(uint16_t *buffer, uint16_t new_value)
{
    buffer[buffer_index] = new_value; // Armazena o novo valor no buffer
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
    {
        sum += buffer[i]; // Soma os valores no buffer
    }
    return sum / FILTER_SIZE; // Retorna a média dos valores
}

// Função principal
int main()
{
    stdio_init_all(); // Inicializa a comunicação serial
    setup_display();  // Configura o display OLED
    setup_leds_and_buttons(); // Configura os LEDs e os botões A e B
    setup_joystick(); // Configura o joystick
    setup_pwm(LED_BLUE, &slice_led_b, led_b_level); // Configura o PWM para o LED azul
    setup_pwm(LED_RED, &slice_led_r, led_r_level);  // Configura o PWM para o LED vermelho
    uint16_t vrx_value, vry_value;

    // Variáveis para controlar a posição do quadrado no display
    int square_y = 14; // Posição inicial do quadrado no eixo Y
    int square_x = 70; // Posição inicial do quadrado no eixo X

    while (1)
    {
        joystick_read_axis(&vrx_value, &vry_value); // Lê os valores dos eixos do joystick

        // Aplica o filtro de média móvel para suavizar as leituras
        vrx_value = get_filtered_value(vrx_buffer, vrx_value);
        vry_value = get_filtered_value(vry_buffer, vry_value);
        buffer_index = (buffer_index + 1) % FILTER_SIZE; // Atualiza o índice do buffer

        // Calcula a variação em relação ao centro do joystick
        int16_t delta_x = vrx_value - VRX_CENTER;
        int16_t delta_y = vry_value - VRY_CENTER;

        // Controle do LED Azul com base no movimento do eixo Y
        if (abs(delta_y) < DEADZONE)
        {
            pwm_set_gpio_level(LED_BLUE, 0); // Desliga o LED Azul se dentro da zona morta
        }
        else
        {
            // Mapeia o movimento do eixo Y para o brilho do LED Azul
            uint16_t brightness_blue = (abs(delta_y) * PERIOD) / 4095;
            pwm_set_gpio_level(LED_BLUE, brightness_blue);
        }

        // Controle do LED Vermelho com base no movimento do eixo X
        if (abs(delta_x) < DEADZONE)
        {
            pwm_set_gpio_level(LED_RED, 0); // Desliga o LED Vermelho se dentro da zona morta
        }
        else
        {
            // Mapeia o movimento do eixo X para o brilho do LED Vermelho
            uint16_t brightness_red = (abs(delta_x) * PERIOD) / 4095;
            pwm_set_gpio_level(LED_RED, brightness_red);
        }

        // Movimentação do quadrado no display com base nos valores do joystick
        square_y -= delta_y / 256; // Ajusta a posição Y do quadrado
        square_x += delta_x / 256; // Ajusta a posição X do quadrado

        // Verifica se o quadrado deve atravessar as bordas ou ser limitado por elas
        if (wrap_around)
        {
            // Faz o quadrado "atravessar" as bordas do display
            if (square_y < 0)
                square_y = 56; // Aparece na parte inferior
            if (square_y > 56)
                square_y = 0; // Aparece do lado esquerdo
            if (square_x < 0)
                square_x = 120; // Aparece do lado direito
            if (square_x > 120)
                square_x = 0; // Aparece na parte superior
        }
        else
        {
            // Limita a posição do quadrado com base na borda desenhada
            if (border_style)
            {
                // Borda dupla: limites internos (3, 3) a (116, 52)
                if (square_y < 3)
                    square_y = 3;
                if (square_y > 52)
                    square_y = 52;
                if (square_x < 3)
                    square_x = 3;
                if (square_x > 116)
                    square_x = 116;
            }
            else
            {
                // Borda simples: limites externos (1, 1) a (118, 54)
                if (square_y < 1)
                    square_y = 1;
                if (square_y > 54)
                    square_y = 54;
                if (square_x < 1)
                    square_x = 1;
                if (square_x > 118)
                    square_x = 118;
            }
        }

        // Limpa o display
        ssd1306_fill(&ssd, false);

        // Desenha a borda (se ativada)
        if (border_style)
        {
            // Borda dupla
            ssd1306_rect(&ssd, 0, 0, 127, 63, true, false); // Borda externa
            ssd1306_rect(&ssd, 2, 2, 123, 59, true, false); // Borda interna
        }
        else
        {
            // Borda simples
            ssd1306_rect(&ssd, 0, 0, 127, 63, true, false); // Borda única
        }

        // Desenha o quadrado na posição atual
        ssd1306_rect(&ssd, square_y, square_x, 8, 8, true, false);

        // Atualiza o display com as novas informações
        ssd1306_send_data(&ssd);

        sleep_ms(50); // Espera antes da próxima leitura
    }
}