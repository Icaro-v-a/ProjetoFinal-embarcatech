# Atividade: Conversores A/D

Objetivos
- Compreender o funcionamento do conversor analógico-digital (ADC) no RP2040.
- Utilizar o PWM para controlar a intensidade de dois LEDs RGB com base nos valores do joystick.
- Representar a posição do joystick no display SSD1306 por meio de um quadrado móvel.
- Aplicar o protocolo de comunicação I2C na integração com o display.

## Descrição do Projeto

O joystick fornecerá valores analógicos correspondentes aos eixos X e Y, que serão utilizados para:
Controlar a intensidade luminosa dos LEDs RGB, onde:

- O LED Azul terá seu brilho ajustado conforme o valor do eixo Y. Quando o joystick estiver solto (posição central - valor 2048), o LED permanecerá apagado. À medida que o joystick for movido para cima (valores menores) ou para baixo (valores maiores), o LED aumentará seu brilho gradualmente, atingindo a intensidade máxima nos extremos (0 e 4095).
- O LED Vermelho seguirá o mesmo princípio, mas de acordo com o eixo X. Quando o joystick estiver solto (posição central - valor 2048), o LED estará apagado. Movendo o joystick para a esquerda (valores menores) ou para a direita (valores maiores), o LED aumentará de brilho, sendo mais intenso nos extremos (0 e 4095).
- Os LEDs serão controlados via PWM para permitir variação suave da intensidade luminosa.
  
Exibir no display SSD1306 um quadrado de 8x8 pixels, inicialmente centralizado, que se moverá proporcionalmente aos valores capturados pelo joystick.

Adicionalmente, o botão do joystick terá as seguintes funcionalidades:

- Alternar o estado do LED Verde a cada acionamento.
- Modificar a borda do display para indicar quando foi pressionado, alternando entre diferentes estilos de borda a cada novo acionamento.
  
Finalmente, o botão A terá a seguinte funcionalidade:

- Ativar ou desativar os LED PWM a cada acionamento.

Neste projeto, deverá ser utilizados os seguintes componentes já interconectados à placa BitDogLab:
- LED RGB, com os pinos conectados às GPIOs (11, 12 e 13).
- Botão do Joystick conectado à GPIO 22.
- Joystick conectado aos GPIOs 26 e 27.
- Botão A conectado à GPIO 5.
- Display SSD1306 conectado via I2C (GPIO 14 e GPIO15)

### Requisitos do Projeto

Para o desenvolvimento, devem ser seguidos os seguintes requisitos:
1. Uso de interrupções: Todas as funcionalidades relacionadas aos botões devem ser implementadas utilizando rotinas de interrupção (IRQ).
2. Debouncing: É obrigatório implementar o tratamento do bouncing dos botões via software.
3. Utilização do Display 128 x 64: A utilização de ferramentas gráficas demonstrará o entendimento do princípio de funcionamento do display, bem como, a utilização do protocolo I2C.
4. Organização do código: O código deve estar bem estruturado e comentado para facilitar o entendimento.
   
## Tecnologias Utilizadas
- **Microcontrolador:** Raspberry Pi Pico W
- **Ambiente de Desenvolvimento:** VS Code
- **Linguagem:** C (com Pico SDK)

## Autor: Ícaro Vasconcelos Alvim

## Como Executar
1. Clone o repositório:
   ```bash
   git clone https://github.com/Icaro-v-a/U4C8-embarcatech.git

2. Navegue até o diretório do projeto:
    ```bash
        cd U4C8-embarcatech

3. Compile e execute o projeto utilizando o Pico SDK no VS Code.

## Link do Vídeo
O vídeo do experimento com o emprego da Ferramenta Educacional BitDogLab pode ser acessado [aqui](https://www.youtube.com/shorts/16MM5i9jo0k).
