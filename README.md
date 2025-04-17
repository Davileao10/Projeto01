Jogo de Controle Pixel com BitDogLab

📖 Visão Geral
Este projeto é um jogo interativo desenvolvido para a placa BitDogLab (baseada no Raspberry Pi Pico). O jogo utiliza um joystick para mover um quadrado no display OLED SSD1306, controla uma matriz de LEDs WS2812 para refletir a posição do quadrado, e usa LEDs RGB e um buzzer para feedback interativo. O botão do joystick alterna a borda do display, e os botões A e B controlam o buzzer.

📂 Estrutura do Projeto

main.c: Código principal do jogo.
CMakeLists.txt: Arquivo de configuração do CMake para compilar o projeto.
pico_sdk_import.cmake: Arquivo auxiliar para importar o Pico SDK.
ws2812.pio: Programa PIO para controlar a matriz de LEDs WS2812.
lib/:
ssd1306.c, ssd1306.h: Biblioteca para controlar o display OLED SSD1306.
font.h: Tabela de fontes para exibir caracteres no display.



🛠️ Pré-requisitos
Antes de começar, certifique-se de ter as seguintes dependências instaladas:

Pico SDK: Versão 2.1.1 (instalado em C:\Users\<SEU_USUARIO>\.pico-sdk\sdk\2.1.1).
Ferramentas:
CMake (versão mínima 3.13).
Ninja (para compilação).
Compilador GCC (ex.: MinGW com gcc e g++ para Windows).


Hardware:
Placa BitDogLab (com Raspberry Pi Pico).
Display OLED SSD1306 (128x64).
Matriz de LEDs WS2812 (5x5, 25 LEDs).
Joystick, LEDs RGB, buzzer e botões conectados conforme os pinos definidos no ADC_DisplayC.c.



🚀 Como Compilar e Executar
Siga os passos abaixo para compilar e executar o projeto:

Clone o Repositório:
git clone https://github.com/<SEU_USUARIO>/<SEU_REPOSITORIO>.git
cd <SEU_REPOSITORIO>


Configure o Ambiente:

Verifique se o Pico SDK está instalado no caminho correto.
Certifique-se de que o CMake, Ninja e GCC estão instalados e no PATH do sistema.


Compile o Projeto:Abra um terminal na pasta do projeto e execute:
cmake -B build
cmake --build build

Isso gerará o arquivo Teste_ADC_Display.uf2 na pasta build.

Carregue na Placa:

Conecte a BitDogLab ao computador via USB.
Copie o arquivo Teste_ADC_Display.uf2 para a placa (ela aparecerá como uma unidade de armazenamento).


🎮 Como Usar

Joystick: Mova o joystick para controlar o quadrado no display OLED. A posição do quadrado é refletida na matriz de LEDs WS2812.
Botão do Joystick (PB): Alterna a borda no display (linhas ou retângulo) e o LED verde.
Botão A: Liga o buzzer (som de 500 Hz).
Botão B: Desliga o buzzer.
LEDs RGB:
LEDs vermelho e azul: Variam sua intensidade (via PWM) com base no movimento do joystick.
LED verde: Acende/desliga ao pressionar o botão do joystick.



