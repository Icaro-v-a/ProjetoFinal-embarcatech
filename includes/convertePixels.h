// Converte um código hexadecimal para decimal em no padrão RGB.
// O código gerado será utilizado para exibir imagens na matriz de leds neopixel.
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "ws2818b.pio.h"

#define MATRIX_ROWS 5
#define MATRIX_COLS 5
#define MATRIX_DEPTH 3


void convertToRGB(int argb, int rgb[3]) {
    rgb[0] = argb & 0xFF;          // Blue
    rgb[2] = (argb >> 16) & 0xFF;  // Red
    rgb[1] = (argb >> 8) & 0xFF;   // Green
}

void convertARGBtoMatriz(uint32_t frame[], int matriz[5][5][3]) {

    for (int i = 0; i < MATRIX_ROWS * MATRIX_COLS; i++) {
        int rgb[3];
        convertToRGB(frame[i], rgb);
        
        int row = i / MATRIX_COLS;    // Cálculo da linha
        int col = i % MATRIX_COLS;    // Cálculo da coluna
        
        // Armazenar os valores RGB na matriz 5x5x3
        matriz[row][col][0] = rgb[0];  // Red
        matriz[row][col][1] = rgb[1];  // Green
        matriz[row][col][2] = rgb[2];  // Blue
    }

}

