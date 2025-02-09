//bibliotecas inclusas
#include "pico/cyw43_arch.h"                                    // Inclui a biblioteca para o módulo CYW4343
#include "pico/cyw43_driver.h"                                  // Inclui a biblioteca para funções do driver CYW43
#include "pico/stdlib.h"                                        // Inclui a biblioteca padrão do Pico
#include "lwip/tcp.h"                                           // Inclui a biblioteca para comunicação TCP/IP
#include <string.h>                                             // Inclui a biblioteca para manipulação de strings
#include <stdio.h>                                              // Inclui a biblioteca padrão de entrada e saída
#include "hardware/adc.h"                                       // Inclui a biblioteca para o ADC
#include "hardware/pio.h"                                       // Inclui a biblioteca para a máquina PIO
#include "hardware/clocks.h"                                    // Inclui a biblioteca para configuração de clocks
#include "ws2818b.pio.h"                                        // Inclui o programa para controle de LEDs WS2818B
#include <stdlib.h>                                             // Inclui a biblioteca padrão
#include <ctype.h>                                              // Inclui a biblioteca para manipulação de caracteres
#include "pico/binary_info.h"                                   // Inclui a biblioteca para informações binárias
#include "inc/ssd1306.h"                                        // Inclui a biblioteca para o display OLED SSD1306
#include "hardware/i2c.h"                                       // Inclui a biblioteca para o protocolo I2C
#include "stdbool.h"                                            // Inclui a biblioteca para variáveis booleanas
#include <time.h>                                               // Inclui a biblioteca para manipulação de tempo
#include "lwip/apps/sntp.h"                                     // Inclui a biblioteca para o cliente SNTP
#include "hardware/pwm.h"                                       // Inclui a biblioteca para o controle de PWM
#include "lwip/dns.h"                                           // Inclui a biblioteca para resolução de DNS
#include "lwip/pbuf.h"                                          // Inclui a biblioteca para buffers de pacotes
#include "lwip/udp.h"                                           // Inclui a biblioteca para comunicação UDP


// Definições de Wi-Fi
#define WIFI_SSID "INSIRA REDE WIFI AQUI"                         // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASS "INSIRA SENHA"                                  // Substitua pela senha da sua rede Wi-Fi

// Definição do número de LEDs e pino.
#define LED_COUNT 25
#define LED_PIN 7
#define LED_RED 13
#define LED_GREEN 11
#define LED_BLUE 12
//definições para o servidor ntp
#define NTP_SERVER "pool.ntp.org"                               // Servidor NTP
#define NTP_PORT 123                                            // Porta padrão do NTP
#define NTP_MSG_LEN 48                                          // Tamanho da mensagem NTP
#define NTP_DELTA 2208988800UL                                  // 1970-01-01 00:00:00

// Definições dos pinos para o joystick e botão
#define VRX_PIN 26                                              // Define o pino GP26 para o eixo X do joystick (Canal ADC0).
#define VRY_PIN 27                                              // Define o pino GP27 para o eixo Y do joystick (Canal ADC1).
#define SW_PIN 22                                               // Define o pino GP22 para o botão do joystick (entrada digital).
#define BUTTON_A_PIN 5                                          // Define o pino GP5 para o botão A (entrada digital).
#define BUTTON_B_PIN 6                                          // Define o pino GP6 para o botão B (entrada digital).
// Valores de referência para o estado neutro do joystick
#define DEADZONE 200                                            // Define uma zona morta para evitar oscilações pequenas
#define CENTER_X 2048                                           // Centro teórico do ADC (meio de 0-4095)
#define CENTER_Y 2048                                           // Centro teórico do ADC (meio de 0-4095)    
#define THRESHOLD 1000                                          // Limiar para detectar um movimento significativo
#define UPDATE_INTERVAL 200000                                  // Intervalo em microssegundos (200ms)
#define AUTHORIZED_CLIENT "ESP8266-NODEMCU"                     // Nome do client esperado

//conexao i2c para a tela
#define I2C_SDA 14 
#define I2C_SCL 15
// Configuração do pino do buzzer
#define BUZZER_PIN 21

// Variáveis globais
time_t start_time; // Variável para armazenar o tempo inicial
uint32_t last_update_time_clock; // Variável para armazenar o tempo da última atualização do relógio

void led_buffer();
void npClear();
void npWrite();
int change_led_state(int led_addr_c, int led_addr_l, int state);
void npInit(uint pin);
void generate_status_string(char *buffer);
void process_request(char *req, struct tcp_pcb *tpcb);
void display( char *texto, int linha);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~funções para acionar o irrigamento~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int matriz_acionar_irrig[5][5] = {0};                   //matriz para acionar o irrigamento
void generate_irrig_string(char *output) {              //Gera uma string com a matriz de acionamento do irrigamento
    char buffer[128] = "";
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            char temp[4];
            snprintf(temp, sizeof(temp), "%d", matriz_acionar_irrig[i][j]);
            strcat(buffer, temp);
            if (j < 4) strcat(buffer, "|"); // Coloca "|" entre os valores
        }
        if (i < 4) strcat(buffer, "/"); // Adiciona "/" entre as linhas
    }
    strcpy(output, buffer);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~funções para controlar o buzzer~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
// Definição de uma função para inicializar o PWM no pino do buzzer
void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta divisor de clock
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0); // Desliga o PWM inicialmente
}
// Definição de uma função para emitir um beep com duração especificada
void play_tone(uint pin, uint frequency, uint duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t top = clock_freq / frequency - 1;

    pwm_set_wrap(slice_num, top);
    pwm_set_gpio_level(pin, top / 2); // 50% de duty cycle

    sleep_ms(duration_ms);

    pwm_set_gpio_level(pin, 0); // Desliga o som após a duração
    sleep_ms(50); // Pausa entre notas
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~funções para controlar a matriz de leds~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool led_flag = true;                                     //flag para ligar/desligar a matriz dos leds
bool select_flag = false;                                //flag para selecionar o led
bool blink = 1;                                          //controlar estado do seletor
int r,g,b = 0;                                           //variaveis para controlar a cor do led
int matriz_led[5][5][3] ={0};                            //matriz para organizar a cor dos leds
int matriz_state[5][5]={0};                              //matriz para guardar o estado de cada ponto
struct pixel_t {                                         // Definição de pixel GRB
  uint8_t G, R, B;                                       // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;                           // Definição de pixel_t como um tipo de dado.
typedef pixel_t npLED_t;                                  // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.
npLED_t leds[LED_COUNT];                                  // Declaração do buffer de pixels que formam a matriz_led.
PIO np_pio;                                               // Variáveis para uso da máquina PIO.
uint sm;                                                  // Variáveis para uso da máquina PIO.
void npInit(uint pin) {                                  // Inicializa a máquina PIO para controle da matriz_led de LEDs.

  // Cria programa PIO.
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  // Toma posse de uma máquina PIO.
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
  }

  // Inicia programa na máquina PIO obtida.
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Limpa buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {// Atribui uma cor RGB a um LED.
  leds[index].R = r;
  leds[index].G = g;
  leds[index].B = b;
}
void npClear() {                                          //Limpa o buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
    memset(matriz_led, 0, sizeof(matriz_led));
}
void npWrite() {                                          // Escreve os dados do buffer nos LEDs.
  // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}
int getIndex(int x, int y) {                              //recebe as cordenadas e converte p/ o index da matriz linear
    int newY = y; // Mantém a ordem das linhas natural
    if (newY % 2 == 0) {
        return newY * 5 + (4 - x); // Linha par (da direita para a esquerda)
    } else {
        return newY * 5 + x; // Linha ímpar (da esquerda para a direita)
    }
}
void getXY(int index, int *x, int *y) {                   //recebe o index e retorna as cordenadas
    index = 24 - index; // Reverter o mapeamento

    *y = index / 5; // Calcula a linha
    int pos = index % 5; // Calcula a posição na linha

    // Se a linha for par, a ordem é esquerda → direita
    // Se a linha for ímpar, a ordem é direita → esquerda
    if (*y % 2 == 0) {
        *x = pos;
    } else {
        *x = 4 - pos;
    }
}
void led_buffer() {                                       //atualiza o buffer de leds
    for (int linha = 0; linha < 5; linha++) {
        for (int coluna = 0; coluna < 5; coluna++) {
            int posicao = getIndex(coluna, linha);
           
            
            npSetLED(posicao, matriz_led[coluna][linha][0], matriz_led[coluna][linha][1], matriz_led[coluna][linha][2]);
        }
    }
}
int change_led_color(int led_addr_x, int led_addr_y, int cor, int bright){//muda a cor do led conforme a cor e o brilho
    matriz_led[led_addr_x][led_addr_y][cor]=bright;
    led_buffer();
    npWrite();

}
int change_led_state(int led_addr_x, int led_addr_y, int state) {          //muda o cor do led conforme o estado
    //printf("Alterando estado do LED [%d][%d] para %d\n", led_addr_x, led_addr_y, state);
    if (state == 0) { // off
        matriz_led[led_addr_x][led_addr_y][0] = 0;
        matriz_led[led_addr_x][led_addr_y][1] = 0;
        matriz_led[led_addr_x][led_addr_y][2] = 0;
    } else if (state == 1) { // seco (vermelho)
        matriz_led[led_addr_x][led_addr_y][0] = 2;
        matriz_led[led_addr_x][led_addr_y][1] = 0;
        matriz_led[led_addr_x][led_addr_y][2] = 0;
    } else if (state == 2) { // meio seco (magenta)
        matriz_led[led_addr_x][led_addr_y][0] = 2;
        matriz_led[led_addr_x][led_addr_y][1] = 0;
        matriz_led[led_addr_x][led_addr_y][2] = 2;
    } else if (state == 3) { // mediano (amarelo)
        matriz_led[led_addr_x][led_addr_y][0] = 2;
        matriz_led[led_addr_x][led_addr_y][1] = 2;
        matriz_led[led_addr_x][led_addr_y][2] = 0;
    } else if (state == 4) { // humido (ciano)
        matriz_led[led_addr_x][led_addr_y][0] = 0;
        matriz_led[led_addr_x][led_addr_y][1] = 2;
        matriz_led[led_addr_x][led_addr_y][2] = 2;
    } else if (state == 5) { // muito humido (verde)
        matriz_led[led_addr_x][led_addr_y][0] = 0;
        matriz_led[led_addr_x][led_addr_y][1] = 2;
        matriz_led[led_addr_x][led_addr_y][2] = 0;
    }

    if (led_flag == true) {
        led_buffer();
        npWrite();
        //printf("Matriz de LEDs atualizada\n");
    } else {
        //printf("Matriz de LEDs desligada\n");
    }
}

int select_state(int led_x, int led_y, int estado_inicial) {    //função para piscar o led
    if (led_flag == false) {
        return estado_inicial;
    }

    if (estado_inicial != 0) {
        if (blink == 1) {
            change_led_state(led_x, led_y, 0);
            blink = 0;
        } else {
            change_led_state(led_x, led_y, estado_inicial);
            blink = 1;
        }
        return estado_inicial;
    } else {
        if (blink == 1) {
            change_led_color(led_x, led_y, 2, 1);
            blink = 0;
        } else {
            change_led_color(led_x, led_y, 2, 0);
            blink = 1;
        }
        return estado_inicial;
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~funções para o ntp~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct udp_pcb *ntp_pcb;                                        // PCB para comunicação UDP com o servidor NTP
ip_addr_t ntp_server_address;                                   // Endereço IP do servidor NTP
char ntp_time_str[30];                                          // Variável global para armazenar a hora formatada
int timezone_offset = -3 * 3600;                                // Ajuste para UTC-3 (Brasília)
void ntp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {// Callback para receber a resposta NTP
    if (p && p->tot_len == NTP_MSG_LEN) {
        uint8_t ntp_data[4];
        pbuf_copy_partial(p, ntp_data, 4, 40);
        uint32_t seconds_since_1970 = (ntp_data[0] << 24 | ntp_data[1] << 16 | ntp_data[2] << 8 | ntp_data[3]) - NTP_DELTA;
        
        time_t raw_time = seconds_since_1970 + timezone_offset;
        struct tm *time_info = gmtime(&raw_time);
        
        snprintf(ntp_time_str, sizeof(ntp_time_str), "%02d/%02d/%04d %02d:%02d:%02d UTC",
                 time_info->tm_mday, time_info->tm_mon + 1, time_info->tm_year + 1900,
                 time_info->tm_hour, time_info->tm_min, time_info->tm_sec);

        //printf("NTP Time: %s\n", ntp_time_str);
    }
    
    pbuf_free(p);
    udp_remove(pcb);  // Libera o PCB após processar a resposta
}
void ntp_request() {                                            // Envia uma requisição NTP para o servidor
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    if (!p) return;

    uint8_t *req = (uint8_t *) p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1B; // Primeiro byte da mensagem NTP

    udp_sendto(ntp_pcb, p, &ntp_server_address, NTP_PORT);
    pbuf_free(p);
}
void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {  // Callback para resolver o endereço do servidor NTP
    if (ipaddr) {
        ntp_server_address = *ipaddr;
        ntp_request();
    } else {
        printf("DNS lookup failed\n");
    }
}
const char* start_ntp() {                                       // Inicia o processo de obtenção da hora via NTP
    if (ntp_pcb) {
        udp_remove(ntp_pcb);  // Libera o UDP PCB anterior antes de criar um novo
    }

    ntp_pcb = udp_new();
    if (!ntp_pcb) {
        printf("Falha ao criar UDP PCB\n");
        return NULL;
    }
    udp_recv(ntp_pcb, ntp_recv_callback, NULL);

    // Configura o DNS manualmente
    ip_addr_t dnsserver;
    IP4_ADDR(&dnsserver, 8, 8, 8, 8);
    dns_setserver(0, &dnsserver);

    // Tenta resolver o endereço do servidor NTP
    err_t err = dns_gethostbyname(NTP_SERVER, &ntp_server_address, ntp_dns_found, NULL);
    if (err == ERR_OK) {
        //printf("DNS resolvido imediatamente.\n");
        ntp_request();  // Se já tivermos um IP, enviamos a requisição NTP
    } else if (err == ERR_INPROGRESS) {
        printf("Aguardando resposta do DNS...\n");
    } else {
        printf("Erro na busca DNS!\n");
        return NULL;
    }

    return ntp_time_str; // Retorna a string formatada com a hora
}
char horarios[5][5][30] = {""};                                 // matriz para guardar o horario de atualização de cada ponto


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~funções para o joystick~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int pos_x=2;                                                    //posição inicial do seletor no eixo x
int pos_y=2;                                                    //posição inicial do seletor no eixo y
bool last_button1 = false;
bool last_button2 = false;
uint32_t last_update_time = 0;
void on_move_left() {
     printf("Movimento detectado: ESQUERDA\n"); 
     if (pos_x > 0){
         pos_x--;
     }else{
        pos_x =0;
     }
     printf("%d",pos_x);
     }
void on_move_right() { 
    printf("Movimento detectado: DIREITA\n");
     if (pos_x < 4){
         pos_x++;
     }else{
        pos_x =4;
     }
     printf("%d",pos_x);
     }
void on_move_up() { 
    printf("Movimento detectado: CIMA\n");
     if (pos_y < 4){
         pos_y++;
     }else{
        pos_y =4;
     }
     printf("%d",pos_y);
 }
void on_move_down() {
     
    printf("Movimento detectado: BAIXO\n");
     if (pos_y > 0){
         pos_y--;
     }else{
        pos_y =0;
     }
     printf("%d",pos_y);
 }
void on_joystick_move(int x, int y) {
    if(led_flag){
        change_led_state(pos_x,pos_y,matriz_state[pos_x][pos_y]);// para garantir que o led retorne ao estado inicial quando movimentar 
    }
   
    //matriz_state[pos_x][pos_y] = select_state(pos_x,pos_y,matriz_state[pos_x][pos_y]); 
    if (x < CENTER_X - THRESHOLD) {
        on_move_down();
    } else if (x > CENTER_X + THRESHOLD) {
        on_move_up();
    }
    if (y < CENTER_Y - THRESHOLD) {
         on_move_left();
    } else if (y > CENTER_Y + THRESHOLD) {
        on_move_right();
    }

}

void on_button_press() {
    printf("Botão do joystick pressionado!\n");
    led_flag = !led_flag;
    if (led_flag == true) {
        display("Matriz LED ON! ", 6);
        //Atualiza o estado de todos os LEDs conforme a matriz_state
        for (int x = 0; x < 5; x++) {
            for (int y = 0; y < 5; y++) {
                change_led_state(x, y, matriz_state[x][y]);
            }
        }
        led_buffer();
        npWrite();
    } else {
        display("Matriz LED OFF!", 6);
        npClear();
        npWrite();
        gpio_put(LED_GREEN, 0);
        gpio_put(LED_RED, 0);
    }
}

void on_button_A_press() {
    printf("Botão A pressionado!\n");
    if(select_flag == true){
        matriz_acionar_irrig[pos_x][pos_y] = !matriz_acionar_irrig[pos_x][pos_y];
    }

}
void on_button_B_press() {
    printf("Botão B pressionado!\n");
    select_flag = !select_flag;
    printf("fim função B\n");
}
void button_callback(uint gpio, uint32_t events) {
    if (gpio == SW_PIN) {
        on_button_press();
    } else if (gpio == BUTTON_A_PIN) {
        on_button_A_press();
    } else if (gpio == BUTTON_B_PIN) {
        on_button_B_press();
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~funções para o display oled~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t ssd[ssd1306_buffer_length];                     // Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
struct render_area frame_area = {                       //área de renderização
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};
void display( char *texto, int linha) {                 //escrever uma linha
    int y = linha * 8;  // Calcula a posição vertical com base na linha desejada
    ssd1306_draw_string(ssd, 0, y, texto); // Passa o ponteiro para a string
    render_on_display(ssd, &frame_area);
}
void display_multiline(char *textos[], int tamanho) {   //escrever varias linhas de uma vez
    int y = 0;
    for (int i = 0; i < tamanho; i++) {
        ssd1306_draw_string(ssd, 0, y, textos[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);
}
void clear_display(uint8_t *ssd, int linha) {           //limpar linha
    // Se linha for 0, limpa o display inteiro
    if (linha == 0) {
        memset(ssd, 0, 128 * 8); // Limpa o display inteiro (assumindo que a largura é 128 e a altura é 8)
    } else {
        int y = linha * 8;  // Calcula a posição da linha
        // Limpa a linha específica (assumindo que o display tem 128 pixels de largura)
        memset(ssd + (y * 128), 0, 128); // Limpa a linha no display
    }

    render_on_display(ssd, &frame_area); // Atualiza o display
}
void exibir_horario(int x, int y) {                     //mostra o horario diretamente no display
    display("ATUALIZADO EM: ",4);
    if(strlen(horarios[x][y]) == 0){
        // Exibe no OLED
        display("  Sem Dados!   ",5);

    } else {
        char linha1[16] = {0};
        char linha2[16] = {0};
        // Pegando o horário salvo na matriz
        char* horario = horarios[x][y];
        // Separa em duas partes
        strncpy(linha1, horario + 11, 8); // Hora (Ex: "14:30:00")
        strncpy(linha2, horario, 5);  // Data (Ex: "02/02")
        // Formata a string com o caractere "→" antes da hora
        char formatted_line[16] = {0};
        snprintf(formatted_line, sizeof(formatted_line), "%s-%s", linha1, linha2);

        printf("%s\n", formatted_line); // Corrigido para evitar comportamento indefinido
        display(formatted_line, 5);
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~funções para o servidor~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   
//pagina html

const char *HTTP_HTML_PAGE ="" \        
"<html><head><title>Matriz de Irrigacao</title></head><body>" \
"<h1>Irrigadores Acionados</h1>" \
"<p>(0 = desligado, 1 = ligado)</p>" \
"<table border='1'>" \
"<script>" \
"function toggleIrrig(x, y) {" \
"    let cell = document.querySelector(`td[data-x='${x}'][data-y='${y}']`);" \
"    let currentValue = parseInt(cell.innerText);" \
"    let newValue = currentValue === 1 ? 0 : 1;" \
"    fetch(`/irrig/${x}/${y}/${newValue}`)" \
"        .then(response => {" \
"             location.reload();" \
"        })" \
"        .catch(error => console.error('Erro na requisição:', error));" \
"location.reload();"\
"}" \
"</script>" \
"<tbody>%s</tbody></table></body></html>";


bool is_esp8266(const char *request) {                                                      // Verifica se a requisição veio do ESP8266
   return strstr(request, "User-Agent: ESP") != NULL;
}
void generate_html_page(char *buffer) { 
    char temp[256];
    buffer[0] = '\0';
    strcat(buffer, "<tr>");
    for (int i = 4; i >= 0; i--) {
        for (int j = 0; j < 5; j++) {
            sprintf(temp, "<td data-x=\"%d\" data-y=\"%d\" onclick=\"toggleIrrig(%d,%d)\">%d</td>",   // Gera a tabela HTML com os valores da matriz de acionamento
                    j,i, j, i, matriz_acionar_irrig[j][i]);
            strcat(buffer, temp);
        }
        strcat(buffer, "</tr>");
    }
}
void generate_status_string(char *buffer) {                                           // Gera uma string com a matriz de status para retornar ao ESP
    char temp[64];
    buffer[0] = '\0';
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            sprintf(temp, "%d|", matriz_state[i][j]);
            strcat(buffer, temp);
        }
        strcat(buffer, "/");
    }
}
static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {    // Callback para requisições HTTP
    if (p == NULL) {
        tcp_close(tpcb);
        return ERR_OK;
    }
    char *request = (char *)p->payload; 
    process_request(request, tpcb);
    pbuf_free(p);
    return ERR_OK;
}
void process_request(char *req, struct tcp_pcb *tpcb) {                                     // Processa a requisição HTTP
    int pos_x, pos_y, state;
    //printf("esse aqui é oq entrou:%s",req);
    bool esp = is_esp8266(req);
    char *start = strstr(req, "GET "); 
       if (start) {
        start += 4; 
        char *end = strchr(start, ' ');
        if (end) {
            *end = '\0';
        }
        printf("Requisição recebida: %s\n", start);
    }
    
    if (strcmp(start, "/status") == 0) {  
        if (esp) {
            // printf("Recebida requisição do ESP8266\n");
            char status_str[512];
            generate_irrig_string(status_str);
            char response[1024];
            snprintf(response, sizeof(response),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/plain\r\n"
                    "Content-Length: %d\r\n"
                    "Connection: close\r\n\r\n"
                    "%s",
                    (int)strlen(status_str), status_str);
            tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
       }else {
            char html_buffer[2048];
            generate_html_page(html_buffer);
            char buffer[4096];
            // Monta a página HTML completa com a tabela dinâmica
            snprintf(buffer, sizeof(buffer),
                    HTTP_HTML_PAGE,
                    html_buffer);
            tcp_write(tpcb, (const char *)buffer, strlen(buffer), TCP_WRITE_FLAG_COPY);
        }
    } else if (sscanf(start, "/%d/%d/%d", &pos_x, &pos_y, &state) == 3) {  
        // Atualiza a matriz com os valores recebidos
        if (pos_x >= 0 && pos_x < 5 && pos_y >= 0 && pos_y < 5 && state >= 0 && state < 6) {
            matriz_state[pos_x][pos_y] = state;
            if (state == 0) {
               play_tone(BUZZER_PIN, 220, 100); // Bipe de 100ms, Lá 3 , indicando que algum ponto foi desligado
            } else if (state == 1) {
                play_tone(BUZZER_PIN, 300, 250); // Bipe de 250ms, Mi 3 , indicando que algum ponto esta seco
            }
            const char* horario_atual = start_ntp();
            strncpy(horarios[pos_x][pos_y], horario_atual, sizeof(horarios[pos_x][pos_y]));
            change_led_state(pos_x, pos_y, state);
            printf("%s\n", horarios[pos_x][pos_y]);
            printf("Pos [%d][%d] atualizado para %d (Horário: %s)\n", 
                   pos_x, pos_y, state, horarios[pos_x][pos_y]);
            
            char response[] = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nLED atualizado!";
            tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
        } else {
            char response[] = "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\n\r\nErro: LED ou estado inválido!";
            tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
        }
    } else if (strncmp(start, "/irrig/", 7) == 0) {
        int x, y, estado;
        if (sscanf(start, "/irrig/%d/%d/%d", &x, &y, &estado) == 3) {
            if (x >= 0 && x < 5 && y >= 0 && y < 5) {
                matriz_acionar_irrig[x][y] = estado;
                printf("Irrigação em [%d][%d] alterada para %d\n", x, y, estado);
            }
        }
        // Após alterar, gere a nova página HTML
        char html_buffer[2048];
        generate_html_page(html_buffer);
        snprintf((char *)html_buffer, sizeof(html_buffer), HTTP_HTML_PAGE, html_buffer);
        tcp_write(tpcb, html_buffer, strlen(html_buffer), TCP_WRITE_FLAG_COPY);
    } else {
         char html_buffer[2048];
                generate_html_page(html_buffer);
                char buffer[4096];
                // Monta a página HTML completa com a tabela dinâmica
                snprintf(buffer, sizeof(buffer),
                        HTTP_HTML_PAGE,
                        html_buffer);
                tcp_write(tpcb, (const char *)buffer, strlen(buffer), TCP_WRITE_FLAG_COPY);
    }
}
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {            // Callback para conexões HTTP
    tcp_recv(newpcb, http_callback);
    return ERR_OK;
}
static void start_http_server(void) {                                                       // Inicialização do servidor HTTP
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB\n");
        return;
    }

    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }

    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}
time_t start_time;    // Variável para armazenar o tempo de início do programa
uint32_t last_update_time_clock; // Variável para armazenar o tempo da última atualização do relógio

bool is_wifi_connected() {
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

void check_wifi_connection() {// Verifica a conexão Wi-Fi e tenta reconectar se necessário
    while (!is_wifi_connected()) {
        printf("Wi-Fi desconectado, tentando reconectar...\n");
        clear_display(ssd,0);
        display("Conexao WIFI", 1);
        display("Perdida!", 2);
        display("Tentando se", 3);
        display("Reconectar...", 4);
        display("Aguarde...", 5);
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000) == 0) {
            printf("Reconectado ao Wi-Fi.\n");
            display("Wi-Fi OK!", 6);
            sleep_ms(1000);
            clear_display(ssd, 0);

            // Atualiza o horário do relógio
            const char* ntp_time = start_ntp();
            if (ntp_time != NULL) {
                struct tm time_info;
                sscanf(ntp_time, "%d/%d/%d %d:%d/%d", &time_info.tm_mday, &time_info.tm_mon, &time_info.tm_year, &time_info.tm_hour, &time_info.tm_min, &time_info.tm_sec);
                time_info.tm_year -= 1900; // Ajusta o ano
                time_info.tm_mon -= 1; // Ajusta o mês
                start_time = mktime(&time_info);
                last_update_time_clock = time_us_32(); // Atualiza o tempo da última atualização do relógio
            } else {
                display("Erro ao obter hora", 7);
            }
            break;
        }
        sleep_ms(2000); // Espera 5 segundos antes de tentar novamente
    }
}

time_t get_valid_ntp_time() {
    time_t valid_time = 0;
    while (valid_time == 0) {
        const char* ntp_time = start_ntp();
        if (ntp_time != NULL) {
            struct tm time_info;
            sscanf(ntp_time, "%d/%d/%d %d/%d/%d", &time_info.tm_mday, &time_info.tm_mon, &time_info.tm_year, &time_info.tm_hour, &time_info.tm_min, &time_info.tm_sec);
            time_info.tm_year -= 1900; // Ajusta o ano
            time_info.tm_mon -= 1; // Ajusta o mês
            valid_time = mktime(&time_info);
            if (time_info.tm_hour == 0 && time_info.tm_min == 0 && time_info.tm_sec == 0) {
                valid_time = 0; // Reinicia se o horário for 00:00:00
            }
        }
        if (valid_time == 0) {
            display("Obtendo hora válida...", 7);
            sleep_ms(2000); // Espera 2 segundos antes de tentar novamente
        }
    }
    return valid_time;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ \ main /~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main() {
    stdio_init_all();  // Inicializa a saída padrão
    adc_init(); // Inicializa o ADC
    npInit(LED_PIN); // Inicializa a matriz de LEDs
    pwm_init_buzzer(BUZZER_PIN);                // Inicializa o PWM no pino do buzzer  
    gpio_init(LED_RED);                        // Inicializa o pino do LED vermelho
    gpio_set_dir(LED_RED, GPIO_OUT);        // Configura o pino do LED vermelho como saída
    gpio_init(LED_GREEN);                      // Inicializa o pino do LED verde
    gpio_set_dir(LED_GREEN, GPIO_OUT);      // Configura o pino do LED verde como saída
    gpio_set_dir(LED_BLUE, GPIO_OUT);       // Configura o pino do LED azul como saída
    adc_gpio_init(VRY_PIN);                     // Inicializa o pino do eixo Y do joystick
    adc_gpio_init(VRX_PIN);                     // Inicializa o pino do eixo X do joystick
    gpio_init(SW_PIN);                          // Inicializa o pino do botão do joystick
    gpio_set_dir(SW_PIN, GPIO_IN);              // Configura o pino do botão como entrada
    gpio_pull_up(SW_PIN);                       // Habilita o pull-up interno
    gpio_init(BUTTON_A_PIN);                    // Inicializa o pino do botão A
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);        // Configura o pino do botão como entrada
    gpio_pull_up(BUTTON_A_PIN);                 // Habilita o pull-up interno
    gpio_set_irq_enabled_with_callback(BUTTON_A_PIN, GPIO_IRQ_EDGE_FALL, true, &button_callback);   // Habilita a interrupção no botão A
    gpio_init(BUTTON_B_PIN);                    // Inicializa o pino do botão B
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);        // Configura o pino do botão como entrada
    gpio_pull_up(BUTTON_B_PIN);                 // Habilita o pull-up interno
    gpio_set_irq_enabled_with_callback(BUTTON_B_PIN, GPIO_IRQ_EDGE_FALL, true, &button_callback);   // Habilita a interrupção no botão B
    int last_x = CENTER_X, last_y = CENTER_Y;   // Última posição do joystick
    bool last_sw = true;                        // Último estado do botão
    // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);   // Inicializa o I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  // Configura os pinos do I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);  // Configura os pinos do I2C
    gpio_pull_up(I2C_SDA);                      // Habilita o pull-up nos pinos do I2C
    gpio_pull_up(I2C_SCL);                      // Habilita o pull-up nos pinos do I2C
    // Processo de inicialização do OLED SSD1306
    ssd1306_init();
    calculate_render_area_buffer_length(&frame_area);
    memset(ssd, 0, ssd1306_buffer_length);  // Limpa o buffer do display
    render_on_display(ssd, &frame_area);    // Atualiza o display
    display("Power On",1);
    restart:                                //reinicia o programa
    sleep_ms(2000);
    // Inicializa o Wi-Fi
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar o Wi-Fi\n");
        display("Erro no Wifi",3);
        return 1;
    }
    cyw43_arch_enable_sta_mode();           // Habilita o modo estação (cliente)
    clear_display(ssd,0);
    printf("Conectando ao Wi-Fi...\n");
    display("Conectando Wifi...",1);
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000) != 0) { // Tenta conectar ao Wi-Fi
        printf("Wi-Fi connect failed, tentando novamente...\n");
        display("tentando...",2);
        sleep_ms(100);
    }
    printf("Conectado ao Wi-Fi.\n");

    // Exibe o endereço IP
    uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_string[16]; // Espaço suficiente para "255.255.255.255\0"
    sprintf(ip_string, "%d.%d.%d.%d", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    printf("Endereço IP: %s\n", ip_string);
    display("Wi-Fi OK! IP:",2);
    display(ip_string,3);
    sleep_ms(3000);
    clear_display(ssd,0);
    // Inicia o servidor HTTP    
    printf("Iniciando servidor HTTP\n");
    display("Iniciando",1);
    display("servidor HTTP..",2);
    start_http_server(); 
    start_ntp(); // Inicia o NTP
    display("OK!",3);
    sleep_ms(1000);
    clear_display(ssd,0);

    // Obter a hora atual do NTP uma vez
    const char* ntp_time = start_ntp();
    time_t start_time;
    if (ntp_time != NULL) {
        struct tm time_info;
        sscanf(ntp_time, "%d/%d/%d %d:%d:%d", &time_info.tm_mday, &time_info.tm_mon, &time_info.tm_year, &time_info.tm_hour, &time_info.tm_min, &time_info.tm_sec);
        time_info.tm_year -= 1900; // Ajusta o ano
        time_info.tm_mon -= 1; // Ajusta o mês
        start_time = mktime(&time_info);
    } else {
        display("Erro ao obter hora",7);
        return 1;
    }

    uint32_t last_update_time_clock = time_us_32(); // Tempo da última atualização do relógio em microssegundos
    uint32_t last_update_time_joystick = time_us_32(); // Tempo da última atualização do joystick em microssegundos
    bool wifi_connected = true; // Estado da conexão Wi-Fi

    while (true) { // Loop principal
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {        // Verifica se a conexão Wi-Fi está ativa
            if (wifi_connected) {
                wifi_connected = false;
                check_wifi_connection(); // Verifica e reconecta ao Wi-Fi se necessário
                wifi_connected = true;
            }
        }
        uint32_t current_time = time_us_32(); // Pega o tempo atual em microssegundos
        bool sw_value = gpio_get(SW_PIN) == 0; // Lê o valor do botão
        // Incrementa o tempo
        time_t current_time_sec = start_time + (current_time - last_update_time_clock) / 1000000;
        // Formata a hora para exibir no display
        struct tm *time_info = localtime(&current_time_sec);
        char display_text[32];
        snprintf(display_text, sizeof(display_text), "Hora: %02d:%02d:%02d", time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
        display(display_text, 7);
        if (select_flag == false) {                             //se estiver com o cursor desabilitado
            display("Monitor de     ",1);
            display("Irrigacao      ",2);
            display("Pressione Botao",3);
            display("B para Liberar ",4);
            display("O cursor       ",5);
            gpio_put(LED_GREEN, 0);
            gpio_put(LED_RED, 0);
            change_led_state(pos_x, pos_y, matriz_state[pos_x][pos_y]); // garantir que o led retorne ao estado inicial caso esteja piscando ao pressionar o botão B
        } else {
                if(led_flag){
                    select_state(pos_x, pos_y, matriz_state[pos_x][pos_y]); // Coloca a posição do cursor piscando
                }
                display("X | Y |Umidade",1);
                exibir_horario(pos_x, pos_y); // Exibe o horário do ponto selecionado
                if (matriz_acionar_irrig[pos_x][pos_y] == 1) { // Aciona o irrigador
                    display("Irrig |on      ",3);
                    if(led_flag){
                        gpio_put(LED_GREEN, 1);
                        gpio_put(LED_RED, 0);
                    }
                } else {
                    display("Irrig |off     ",3);     
                    if(led_flag){
                        gpio_put(LED_GREEN, 0);
                        gpio_put(LED_RED, 1);
                    }

                }
                char buffer[16]; // Buffer para exibir as coordenadas e o estado do ponto
                char estado[10]; // Buffer para exibir o estado do ponto
                if (matriz_state[pos_x][pos_y] == 1) { // Converte o estado do ponto para string
                    strcpy(estado, "Seco    ");
                    
                } else if (matriz_state[pos_x][pos_y] == 2) {
                    strcpy(estado, "Baixo   ");
                } else if (matriz_state[pos_x][pos_y] == 3) {
                    strcpy(estado, "Medio   ");
                } else if (matriz_state[pos_x][pos_y] == 4) {
                    strcpy(estado, "Bom     ");
                } else if (matriz_state[pos_x][pos_y] == 5) {
                    strcpy(estado, "Alto    ");
                } else if (matriz_state[pos_x][pos_y] == 0) {
                    strcpy(estado, "Offline ");
                }
                sprintf(buffer, "%d | %d |%s", pos_x, pos_y, estado); // Converte as coordenadas e o estado do ponto para string
                //printf(buffer);
                display(buffer,2);
            

            if (current_time - last_update_time_joystick >= UPDATE_INTERVAL) { // Verifica se é hora de atualizar o cursor
                // Lê o eixo X
                adc_select_input(0);
                int vrx_value = adc_read();

                // Lê o eixo Y
                adc_select_input(1);
                int vry_value = adc_read();

                // Verifica se o joystick se moveu além da zona morta
                if (abs(vrx_value - CENTER_X) > DEADZONE || abs(vry_value - CENTER_Y) > DEADZONE) {
                    if (abs(vrx_value - last_x) > DEADZONE || abs(vry_value - last_y) > DEADZONE) {
                        on_joystick_move(vrx_value, vry_value);
                        last_x = vrx_value; // Atualiza a última posição do eixo X
                        last_y = vry_value; // Atualiza a última posição do eixo Y
                    }
                }
                last_update_time_joystick = current_time; // Atualiza o tempo da última atualização do joystick
            }
        }
        if (sw_value && !last_sw) { // Verifica se o botão foi pressionado
            on_button_press();
        }
        last_sw = sw_value; // Atualiza o valor do botão
        cyw43_arch_poll(); // Necessário para manter o Wi-Fi ativo
        sleep_ms(50);
    }

    cyw43_arch_deinit(); // Desliga o Wi-Fi (não será chamado, pois o loop é infinito)
    return 0;
}
