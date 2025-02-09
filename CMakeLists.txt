# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Definindo os padrões de C e C++
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Inicializa o Pico SDK com a extensão do VS Code
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 1.5.1)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)

# Definindo o caminho do arquivo pico-vscode.cmake
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# Defina a placa para Pico W
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Importa o SDK do Pico
include(pico_sdk_import.cmake)

# Nome do projeto
project(projeto C CXX ASM)

# Inicializa o SDK do Pico
pico_sdk_init()

# Adiciona o executável do projeto
add_executable(projeto projeto.c inc/ssd1306_i2c.c lwip/src/apps/sntp/sntp.c)

# Defina o nome e a versão do programa
pico_set_program_name(projeto "projeto")
pico_set_program_version(projeto "0.1")

# Configura a saída padrão para USB e UART
pico_enable_stdio_uart(projeto 0)
pico_enable_stdio_usb(projeto 1)

# Vincula as bibliotecas necessárias
target_link_libraries(projeto
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background  # Para conectividade Wi-Fi
    hardware_adc  # Para I2C, se necessário para o PN532
    hardware_i2c
    hardware_pwm
)


# Adiciona diretórios de cabeçalhos
target_include_directories(projeto PRIVATE ./lwip
    ${CMAKE_CURRENT_LIST_DIR}
)




# Gera os artefatos extras, como o arquivo UF2
pico_add_extra_outputs(projeto)
