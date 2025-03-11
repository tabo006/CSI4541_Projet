#include "esp8266.h"
#include "usart.h"
#include <string.h>

// Global buffer for ESP8266 responses
char buffer[600];
int buffer_index = 0;
volatile uint8_t intrusion_detected = 0; // 1 = Detected, 0 = Normal

void ESP_Server_Init() {
    // Connect to Wi-Fi
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CWJAP=\"Your_WiFi_SSID\",\"Your_WiFi_Password\"\r\n", 38, HAL_MAX_DELAY);
    HAL_Delay(5000);

    // Enable multiple connections
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPMUX=1\r\n", 13, HAL_MAX_DELAY);
    HAL_Delay(1000);

    // Start TCP Server on ESP8266
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPSERVER=1,80\r\n", 19, HAL_MAX_DELAY);
    HAL_Delay(2000);

    // Enable UART Receive Interrupt
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void messageHandler() {
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE); // Disable interrupt

    if (string_contains((char*)buffer, "GET", buffer_index) != -1) {
        sendData();
    } else if (string_contains((char*)buffer, "+CWJAP:", buffer_index) != -1
               && (string_contains((char*)buffer, "FAIL", buffer_index) != -1
               || string_contains((char*)buffer, "DISCONNECT", buffer_index) != -1)) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CWJAP=\"Your_WiFi_SSID\",\"Your_WiFi_Password\"\r\n", 38, HAL_MAX_DELAY);
    }

    ESP_Clear_Buffer();
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // Re-enable interrupt
}

void sendData() {
    char outputString[300], cipsend[50], response[600];

    sprintf(outputString, "<html><head><meta http-equiv=\"refresh\" content=\"5\">"
                          "<title>Intrusion Detection</title></head>"
                          "<body><h1 style='color:red;'>%s</h1></body></html>",
                          intrusion_detected ? "Intrusion Detected!" : "No Intrusion");

    sprintf(response, "HTTP/1.1 200 OK\r\nContent-Length: %i\r\nContent-Type: text/html\r\n\r\n%s",
            strlen(outputString), outputString);
    sprintf(cipsend, "AT+CIPSEND=0,%i\r\n", strlen(response));

    HAL_UART_Transmit(&huart2, (uint8_t*)cipsend, strlen(cipsend), 100);
    HAL_Delay(50);
    HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
    HAL_Delay(50);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPCLOSE=0\r\n", strlen("AT+CIPCLOSE=0\r\n"), 100);
}

void ESP_Clear_Buffer() {
    memset(buffer, 0, sizeof(buffer));
    buffer_index = 0;
}

// Utility function to check if a substring exists in a string
int string_contains(const char *str, const char *sub, int length) {
    for (int i = 0; i <= length - strlen(sub); i++) {
        if (strncmp(&str[i], sub, strlen(sub)) == 0) {
            return i;
        }
    }
    return -1;
}