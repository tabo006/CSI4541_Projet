#include "esp8266.h"
#include "usart.h"
#include <string.h>

// Global buffer for ESP8266 responses
char buffer[600];
int buffer_index = 0;
volatile uint8_t intrusion_detected = 0; // 1 = Detected, 0 = Normal

void send_AT_command(char *cmd, const char *expectedResponse, uint32_t timeout) {
    printf("Sending: %s", cmd);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);

    if (!waitFor(expectedResponse, timeout)) {
        printf("Warning: No valid response for: %s\n", cmd);
    }
}
int waitFor(const char *expectedResponse, uint32_t timeout) {
    uint8_t rxBuffer[200] = {0};  // Increased buffer size
    uint32_t startTick = HAL_GetTick();
    int index = 0;

    while ((HAL_GetTick() - startTick) < timeout) {  // Timeout check
        if (HAL_UART_Receive(&huart1, &rxBuffer[index], 1, 100) == HAL_OK) {
            if (rxBuffer[index] == '\n' || rxBuffer[index] == '\r') {  // Stop at newline
                rxBuffer[index] = '\0';  // Null terminate string

                if (strstr((char *)rxBuffer, expectedResponse)) {
                	printf("ESP Response: %s\n", rxBuffer);// Check if expected response exists
                    return 1;  // Found the expected response
                }

                index = 0;  // Reset buffer if response not matched
            } else {
                index++;
                if (index >= sizeof(rxBuffer) - 1) index = 0;  // Prevent buffer overflow
            }
        }
    }
    printf("Timeout waiting for: %s\n", expectedResponse);
    return 0;  // Timeout reached without expected response
}
void ESP_Server_Init() {
    printf("RESETTING ESP8266...\n");
    send_AT_command("AT+RST\r\n", "OK", 5000);

    printf("Testing AT Command...\n");
    send_AT_command("AT\r\n", "OK", 5000);

    printf("Connecting to Wi-Fi...\n");
    send_AT_command("AT+CWJAP=\"Abraham_Nguero\",\"@\"\r\n", "WIFI CONNECTED", 15000);

    printf("Checking assigned IP address...\n");
    send_AT_command("AT+CIFSR\r\n", "+CIFSR", 5000);  // Wait for IP Address

    printf("Enabling multiple connections...\n");
    send_AT_command("AT+CIPMUX=0\r\n", "OK", 2000);

    //printf("Starting TCP Server...\n");
    //send_AT_command("AT+CIPSERVER=1,80\r\n", "OK", 3000);

    // Enable UART Receive Interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
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

void sendDataToThingSpeak(const char *apiKey, int fieldNumber, int value) {
    char cmdBuffer[200];
    char responseBuffer[100];
    printf("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
    // **Step 1: Connect to ThingSpeak's Server**
    sprintf(cmdBuffer, "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
    send_AT_command(cmdBuffer, "OK", 5000);

    printf("GET /update");
    // **Step 2: Prepare HTTP GET Request with API Key and Field Number**
    sprintf(cmdBuffer, "GET /update?api_key=%s&field%d=%d\r\n", apiKey, fieldNumber, value);
    int requestLength = strlen(cmdBuffer);

    // **Step 3: Send the Length of the Request**
    printf("AT+CIPSEND");
    sprintf(responseBuffer, "AT+CIPSEND=%d\r\n", requestLength);
    send_AT_command(responseBuffer, ">", 5000);

    printf("Data sent to ThingSpeak: API_KEY=%s, Field=%d, Value=%d\n", apiKey, fieldNumber, value);
    // **Step 4: Send the Actual Data**
    send_AT_command(cmdBuffer, "SEND OK", 5000);

    printf("AT+CIPCLOSE");
    // **Step 5: Close the Connection**
    send_AT_command("AT+CIPCLOSE\r\n", "OK",5000);  // Send AT command and print response
}

/*void sendData() {
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
*/
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
