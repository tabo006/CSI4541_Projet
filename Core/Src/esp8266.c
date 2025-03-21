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
void ESP_Server_Init(const char *ssid, const char *password) {
    char cmdBuffer[100];  // Buffer for AT commands

    printf("RESETTING ESP8266...\n");
    send_AT_command("AT+RST\r\n", "OK", 5000);

    printf("Testing AT Command...\n");
    send_AT_command("AT\r\n", "OK", 5000);

    printf("Connecting to Wi-Fi...\n");
    sprintf(cmdBuffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
    send_AT_command(cmdBuffer, "WIFI CONNECTED", 15000);

    printf("Checking assigned IP address...\n");
    send_AT_command("AT+CIFSR\r\n", "+CIFSR", 5000);  // Wait for IP Address

    printf("Enabling single connections...\n");
    send_AT_command("AT+CIPMUX=0\r\n", "OK", 2000);

    // Enable UART Receive Interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void sendHTMLToLocalServer(const char *serverIP, const char *htmlContent) {
    char cmdBuffer[500];
    char responseBuffer[100];

    // Step 1: Connect to Local Server
    sprintf(cmdBuffer, "AT+CIPSTART=\"TCP\",\"%s\",8080\r\n", serverIP);
    send_AT_command(cmdBuffer, "OK", 2000);

    // Step 2: Prepare HTTP POST Request
    sprintf(cmdBuffer,
            "POST /status.html HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: %d\r\n\r\n"
            "%s\r\n",
            serverIP, strlen(htmlContent), htmlContent);

    int requestLength = strlen(cmdBuffer);

    // Step 3: Send the Length of the Request
    sprintf(responseBuffer, "AT+CIPSEND=%d\r\n", requestLength);
    send_AT_command(responseBuffer, ">", 500);

    // Step 4: Send the Actual Data
    send_AT_command(cmdBuffer, "SEND OK", 1000);

    // Step 5: Close the Connection
    send_AT_command("AT+CIPCLOSE\r\n", "OK", 500);

    printf("HTML sent to Local Server: %s\n", htmlContent);
}

/*
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
*/

