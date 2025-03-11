#ifndef ESP8266_H
#define ESP8266_H

#include "main.h"   // Includes HAL libraries automatically
#include "usart.h"  // Required for UART communication
#include <string.h> // Required for string operations

// Function Prototypes
void ESP_Server_Init(void);
void messageHandler(void);
void sendData(void);
void ESP_Clear_Buffer(void);
int string_contains(const char *str, const char *sub, int length);

#endif /* ESP8266_H */