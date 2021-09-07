/*
 * usart.h
 *
 *  Created on: Sep 8, 2021
 *      Author: bot
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#define DBG_BUF_LEN     512

UART_HandleTypeDef huart1;

char DBG_BUFFER[DBG_BUF_LEN];

#define SYSTEM_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    strcpy(DBG_BUFFER, "[SYSTEM]: "); \
    sprintf(DBG_BUFFER + strlen("[SYSTEM]: "),FORMAT,##__VA_ARGS__); \
	if(debugenable.system)\
		HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}
#define SIMPLE_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
	HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}


#endif /* INC_USART_H_ */
