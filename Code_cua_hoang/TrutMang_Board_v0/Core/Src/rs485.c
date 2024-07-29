/*
 * rs485.h version 2
 * Created on: 25-June-2024
 * Author: Le Huu An
 */


#include <rs485.h>
#include <string.h>

/*
BRIEF NOTE: Ver2 bo sung ham send uart theo tung byte

		+ Nhan ngat UART (Put in CODE BEGIN 0)
					void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {  
						if(huart->Instance == USART1) 
						{    
							RS485_Read(&vRS485);
							HAL_UART_Receive_IT(&huart1, &vRS485.rxByte, 1); 			
						}
					}
					
		+ Put in CODE BEGIN 2
			HAL_UART_Receive_IT(&huart3, &vRS485.rxByte, 1);
					
		+ RX (put in While): ham nay thuc hien khi co Rx dc set
		
			if(vRS485.RxFlag == 1){   
				HAL_GPIO_WritePin(DE_RS485_GPIO_Port, DE_RS485_Pin, GPIO_PIN_SET);
				vRS485.RxFlag = 0;   
				
				//Xu ly data nhan duoc o day

				//for(uint16_t index=0; index<1000; index++);			//delay with for 200us
				
				HAL_UART_Transmit(&huart1, vRS485.txBuff, SIZE_RS485, HAL_MAX_DELAY);
				HAL_GPIO_WritePin(DE_RS485_GPIO_Port, DE_RS485_Pin, GPIO_PIN_RESET);  
				vRS485.STATE = USART_WAIT;
			}	

		+ TX (put in While):	ham nay thuc hien khi co Tx dc set
		
			if(vRS485.TxFlag == 1){
				RS485_SendData(&vRS485, USART1, 2000);
			}
	
*/
/**********************************************************************************************************************************/

/******EXTERN*******/
extern UART_HandleTypeDef huart1;


/**********************************************************************************************************************************/
/*******DECLARE VARIABLE********/
rs485_t vRS485;		// variable RS485
extern uint16_t address_chute;
/**********************************************************************************************************************************/
/*******FUNCTION********/

/*______________________________________________________________________________*/
	void RS485_Read(rs485_t *pRx){			 //Read Data from RS485
	 //Process receive data 
		switch(pRx->STATE) 
		{
			case  USART_WAIT:                  //Wait for start of new message   
			{
				if(pRx->rxByte == pRx->header){       //check header
					pRx->rxBuff[0] = pRx->rxByte;
					pRx->STATE = USART_READING;      //switch to Reading state    
					pRx->rxPointer = 1;
				}   
			break;
			}  
			//-------------------------------------------------------------------------------------------
			case USART_READING:  
			{
				pRx->rxBuff[pRx->rxPointer] = pRx->rxByte; 			//Read data   
				if(pRx->rxPointer == SIZE_RS485-1)              //Check the last byte
				{    
					if(pRx->rxBuff[pRx->rxPointer] == pRx->ender && pRx->rxBuff[2] == (address_chute>>8&0xFF) && pRx->rxBuff[3] == (address_chute&0xFF)){    //Success -> Turn on the Flag and process data			  
						pRx->RxFlag = 1;  
						pRx->cnt_mess_received++;						
					}
					else{ 										//Fail -> Reset buffer, pointer and move to WAIT state                 
						pRx->rxPointer = 0;     
						pRx->RxFlag = 0;
						memset(pRx->rxBuff, 0x00, SIZE_RS485); 
				/* them dong lenh nay */		pRx->STATE = USART_WAIT;						
					}  
				}
				else{
					pRx->rxPointer = pRx->rxPointer + 1;  //Increse the pointer    
					if(pRx->rxPointer > SIZE_RS485-1){           //Fail
						pRx->rxPointer = 0;
						pRx->RxFlag = 0;     
						memset(pRx->rxBuff, 0x00, SIZE_RS485);
						pRx->STATE = USART_WAIT;
					}   
				}   
			 break;  
			} 
		}
	}

/*______________________________________________________________________________*/
	void RS485_Init(rs485_t *pRS, UART_HandleTypeDef *huart, uint8_t _ID_Board){
		pRS->huart = huart;
		__HAL_UART_CLEAR_FLAG(pRS->huart, UART_FLAG_TC);
		
		pRS->RxFlag = 0;		
		pRS->rxByte = 0; 
		pRS->rxPointer = 0;
		pRS->STATE = USART_WAIT;
		pRS->cmd_type = 0x00;
		pRS->cnt_mess_received = 0;
		
		pRS->TxFlag = 0;
		pRS->cnt_mess_sent = 0;
		pRS->cnt_byte_sent = 0;
		pRS->header = 0xFF;
		pRS->ender = 0xFC;
		memset(pRS->rxBuff, 0x00, SIZE_RS485);
		memset(pRS->txBuff, 0x00, SIZE_RS485);
		
		pRS->ID_Board =_ID_Board;
		
	}
	
/*______________________________________________________________________________*/
	static uint8_t isSending = 0;
	static uint8_t TC = 0;
	
	void RS485_SendData(rs485_t *pRS, USART_TypeDef * USARTx, uint16_t timeout){
		uint8_t cnt_clear_TC = 10;
		//TC = USARTx->SR & USART_SR_TC;
		
		if((isSending == 1) && ((USARTx->SR & USART_SR_TC) != 0)){
			if(++pRS->cnt_byte_sent == SIZE_RS485){
				pRS->TxFlag = 0;
				pRS->cnt_mess_sent++;
				pRS->cnt_byte_sent = 0;
				
				HAL_Delay(3);
				HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET); 	
				HAL_UART_Receive_IT(&huart1, &pRS->rxByte, 1);
			}
			__HAL_UART_CLEAR_FLAG(pRS->huart, UART_FLAG_TC);	
			isSending = 0;
		}

		if(isSending == 0&&pRS->TxFlag == 1){
				//USART1->SR &= ~USART_SR_TC;		// clear TC flag
			__HAL_UART_CLEAR_FLAG(pRS->huart, UART_FLAG_TC);		
			
			// Confirm TC is off
			while(!(USARTx->SR & USART_SR_TXE)){		
				if(--cnt_clear_TC == 0) break;
			}
			
			// transmit byte
			USARTx->DR = pRS->txBuff[pRS->cnt_byte_sent];
	
			isSending = 1;
		}	
	}
	
/*______________________________________________________________________________*/
	
	
	
	
	
	
/*______________________________________________________________________________*/
	
	
	
	
/*______________________________________________________________________________*/




/*______________________________________________________________________________*/
/*Code test, flag_sent dat trong ngat voi chu ky 5ms*/
	
//if(flag_sent >= 5){
//	HAL_GPIO_WritePin(DE_RS485_GPIO_Port, DE_RS485_Pin, GPIO_PIN_SET);
//	flag_sent = 0;
//	vRS485.txBuff[0] = vRS485.header;
//	
//	vRS485.txBuff[1] = 0xAA; 
//	
//	tx_buffer[SIZE_RS485-1] = vRS485.ender;
//	for(uint8_t i = 2; i<SIZE_RS485-3; i++){
//		vRS485.txBuff[i] = i;
//	}
//	vRS485.txBuff[SIZE_RS485-2] = rand() % 0xFF;
//	vRS485.txBuff[SIZE_RS485-3] = rand() % 0xFF;
//	
//	HAL_UART_Transmit(&huart3, vRS485.txBuff, SIZE_RS485, HAL_MAX_DELAY);
//	
//	HAL_GPIO_WritePin(DE_RS485_GPIO_Port, DE_RS485_Pin, GPIO_PIN_RESET);
//	vRS485.STATE = USART_WAIT;
//}




