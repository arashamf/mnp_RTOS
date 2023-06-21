
/* Includes ------------------------------------------------------------------*/
#include "MNP_msg.h"
#include "usart.h"
#include "typedef.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
//Private defines -------------------------------------------------------------------------------------//
//Constants -------------------------------------------------------------------------------------------//
//Private variables -----------------------------------------------------------------------------------//
MNP_M7_CFG_t MNP_M7_CFG = //иницализация шаблона структуры перезагрузки и настройки приёмника
{
	.GPS_RST = &GPS_RST, //ф-я аппаратной перезагрузки приёмника
	.rst_delay = GPS_RST_DELAY, //задержка для перезагрузки приёмника
};

//-----------------------------------------------------------------------------------------------------//
uint16_t MNP_CalcChkSum	(uint16_t *Array, int WordsCount)
{
  uint16_t chksum = 0;
  uint32_t count;

	for( count = 0 ; count < WordsCount; count++ )
	{
		chksum += Array[count];
	}
	
	return (uint16_t)(0 - chksum);
}

//-----------------------------------------------------------------------------------------------------//
static void MNP_PutMessage (USART_TypeDef *USARTx, MNP_MSG_t *Msg, uint16_t MsgId, uint16_t WordsCount)
{
	
	Msg->msg_header.MNP_header.sync = MNP_SYNC_CHAR;
	Msg->msg_header.MNP_header.msg_id = MsgId; //id MNP-сообщения 
	Msg->msg_header.MNP_header.data_size = WordsCount; //количество 16 битных слов в теле сообщения
	Msg->msg_header.MNP_header.dummy = 0;
	Msg->msg_header.MNP_header.chksum = MNP_CalcChkSum((uint16_t *)&Msg->msg_header.MNP_header, (sizeof(HEAD_MNP_MSG_t)-2)/2);
	
	Msg->payload.raw_words[WordsCount] = MNP_CalcChkSum((uint16_t *)&Msg->payload, WordsCount); //сохранение КС
	
	MNP_UART_Puts(MNP_UART, (uint8_t *)Msg, (sizeof(HEAD_MNP_MSG_t) + (WordsCount+1)*2)); //количество передаваемых символов = количество байт в заголовке + количество передаваемых байт в самом сообщении + 2 байта КС
}

//-----------------------------------------------------------------------------------------------------//
static void MNP_PutMessageParse (USART_TypeDef *USARTx, MNP_MSG_t *Msg, uint16_t MsgId, uint16_t WordsCount)
{
	
	Msg->msg_header.MNP_header.sync = MNP_SYNC_CHAR;
	Msg->msg_header.MNP_header.msg_id = MsgId; //id MNP-сообщения 
	Msg->msg_header.MNP_header.data_size = WordsCount; //количество 16 битных слов в теле сообщения
	Msg->msg_header.MNP_header.dummy = 0;
	Msg->msg_header.MNP_header.chksum = MNP_CalcChkSum((uint16_t *)&Msg->msg_header.MNP_header, (sizeof(HEAD_MNP_MSG_t)-2)/2);
	
	Msg->payload.raw_words[WordsCount] = MNP_CalcChkSum((uint16_t *)&Msg->payload, WordsCount); //сохранение КС
	
	MNP_UART_Puts(MNP_UART, (uint8_t *)Msg, (sizeof(HEAD_MNP_MSG_t) + (WordsCount+1)*2)); //количество передаваемых символов = количество байт в заголовке + количество передаваемых байт в самом сообщении + 2 байта КС
	sprintf (buffer_TX_UART2, (char *)"%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x\r\n", Msg->msg_header.header_bytes[0],
	Msg->msg_header.header_bytes[1], Msg->msg_header.header_bytes[2], Msg->msg_header.header_bytes[3], Msg->msg_header.header_bytes[4],
	Msg->msg_header.header_bytes[5], Msg->msg_header.header_bytes[6], Msg->msg_header.header_bytes[7], Msg->msg_header.header_bytes[8],
	Msg->msg_header.header_bytes[9], Msg->payload.raw_bytes[0],Msg->payload.raw_bytes[1], Msg->payload.raw_bytes[2],
	Msg->payload.raw_bytes[3], Msg->payload.raw_bytes[4],Msg->payload.raw_bytes[5]);	
	//sprintf (buffer_TX_UART2, (char *)"%u\r\n", (sizeof(HEAD_MNP_MSG_t) + (WordsCount+1)*2));
	UART2_PutString (buffer_TX_UART2);
}

//-----------------------------------------------------------------------------------------------------//
static void MNP_M7_init ( MNP_MSG_t *Msg)
{
	
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //источник - RAM приёмника
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //источник - flash приёмника
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x01; //запись уставок
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //6 32-разрядных слов конфигурации
	
	Msg->payload.msg3006.command_code.dummy = 0x0; //резерв
	
	Msg->payload.msg3006.config.protocol0 = 0x1; //протокол канала 0 - MNP_binary
	Msg->payload.msg3006.config.baud_divider0 = 460800 / 115200; //предделитель UART0=4, скорость 115200 
	Msg->payload.msg3006.config.protocol1 = 0x1; //протокол канала 1 - MNP_binary
	Msg->payload.msg3006.config.baud_divider1 = 460800 / 115200; //предделитель UART1=4, скорость 115200 
	Msg->payload.msg3006.config.tropo_corr = 0x1; //включение использования модели тропосферы
	Msg->payload.msg3006.config.use_difc = 0x1; //включение использования дифференциальных поправок
	Msg->payload.msg3006.config.dif_only = 0x0; //отключение принудительного использования дифференциального режима
	Msg->payload.msg3006.config.sol_smooth = 0x0; //отключение фиксации координат при скорости менее 1 м/c
	Msg->payload.msg3006.config.sol_filter = 0x1; //включение сглаживание решения
	Msg->payload.msg3006.config.meas_filter = 0x1; //включение совмещенной фильтрации по коду и фазе несущей
	Msg->payload.msg3006.config.iono_corr = 0x1; // Разрешение ионосферной коррекции
	Msg->payload.msg3006.config.disable_2D = 0x0; //отключение запрета двумерной навигации
	Msg->payload.msg3006.config.use_RAIM = 0x1; //включение алгоритма RAIM
	Msg->payload.msg3006.config.enable_warm_startup = 0x1; //включение быстрого «горячего» старта
	Msg->payload.msg3006.config.sys_time = 0x0; // Привязка секундной метки к времени навигационной системы 
	Msg->payload.msg3006.config.glo_time = 0x0; // Секундная метка ГЛОНАСС / UTC(SU)
	Msg->payload.msg3006.config.shift_meas = 0x0; // Привязка измерений к секундной метке
	Msg->payload.msg3006.config.enable_SBAS = 0x0; //Разрешение SBAS
	Msg->payload.msg3006.config.enable_iono_SBAS = 0x0; //Разрешение модели ионосферы SBAS
	Msg->payload.msg3006.config.GPS_compatibility = 0x0; //Режим совместимости с приемниками GPS
	Msg->payload.msg3006.config.wr_alms = 0x1; //включение сохранения в flash альманахов 
	Msg->payload.msg3006.config.wr_ephs = 0x1; //включение сохранения в flash эфемерид 
	Msg->payload.msg3006.config.wr_ionoutc = 0x1; //включение сохранения в flash модели UTC GPS
	Msg->payload.msg3006.config.wr_coords = 0x1; //включение сохранения в flash координат
	Msg->payload.msg3006.config.enable_3000_1000_GGA_0 = 0x0; //включение кадра 3000/1000/«GGA» по каналу 0
	Msg->payload.msg3006.config.enable_3011_1002_GSA_0 = 0x0; //отключение кадра 3011/1002/«GSA» по каналу 0
	Msg->payload.msg3006.config.enable_3002_1003_GSV_0 = 0x0; //отключение кадра 3002/1003/«GSV» по каналу 0
	Msg->payload.msg3006.config.enable_3003_1012_RMC_0 = 0x0; //отключение кадра 3003/1012/«RMC» по каналу 0
	Msg->payload.msg3006.config.enable_3000_1000_GGA_1 = 0x0; //отключение кадра 3000/1000/«GGA» по каналу 1
	Msg->payload.msg3006.config.enable_3011_1002_GSA_1 = 0x0; //отключение кадра 3011/1002/«GSA» по каналу 1
	Msg->payload.msg3006.config.enable_3002_1003_GSV_1 = 0x0; //отключение кадра 3002/1003/«GSV» по каналу 1
	Msg->payload.msg3006.config.enable_3003_1012_RMC_1 = 0x0; //отключение кадра 3003/1012/«RMC» по каналу 1
	
	/*sprintf (buffer_TX_UART2, (char *)"size=%uwords\r\n", ((sizeof(Msg->payload.msg3006.config) + sizeof(Msg->payload.msg3006.command_code))/ 2));				
	UART2_PutString (buffer_TX_UART2);*/
	MNP_PutMessage (MNP_UART, Msg, MSG_3006, ((sizeof(Msg->payload.msg3006.config) + sizeof(Msg->payload.msg3006.command_code))/ 2)); //запись настроек в RAM приёмника
}

//-----------------------------------------------------------------------------------------------------//
void read_config_MNP ( MNP_MSG_t *Msg)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //1-источник - RAM приёмника
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //1-источник - flash приёмника
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x00; //0-чтение уставок, 1-запись уставок
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //6 32-разрядных слов конфигурации
	Msg->payload.msg3006.command_code.dummy = 0x00; //резерв
	
	MNP_PutMessageParse (MNP_UART, Msg, MSG_3006, (sizeof(Msg->payload.msg3006.command_code)/ 2)); //чтение настроек из RAM приёмника
	//sprintf (buffer_TX_UART2, (char *)"size=%ubyte, size=%uwords\r\n", sizeof(Msg->payload.msg3006.command_code), sizeof(Msg->payload.msg3006.command_code)/2);				
	//UART2_PutString (buffer_TX_UART2);
}

//-----------------------------------------------------------------------------------------------------//
void read_flash_MNP ( MNP_MSG_t *Msg)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x00; //1-источник - RAM приёмника
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x01; //1-источник - flash приёмника
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x00; //0-чтение уставок, 1-запись уставок
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //6 32-разрядных слов конфигурации
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.dummy = 0x00; //резерв
	
	MNP_PutMessage (MNP_UART, Msg, MSG_3006, (sizeof(Msg->payload.msg3006.command_code)/ 2)); //чтение настроек из flash приёмника
}

//-----------------------------------------------------------------------------------------------------//
void put_msg2000 (MNP_MSG_t *Msg)
{
	MNP_PutMessage (MNP_UART, Msg, MSG_2000, 0);
}

//-----------------------------------------------------------------------------------------------------//
uint8_t get_msg (MNP_MSG_t * msg, uint16_t byte_i)
{
	int8_t result = -1;
//	 byte_i = Parse_MNP_MSG (msg, byte_i);
	byte_i = Parse_MNP_MSG (msg);
//	return result;
	 return byte_i;
}

//--------------------------------------Парсер сообщение от МНП-М7--------------------------------------//
int8_t Parse_MNP_MSG (MNP_MSG_t * MSG)
{
	uint8_t byte; //полученный из кольцевого буффера байт
	static uint16_t byte_i; //счётчик принятых байтов тела сообщения
	int8_t ret = -1; //результат парсинга
	float tDOP = 0;
	
	MSG->msg_header.MNP_header.sync = 0x81FF;
	//MSG->rx_state = __SYNC_BYTE1; //стадия получения младшего байта синхрослова
	
	while ( RING_GetCount(&RING_buffer) > 0 )
//	do
	{		
		byte = RING_Pop(&RING_buffer); //получение из кольцевого буфера байта
		switch ( MSG->rx_state ) //проверка стадии получения сообщения
		{
			case __SYNC_BYTE1: //стадия получения младшего байта синхрослова
				if ( byte == (uint8_t)MSG->msg_header.header_bytes[0]) //если полученный байт равен 0xFF (младшая часть синхрослова)
					{MSG->rx_state = __SYNC_BYTE2;}//переход на стадию получения старшего байта синхрослова
				break;
				
			case __SYNC_BYTE2: //стадия получения старшего байта синхрослова
				if ( byte == (uint8_t)MSG->msg_header.header_bytes[1] ) //если полученный байт равен 0x81 (старшая часть синхрослова) 
					{MSG->rx_state = __TYPE_ID_BYTE1;} //переход на стадию получения младшего байта идентификатора типа кадра
				else 
				{
					MSG->rx_state = __SYNC_BYTE1; //переход на стадию получения младшего байта синхрослова
				} 					
				break;	
				
			case __TYPE_ID_BYTE1:  //стадия получения младшего байта идентификатора типа кадра
				MSG->rx_state = __TYPE_ID_BYTE2; //переход на стадию получения старшего байта идентификатора типа кадра
				MSG->msg_header.MNP_header.ID_B = byte; //сохранение младшего байта поля идентификатора типа кадра			
				break;
			
			case __TYPE_ID_BYTE2:  //стадия получения старшего байта идентификатора типа кадра
				MSG->rx_state = __LENGTH_BYTE1; //переход на стадию получения младшего байта байта поля длины сообщения
				MSG->msg_header.MNP_header.ID_A = byte; //сохранение старшего байта идентификатора типа кадра
				
			
				//sprintf (buffer_TX_UART2, (char *)"id=%x\r\n", MSG->msg_header.MNP_header.msg_id);	
			//	UART2_PutString (buffer_TX_UART2);
				break;
			
			case __LENGTH_BYTE1: //стадия получения младшего байта поля длины сообщения
				MSG->rx_state = __LENGTH_BYTE2; //переход на стадию получения старшего байта поля длины сообщения
				MSG->msg_header.MNP_header.length_B = byte; //сохранение младшего байта поля длины сообщения
				break;	
			
			case __LENGTH_BYTE2: //стадия получения старшего байта поля длины сообщения
				MSG->rx_state = __RESERVE_BYTE1;  //переход на стадию получения 1 байта резервного слова заголовка
				MSG->msg_header.MNP_header.length_A = byte; //сохранение старшего байта поля длины сообщения							
				byte_i = 0; //обнуление счётчика принятых байтов полезной части сообщения
			
				break;
			
			case __RESERVE_BYTE1: //стадия получения 1 байта резервного слова заголовка
				MSG->msg_header.MNP_header.dummy_B =  byte; 
				MSG->rx_state = __RESERVE_BYTE2;  //переход на стадию получения 2 байта резервного слова заголовка
				break;
			
			case __RESERVE_BYTE2: //стадия получения 2 байта резервного слова заголовка
				MSG->msg_header.MNP_header.dummy_A =  byte; 
				MSG->rx_state = __CK_HEAD_BYTE1;  //переход на стадию получения младшего байта КС заголовка
				break;
			
			case __CK_HEAD_BYTE1: //стадия получения младшего байта КС заголовка
				MSG->msg_header.MNP_header.chksum_B = byte; //охранение младшего байта КС заголовка
				MSG->rx_state = __CK_HEAD_BYTE2;  //переход на стадию получения старшего байта КС заголовка
				break;
			
			case __CK_HEAD_BYTE2: //стадия получения старшего байта КС заголовка
				MSG->msg_header.MNP_header.chksum_A = byte; //охранение старшего байта КС заголовка
				sprintf (buffer_TX_UART2, "get_byte=%u\r\n", RING_GetCount(&RING_buffer));
				UART2_PutString (buffer_TX_UART2);
				MSG->rx_state = __PAYLOAD;  //переход на стадию получения полезной части сообщения
				break;
			
			case __PAYLOAD: //стадия получения полезной части сообщения
				/*if (MSG->msg_header.MNP_header.msg_id == MSG_2200)
				{
					UART2_PutString ("get_msg2200\r\n");
					MSG->rx_state = __PARSER;
				} //переход на стадию парсинга сообщения
				else
				{*/
					if ( byte_i < 2*MSG->msg_header.MNP_header.data_size ) //если счётчик принятых байтов меньше полученной длины сообщения
					{
						MSG->payload.raw_bytes[byte_i] = byte; //сохранения принятого байта					
						byte_i++;
						if (byte_i >= BUFFER_SIZE)
							{byte_i = 0;}
					} 
					else //если полезная часть сообщения получена полностью
						{
							byte_i = 0;
							MSG->rx_state = __PARSER; //переход на стадию парсинга сообщения		
						} 
				//}		
				break;				
			
			case __PARSER: //стадия парсинга полученного сообщения
				MSG->rx_state = __SYNC_BYTE1;	//стадия получения 1 синхрослова	
				RING_Clear(&RING_buffer); 
				if ( MNP_CalcChkSum((uint16_t*)&MSG->msg_header, (sizeof(HEAD_MNP_MSG_t)-2)/2) == (MSG->msg_header.MNP_header.chksum)) // проверка контрольной суммы			
				{	
					switch ( MSG->msg_header.MNP_header.msg_id ) //проверка  идентификатора типа кадра
					{
						case MSG_3000:
							//sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x, crc=%x, dummy=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size,
						//	MSG->msg_header.MNP_header.chksum, MSG->msg_header.MNP_header.dummy);				
						//	UART2_PutString (buffer_TX_UART2);
						
							sprintf (buffer_TX_UART2, (char *)"hour=%d,min=%d,sec=%d\r\n", MSG->payload.msg3000.hour,  
							MSG->payload.msg3000.minute, MSG->payload.msg3000.second);
							UART2_PutString (buffer_TX_UART2);

						//	tDOP = sqrt ((pow(MSG->payload.msg3000.gDOP,2) - pow(MSG->payload.msg3000.pDOP,2)));
						//	sprintf (buffer_TX_UART2, (char *)"tDOP=%0.2f, pDOP=%0.2f, gDOP=%0.2f\r\n", tDOP, MSG->payload.msg3000.pDOP, MSG->payload.msg3000.gDOP);
						//	UART2_PutString (buffer_TX_UART2);
							ret = 1; //результат парсинга равен 1 
							break;
						
						case MSG_3001: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 2; //результат парсинга равен 2
							break;
						
						case MSG_3011: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 3; //результат парсинга равен 3
							break;
						
						case MSG_3002:
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 4; //результат парсинга равен 4
							break;
						
						case MSG_3003: 	
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 5; //результат парсинга равен 5
							break;
						
						case MSG_3006: 	
							sprintf (buffer_TX_UART2, (char *)"get msg3006, id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 6; //результат парсинга равен 6
							break;
						
						case MSG_2200: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x, dummy=%x\r\n", MSG->msg_header.MNP_header.msg_id, 
							MSG->msg_header.MNP_header.data_size, MSG->msg_header.MNP_header.dummy);				
							UART2_PutString (buffer_TX_UART2);
							ret = 7; //результат парсинга равен 7
							break;
					
						default:
							UART2_PutString ("parcing_error\r\n");
							break;	
						
					}
				}
				else
					{
						sprintf (buffer_TX_UART2, "header_CRC_error, %x!=%x\r\n", MNP_CalcChkSum((uint16_t*)&MSG->msg_header, (sizeof(HEAD_MNP_MSG_t)-2)/2), MSG->msg_header.MNP_header.chksum);
						UART2_PutString (buffer_TX_UART2);
					}
				break;				
				
			default:
			break;			
		}
//		byte = 0;	//обнуление принятого байта	
	} 
//	while ( RING_GetCount(&RING_buffer) > 0 ); //пока в кольцевом буфере счётчик принятых байтов больше счётчика обработанных байтов
//	RING_Clear(&RING_buffer); //сброс обоих счётчиков, очистка приёмного буффера
	return ret; //возврат результата парсинга 
//	return byte_i;
}

//-------------------------------------------Инициализация МНП-М7-------------------------------------------//
void GPS_Init(MNP_MSG_t *Msg)
{
	MNP_M7_init (Msg);		
	/*HAL_Delay (1000);
	
	read_config_MNP (&MNP_PUT_MSG);
	HAL_Delay (1000);*/

	//xTimerGPSUARTTimeout = xTimer_Create(5000, DISABLE, &vTimerGPSUARTTimeoutCallback, ENABLE); // таймаут на отсутствие обмена по UART с GPS приемником
}

//--------------------------------------аппаратная перезагрузка приемника--------------------------------------//
static void GPS_RST(FunctionalState NewState)
{
	/*if ( NewState != DISABLE ) 
		{PORT_ResetBits(GPS_PORT_nRST, GPS_PIN_nRST);} 
	else 
		{PORT_SetBits(GPS_PORT_nRST, GPS_PIN_nRST);}*/
}

//------------------перезагрузка и настройка GPS приемника------------------//
void MNP_Reset(MNP_MSG_t *Msg)
{
	MNP_MakeMessage_Reset(Msg);
	MNP_PutMessage(MNP_UART, Msg, 3006, (sizeof(Msg->payload.msg3006.command_code)+sizeof(Msg->payload.msg3006.reset_mask))/ 2);
//	MNP_M7_CFG.GPS_RST(ENABLE); //включить аппаратную перезагрузку приёмника
}


//---------------------------------------------------------------------------
static void MNP_MakeMessage_Reset(MNP_MSG_t *Msg)
{
	memset ((void *)&Msg->payload.msg3006, 0, (sizeof(Msg->payload.msg3006.command_code)+sizeof(Msg->payload.msg3006.reset_mask)));

	Msg->payload.msg3006.command_code.specialcmd_3006.special_code = SPECIAL_CMD_CODE_RESET;
	Msg->payload.msg3006.command_code.specialcmd_3006.id = SPECIAL_CMD_ID; 
}
