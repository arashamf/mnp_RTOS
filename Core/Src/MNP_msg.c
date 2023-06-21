
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
MNP_M7_CFG_t MNP_M7_CFG = //������������ ������� ��������� ������������ � ��������� ��������
{
	.GPS_RST = &GPS_RST, //�-� ���������� ������������ ��������
	.rst_delay = GPS_RST_DELAY, //�������� ��� ������������ ��������
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
	Msg->msg_header.MNP_header.msg_id = MsgId; //id MNP-��������� 
	Msg->msg_header.MNP_header.data_size = WordsCount; //���������� 16 ������ ���� � ���� ���������
	Msg->msg_header.MNP_header.dummy = 0;
	Msg->msg_header.MNP_header.chksum = MNP_CalcChkSum((uint16_t *)&Msg->msg_header.MNP_header, (sizeof(HEAD_MNP_MSG_t)-2)/2);
	
	Msg->payload.raw_words[WordsCount] = MNP_CalcChkSum((uint16_t *)&Msg->payload, WordsCount); //���������� ��
	
	MNP_UART_Puts(MNP_UART, (uint8_t *)Msg, (sizeof(HEAD_MNP_MSG_t) + (WordsCount+1)*2)); //���������� ������������ �������� = ���������� ���� � ��������� + ���������� ������������ ���� � ����� ��������� + 2 ����� ��
}

//-----------------------------------------------------------------------------------------------------//
static void MNP_PutMessageParse (USART_TypeDef *USARTx, MNP_MSG_t *Msg, uint16_t MsgId, uint16_t WordsCount)
{
	
	Msg->msg_header.MNP_header.sync = MNP_SYNC_CHAR;
	Msg->msg_header.MNP_header.msg_id = MsgId; //id MNP-��������� 
	Msg->msg_header.MNP_header.data_size = WordsCount; //���������� 16 ������ ���� � ���� ���������
	Msg->msg_header.MNP_header.dummy = 0;
	Msg->msg_header.MNP_header.chksum = MNP_CalcChkSum((uint16_t *)&Msg->msg_header.MNP_header, (sizeof(HEAD_MNP_MSG_t)-2)/2);
	
	Msg->payload.raw_words[WordsCount] = MNP_CalcChkSum((uint16_t *)&Msg->payload, WordsCount); //���������� ��
	
	MNP_UART_Puts(MNP_UART, (uint8_t *)Msg, (sizeof(HEAD_MNP_MSG_t) + (WordsCount+1)*2)); //���������� ������������ �������� = ���������� ���� � ��������� + ���������� ������������ ���� � ����� ��������� + 2 ����� ��
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
	
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x01; //������ �������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //6 32-��������� ���� ������������
	
	Msg->payload.msg3006.command_code.dummy = 0x0; //������
	
	Msg->payload.msg3006.config.protocol0 = 0x1; //�������� ������ 0 - MNP_binary
	Msg->payload.msg3006.config.baud_divider0 = 460800 / 115200; //������������ UART0=4, �������� 115200 
	Msg->payload.msg3006.config.protocol1 = 0x1; //�������� ������ 1 - MNP_binary
	Msg->payload.msg3006.config.baud_divider1 = 460800 / 115200; //������������ UART1=4, �������� 115200 
	Msg->payload.msg3006.config.tropo_corr = 0x1; //��������� ������������� ������ ����������
	Msg->payload.msg3006.config.use_difc = 0x1; //��������� ������������� ���������������� ��������
	Msg->payload.msg3006.config.dif_only = 0x0; //���������� ��������������� ������������� ����������������� ������
	Msg->payload.msg3006.config.sol_smooth = 0x0; //���������� �������� ��������� ��� �������� ����� 1 �/c
	Msg->payload.msg3006.config.sol_filter = 0x1; //��������� ����������� �������
	Msg->payload.msg3006.config.meas_filter = 0x1; //��������� ����������� ���������� �� ���� � ���� �������
	Msg->payload.msg3006.config.iono_corr = 0x1; // ���������� ����������� ���������
	Msg->payload.msg3006.config.disable_2D = 0x0; //���������� ������� ��������� ���������
	Msg->payload.msg3006.config.use_RAIM = 0x1; //��������� ��������� RAIM
	Msg->payload.msg3006.config.enable_warm_startup = 0x1; //��������� �������� ��������� ������
	Msg->payload.msg3006.config.sys_time = 0x0; // �������� ��������� ����� � ������� ������������� ������� 
	Msg->payload.msg3006.config.glo_time = 0x0; // ��������� ����� ������� / UTC(SU)
	Msg->payload.msg3006.config.shift_meas = 0x0; // �������� ��������� � ��������� �����
	Msg->payload.msg3006.config.enable_SBAS = 0x0; //���������� SBAS
	Msg->payload.msg3006.config.enable_iono_SBAS = 0x0; //���������� ������ ��������� SBAS
	Msg->payload.msg3006.config.GPS_compatibility = 0x0; //����� ������������� � ����������� GPS
	Msg->payload.msg3006.config.wr_alms = 0x1; //��������� ���������� � flash ���������� 
	Msg->payload.msg3006.config.wr_ephs = 0x1; //��������� ���������� � flash �������� 
	Msg->payload.msg3006.config.wr_ionoutc = 0x1; //��������� ���������� � flash ������ UTC GPS
	Msg->payload.msg3006.config.wr_coords = 0x1; //��������� ���������� � flash ���������
	Msg->payload.msg3006.config.enable_3000_1000_GGA_0 = 0x0; //��������� ����� 3000/1000/�GGA� �� ������ 0
	Msg->payload.msg3006.config.enable_3011_1002_GSA_0 = 0x0; //���������� ����� 3011/1002/�GSA� �� ������ 0
	Msg->payload.msg3006.config.enable_3002_1003_GSV_0 = 0x0; //���������� ����� 3002/1003/�GSV� �� ������ 0
	Msg->payload.msg3006.config.enable_3003_1012_RMC_0 = 0x0; //���������� ����� 3003/1012/�RMC� �� ������ 0
	Msg->payload.msg3006.config.enable_3000_1000_GGA_1 = 0x0; //���������� ����� 3000/1000/�GGA� �� ������ 1
	Msg->payload.msg3006.config.enable_3011_1002_GSA_1 = 0x0; //���������� ����� 3011/1002/�GSA� �� ������ 1
	Msg->payload.msg3006.config.enable_3002_1003_GSV_1 = 0x0; //���������� ����� 3002/1003/�GSV� �� ������ 1
	Msg->payload.msg3006.config.enable_3003_1012_RMC_1 = 0x0; //���������� ����� 3003/1012/�RMC� �� ������ 1
	
	/*sprintf (buffer_TX_UART2, (char *)"size=%uwords\r\n", ((sizeof(Msg->payload.msg3006.config) + sizeof(Msg->payload.msg3006.command_code))/ 2));				
	UART2_PutString (buffer_TX_UART2);*/
	MNP_PutMessage (MNP_UART, Msg, MSG_3006, ((sizeof(Msg->payload.msg3006.config) + sizeof(Msg->payload.msg3006.command_code))/ 2)); //������ �������� � RAM ��������
}

//-----------------------------------------------------------------------------------------------------//
void read_config_MNP ( MNP_MSG_t *Msg)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //1-�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //1-�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x00; //0-������ �������, 1-������ �������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //6 32-��������� ���� ������������
	Msg->payload.msg3006.command_code.dummy = 0x00; //������
	
	MNP_PutMessageParse (MNP_UART, Msg, MSG_3006, (sizeof(Msg->payload.msg3006.command_code)/ 2)); //������ �������� �� RAM ��������
	//sprintf (buffer_TX_UART2, (char *)"size=%ubyte, size=%uwords\r\n", sizeof(Msg->payload.msg3006.command_code), sizeof(Msg->payload.msg3006.command_code)/2);				
	//UART2_PutString (buffer_TX_UART2);
}

//-----------------------------------------------------------------------------------------------------//
void read_flash_MNP ( MNP_MSG_t *Msg)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x00; //1-�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x01; //1-�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x00; //0-������ �������, 1-������ �������
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //6 32-��������� ���� ������������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.dummy = 0x00; //������
	
	MNP_PutMessage (MNP_UART, Msg, MSG_3006, (sizeof(Msg->payload.msg3006.command_code)/ 2)); //������ �������� �� flash ��������
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

//--------------------------------------������ ��������� �� ���-�7--------------------------------------//
int8_t Parse_MNP_MSG (MNP_MSG_t * MSG)
{
	uint8_t byte; //���������� �� ���������� ������� ����
	static uint16_t byte_i; //������� �������� ������ ���� ���������
	int8_t ret = -1; //��������� ��������
	float tDOP = 0;
	
	MSG->msg_header.MNP_header.sync = 0x81FF;
	//MSG->rx_state = __SYNC_BYTE1; //������ ��������� �������� ����� �����������
	
	while ( RING_GetCount(&RING_buffer) > 0 )
//	do
	{		
		byte = RING_Pop(&RING_buffer); //��������� �� ���������� ������ �����
		switch ( MSG->rx_state ) //�������� ������ ��������� ���������
		{
			case __SYNC_BYTE1: //������ ��������� �������� ����� �����������
				if ( byte == (uint8_t)MSG->msg_header.header_bytes[0]) //���� ���������� ���� ����� 0xFF (������� ����� �����������)
					{MSG->rx_state = __SYNC_BYTE2;}//������� �� ������ ��������� �������� ����� �����������
				break;
				
			case __SYNC_BYTE2: //������ ��������� �������� ����� �����������
				if ( byte == (uint8_t)MSG->msg_header.header_bytes[1] ) //���� ���������� ���� ����� 0x81 (������� ����� �����������) 
					{MSG->rx_state = __TYPE_ID_BYTE1;} //������� �� ������ ��������� �������� ����� �������������� ���� �����
				else 
				{
					MSG->rx_state = __SYNC_BYTE1; //������� �� ������ ��������� �������� ����� �����������
				} 					
				break;	
				
			case __TYPE_ID_BYTE1:  //������ ��������� �������� ����� �������������� ���� �����
				MSG->rx_state = __TYPE_ID_BYTE2; //������� �� ������ ��������� �������� ����� �������������� ���� �����
				MSG->msg_header.MNP_header.ID_B = byte; //���������� �������� ����� ���� �������������� ���� �����			
				break;
			
			case __TYPE_ID_BYTE2:  //������ ��������� �������� ����� �������������� ���� �����
				MSG->rx_state = __LENGTH_BYTE1; //������� �� ������ ��������� �������� ����� ����� ���� ����� ���������
				MSG->msg_header.MNP_header.ID_A = byte; //���������� �������� ����� �������������� ���� �����
				
			
				//sprintf (buffer_TX_UART2, (char *)"id=%x\r\n", MSG->msg_header.MNP_header.msg_id);	
			//	UART2_PutString (buffer_TX_UART2);
				break;
			
			case __LENGTH_BYTE1: //������ ��������� �������� ����� ���� ����� ���������
				MSG->rx_state = __LENGTH_BYTE2; //������� �� ������ ��������� �������� ����� ���� ����� ���������
				MSG->msg_header.MNP_header.length_B = byte; //���������� �������� ����� ���� ����� ���������
				break;	
			
			case __LENGTH_BYTE2: //������ ��������� �������� ����� ���� ����� ���������
				MSG->rx_state = __RESERVE_BYTE1;  //������� �� ������ ��������� 1 ����� ���������� ����� ���������
				MSG->msg_header.MNP_header.length_A = byte; //���������� �������� ����� ���� ����� ���������							
				byte_i = 0; //��������� �������� �������� ������ �������� ����� ���������
			
				break;
			
			case __RESERVE_BYTE1: //������ ��������� 1 ����� ���������� ����� ���������
				MSG->msg_header.MNP_header.dummy_B =  byte; 
				MSG->rx_state = __RESERVE_BYTE2;  //������� �� ������ ��������� 2 ����� ���������� ����� ���������
				break;
			
			case __RESERVE_BYTE2: //������ ��������� 2 ����� ���������� ����� ���������
				MSG->msg_header.MNP_header.dummy_A =  byte; 
				MSG->rx_state = __CK_HEAD_BYTE1;  //������� �� ������ ��������� �������� ����� �� ���������
				break;
			
			case __CK_HEAD_BYTE1: //������ ��������� �������� ����� �� ���������
				MSG->msg_header.MNP_header.chksum_B = byte; //��������� �������� ����� �� ���������
				MSG->rx_state = __CK_HEAD_BYTE2;  //������� �� ������ ��������� �������� ����� �� ���������
				break;
			
			case __CK_HEAD_BYTE2: //������ ��������� �������� ����� �� ���������
				MSG->msg_header.MNP_header.chksum_A = byte; //��������� �������� ����� �� ���������
				sprintf (buffer_TX_UART2, "get_byte=%u\r\n", RING_GetCount(&RING_buffer));
				UART2_PutString (buffer_TX_UART2);
				MSG->rx_state = __PAYLOAD;  //������� �� ������ ��������� �������� ����� ���������
				break;
			
			case __PAYLOAD: //������ ��������� �������� ����� ���������
				/*if (MSG->msg_header.MNP_header.msg_id == MSG_2200)
				{
					UART2_PutString ("get_msg2200\r\n");
					MSG->rx_state = __PARSER;
				} //������� �� ������ �������� ���������
				else
				{*/
					if ( byte_i < 2*MSG->msg_header.MNP_header.data_size ) //���� ������� �������� ������ ������ ���������� ����� ���������
					{
						MSG->payload.raw_bytes[byte_i] = byte; //���������� ��������� �����					
						byte_i++;
						if (byte_i >= BUFFER_SIZE)
							{byte_i = 0;}
					} 
					else //���� �������� ����� ��������� �������� ���������
						{
							byte_i = 0;
							MSG->rx_state = __PARSER; //������� �� ������ �������� ���������		
						} 
				//}		
				break;				
			
			case __PARSER: //������ �������� ����������� ���������
				MSG->rx_state = __SYNC_BYTE1;	//������ ��������� 1 �����������	
				RING_Clear(&RING_buffer); 
				if ( MNP_CalcChkSum((uint16_t*)&MSG->msg_header, (sizeof(HEAD_MNP_MSG_t)-2)/2) == (MSG->msg_header.MNP_header.chksum)) // �������� ����������� �����			
				{	
					switch ( MSG->msg_header.MNP_header.msg_id ) //��������  �������������� ���� �����
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
							ret = 1; //��������� �������� ����� 1 
							break;
						
						case MSG_3001: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 2; //��������� �������� ����� 2
							break;
						
						case MSG_3011: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 3; //��������� �������� ����� 3
							break;
						
						case MSG_3002:
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 4; //��������� �������� ����� 4
							break;
						
						case MSG_3003: 	
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 5; //��������� �������� ����� 5
							break;
						
						case MSG_3006: 	
							sprintf (buffer_TX_UART2, (char *)"get msg3006, id=%x, data_size=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 6; //��������� �������� ����� 6
							break;
						
						case MSG_2200: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x, dummy=%x\r\n", MSG->msg_header.MNP_header.msg_id, 
							MSG->msg_header.MNP_header.data_size, MSG->msg_header.MNP_header.dummy);				
							UART2_PutString (buffer_TX_UART2);
							ret = 7; //��������� �������� ����� 7
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
//		byte = 0;	//��������� ��������� �����	
	} 
//	while ( RING_GetCount(&RING_buffer) > 0 ); //���� � ��������� ������ ������� �������� ������ ������ �������� ������������ ������
//	RING_Clear(&RING_buffer); //����� ����� ���������, ������� �������� �������
	return ret; //������� ���������� �������� 
//	return byte_i;
}

//-------------------------------------------������������� ���-�7-------------------------------------------//
void GPS_Init(MNP_MSG_t *Msg)
{
	MNP_M7_init (Msg);		
	/*HAL_Delay (1000);
	
	read_config_MNP (&MNP_PUT_MSG);
	HAL_Delay (1000);*/

	//xTimerGPSUARTTimeout = xTimer_Create(5000, DISABLE, &vTimerGPSUARTTimeoutCallback, ENABLE); // ������� �� ���������� ������ �� UART � GPS ����������
}

//--------------------------------------���������� ������������ ���������--------------------------------------//
static void GPS_RST(FunctionalState NewState)
{
	/*if ( NewState != DISABLE ) 
		{PORT_ResetBits(GPS_PORT_nRST, GPS_PIN_nRST);} 
	else 
		{PORT_SetBits(GPS_PORT_nRST, GPS_PIN_nRST);}*/
}

//------------------������������ � ��������� GPS ���������------------------//
void MNP_Reset(MNP_MSG_t *Msg)
{
	MNP_MakeMessage_Reset(Msg);
	MNP_PutMessage(MNP_UART, Msg, 3006, (sizeof(Msg->payload.msg3006.command_code)+sizeof(Msg->payload.msg3006.reset_mask))/ 2);
//	MNP_M7_CFG.GPS_RST(ENABLE); //�������� ���������� ������������ ��������
}


//---------------------------------------------------------------------------
static void MNP_MakeMessage_Reset(MNP_MSG_t *Msg)
{
	memset ((void *)&Msg->payload.msg3006, 0, (sizeof(Msg->payload.msg3006.command_code)+sizeof(Msg->payload.msg3006.reset_mask)));

	Msg->payload.msg3006.command_code.specialcmd_3006.special_code = SPECIAL_CMD_CODE_RESET;
	Msg->payload.msg3006.command_code.specialcmd_3006.id = SPECIAL_CMD_ID; 
}