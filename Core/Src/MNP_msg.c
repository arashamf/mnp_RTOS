
/* Includes ------------------------------------------------------------------*/
#include "MNP_msg.h"
#include "usart.h"
#include "typedef.h"
#include "protocol.h"
#include "pins.h"
#include "time64.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
//Private defines -------------------------------------------------------------------------------------//
//Constants -------------------------------------------------------------------------------------------//
//Private variables -----------------------------------------------------------------------------------//
MKS2_t MKS2 = //������������� ��������� ���� MKS2_t
{
	.tmContext.Time2k = 0, //���������� ������ � 01.01.2000
	.tmContext.TAI_UTC_offset = 0, //������� ����� ������� �������� � �������� UTC
	.tmContext.LeapS_59 = 0,									//���������� ������� 
	.tmContext.LeapS_61 = 0,									//���������� �������
	.tmContext.ValidTHRESHOLD = 0,						//���������� ������������� ������		
	.tmContext.Valid = 0,                  		//����� ����� ������������� ������		
	.tmContext.Max_gDOP = DEFAULT_MAX_gDOP, 	//��������� ����������� ����������� �������� gDOP
		
	.canContext.ID = MODULE_TYPE_MKNS, 					//������������� ���� ������
	.canContext.GetAddr = &Get_Module_Address, //������������� ��������� �� ������� ��������� ������ � �����-�����
//	.canContext.Addr = Get_Module_Address(),
	
	.fContext.Fail = 0, //��� ����� ���������� �������������� ��������
};


//-----------------------------------------------------------------------------------------------------//
static uint16_t MNP_CalcChkSum	(uint16_t *Array, int WordsCount)
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
void MNP_PutMessage (MNP_MSG_t *Msg, uint16_t MsgId, uint16_t WordsCount)
{
	
	Msg->msg_header.MNP_header.sync = MNP_SYNC_CHAR;
	Msg->msg_header.MNP_header.msg_id = MsgId; //id MNP-��������� 
	Msg->msg_header.MNP_header.data_size = WordsCount; //���������� 16 ������ ���� � ���� ���������
	Msg->msg_header.MNP_header.dummy = 0;
	Msg->msg_header.MNP_header.chksum = MNP_CalcChkSum((uint16_t *)&Msg->msg_header.MNP_header, (sizeof(HEAD_MNP_MSG_t)-2)/2);
	
	Msg->payload.raw_words[WordsCount] = MNP_CalcChkSum((uint16_t *)&Msg->payload, WordsCount); //���������� ��
	
	MNP_UART_MSG_Puts (Msg->msg_header.header_bytes, sizeof(HEAD_MNP_MSG_t));
	MNP_UART_MSG_Puts (Msg->payload.raw_bytes, (WordsCount+1)*2);
	
}


//-----------------------------------------------------------------------------------------------------//
static void MNP_M7_init ( MNP_MSG_t *Msg)
{
	
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x01; //������ �������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //�������� 6 32-��������� ���� � ��������� � ������������ ��������
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
	Msg->payload.msg3006.config.enable_warm_startup = 0x0; //��������� �������� ��������� ������
	Msg->payload.msg3006.config.sys_time = 0x1; // �������� ��������� ����� � ������� ������������� ������� 
	Msg->payload.msg3006.config.glo_time = 0x0; // ��������� ����� ������� / UTC(SU)
	Msg->payload.msg3006.config.shift_meas = 0x1; // �������� ��������� � ��������� �����
	Msg->payload.msg3006.config.enable_SBAS = 0x0; //���������� SBAS
	Msg->payload.msg3006.config.enable_iono_SBAS = 0x0; //���������� ������ ��������� SBAS
	Msg->payload.msg3006.config.GPS_compatibility = 0x0; //����� ������������� � ����������� GPS
	Msg->payload.msg3006.config.wr_alms = 0x1; //��������� ���������� � flash ���������� 
	Msg->payload.msg3006.config.wr_ephs = 0x1; //��������� ���������� � flash �������� 
	Msg->payload.msg3006.config.wr_ionoutc = 0x1; //��������� ���������� � flash ������ UTC GPS
	Msg->payload.msg3006.config.wr_coords = 0x1; //��������� ���������� � flash ���������
	Msg->payload.msg3006.config.enable_3000_1000_GGA_0 = 0x1; //��������� ����� 3000/1000/�GGA� �� ������ 0
	Msg->payload.msg3006.config.enable_3011_1002_GSA_0 = 0x0; //���������� ����� 3011/1002/�GSA� �� ������ 0
	Msg->payload.msg3006.config.enable_3002_1003_GSV_0 = 0x0; //���������� ����� 3002/1003/�GSV� �� ������ 0
	Msg->payload.msg3006.config.enable_3003_1012_RMC_0 = 0x0; //���������� ����� 3003/1012/�RMC� �� ������ 0
	Msg->payload.msg3006.config.enable_3000_1000_GGA_1 = 0x1; //���������� ����� 3000/1000/�GGA� �� ������ 1
	Msg->payload.msg3006.config.enable_3011_1002_GSA_1 = 0x0; //���������� ����� 3011/1002/�GSA� �� ������ 1
	Msg->payload.msg3006.config.enable_3002_1003_GSV_1 = 0x0; //���������� ����� 3002/1003/�GSV� �� ������ 1
	Msg->payload.msg3006.config.enable_3003_1012_RMC_1 = 0x0; //���������� ����� 3003/1012/�RMC� �� ������ 1
	
	MNP_PutMessage (Msg, MSG_3006, ((sizeof(Msg->payload.msg3006.config) + sizeof(Msg->payload.msg3006.command_code))/2)); //������ �������� � RAM ��������
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
	
	MNP_PutMessage (Msg, MSG_3006, (sizeof(Msg->payload.msg3006.command_code)/ 2)); //������ �������� �� RAM ��������
}

//-----------------------------------------------------------------------------------------------------//
void read_flash_MNP ( MNP_MSG_t *Msg)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x00; //1-�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x01; //1-�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x00; //0-������ �������, 1-������ �������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x02; //6 32-��������� ���� ������������
	Msg->payload.msg3006.command_code.dummy = 0x00; //������
	
	MNP_PutMessage (Msg, MSG_3006, (sizeof(Msg->payload.msg3006.command_code)/ 2)); //������ �������� �� flash ��������
}

//-----------------------------------------------------------------------------------------------------//
void put_msg2000 (MNP_MSG_t *Msg)
{
	MNP_PutMessage (Msg, MSG_2000, 0);
}

//-----------------------------------------------------------------------------------------------------//
void Set_GNSS_interval (MNP_MSG_t *Msg, uint32_t inerval)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x01; //������ �������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;	
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x07; //��������� ����� ��������� ���������
	Msg->payload.msg3006.command_code.dummy = 0x00; //������
	
	Msg->payload.msg3006.interval = inerval; //����� ��������� ���������, 2000 ������������� 1�
	
	MNP_PutMessage (Msg, MSG_3006, (sizeof(Msg->payload.msg3006.interval) + sizeof(Msg->payload.msg3006.command_code))/2);
/*	sprintf (buffer_TX_UART2, (char *)"put_MsgId=%u, byte=%u\r\n", Msg->msg_header.MNP_header.msg_id, 
	(sizeof(Msg->payload.msg3006.interval) + sizeof(Msg->payload.msg3006.command_code)));
	UART2_PutString (buffer_TX_UART2);*/
}

//-----------------------------------------------------------------------------------------------------//
void Get_GNSS_interval (MNP_MSG_t *Msg, uint32_t inerval)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x00; //������ �������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;	
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x07; //��������� ����� ��������� ���������
	Msg->payload.msg3006.command_code.dummy = 0x00; //������
	
	Msg->payload.msg3006.interval = inerval; //����� ��������� ���������, 2000 ������������� 1�
	
	MNP_PutMessage (Msg, MSG_3006, (sizeof(Msg->payload.msg3006.interval) + sizeof(Msg->payload.msg3006.command_code))/2);
/*	sprintf (buffer_TX_UART2, (char *)"put_MsgId=%u, byte=%u\r\n", Msg->msg_header.MNP_header.msg_id, 
	(sizeof(Msg->payload.msg3006.interval) + sizeof(Msg->payload.msg3006.command_code)));
	UART2_PutString (buffer_TX_UART2);*/
}

//-----------------------------------------------------------------------------------------------------//
void Read_SN (MNP_MSG_t *Msg)
{
	Msg->payload.msg3006.command_code.cmd_3006.RAM = 0x01; //�������� - RAM ��������
	Msg->payload.msg3006.command_code.cmd_3006.flash = 0x00; //�������� - flash ��������
	Msg->payload.msg3006.command_code.cmd_3006.write = 0x00; //������ �������
	Msg->payload.msg3006.command_code.cmd_3006.dummy1 = 0x00;
	Msg->payload.msg3006.command_code.cmd_3006.dummy2 = 0x00;	
	Msg->payload.msg3006.command_code.cmd_3006.code = 0x16; //������ ��������� ������ ��������
	Msg->payload.msg3006.command_code.dummy = 0x00; //������
	
	
	MNP_PutMessage (Msg, MSG_3006, ((sizeof(Msg->payload.msg3006.command_code))/2));
}

//-------------------------------------------������������� ���-�7-------------------------------------------//
void GPS_Init(MNP_MSG_t *Msg)
{
	MNP_M7_init (Msg);		
}

//--------------------------------------���������� ������������ ���������--------------------------------------//
static void GPS_RST(FunctionalState NewState)
{
	if ( NewState != DISABLE ) {}
		//{PORT_ResetBits(GPS_PORT_nRST, GPS_PIN_nRST);} 
	else {}
	//	{PORT_SetBits(GPS_PORT_nRST, GPS_PIN_nRST);}
}

//-----------------------------------��������� ������������ GPS ���������-----------------------------------//
void MNP_Reset(MNP_MSG_t *Msg)
{
	memset ((void *)&Msg->payload.msg3006, 0, (sizeof(Msg->payload.msg3006.command_code)+sizeof(Msg->payload.msg3006.reset_mask)));

	Msg->payload.msg3006.command_code.specialcmd_3006.special_code = SPECIAL_CMD_CODE_RESET;
	Msg->payload.msg3006.command_code.specialcmd_3006.id = SPECIAL_CMD_ID; 
	MNP_PutMessage (Msg, MSG_3006, (sizeof(Msg->payload.msg3006.command_code)+sizeof(Msg->payload.msg3006.reset_mask))/ 2);
}

//--------------------------------------������ ��������� �� ���-�7--------------------------------------//
static int8_t Parse_MNP_MSG (MNP_MSG_t * Msg)
{
	uint8_t byte; //���������� �� ���������� ������� ����
	static uint16_t byte_i; //������� �������� ������ ���� ���������
	int8_t ret = -1; //��������� ��������
//	float tDOP = 0;
	
	Msg->msg_header.MNP_header.sync = MNP_SYNC_CHAR;
	
	while (RING_GetCount(&RING_buffer) > 0)
//	do
	{		
		byte = RING_Pop(&RING_buffer); //��������� �� ���������� ������ �����
		switch ( Msg->rx_state ) //�������� ������ ��������� ���������
		{
			case __SYNC_BYTE1: //������ ��������� �������� ����� �����������
				if ( byte == (uint8_t)Msg->msg_header.header_bytes[0]) //���� ���������� ���� ����� 0xFF (������� ����� �����������)
					{Msg->rx_state = __SYNC_BYTE2;}//������� �� ������ ��������� �������� ����� �����������
				break;
				
			case __SYNC_BYTE2: //������ ��������� �������� ����� �����������
				if ( byte == (uint8_t)Msg->msg_header.header_bytes[1] ) //���� ���������� ���� ����� 0x81 (������� ����� �����������) 
					{Msg->rx_state = __TYPE_ID_BYTE1;} //������� �� ������ ��������� �������� ����� �������������� ���� �����
				else 
				{
					Msg->rx_state = __SYNC_BYTE1; //������� �� ������ ��������� �������� ����� �����������
				} 					
				break;	
				
			case __TYPE_ID_BYTE1:  //������ ��������� �������� ����� �������������� ���� �����
				Msg->rx_state = __TYPE_ID_BYTE2; //������� �� ������ ��������� �������� ����� �������������� ���� �����
				Msg->msg_header.MNP_header.ID_B = byte; //���������� �������� ����� ���� �������������� ���� �����			
				break;
			
			case __TYPE_ID_BYTE2:  //������ ��������� �������� ����� �������������� ���� �����
				Msg->rx_state = __LENGTH_BYTE1; //������� �� ������ ��������� �������� ����� ����� ���� ����� ���������
				Msg->msg_header.MNP_header.ID_A = byte; //���������� �������� ����� �������������� ���� �����
				break;
			
			case __LENGTH_BYTE1: //������ ��������� �������� ����� ���� ����� ���������
				Msg->rx_state = __LENGTH_BYTE2; //������� �� ������ ��������� �������� ����� ���� ����� ���������
				Msg->msg_header.MNP_header.length_B = byte; //���������� �������� ����� ���� ����� ���������
				break;	
			
			case __LENGTH_BYTE2: //������ ��������� �������� ����� ���� ����� ���������
				Msg->rx_state = __RESERVE_BYTE1;  //������� �� ������ ��������� 1 ����� ���������� ����� ���������
				Msg->msg_header.MNP_header.length_A = byte; //���������� �������� ����� ���� ����� ���������							
				byte_i = 0; //��������� �������� �������� ������ �������� ����� ���������
			
				break;
			
			case __RESERVE_BYTE1: //������ ��������� 1 ����� ���������� ����� ���������
				Msg->msg_header.MNP_header.dummy_B =  byte; 
				Msg->rx_state = __RESERVE_BYTE2;  //������� �� ������ ��������� 2 ����� ���������� ����� ���������
				break;
			
			case __RESERVE_BYTE2: //������ ��������� 2 ����� ���������� ����� ���������
				Msg->msg_header.MNP_header.dummy_A =  byte; 
				Msg->rx_state = __CK_HEAD_BYTE1;  //������� �� ������ ��������� �������� ����� �� ���������
				break;
			
			case __CK_HEAD_BYTE1: //������ ��������� �������� ����� �� ���������
				Msg->msg_header.MNP_header.chksum_B = byte; //��������� �������� ����� �� ���������
				Msg->rx_state = __CK_HEAD_BYTE2;  //������� �� ������ ��������� �������� ����� �� ���������
				break;
			
			case __CK_HEAD_BYTE2: //������ ��������� �������� ����� �� ���������
				Msg->msg_header.MNP_header.chksum_A = byte; //��������� �������� ����� �� ���������
			//	sprintf (buffer_TX_UART2, "get_byte=%u\r\n", RING_GetCount(&RING_buffer));
			//	UART2_PutString (buffer_TX_UART2);
				Msg->rx_state = __PAYLOAD;  //������� �� ������ ��������� �������� ����� ���������
				break;
			
			case __PAYLOAD: //������ ��������� �������� ����� ���������
				if ( byte_i < 2*Msg->msg_header.MNP_header.data_size ) //���� ������� �������� ������ ������ ���������� ����� ���������
				{
					Msg->payload.raw_bytes[byte_i] = byte; //���������� ��������� �����					
					byte_i++;
					if (byte_i >= BUFFER_SIZE)
						{byte_i = 0;}
				} 
				else //���� �������� ����� ��������� �������� ���������
				{
					byte_i = 0;
					Msg->rx_state = __PARSER; //������� �� ������ �������� ���������		
				} 
				break;				
			
			case __PARSER: //������ �������� ����������� ���������
				Msg->rx_state = __SYNC_BYTE1;	//������ ��������� 1 �����������	
				if ( MNP_CalcChkSum((uint16_t*)&Msg->msg_header, (sizeof(HEAD_MNP_MSG_t)-2)/2) == (Msg->msg_header.MNP_header.chksum)) // �������� ����������� �����			
				{	
					switch ( Msg->msg_header.MNP_header.msg_id ) //��������  �������������� ���� �����
					{
						case MSG_3000:
						//	sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x, crc=%x, dummy=%x\r\n", MSG->msg_header.MNP_header.msg_id, MSG->msg_header.MNP_header.data_size,
						//	MSG->msg_header.MNP_header.chksum, MSG->msg_header.MNP_header.dummy);				
						//	UART2_PutString (buffer_TX_UART2);
						
							sprintf (buffer_TX_UART2, (char *)"hour=%d,min=%d,sec=%d\r\n", Msg->payload.msg3000.hour,  
							Msg->payload.msg3000.minute, Msg->payload.msg3000.second);
							UART2_PutString (buffer_TX_UART2);
							ret = 1; //��������� �������� ����� 1 
							break;
						
						case MSG_3001: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", Msg->msg_header.MNP_header.msg_id, Msg->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 2; //��������� �������� ����� 2
							break;
						
						case MSG_3011: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", Msg->msg_header.MNP_header.msg_id, Msg->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 3; //��������� �������� ����� 3
							break;
						
						case MSG_3002:
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", Msg->msg_header.MNP_header.msg_id, Msg->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 4; //��������� �������� ����� 4
							break;
						
						case MSG_3003: 	
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x\r\n", Msg->msg_header.MNP_header.msg_id, Msg->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							ret = 5; //��������� �������� ����� 5
							break;
						
						case MSG_3006: 	
							sprintf (buffer_TX_UART2, (char *)"get msg3006, data_size=%x, code=%u\r\n", Msg->msg_header.MNP_header.data_size, 
							Msg->payload.msg3006.command_code.cmd_3006.code);				
							UART2_PutString (buffer_TX_UART2);
							if (Msg->payload.msg3006.command_code.cmd_3006.code == 7)
							{
								sprintf (buffer_TX_UART2, (char *)"interval=%u\r\n",Msg->payload.msg3006.interval);
								UART2_PutString (buffer_TX_UART2);
							}
							ret = 6; //��������� �������� ����� 6
							break;
						
						case MSG_2200: 
							sprintf (buffer_TX_UART2, (char *)"id=%x, data_size=%x, dummy=%x\r\n", Msg->msg_header.MNP_header.msg_id, 
							Msg->msg_header.MNP_header.data_size, Msg->msg_header.MNP_header.dummy);				
							UART2_PutString (buffer_TX_UART2);
							ret = 7; //��������� �������� ����� 7
							break;
					
						default:
							sprintf (buffer_TX_UART2, (char *)"parcing_error,id=%x,data_size=%x\r\n", Msg->msg_header.MNP_header.msg_id, Msg->msg_header.MNP_header.data_size);				
							UART2_PutString (buffer_TX_UART2);
							break;						
					}
				}
				else
				{
					sprintf (buffer_TX_UART2, "header_CRC_error, %x!=%x\r\n", MNP_CalcChkSum((uint16_t*)&Msg->msg_header, (sizeof(HEAD_MNP_MSG_t)-2)/2), Msg->msg_header.MNP_header.chksum);
					UART2_PutString (buffer_TX_UART2);
				}
				RING_Clear(&RING_buffer); 
				break;				
				
			default:
			break;			
		}
	} 
//	while ( RING_GetCount(&RING_buffer) > 0 ); //���� � ��������� ������ ������� �������� ������ ������ �������� ������������ ������
//	RING_Clear(&RING_buffer); 
	return ret; //������� ���������� �������� 
}

//----------------------------------��������� ��������� ������ �� GPS ���������----------------------------------//
static void GPS_Read_Data(MNP_MSG_t *Msg)
{
	struct TM TimeStamp;
	//Time64_T 	UNIX_Time64;
	int64_t UNIX_Time64;
	float gDOP;
	
	gDOP = Msg->payload.msg3000.gDOP;
	// �������� ��������� gDOP �� ���������� ��������
	if ( gDOP < (MKS2.tmContext.Max_gDOP)) //���� gDOP ��������� � ��������� �� 1 �� 4
	{				
		TimeStamp.tm_mday = Msg->payload.msg3000.day;
		TimeStamp.tm_mon = Msg->payload.msg3000.month;			
		TimeStamp.tm_year = Msg->payload.msg3000.year - 1900;
		TimeStamp.tm_hour = Msg->payload.msg3000.hour;
		TimeStamp.tm_min = Msg->payload.msg3000.minute;
			
		if ( Msg->payload.msg3000.second == 60 ) //��������� ������� ���������� �������
			{TimeStamp.tm_sec = 59;} 
		else 
			{TimeStamp.tm_sec = Msg->payload.msg3000.second;}
			
		UNIX_Time64 = mktime64(&TimeStamp); //������� ���������� ���� � ������ UNIX (01.01.1970)
			
		MKS2.tmContext.Time2k = (uint32_t)(UNIX_Time64 - 946684800); //����� ��������� � 01.01.2000
		if (!(Msg->payload.msg3000.flags.GPS) || !(Msg->payload.msg3000.flags.UTC)) // ������������� ���������� �� GPS ��������� (���� ����������� 1 �� ������ ������������)
		{
			if (!(MKS2.tmContext.ValidTHRESHOLD & DEFAULT_MASK_ValidTHRESHOLD )) //DEFAULT_MASK_ValidTHRESHOLD == 0b100
				{MKS2.tmContext.ValidTHRESHOLD++;} //������ �������� ������� ������ ���� ����������
			else 
				{MKS2.tmContext.Valid = 1;} //��������� ����� ������������� ������
		}
	}
	else
	{
		if (MKS2.tmContext.ValidTHRESHOLD != 0)
		MKS2.tmContext.ValidTHRESHOLD--; //���������� �������� �������������
		MKS2.tmContext.Valid = 0; //����� ����� ������������� ������� 
		sprintf (buffer_TX_UART2, (char *)"pDOP=%0.2f, gDOP=%0.2f, %u %u\r\n", Msg->payload.msg3000.pDOP, 
		Msg->payload.msg3000.gDOP, Msg->payload.msg3000.flags.GPS, Msg->payload.msg3000.flags.UTC);
		UART2_PutString (buffer_TX_UART2);
	}
}

//-------------------------------------------------------------------------------------------------------------//
int8_t GPS_wait_data_Callback (MNP_MSG_t *Msg)
{
	 int8_t result = -1;
	
	if (RING_GetCount(&RING_buffer) > 0)
	{	
		if ((result = Parse_MNP_MSG (Msg)) > 0) //���� ��������� �� �������� �������� ���������
		{
			MKS2.fContext.GPS = 0; //���������� � gps-��������� ������� �����������
			GPS_Read_Data(Msg);
			if (MKS2.tmContext.Valid) // �������� ��������� A ��� ����������� ���������� �� GPS ���������
				{MKS2.canContext.MsgA1Send();} //�������� ��������� ���� �1
			
		}	
	}
	return result;
}

//-------------------------------------------------------------------------------------------------------------//
