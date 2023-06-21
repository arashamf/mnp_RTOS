#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#define CAN_BAUDRATE                                 125000
#define CAN_SAMPLEPOINT_NUM                          75
#define CAN_SAMPLEPOINT_DENOM                        100

#define MODULE_TYPE_MIUx                             0x01
#define MODULE_TYPE_MIU3                             MODULE_TYPE_MIUx
#define MODULE_TYPE_MIU_V                            MODULE_TYPE_MIUx
#define MODULE_TYPE_MIU_N                            MODULE_TYPE_MIUx
#define MODULE_TYPE_MH                               0x04
#define MODULE_TYPE_MPI3                             0x05
#define MODULE_TYPE_M485                             0x07
#define MODULE_TYPE_MRS485                           0x07
#define MODULE_TYPE_MKNS                             0x08
#define MODULE_TYPE_MOG2                             0x09
#define MODULE_TYPE_MPI3_01                          0x0A
#define MODULE_TYPE_MMPI                             0x0A
#define MODULE_TYPE_MPI4                             0x0B
#define MODULE_TYPE_MRS232                           0x0B
#define MODULE_TYPE_MLC2                             0x0C
#define MODULE_TYPE_MLC3                             0x0C
#define MODULE_TYPE_MO_V                             0x0E
#define MODULE_TYPE_MO_N                             0x0F
#define MODULE_TYPE_MNTP                             0x10
#define MODULE_TYPE_MPTP                             0x11
#define MODULE_TYPE_MKM                              0x12
#define MODULE_TYPE_MKK                              0x13
#define MODULE_TYPE_MTAB                             0x14
#define MODULE_TYPE_MKIP                             0x15
#define MODULE_TYPE_ANY                              0x1F
//#define MODULE_ABSENT                                -1

#define MSG_TYPE_A0                                  0x000 // 000000
#define MSG_TYPE_A1                                  0x001 // 000001
#define MSG_TYPE_A2                                  0x002 // 000010
#define MSG_TYPE_A3                                  0x003 // 000011
#define MSG_TYPE_B                                   0x008 // 001000
#define MSG_TYPE_C                                   0x010 // 010000
#define MSG_TYPE_D                                   0x020 // 100000

#define MODULE_ADDR_ALL                              0x1F

#define INVALID_ZKA_ADDRESS                          0x1F

#define MIU_V_IP4                                    235
#define MIU_N_IP4                                    236
#define MIU_MOST_2_IP4                               235
#define MNTP_IP4                                     237
#define MPTP_IP4                                     239
#define MTAB_IP4                                     238


//#include <devcfg.h>

#if defined(__cplusplus)
extern "C"
{
#endif

#if defined(__EMULATION__)
#define MAKE_MSG_ID(__msg_type, __module_addr) ((((uint32_t)((__msg_type) & 0x3F)) << 5) | ((__module_addr) & MODULE_ADDR_ALL))
#define MODULE_ADDR(__msg_id) ((__msg_id) & 0x1F)
#define MSG_TYPE(__msg_id) (((__msg_id) >> 5) & 0x3F)
#elif defined(USE_MDR32)
#define MAKE_MSG_ID(__msg_type, __module_addr) (((((uint32_t)((__msg_type) & 0x3F)) << 5) | ((__module_addr) & MODULE_ADDR_ALL)) << 18)
#define MODULE_ADDR(__msg_id) (((__msg_id) >> 18) & 0x1F)
#define MSG_TYPE(__msg_id) (((__msg_id) >> 23) & 0x3F)
#elif defined(STM32F4xx)
#define MAKE_MSG_ID(__msg_type, __module_addr) ((((uint32_t)((__msg_type) & 0x3F)) << 5) | (((__module_addr) & MODULE_ADDR_ALL)))
#define MODULE_ADDR(__msg_id) ((__msg_id) & 0x1F)
#define MSG_TYPE(__msg_id) (((__msg_id) >> 5) & 0x3F)
//#else
//#error !!!
#endif

#define	MSG_A_DATA_TYPE_TIME0                        0
#define	MSG_A_DATA_TYPE_TIME1                        1
#define	MSG_A_DATA_TYPE_TIME2                        2
#define	MSG_A_DATA_TYPE_TIME3                        3

#define MSG_B_MPI3_RELIABILITY_CMD                   0x1
#define MSG_B_MPI3_RELIABILITY_CMD_SET               0x2

#define MSG_B_MPI3_RELIABILITY_SET                   (MSG_B_MPI3_RELIABILITY_CMD | MSG_B_MPI3_RELIABILITY_CMD_SET)
#define MSG_B_MPI3_RELIABILITY_CLEAR                 MSG_B_MPI3_RELIABILITY_CMD

#define MSG_B_MOG2_ENABLE                            0
#define MSG_B_MOG2_DISABLE                           1

#define MSG_C_MPI3_IS_LPI_V_NO_CONNECTION(msgC_MPI3) ((msgC_MPI3).per_type == 0 && (msgC_MPI3).per_status == 0x0f)

#define MSG_C_MPI4_ERROR_MASK                        0x001F
#define MSG_C_MPI4_ERROR_SEND_TIME_DATA_FAILED       0x0001
#define MSG_C_MPI4_ERROR_LONG_PPS1_VALUE_INCORRECT   0x0002
#define MSG_C_MPI4_ERROR_LONG_PPS2_VALUE_INCORRECT   0x0004
#define MSG_C_MPI4_ERROR_SEND_MESSAGEC_FAILED        0x0008
#define MSG_C_MPI4_ERROR_NO_SHORT_PPS_RECEIVED       0x0010
#define MSG_C_MPI4_WARNING_TIME_DATA_NOT_RECEIVED    0x0020
#define MSG_C_MPI4_WARNING_MESSAGEC_NOT_RECEIVED     0x0040

/*#if defined(__BORLANDC__)
#pragma pack(push, 1)
#else
#pragma push
#pragma pack(1)
#endif*/
typedef struct _MODULE_MSG
{
	uint8_t data_type: 3;
	uint8_t module_type: 5;
	union
	{
		uint8_t message_data[7];
		struct
		{
			uint32_t time2k;
			int8_t d_tai;
			uint8_t	d_msk : 3;
			uint8_t stratum	: 2;
			uint8_t : 1;
			uint8_t leap_info : 2;
			int8_t d_mestn;
		} msgA_0; // MSG_TYPE_A0, data_type == MSG_A_DATA_TYPE_TIME0
		struct
		{
			uint32_t time2k;
			int8_t d_tai;
			uint8_t	d_msk : 3;
			uint8_t : 3;
			uint8_t leap_info : 2;
			int8_t d_mestn;
		} msgA_1; // MSG_TYPE_A1, MSG_TYPE_A2, MSG_TYPE_A3, data_type == MSG_A_DATA_TYPE_TIME1
		struct
		{
			uint8_t year2k; // год столетия, year == year2k + 2000
			uint8_t month;  // 1..12 
			uint8_t day;    // 1..
			uint8_t hour;   // 0..23
			uint8_t min;    // 0..59
			uint8_t sec;    // 0..59
			int8_t local_tz: 5;
			uint8_t moscow_tz: 3;
		} msgA_2; // data_type == MSG_A_DATA_TYPE_TIME2
		struct
		{
			uint32_t time2k;
			uint16_t : 12;
			uint16_t unreliability : 1;
			uint16_t moscow_tz : 3;	// в часах
			int8_t local_tz;        // в пятнадцатиминутных интервалах
		} msgA_3; // data_type == MSG_A_DATA_TYPE_TIME3
/*
		struct
		{
			uint32_t time2k;
			uint16_t : 8;
			uint16_t time_source : 4;
			uint16_t unreliability : 1;
			uint16_t moscow_tz : 3;	// в часах
			int8_t local_tz : 8;    // в пятнадцатиминутных интервалах
		} msgA1; // data_type == MSG_A_DATA_TYPE_TIME3
*/
		struct
		{
			uint8_t : 5;
			uint8_t shutdown : 1;
			uint8_t stop : 1;
			uint8_t reset : 1;
		} msgB_Common; // data_type == 0x07
		struct
		{
			uint8_t reliability	: 2;
			uint8_t const_0 : 2;
			uint8_t status : 4;
		} msgB_MPI3; // (module_type == MODULE_TYPE_MPI3 || module_type == MODULE_TYPE_ANY) && data_type == 0
		struct
		{
			uint8_t const_0 : 4;
			uint8_t status : 4;
#define MPI3_01_STATUS_OK            0
#define MPI3_01_STATUS_FAULT         1	
#define MPI3_01_STATUS_PKN_FAULT     2
#define MPI3_01_STATUS_PKN_NO_LINK   3
			uint8_t rt_address : 5;
		} msgB_MPI3_01; // (module_type == MODULE_TYPE_MPI3_01 || module_type == MODULE_TYPE_ANY) && data_type == 0
		struct
		{
			uint8_t command;
		} msgB_MOG2;
		struct
		{
#define MKM_COMMAND_NOTHING       0
#define MKM_COMMAND_STOP          1	
#define MKM_COMMAND_START         2
#define MKM_COMMAND_SET_POSITIONS 3
			uint8_t : 8;
			uint16_t chnl1_command : 3;
			uint16_t : 1;
			uint16_t chnl1_phase : 1;
			uint16_t chnl1_min : 11;
			uint16_t chnl2_command : 3;
			uint16_t : 1;
			uint16_t chnl2_phase : 1;
			uint16_t chnl2_min : 11;
		} msgB_MKM;
		struct
		{
#define MKK_COMMAND_NOTHING       0
#define MKK_COMMAND_STOP          1	
#define MKK_COMMAND_START         2
			uint8_t : 8;
			uint16_t chnl1_command : 3;
			uint16_t : 13;
			uint16_t chnl2_command : 3;
			uint16_t : 13;
		} msgB_MKK;
		struct
		{
			uint8_t channel_no : 3;
			uint8_t : 3;
			uint8_t cmd : 2;
			uint8_t hour : 5;
			uint8_t : 1;
			uint8_t start : 1;
			uint8_t polarity : 1;
			uint8_t min : 6;
			uint8_t const_0: 2;
			uint8_t amperage_min;
		} msgB_MUS;
		struct
		{
			uint8_t ant_power : 2;
			uint8_t sent_A : 1;
			uint8_t : 5;
			int8_t local_tz : 5;
			uint8_t : 3;
		} msgB_MPV;
		struct
		{
			uint8_t addr : 6;
			uint8_t : 2;
			uint8_t brightness;
		} msgB_M485;
		struct
		{
			uint8_t ip[4];
		} msgB_MNTP_net_ip;
		struct
		{
			uint8_t mask[4];
		} msgB_MNTP_net_mask;
		struct
		{
			uint8_t gw[4];
		} msgB_MNTP_net_gw;
		struct
		{
			uint8_t ip[4];
			int8_t poll;
			uint8_t : 7;
			uint8_t enable : 1;
		} msgB_MNTP_broadcast;
		struct
		{
			uint8_t ip[4];
		} msgB_MPTP_net_ip;
		struct
		{
			uint8_t mask[4];
		} msgB_MPTP_net_mask;
		struct
		{
			uint8_t gw[4];
		} msgB_MPTP_net_gw;
		struct
		{
			uint8_t domain_number;
			uint8_t : 4;
			uint8_t transport : 1;
#define MPTP_TRANSPORT_IPV4      0
#define MPTP_TRANSPORT_ETHERNET  1
			uint8_t delay_mechanism : 1;
#define MPTP_DELAY_MECHANISM_E2E  0
#define MPTP_DELAY_MECHANISM_P2P  1
			uint8_t domain_addr_fix : 2; // domain_addr = 224.0.1.129 + domain_addr_fix
			int16_t ptp_utc_offset;
		} msgB_MPTP_ptp;
		struct
		{
			uint8_t : 6;
			uint8_t warn : 1;
			uint8_t err : 1;
		} msgB_MO_N;
		struct
		{
			uint8_t pps_src : 1; // 0 - PD5, 1 - PB4
			uint8_t /*msgA_type*/ : 1; // 0 - MSG_TYPE_A, 1 - MSG_TYPE_A1
			uint8_t msgA_tx_addr : 5;
			uint8_t : 1;
			uint8_t port_hardware_flow_control : 4; // None, RTS, CTS, RTS_CTS
			uint8_t port_stop_bits : 2; // 1, 2
			uint8_t port_word_length : 2; // 5, 6, 7, 8
			uint8_t port_parity : 3; // No, Even, Odd
			uint8_t port_baud: 5; // 50, 75, 110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
		} msgB_MPI4;
		struct
		{
			uint8_t ip[4];
		} msgB_MTAB_net_ip;
		struct
		{
			uint8_t mask[4];
		} msgB_MTAB_net_mask;
		struct
		{
			uint8_t gw[4];
		} msgB_MTAB_net_gw;
		struct
		{
			uint8_t : 7;
			uint8_t err	: 1;
		} msgC_Any;
		struct
		{
			uint8_t unreliable : 1;
			uint8_t failure1MHz	: 1;
			uint8_t : 5;
			uint8_t err	: 1;
			uint8_t vhp_addr : 5;
			uint8_t : 3;
			uint8_t per_addr : 5;
			uint8_t : 3;
			uint8_t per_type : 5;
			uint8_t : 3;
			uint8_t per_status : 4;
			uint8_t : 4;
		} msgC_MPI3; // module_type == MODULE_TYPE_MPI3 || && data_type == 0
		struct
		{
			uint8_t : 7;
			uint8_t err	: 1;
			uint8_t rt_address : 5;
			uint8_t : 3;
		} msgC_MPI3_01; // module_type == MODULE_TYPE_MPI3_01 || && data_type == 0
		struct
		{
			uint8_t inicialization	: 1;
			uint8_t master : 1;
			uint8_t no_reserve : 1;
			uint8_t : 4;
			uint8_t err	: 1;
			uint16_t ustavka_DAC;
			int16_t test;
		} msgC_MOG2;
		struct
		{
			uint8_t hw_init_fault : 1;
#define MPI4_HW_INIT_FAULT_MSK   (0x01 << 8)
			uint8_t config_wait : 1;
#define MPI4_CONFIG_WAIT_MSK     (0x02 << 8)
			uint8_t pps0_out_fault : 1;
#define MPI4_PPS0_OUT_FAULT_MSK  (0x04 << 8)
			uint8_t uart_out_fault : 1;
#define MPI4_UART_OUT_FAULT_MSK  (0x08 << 8)
			uint8_t hw_fault : 1;
#define MPI4_HW_FAULT_MSK        (0x10 << 8)
			uint8_t pps1_out_fault : 1;
#define MPI4_PPS1_OUT_FAULT_MSK  (0x20 << 8)
			uint8_t : 1;
			uint8_t err	: 1;
			uint8_t fail_1mhz : 1;
#define MPI4_FAIL_1MHZ_MSK       (0x01 << 16)
			uint8_t fail_pps : 1;
#define MPI4_FAIL_PPS_MSK        (0x02 << 16)
			uint8_t fail_msga : 1;
#define MPI4_FAIL_MSGA_MSK       (0x04 << 16)
			uint8_t unreliable_time: 1;
#define MPI4_UNRELIABLE_TIME_MSK (0x08 << 16)
#define MPI4_ALL_ERRORS_MSK      (MPI4_HW_INIT_FAULT_MSK | MPI4_PPS0_OUT_FAULT_MSK | MPI4_UART_OUT_FAULT_MSK | MPI4_HW_FAULT_MSK | MPI4_PPS1_OUT_FAULT_MSK)
#define MPI4_ALL_WARNINGS_MSK    (MPI4_CONFIG_WAIT_MSK | MPI4_FAIL_PPS_MSK | MPI4_FAIL_MSGA_MSK | MPI4_UNRELIABLE_TIME_MSK)
		} msgC_MPI4;
		struct
		{
			uint8_t channel_no : 3;
			uint8_t reserve_type : 2;
			uint8_t amperage_lt6mA : 1;
			uint8_t amperage_gt1_2mA : 1;
			uint8_t err	: 1;
			uint8_t hour : 5;
			uint8_t status : 1;
			uint8_t polarity: 1;
			uint8_t hi_I;
			uint8_t lo_I;
			uint8_t hi_U;
			uint8_t lo_U;
		} msgC_MUS;
		struct
		{
			uint8_t channel_no : 3;
			uint8_t : 3;
			uint8_t amperage_gt1_2mA : 1;
			uint8_t err	: 1;
			uint8_t amperage;
			uint8_t voltage_60V_vs_imp_diff;
			uint8_t voltage_60V;
			uint8_t voltage_rev_line;
			uint8_t voltage_5V;
		} msgC_MSI;
		struct
		{
			uint8_t reliability : 3;
			uint8_t : 4;
			uint8_t err	: 1;
			uint8_t time[4];
			uint8_t voltage_ant;
			uint8_t voltage_GPS;
		} msgC_MPV;
		struct
		{
			uint8_t channel_no : 2;
			uint8_t : 5;
			uint8_t err	: 1;
			uint32_t clock_status;
		} msgC_485;
		struct
		{
			uint8_t pkn_warning : 1;
			uint8_t pkn_fault	: 1;
			uint8_t : 3;
			uint8_t sfp_tx_fault : 1;
			uint8_t fpga_fault : 1;
			uint8_t err : 1;
			uint8_t : 1;
			uint8_t disabled : 1;
			uint8_t sfp_ddm: 1;
			uint8_t sfp_link : 1;
			uint8_t sfp_detect : 1;
			uint8_t sync_timeinfo	: 1;
			uint8_t sync_1mhz : 2;
			uint8_t sfp_temperature;
			uint8_t sfp_vcc;
			uint8_t sfp_tx_bias;
			uint8_t sfp_tx_power;
			uint8_t sfp_rx_power;
		} msgC_MO_x;
		struct
		{
			uint8_t hw_init_fault : 1;
#define MIU_HW_INIT_FAULT_MSK   (0x01 << 8)
			uint8_t invalid_config : 1;
#define MIU_INVALID_CONFIG_MSK  (0x02 << 8)
			uint8_t : 5;
			uint8_t err : 1;
#define MIU_ERROR_MSK           (0x80 << 8)
		} msgC_MIU_x;
		struct
		{
			uint8_t no_sync : 1;        // 1 - не синхронизирован
			uint8_t config_wait : 1;    // 1 - процес получения конфигурации не завершен
			uint8_t : 2;
			uint8_t fail_hw_mac : 1;    // 1 - произошел отказ 25AA02E48 (внешней MAC)
			uint8_t fail_eth_phy : 1; 	// 1 - произошел отказ блока Ethernet PHY
			uint8_t fail_1mhz : 1;      // 1 - отказ 1 МГц выдаваемого опорным гениратором
			uint8_t err : 1;            // 1 - интегральный отказ, при наличие хотябы одного отказа
			uint8_t year: 8;
			uint8_t month: 8;
			uint8_t day: 8;
			uint8_t hour: 8;
			uint8_t min: 8;
			uint8_t sec: 8;
		} msgC_MNTP;
		struct
		{
			uint8_t no_sync : 1;        // 1 - не синхронизирован
			uint8_t config_wait : 1;    // 1 - процес получения конфигурации не завершен
			uint8_t : 2;
			uint8_t fail_hw_mac : 1;    // 1 - произошел отказ 25AA02E48 (внешней MAC)
			uint8_t fail_eth_phy : 1; 	// 1 - произошел отказ блока Ethernet PHY
			uint8_t fail_pll : 1;       // 1 - отказ PLL CDCE906
			uint8_t err : 1;            // 1 - интегральный отказ, при наличие хотябы одного отказа
			uint8_t : 8;
			uint8_t : 8;
			uint8_t : 8;
		} msgC_MPTP;
		struct
		{
			uint8_t : 7;
			uint8_t err : 1;
			uint16_t dac_value;
			uint16_t reserved;
			uint16_t poweron_hours;
		} msgC_MH;
		struct
		{
			uint8_t : 5;
			uint8_t fail_gps_ant : 1;   // 1 - КЗ GPS антены  
			uint8_t fail_gps : 1;       // 1 - отказ GPS приемника
			uint8_t err : 1;            // 1 - интегральный отказ, при наличие хотябы одного отказа
			uint8_t : 7;
			uint8_t gps_ant_disc : 1;   // 1 - GPS антена неподключена
			uint8_t : 8;
			uint8_t : 8;
			uint8_t : 8;
			uint8_t : 8;
			uint8_t : 8;
		} msgC_MKNS;
		struct
		{
#define MKM_CHANNEL_STATUS_WAITING_CONFIG  0
#define MKM_CHANNEL_STATUS_LINE_BREAK      1
#define MKM_CHANNEL_STATUS_LINE_OVERLOAD   2
#define MKM_CHANNEL_STATUS_NORMA           3
			uint8_t : 6;
			uint8_t no_sync : 1;
			uint8_t err : 1;
			uint16_t chnl1_status : 3;
			uint16_t chnl1_mode : 1;
			uint16_t chnl1_phase : 1;
			uint16_t chnl1_min : 11;
			uint16_t chnl2_status : 3;
			uint16_t chnl2_mode : 1;
			uint16_t chnl2_phase : 1;
			uint16_t chnl2_min : 11;
		} msgC_MKM;
		struct
		{
#define MKK_CHANNEL_STATUS_LINE_BREAK      1
#define MKK_CHANNEL_STATUS_LINE_OVERLOAD   2
#define MKK_CHANNEL_STATUS_NORMA           3
			uint8_t : 6;
			uint8_t no_sync : 1;
			uint8_t err : 1;
			uint16_t chnl1_status : 3;
			uint16_t chnl1_mode : 1;
			uint16_t : 12;
			uint16_t chnl2_status : 3;
			uint16_t chnl2_mode : 1;
			uint16_t : 12;
		} msgC_MKK;
		struct
		{
			uint8_t hw_init_fault : 1;
#define MTAB_HW_INIT_FAULT_MSK   (0x01 << 8)
			uint8_t config_wait : 1;
#define MTAB_CONFIG_WAIT_MSK     (0x02 << 8)
			uint8_t ready_for_shutdown : 1;
#define MTAB_READY_FOR_SHUTDOWN_MSK (0x04 << 8)
			uint8_t  : 1;
			uint8_t hw_fault : 1;
#define MTAB_HW_FAULT_MSK        (0x10 << 8)
			uint8_t  : 1;
			uint8_t : 1;
			uint8_t err	: 1;
			uint8_t fail_1mhz : 1;
#define MPI4_FAIL_1MHZ_MSK       (0x01 << 16)
			uint8_t fail_pps : 1;
#define MTAB_FAIL_PPS_MSK        (0x02 << 16)
			uint8_t fail_msga : 1;
#define MTAB_FAIL_MSGA_MSK       (0x04 << 16)
			uint8_t unreliable_time: 1;
#define MTAB_UNRELIABLE_TIME_MSK (0x08 << 16)
		} msgC_MTAB;
		struct
		{
			uint8_t : 4;
			uint8_t status : 4;
		} msgC_MKIP;
		struct
		{
			uint8_t inicialization : 1;
			uint8_t master : 1;
			uint8_t line_no : 1;
			uint8_t : 5;
		} msgD_MOG2; // module_type == MODULE_TYPE_MOG2 || && data_type == 0
		struct
		{
			uint8_t const_0x06;
			int8_t local_tz: 5;
			uint8_t moscow_tz: 3;			
		} msgD_TZ;
		struct
		{
			uint8_t const_0x07;
			int8_t d_std;
			uint32_t leap_info	: 2;
			uint32_t : 3;
			uint32_t hour_std : 4;
			uint32_t month_std : 4;
			uint32_t week_day_std : 3;
			uint32_t week_num_std : 3;
			uint32_t hour_dst : 3;
			uint32_t month_dst : 4;
			uint32_t week_day_dst : 3;
			uint32_t week_num_dst : 3;
			int8_t d_tai;
		} msgD_TimeCfg;
	} d;
} MODULE_MSG;
/*#if defined(__BORLANDC__)
#pragma pack(pop)
#else
#pragma pop
#endif*/

#if defined(__cplusplus)
}
#endif

#endif

