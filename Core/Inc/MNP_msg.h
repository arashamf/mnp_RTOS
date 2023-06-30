
#ifndef __MNP_MSG_H__
#define __MNP_MSG_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes ------------------------------------------------------------------------//
#include "main.h"
#include "typedef.h"
//Private defines ------------------------------------------------------------------//
#define SPECIAL_CMD_ID 1
#define SPECIAL_CMD_CODE_RESET 12
#define CMD_CODE_CONFIG 2
#define CMD_CODE_SATTELITES_MASK 6

//Exported types -----------------------------------------------------------------//
typedef enum
{
	MSG_3000      = 0x0BB8, //messages 3000
	MSG_3001      = 0x0BB9, //messages 3001
	MSG_3011      = 0x0BC3, //messages 3011
	MSG_3002      = 0x0BBA, //messages 3002
	MSG_3003      = 0x0BBB, //messages 3003
	MSG_3006      = 0x0BBE, //messages 3006
	MSG_2000      = 0x07D0, //messages 2000
	MSG_2200      = 0x0898  //messages 2200
} HEAD_MNP_MSG_ID_t;

//----------------------------уставки и спец. команды----------------------------//
typedef struct 
{
	struct
	{
		union //код команды 3006
		{
	    struct 
			{
				uint16_t RAM		: 1;	//Текущие уставки в RAM приемника
				uint16_t dummy1	: 1;	//резерв
				uint16_t flash	: 1;	//уставки в FLASH памяти
				uint16_t dummy2	: 4;	//резерв
				uint16_t write	: 1; // 0 - чтение 1 - запись
				uint16_t code		: 8; // Код команды (0x2 - шесть 32-х разрядных словконфигурации)
			} cmd_3006;
			
		  struct 
			{
				uint16_t special_code	: 8; // код спец. команды (0x0C - сброс)
				uint16_t id						: 8; // для спец. команд должен быть равен (SPECIAL_CMD_ID = 0x1)
			} specialcmd_3006;
			
			uint8_t code_cmd [2]; //массив для побайтного чтения кода команды
		};
		
		uint16_t	dummy; //2 пустых байта (резерв)
		
	}command_code;
	
	union
	{
		struct // Команда CMD_CODE_CONFIG (Установка конфигурации)
		{		
			//нулевое 32 разрядное слово
			uint32_t      					: 32; //1 слово - резерв (0x0)
			//первое 32 разрядное слово
			uint32_t protocol0			: 8; 	// Протокол обмена канала0. 0x1-MNP binary, 0х8-NMEA-0183 (0x8)
			uint32_t      					: 14; //Резерв (0x0)
			uint32_t baud_divider0	: 10; //предделитель UART0 = 460800/baudrate (0x60)
			//второе 32 разрядное слово
			uint32_t protocol1			: 8; // Протокол обмена канала1 - MNP binary, 8 - NMEA-0183 (0x1)
			uint32_t      					: 14; //Резерв (0x0)
			uint32_t baud_divider1	: 10; //предделитель UART1 = 460800/baudrate (0x4)
			//третье 32 разрядное слово
			uint32_t      					: 32;  //Резерв (0x0)
			//четвёртое 32 разрядное слово
			uint32_t      					: 2;  //Резерв (0x0)
			uint32_t tropo_corr			: 1; // Разрешение использования модели тропосферы (0x1)
			uint32_t use_difc				: 1; // Разрешение использования дифференциальных поправок (0x1)
			uint32_t      					: 2; //Резерв (0x0)
			uint32_t dif_only				: 1; // Принудительное использование дифференциального режима (0x0) 
			uint32_t sol_smooth			: 1; // Фиксация координат при скорости менее 1 м/c (0x0) 
			uint32_t sol_filter			: 1; // Сглаживание решения (0x1)
			uint32_t meas_filter		: 1; // Совмещенная фильтрация по коду и фазе несущей (0x1)
			uint32_t iono_corr			: 1; // Разрешение ионосферной коррекции (0x1)
			uint32_t disable_2D			: 1; // Запрет двумерной навигации (0x0)
			uint32_t use_RAIM				: 1; // Включение алгоритма RAIM (0x1)
			uint32_t enable_warm_startup	: 1; // Разрешение быстрого «горячего» старта (0x1)
			uint32_t      					: 2; //Резерв  (0x1)  
			// биты  sys_time и glo_time определяют привязку  измерений и фронта секундной метки времени:
			// glo_time | sys_time | привязка
			// --------------------------------
			//  0       |	0      | UTC(USNO)
			//  0       |	1      | GPS
			//  1       |	0      | UTC(SU)
			//  1       |	1      | ГЛОНАСС
			uint32_t sys_time						: 1; // Привязка секундной метки к времени навигационной системы (0x0)
			uint32_t glo_time						: 1; // Секундная метка ГЛОНАСС / UTC(SU) (0x0)   
			uint32_t shift_meas					: 1; // Привязка измерений к секундной метке (0x1)
			uint32_t enable_SBAS				: 1; //Разрешение SBAS (0x0)
			uint32_t enable_iono_SBAS		: 1; //Разрешение модели ионосферы SBAS (0x0)
			uint32_t GPS_compatibility	: 1; //Режим совместимости с приемниками GPS. В сообщениях NMEA-0183 всегда принудительно ставится префикс $GP (0x0)
			uint32_t      							: 2; //Резерв (0x0)
			uint32_t wr_alms 						: 1; // Разрешение сохранения в flash альманахов (0x1)
			uint32_t wr_ephs   					: 1; // Разрешение сохранения в flash эфемерид (0x1)
			uint32_t wr_ionoutc					: 1; // Разрешение сохранения в flash модели UTC GPS (0x1)
			uint32_t wr_coords					: 1; // Разрешение сохранения в flash координат (0x1)
			uint32_t      							: 4;  //Резерв (0x1)
			//пятое слово
			uint32_t enable_3000_1000_GGA_0	: 1; //Разрешение кадра 3000/1000/«GGA» по каналу 0 (0x1)
			uint32_t enable_3011_1002_GSA_0	: 1; //Разрешение кадра 3011/1002/«GSA» по каналу 0 (0x1)
			uint32_t enable_3002_1003_GSV_0	: 1; //Разрешение кадра 3002/1003/«GSV» по каналу 0 (0x1)
			uint32_t enable_3003_1012_RMC_0	: 1; //Разрешение кадра 3003/1012/«RMC» по каналу 0 (0x1)
			uint32_t      									: 4;  //Резерв (0x0)
			uint32_t enable_3000_1000_GGA_1	: 1; //Разрешение кадра 3000/1000/«GGA» по каналу 1 (0x1)
			uint32_t enable_3011_1002_GSA_1	: 1; //Разрешение кадра 3011/1002/«GSA» по каналу 1 (0x1)
			uint32_t enable_3002_1003_GSV_1	: 1; //Разрешение кадра 3002/1003/«GSV» по каналу 1 (0x1)
			uint32_t enable_3003_1012_RMC_1	: 1; //Разрешение кадра 3003/1012/«RMC» по каналу 1 (0x0)
			uint32_t      									: 20;  //Резерв (0x0)

		} config;

		struct // Команда CMD_CODE_SATTELITES_MASK (Маска по спутникам), код 0х6
		{				
			uint32_t	GPS;
			uint32_t	GLONASS;
			/*uint32_t	SBAS; 
			uint32_t	Galileo;
			uint32_t	P_GPS;
			uint32_t	P_GLONASS;
			uint32_t	QZSS_GPS;
			uint32_t	QZSS_SBAS;
			uint32_t	BDS;*/
		} sat_mask;
		
		uint32_t	interval; //Длина интервала измерения (2000=1c), код команды 0х07
		
		struct //Маска сброса (использовать с кодом команды specialcmd_3006)
		{
			uint32_t rtc    				: 1; //сброс RTC
			uint32_t coord    			: 1; // стирание запомненных координат
			uint32_t ephs    				: 1; //стирание эфемерид
			uint32_t alm    				: 1; // стирание альманаха
			uint32_t iono_model			: 1; //обнуление модели ионосферы/UTC GPS
			uint32_t 								: 27; //Резерв
		} reset_mask;
	};

}MNP_MSG_3006_t;

//--------------------Описание кадра 3000 - навигационное решение--------------------//
typedef struct 
{
	struct 
	{
		double 				longitude;       // Широта D рад
		double 				latitude;        // Долгота D рад
		double 				hor_height;	     // Высота D м
		double 				hor_vel;		 			// Модуль наземной скорости D м/с
		double 				azimuth;		 		// Азимут D рад
		double 				vert_vel;		 		// Скорость подъема D м/с
		uint32_t      channels;		 		// Каналы в решении
		uint32_t      diff_corr;       // Наличие дифференциальных поправок
		int32_t 			year;			 			// Год DI год -
		int32_t 			month;					 // Месяц DI месяц 1-12
		int32_t 			day;             // День DI день 1-31
		int32_t 			hour;			 			// Час DI ч 0-23
		int32_t 			minute;			 			// Минута DI мин 0-59
		int32_t 			second;			 			// Секунда DI с 0-60
		uint32_t 			msec;			 			// Внутреннее время приемника UDI мс - Цена младшего разряда 0,5 мс
		float					Hz;              // Отстройка генератора F Гц -
		float					gDOP;						 // GDOP F - -
		float					pDOP;			 			// PDOP F - -
		double	    	fil_lon;		 		// Фильтрованная широта D рад -
		double		    fil_lat;		 		// Фильтрованная долгота D рад -
		double      	fil_height;      // Фильтрованная высота D м -
		double        fil_vel_module;	 // Фильтрованная скорость (модуль) D м/с -
		double        fil_vel_az;		 // Фильтрованная скорость (азимут) D рад -
		double        fil_vel_vert;    // Фильтрованная скорость (подъем) D м/с -
			
		struct  // Флаги 
		{                     
			uint32_t two_dim_sol	: 1; // Двухмерное решение
			uint32_t fix_diff			: 1; // Фиксированная невязка времени между GPS и ГЛОНАСС
			uint32_t ellipsoid		: 2; // Эллипсоид 0-WGS-84, 1-ПЗ-90, 2- Красовского, 3 – пользователя
			uint32_t GPS					: 1; // Годность решения
			uint32_t UTC					: 1; // Годность времени 
			uint32_t							: 4; // Резерв
			uint32_t system_coord	: 3; // Система координат 0 - WGS-84, 1- ПЗ-90, 2 - СК-42, 3 - СК-95, 4 - определяется пользователем, 5-7 резерв
			uint32_t diff_mode		: 1; // Дифференциальный режим
			uint32_t 							: 2; // Резерв
			uint32_t							: 2; // Резерв
			uint32_t flags_ver		: 4; // Версия флагов
		}flags;	

		uint32_t		ephemerides;       // Наличие эфемерид 
		float				temperature;       // Температура °C - При наличии термодатчика
		uint32_t		failed_meas;       // Отбракованные измерения. 1 – измерения отбракованы алгоритмом RAIM
	};	
}MNP_MSG_3000_t;

//--------------------шаблон структуры заголовка сообщения протокола MNP-binary--------------------//
typedef struct 
{
	uint16_t sync;
	union 
	{	
		struct 
		{
			uint8_t ID_B; //младшая часть ID типа сообщения
			uint8_t ID_A; //старшая часть ID типа сообщения
		};
		uint16_t msg_id;
	};	
	
	union 
	{	
		struct 
		{
			uint8_t length_B; //младшая часть длины сообщения
			uint8_t length_A; //старшая часть длины сообщения
		};
		uint16_t data_size; //длина полезной части сообщения
	};	
	
		union 
	{	
		struct 
		{
			uint8_t  dummy_B; 
			uint8_t  dummy_A; 
		};
		uint16_t dummy; 
	};	
	
	
	union 
	{	
		struct 
		{
			uint8_t	chksum_B; //младшая часть контрольной суммы
			uint8_t chksum_A; //старшая часть контрольной суммы
		};
		uint16_t chksum;
	};
	
} HEAD_MNP_MSG_t;

//-------------------------шаблон структуры сообщения протокола MNP-binary-------------------------//
typedef struct 
{
	union
	{
		HEAD_MNP_MSG_t MNP_header;
		uint16_t header_words[5];
		uint8_t header_bytes[10];
	} msg_header;
	
	union
	{
		MNP_MSG_3000_t	msg3000;
		MNP_MSG_3006_t	msg3006;
		uint16_t raw_words[BUFFER_SIZE/2]; //массив для побайтного чтения тела MNP-сообщения
		uint8_t raw_bytes[BUFFER_SIZE]; //массив для пословного (2 байта) чтения тела MNP-сообщения
	}	payload;
		
	enum 
	{
		__SYNC_BYTE1 = 1,
		__SYNC_BYTE2,
		__TYPE_ID_BYTE1,
		__TYPE_ID_BYTE2,
		__LENGTH_BYTE1,
		__LENGTH_BYTE2,
		__RESERVE_BYTE1,
		__RESERVE_BYTE2,
		__CK_HEAD_BYTE1,
		__CK_HEAD_BYTE2,
		__PAYLOAD,
		__PARSER
	}rx_state;	

}MNP_MSG_t;

extern MKS2_t MKS2;
//Constants ----------------------------------------------------------------------//
//Private prototypes--------------------------------------------------------------//
static uint16_t MNP_CalcChkSum( uint16_t *Array, int WordsCount );
void MNP_PutMessage (MNP_MSG_t *Msg, uint16_t MsgId, uint16_t WordsCount);
static void MNP_M7_init (MNP_MSG_t *);
void read_config_MNP ( MNP_MSG_t *);
void read_flash_MNP ( MNP_MSG_t *);
void put_msg2000 (MNP_MSG_t *);
void Set_GNSS_interval (MNP_MSG_t *, uint32_t );
void Get_GNSS_interval (MNP_MSG_t *, uint32_t );
void Read_SN (MNP_MSG_t *);
void GPS_Init(MNP_MSG_t *);
void GPS_rst(FunctionalState );
void MNP_Reset(MNP_MSG_t *);
static int8_t Parse_MNP_MSG (MNP_MSG_t * );
static void GPS_Read_Data(MNP_MSG_t *);
int8_t GPS_wait_data_Callback (MNP_MSG_t *);
#ifdef __cplusplus
}
#endif

#endif /* __MNP_MSG_H__ */

