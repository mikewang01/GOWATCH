#ifndef __UART_PROTOCAL_CMD_H__
#define __UART_PROTOCAL_CMD_H__


#include "oop_hal.h"
#include "stdint.h"
#include "ble_common.h"
#include "oop_hal.h"
#include "ble_service.h"
#include "btle_api.h"
/*INIDICATE DIFFRENT IN CMD LAYER*/
enum uart_cmd_package_type{
        CMD_LOCATION_TYPE,
        CMD_HEALTH_TYPE
};

/*********************************************************************
* MACROS
*/
/*to be compatible with old uart protocal*/
#define  UART_OLD_PROTOCAL
#define  CLING_ID_LENTH 4
#define  CLING_MAC_LENTH 6
#define  CLING_RSSI_LENTH 1
#define  CLING_CMD_TYPE_LENTH 1



/*information uploaded*/
struct location_data{
        char cling_id[CLING_ID_LENTH]; /*cling device id*/
        char cling_mac[CLING_MAC_LENTH];/*cling mac adress*/
        char cling_rssi[CLING_RSSI_LENTH];/*cling rssi number*/
};
struct cling_location_rev{
        char package_type;/*indicate current pakage type*/
        struct location_data load;
};


#pragma pack(push) 
#pragma pack(1)
struct health_data{
        char cling_mac[CLING_MAC_LENTH];/*cling mac adress*/
        unsigned char null_align; /*for align used*/
        unsigned char  date;   /*the time when cling sendout health data */
        unsigned short int steps;
        unsigned short int distance;
        unsigned short int calories;
        unsigned short int sleep_time;
        unsigned char  heart_rate;
        unsigned char  skin_temp;
};

/*user health information related*/
struct cling_health_rev{
        char package_type;/*indicate current pakage type*/
        struct health_data load;
        //char health_data[sizeof(struct health_data_inf)];
};
#pragma pack(pop) 

struct cling_health_send{

};



/*uart protocal cmd layer package*/
struct cling_cmd_rev{
        char package_type;
        char ppackage[1];
};


struct cling_inf_send{
#ifndef UART_OLD_PROTOCAL
        char package_type;/*indicate current pakage type*/
#endif
        char *ptr;
};


/*object prototype declaration*/
DEF_CLASS(cling_uart)
        int (*init)     (CLASS(cling_uart) *arg);                       /*initiate uart object*/
        int (*de_init)  (CLASS(cling_uart) *arg);                               /*delete uart object*/
        int (*send_data)                (CLASS(cling_uart) *arg, char *pinf, int lenth);/*data send   function*/
        int (*send_cmd)                 (CLASS(cling_uart) *arg, char *pinf, int lenth);/*data send   function*/
        int (*enable_recieving) (CLASS(cling_uart) *arg);
        int (*disable_recieving) (CLASS(cling_uart) *arg);
        int (*task_register)    (CLASS(cling_uart) *arg, unsigned short int task_id);                   /*register  object*/
        int (*recv_callback_register)   (CLASS(cling_uart) *arg, void (*callback)(void *pbuffer));/*register  object*/
        void *user_data;/*point to user private data*/
END_DEF_CLASS(cling_uart)

int  cling_u_data_send(char *pinf, int lenth);
int  set_cling_uart_cmdlayer_revcallback(int (*callback)(char*, unsigned int));


//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
#pragma pack(push)
#pragma pack(1)
struct pay_rev_package {
        struct {
                uint8_t ind;
                uint8_t reserved;
                uint16_t ssc;
                uint16_t cmd_type;
                uint16_t data_lenth;
        } header;

        struct {
                uint8_t* body;
        } data;
        uint16_t crc;

};

struct pay_ack_package {
        struct {
                uint8_t ind;
                uint8_t reserved;
                uint16_t ssc;
                uint16_t cmd_type;
                uint16_t data_lenth;
        } header;

        struct {
                uint8_t* body;
        } ack_data;
        uint16_t crc;
};
#pragma pack(pop)

enum{
   BTC_INFO = 0x0001,
   BTC_BATT,
   BTC_RESERVED1,
   BTC_DATA,
   BTC_AUTH,
   BTC_BIND,
   BTC_UNBIND,
   BTC_DISCONNECT = 0x0101,
   BTC_CONNECT,
   BTC_ATR,
   BTC_APDU = 0x0104,
   BTC_QR_CODE,
   BTC_QR_IMAGE,

   BTC_IO_OK = 0X0000,
   BTC_ILLEAGAL_CMD ,
   BTC_SSC_ERROR = 0X0006,
   BTC_CRC_ERROR = 0X0008,

};


#define SYNC_HEAD                   0x6F

#define ACTIVATE_ISO7816            0x00
#define DEACTIVATE_ISO7816          0x01
#define RETRIEVE_ATR                0x02
#define SEND_APDU                   0x03
#define RETRIEVE_BALANCE            0x04
#define COMMAND_DES                 0x05
#define COMMAND_TDES                0x06
#define COMMAND_SHA1                0x07
#define COMMAND_RAND                0x08
#define COMMAND_RSA_GEN             0x09
#define COMMAND_RSA_CAL             0x0A
#define COMMAND_MAC                 0x0B
#define COMMAND_HMAC                0x0C

void Clingpay_Init_UART(void);
void Clingpay_Activate_ISO7816(void);
void Clingpay_RETRIEVE_ATR(uint8_t *pATR, uint16_t *pATR_Len);
void Clingpay_Send_APDU(uint8_t *pAPDU, uint16_t *pAPDU_Len);
void Clingpay_Retrieve_Balance(uint8_t *pAPDU, uint16_t *pAPDU_Len);
void Clingpay_Command_DES(uint8_t *pDES, uint16_t *pDES_Len);
void Clingpay_Command_TDES(uint8_t *pTDES, uint16_t *pTDES_Len);
void Clingpay_Command_SHA1(uint8_t *pSHA, uint16_t *pSHA_Len);
void Clingpay_Command_Rand(uint8_t *pRND, uint16_t *pRND_Len);
void Clingpay_Command_RSA_Generate(uint8_t *pRSA, uint16_t *pRSA_Len);
void Clingpay_Command_RSA_Calculate(uint8_t *pRSA, uint16_t *pRSA_Len);
void Clingpay_Command_MAC(uint8_t *pMAC, uint16_t *pMAC_Len);
void Clingpay_Command_HMAC(uint8_t *pHMAC, uint16_t *pHMAC_Len);
int clingpay_cmd_init(TASK_HANDLE_SETS *sets);
int reg_cling_pay_cmd_callback(int (*cb)(struct pay_rev_package*));
int set_z32_rev_cb(int (*cb)(uint8_t*, uint16_t));
void Clingpay_Deactivate_ISO7816(void);
#endif

