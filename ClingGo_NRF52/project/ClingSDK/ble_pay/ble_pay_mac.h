#ifndef  __UART_PROTOCAL_MAC_H__
#define __UART_PROTOCAL_MAC_H__

#include "oop_hal.h"
#include "stdint.h"
#include "stdio.h"
struct mac_layer_payload_rev{
        unsigned char *ppayload;/*uart paly load pointer*/
        int lenth;/*data lenth*/
        unsigned char num; /*current state like wai*/
        struct mac_layer_payload_rev *next;
};



struct mac_layer_payload_send{
        unsigned char  *ppayload;       /*uart paly load pointer*/
        int lenth;              /*data lenth*/
        unsigned char  resend_times;/* recoed times the uart has been sended*/
        unsigned char state;            /*current state like wai*/
        struct mac_layer_payload_send *next;
};

/*object prototype declaration*/
DEF_CLASS(uart_implement)
        int (*init)     (CLASS(uart_implement) *arg);                   /*initiate uart object*/
        int (*de_init)  (CLASS(uart_implement) *arg);                           /*delete uart object*/
        int (*get_data)         (char *pinf, int lenth);/*data send   function*/
        int (*send_data)        (char *pinf, int lenth);/*data send   function*/
        int (*enable)           ();/*data send   function*/
        int (*disable)  ();/*data send   function*/
END_DEF_CLASS(uart_implement)


int obtain_payload_from_revlist(struct mac_layer_payload_rev **ppayload);
int mac_send_payload(char *ppayload, int lenth);
void set_recieved_cmd_call_back(void (*callback)(char));
/*maintain serial send list*/
int mac_sendlist_mantain_demon(void);
int register_write_function(int (*write)(uint8_t* , size_t));
int receive_one_char_callback(unsigned char rev_char);
int register_serial_fd(int fd);
extern int clingpay_mac_init(void);
void set_recieved_data_call_back(void (*callback)(uint8_t*, int));
int cling_pay_mac_init(void);
#endif

