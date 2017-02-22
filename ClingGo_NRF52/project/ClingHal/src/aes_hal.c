/******************************************************************************
* Copyright 2013-2014 hicling Systems (MikeWang)
*
* FileName: rtc_hal.c
*
* Description: abstract layer between cling sdk and freertos.
*
* Modification history:
*     2016/7/22, v1.0 create this file mike.
*******************************************************************************/


#include "oop_hal.h"
//#include "ad_flash.h"
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "aes_hal.h"
#include "Aes.h"
/*********************************************************************
* MACROS
*/
#ifdef __cplusplus
extern "C" {
#endif


/*********************************************************************
* TYPEDEFS
*/


/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/
static CLASS(HalAes)* p_instance = NULL;
//static OS_TIMER rtc_timer;
/*********************************************************************
* LOCAL DECLARATION
*/
CLASS(HalAes)* HalAes_get_instance(void);
int HalAes_init(CLASS(HalAes) *arg);
static int decrypt(CLASS(HalAes) *arg, uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb);
static int encrypt(CLASS(HalAes) *arg, uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb);
static int enable_clock(CLASS(HalAes) *arg);
static int disable_clock(CLASS(HalAes) *arg);
/******************************************************************************
* FunctionName :HalAes_get_instance
* Description  : get hal rtc instance
* Parameters   : write: write call back function
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
CLASS(HalAes)* HalAes_get_instance(void)
{
        static CLASS(HalAes) p;
        if (p_instance == NULL) {
                p_instance = &p;
                memset(&p, 0, sizeof(p));
                HalAes_init(p_instance);
        }
        return p_instance;
}

/******************************************************************************
* FunctionName :HalAes_init
* Description  : init hal rtc instance
* Parameters   : object pointer
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
int HalAes_init(CLASS(HalAes) *arg)
{
        arg->aes_decrypt  = decrypt;
        arg->aes_encrypt = encrypt;
        arg->clock_enable = enable_clock;
        arg->clock_disable = disable_clock;
        return 0;
}

static int enable_clock(CLASS(HalAes) *arg)
{
        //hw_aes_hash_enable_clock();
        return 0;
}
static int disable_clock(CLASS(HalAes) *arg)
{
       // hw_aes_hash_disable_clock();
        return 0;
}

/******************************************************************************
* FunctionName : encrypt
* Description  : encrypt the data passed in according to key
* Parameters   : uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb: call back function when decryption finished
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/

static int encrypt(CLASS(HalAes) *arg, uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb)
{
		// Set up hal_aes using new key and init vector
			nrf_ecb_hal_data_t ecb_ctx;
			
			// Set ECB key
			memcpy(ecb_ctx.key, key_p, SOC_ECB_KEY_LENGTH);
			// Set ECB text input
			memcpy(ecb_ctx.cleartext, in_p, SOC_ECB_CLEARTEXT_LENGTH);
			// Encryption
			sd_ecb_block_encrypt(&ecb_ctx);
			
			// get encrypted text
			memcpy(out_p, ecb_ctx.ciphertext, SOC_ECB_CLEARTEXT_LENGTH);
		  return 0;
//        arg->isr_reg;
}

/******************************************************************************
* FunctionName : decrypt
* Description  : decprpt the data passed in according to key
* Parameters   : uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb: call back function when decryption finished
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
static int decrypt(CLASS(HalAes) *arg, uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb)
{
     AES_CTX ctx;

	// Generate sbox code table
		AES_generateSBox();

	// Set keys
		AES_set_key(&ctx, key_p, NULL, AES_MODE_128);

		AES_convert_key(&ctx);

		AES_cbc_decrypt(&ctx, in_p, out_p, AES_BLOCKSIZE*4);
              return 0;
}


#ifdef __cplusplus
}
#endif
