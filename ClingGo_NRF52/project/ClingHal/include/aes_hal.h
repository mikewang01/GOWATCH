/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : aes_hal.h
 * Author        : MikeWang
 * Date          : 2016-03-28
 * Description   : aes_hal.c header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2016-03-28
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef __AES_HAL_H__
#define __AES_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
//#include "hw_aes_hash.h"
//#define __WIRST_DETECT_ENABLE__
#define  _AES_ENABLED_
#define SOC_ECB_KEY_LENGTH                (16)                       /**< ECB key length. */
#define SOC_ECB_CLEARTEXT_LENGTH          (16)                       /**< ECB cleartext length. */
#define SOC_ECB_CIPHERTEXT_LENGTH         (SOC_ECB_CLEARTEXT_LENGTH) /**< ECB ciphertext length. */
#define  hw_aes_hash_cb int
/** An interface for the aes_hal to take media loacated in host device under control
 *
 * @code
 */
/*object prototype declaration*/
/*class defination header*/
DEF_CLASS(HalAes)
        int (*aes_encrypt)(CLASS(HalAes) *arg, uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb);
        int (*aes_decrypt)(CLASS(HalAes) *arg, uint8_t *key_p, uint8_t *in_p, uint8_t *out_p, hw_aes_hash_cb cb);
        int (*clock_enable)(CLASS(HalAes) *arg);
        int (*clock_disable)(CLASS(HalAes) *arg);
END_DEF_CLASS(HalAes)

CLASS(HalAes)* HalAes_get_instance(void);
/*================================CONFIGRATION===============================================*/
#define CLING_HW_ASE_INIT() do{\
        HalAes_get_instance()\
}while(0)


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __AES_HAL_H__ */
