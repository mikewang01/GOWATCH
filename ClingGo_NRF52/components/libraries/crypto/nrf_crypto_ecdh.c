/* Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(NRF_CRYPTO)
 
#include "nrf_crypto_ecdh.h"
#include "nrf_crypto_mem.h"

#if NRF_CRYPTO_BACKEND_MICRO_ECC
#include "micro_ecc_lib_ecdh.h"
#endif

uint32_t nrf_crypto_ecdh_shared_secret_size_get(nrf_ecc_curve_type_t    curve_type,
                                                uint32_t              * p_size)
{
    if(p_size == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    switch(curve_type)
    {
        case NRF_CRYPTO_CURVE_SECP160R1:
            (*p_size)= NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP160R1;
            break;
        
        case NRF_CRYPTO_CURVE_SECP192R1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP192R1;
            break;
        
        case NRF_CRYPTO_CURVE_SECP224R1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP224R1;            
            break;
        
        case NRF_CRYPTO_CURVE_SECP256R1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP256R1;
            break;
        
        case NRF_CRYPTO_CURVE_SECP384R1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP384R1;
            break;
        
        case NRF_CRYPTO_CURVE_SECP521R1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP521R1;
            break;
        
        case NRF_CRYPTO_CURVE_SECP192K1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP192K1;
            break;
        
        case NRF_CRYPTO_CURVE_SECP224K1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP224K1;
            break;
        
        case NRF_CRYPTO_CURVE_SECP256K1:
            (*p_size) = NRF_CRYPTO_ECDH_SHARED_SECRET_SIZE_SECP256K1;
            break;
        
        default:
            (*p_size) = 0;
            return NRF_ERROR_NOT_SUPPORTED;
    }
    
    return NRF_SUCCESS;
}


uint32_t nrf_crypto_ecdh_shared_secret_allocate(nrf_crypto_curve_info_t     curve_type,
                                                nrf_value_length_t        * p_shared_secret,
                                                nrf_value_length_t  const * p_raw_shared_secret)
{
    uint32_t  ret_val;
    uint32_t  shared_secret_size;
    
    if (p_shared_secret == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    ret_val = nrf_crypto_ecdh_shared_secret_size_get(curve_type.curve_type, &shared_secret_size);
    if (ret_val != NRF_SUCCESS)
    {
        return ret_val;
    }
    
    ret_val = nrf_crypto_mem_allocate(shared_secret_size, p_shared_secret);
    if (ret_val == NRF_SUCCESS)
    {
        return ret_val;
    }
    
    return ret_val;
}


uint32_t nrf_crypto_ecdh_shared_secret_free(nrf_value_length_t * p_shared_secret)
{
    uint32_t ret_val;
    
    if (p_shared_secret == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    // Memory was already freed
    if (p_shared_secret->length == 0)
    {
        return NRF_SUCCESS;
    }
    
    ret_val = nrf_crypto_mem_free(p_shared_secret);
    return ret_val;
}


#endif //  NRF_MODULE_ENABLED(NRF_CRYPTO)
