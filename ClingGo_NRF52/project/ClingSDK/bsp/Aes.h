/****************************************************************************
Classification: U//FOUO
*//**
    @file   Aes.h
    @brief  Header for Aes.c
****************************************************************************/

#define AES_MAXROUNDS   14  /**< @brief max potential rounds */
#define AES_BLOCKSIZE   16  /**< @brief Blocksize of aes encryption - 128bit */
#define AES_IV_SIZE     16  /**< @brief Initalization vector size */


#ifndef htons
#define htons(A) ((((uint16_t)(A) & 0xff00) >> 8) | \
(((uint16_t)(A) & 0x00ff) << 8))
#define htonl(A) ((((uint32_t)(A) & 0xff000000) >> 24) | \
(((uint32_t)(A) & 0x00ff0000) >> 8) | \
(((uint32_t)(A) & 0x0000ff00) << 8) | \
(((uint32_t)(A) & 0x000000ff) << 24))
#define ntohs htons
#define ntohl htonl
#endif

#define rot1(x) (((x) << 24) | ((x) >> 8))
#define rot2(x) (((x) << 16) | ((x) >> 16))
#define rot3(x) (((x) <<  8) | ((x) >> 24))

#define mt  0x80808080
#define mh  0xfefefefe
#define mm  0x1b1b1b1b
#define mul2(x,t)	((t)=((x)&mt), ((((x)+(x))&mh)^(((t)-((t)>>7))&mm)))

#define inv_mix_col(x,f2,f4,f8,f9) (\
(f2)=mul2(x,f2), \
(f4)=mul2(f2,f4), \
(f8)=mul2(f4,f8), \
(f9)=(x)^(f8), \
(f8)=((f2)^(f4)^(f8)), \
(f2)^=(f9), \
(f4)^=(f9), \
(f8)^=rot3(f2), \
(f8)^=rot2(f4), \
(f8)^rot1(f9))



/** @brief Specifies keylength */
typedef enum
{
    AES_MODE_128,
    AES_MODE_256
} AES_MODE;

/** @brief This structure holds the AES key context */
typedef struct aes_key_st 
{
    uint16_t rounds;                    /**< @brief # of rounds - based on keysize */
    uint16_t key_size;                  /**< @brief keysize int bits */
    uint32_t ks[(AES_MAXROUNDS+1)*8];   /**< @brief hold the different keystates */
    uint8_t  iv[AES_IV_SIZE];           /**< @brief holds the initilaization vector */
} AES_CTX;



/* ----- static functions ----- */
void AES_encrypt(const AES_CTX *ctx, uint32_t *data);
void AES_decrypt(const AES_CTX *ctx, uint32_t *data);
void AES_hncpy32(uint32_t * dst, uint32_t * src);
uint8_t AES_xtime(uint32_t x);
void AES_generateSBox(void);



/****************************************************************************
AES_set_key
*//**
    @brief  Setup the AES key context

    @param[in]  ctx     AES context structure
    @param[in]  key     The buffer containing the key to set
    @param[in]  iv      Initlaization vector data
    @param[in]  mode 	Enum of supported AES keysizes
****************************************************************************/
void AES_set_key(AES_CTX *ctx, const uint8_t *key, const uint8_t *iv, AES_MODE mode);

/****************************************************************************
AES_convert_key
*//**
    @brief  Converts the AES key context from encryption to decryption

    @param[in]  ctx     AES context structure
****************************************************************************/
void AES_convert_key(AES_CTX *ctx);

/****************************************************************************
AES_cbc_encrypt
*//**
    @brief  Performs CBC encryption

    Performs CBC encryption on the msg buffer, output needs to be a valid
    allocated memory location of approporaite length. The output buffer can
    be the same as the input buffer in which the data will be directly 
    encrypted.

    @param[in]  ctx     Initialized AES context structure
    @param[in]  msg     The buffer to perform encryption on
    @param[out] out     The buffer to put the encryption data on
    @param[in]  length  The length of the msguffer in bytes, must be evenly
                        divisible the AES blocksize of 16.
****************************************************************************/
void AES_cbc_encrypt(AES_CTX *ctx, const uint8_t *msg, uint8_t *out, int length);

/****************************************************************************
AES_cbc_decrypt
*//**
    @brief  Performs CBC decryption

    Performs CBC decryption on the msg buffer, output needs to be a valid
    allocated memory location of approporaite length. The output buffer can
    be the same as the input buffer in which the data will be directly 
    decrypted.

    @param[in]  ctx     Initialized AES context structure
    @param[in]  msg     The buffer to perform decryption on
    @param[out] out     The buffer to put the decryption data on
    @param[in]  length  The length of the msguffer in bytes, must be evenly
                        divisible the AES blocksize of 16.
****************************************************************************/
void AES_cbc_decrypt(AES_CTX *ctx, const uint8_t *msg, uint8_t *out, int length);
