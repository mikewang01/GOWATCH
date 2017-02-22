/****************************************************************************
Classification: U//FOUO
*//**
	@file   Aes.c
	@brief  Handles AES computation.
	@author Andrew Gorczyca
	@date   2012/5/25

	Based on axtls implementation, by Cameron Rich. Improvements made mostly
	to cbc handling, reducing unnecessary copying and intermediary buffers.
	Added code to generate both Rijndael S-box lookups in memory. Reduced
	Round constants to ones actally used. mul2 from Dr. Gladman. I've only 
	personally tested this on little Endian machines, but it might 
	*possibly* work on a big endian platform.
****************************************************************************/


#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "Aes.h"
#include "pedo_api.h"

/* ----- functions ----- */
//static void AES_encrypt(const AES_CTX *ctx, uint32_t *data);
//static void AES_decrypt(const AES_CTX *ctx, uint32_t *data);
//static void AES_hncpy32(uint32_t * dst, uint32_t * src);
//static uint8_t AES_xtime(uint32_t x);
//static void AES_generateSBox(void);


uint8_t *aes_sbox;		/** AES S-box  */
uint8_t *aes_isbox;	/** AES iS-box */

/** AES round constants */
const uint8_t Rcon[]= 
{
	0x01,0x02,0x04,0x08,0x10,0x20,
	0x40,0x80,0x1b,0x36,0x6c,0xd8
};



/* Perform doubling in Galois Field GF(2^8) using the irreducible polynomial
   x^8+x^4+x^3+x+1 */
uint8_t AES_xtime(uint32_t x)
{
	return (x&0x80) ? (x<<1)^0x1b : x<<1;
}

/** Set up AES with the key/iv and cipher size. */
void AES_set_key(AES_CTX *ctx, const uint8_t *key, const uint8_t *iv, AES_MODE mode)
{
	int i, words;
	uint32_t tmp, tmp2;
	const uint8_t * rtmp = Rcon;

	memset(ctx, 0, sizeof(AES_CTX));
	
	switch (mode)
	{
	case AES_MODE_128:
		ctx->rounds = 10;
		words = 4;
		break;
	case AES_MODE_256:
		ctx->rounds = 14;
		words = 8;
		AES_hncpy32((uint32_t *)((ctx->ks)+4), (uint32_t *)(key + AES_BLOCKSIZE));
		break;
	default:        /* fail silently */
		return;
	}

	AES_hncpy32((uint32_t *)(ctx->ks), (uint32_t *)key);
	ctx->key_size = words;

		for (i = words; i <  4 * (ctx->rounds+1); i++)
	{
		tmp = ctx->ks[i-1];

		if ((i % words) == 0)
		{
			tmp2 =(uint32_t)aes_sbox[(tmp    )&0xff]<< 8;
			tmp2|=(uint32_t)aes_sbox[(tmp>> 8)&0xff]<<16;
			tmp2|=(uint32_t)aes_sbox[(tmp>>16)&0xff]<<24;
			tmp2|=(uint32_t)aes_sbox[(tmp>>24)     ];
			tmp=tmp2^(((uint32_t)*rtmp)<<24);
			rtmp++;
		}

		if ((words == 8) && ((i % words) == 4))
		{
			tmp2 =(uint32_t)aes_sbox[(tmp    )&0xff]    ;
			tmp2|=(uint32_t)aes_sbox[(tmp>> 8)&0xff]<< 8;
			tmp2|=(uint32_t)aes_sbox[(tmp>>16)&0xff]<<16;
			tmp2|=(uint32_t)aes_sbox[(tmp>>24)     ]<<24;
			tmp=tmp2;
		}

		ctx->ks[i]=ctx->ks[i-words]^tmp;
	}

	/* copy the iv across */
	for (i = 0; i < AES_BLOCKSIZE; i++) {
		ctx->iv[0] = 0;
	}
}

/** Change a key for decryption. */
void AES_convert_key(AES_CTX *ctx)
{
	int i;
	uint32_t *k,w,t1,t2,t3,t4;

	k = ctx->ks;
	k += 4;

	for (i= ctx->rounds*4; i > 4; i--)
	{
		w= *k;
		w = inv_mix_col(w,t1,t2,t3,t4);
		*k++ =w;
	}
}

void AES_hncpy32(uint32_t * dst, uint32_t * src)
{
	dst[0] = htonl(src[0]);
	dst[1] = htonl(src[1]);
	dst[2] = htonl(src[2]);
	dst[3] = htonl(src[3]);
}
#if 0
/** Encrypt a byte sequence (with a block size 16) using the AES cipher. */
void AES_cbc_encrypt(AES_CTX *ctx, const uint8_t *msg, uint8_t *out, int length)
{
	uint32_t buf[4];

	AES_hncpy32(buf,(uint32_t *)(ctx->iv));
	for (length -= AES_BLOCKSIZE; length >= 0; length -= AES_BLOCKSIZE)
	{
	
		buf[0] = ntohl(((uint32_t *)msg)[0])^buf[0];
		buf[1] = ntohl(((uint32_t *)msg)[1])^buf[1];
		buf[2] = ntohl(((uint32_t *)msg)[2])^buf[2];
		buf[3] = ntohl(((uint32_t *)msg)[3])^buf[3];

		AES_encrypt(ctx, buf);

		AES_hncpy32((uint32_t *)out,buf);
		
		msg += AES_BLOCKSIZE;
		out += AES_BLOCKSIZE;
    }
	out-=AES_BLOCKSIZE;

	memcpy(ctx->iv,out,AES_BLOCKSIZE);
}
#endif
/** Decrypt a byte sequence (with a block size 16) using the AES cipher. */
void AES_cbc_decrypt(AES_CTX *ctx, const uint8_t *msg, uint8_t *out, int length)
{
	uint32_t tin[4], xor[4], buf[4];
	AES_hncpy32(xor,(uint32_t *)(ctx->iv));
	for (length -= AES_BLOCKSIZE; length >= 0; length -= AES_BLOCKSIZE)
	{
		AES_hncpy32(tin,(uint32_t *)msg);
		memcpy(buf,tin,AES_BLOCKSIZE);
		AES_decrypt(ctx, buf);
		
		((uint32_t *)out)[0] = htonl(buf[0]^xor[0]);
		((uint32_t *)out)[1] = htonl(buf[1]^xor[1]);
		((uint32_t *)out)[2] = htonl(buf[2]^xor[2]);
		((uint32_t *)out)[3] = htonl(buf[3]^xor[3]);
		
		memcpy(xor,tin,AES_BLOCKSIZE);

		msg += AES_BLOCKSIZE;
		out += AES_BLOCKSIZE;
	}

	AES_hncpy32((uint32_t *)(ctx->iv),tin);
}
#if 0
/** Encrypt a single block (16 bytes) of data */
void AES_encrypt(const AES_CTX *ctx, uint32_t *data)
{
    /* To make this code smaller, generate the sbox entries on the fly.
     * This will have a really heavy effect upon performance.
     */
    uint32_t tmp[4];
    uint32_t tmp1, old_a0, a0, a1, a2, a3, row;
    int curr_rnd;
    int rounds = ctx->rounds; 
    const uint32_t *k = ctx->ks;

    /* Pre-round key addition */
    for (row = 0; row < 4; row++)
        data[row] ^= *(k++);

    /* Encrypt one block. */
    for (curr_rnd = 0; curr_rnd < rounds; curr_rnd++)
    {
        /* Perform ByteSub and ShiftRow operations together */
        for (row = 0; row < 4; row++)
        {
            a0 = (uint32_t)aes_sbox[(data[row%4]>>24)&0xFF];
            a1 = (uint32_t)aes_sbox[(data[(row+1)%4]>>16)&0xFF];
            a2 = (uint32_t)aes_sbox[(data[(row+2)%4]>>8)&0xFF]; 
            a3 = (uint32_t)aes_sbox[(data[(row+3)%4])&0xFF];

            /* Perform MixColumn iff not last round */
            if (curr_rnd < (rounds - 1))
            {
                tmp1 = a0 ^ a1 ^ a2 ^ a3;
                old_a0 = a0;
                a0 ^= tmp1 ^ AES_xtime(a0 ^ a1);
                a1 ^= tmp1 ^ AES_xtime(a1 ^ a2);
                a2 ^= tmp1 ^ AES_xtime(a2 ^ a3);
                a3 ^= tmp1 ^ AES_xtime(a3 ^ old_a0);
            }

            tmp[row] = ((a0 << 24) | (a1 << 16) | (a2 << 8) | a3);
        }

        /* KeyAddition - note that it is vital that this loop is separate from
           the MixColumn operation, which must be atomic...*/ 
        for (row = 0; row < 4; row++)
            data[row] = tmp[row] ^ *(k++);
    }
}
#endif
/** Decrypt a single block (16 bytes) of data */
void AES_decrypt(const AES_CTX *ctx, uint32_t *data)
{ 
    uint32_t tmp[4];
    uint32_t xt0,xt1,xt2,xt3,xt4,xt5,xt6;
    uint32_t a0, a1, a2, a3, row;
    int curr_rnd;
    int rounds = ctx->rounds;
    const uint32_t *k = ctx->ks + ((rounds+1)*4);

    /* pre-round key addition */
    for (row=4; row > 0;row--)
        data[row-1] ^= *(--k);

    /* Decrypt one block */
	for (curr_rnd = 0; curr_rnd < rounds; curr_rnd++)
	{
		/* Perform ByteSub and ShiftRow operations together */
		for (row = 4; row > 0; row--)
		{
			a0 = aes_isbox[(data[(row+3)%4]>>24)&0xFF];
			a1 = aes_isbox[(data[(row+2)%4]>>16)&0xFF];
			a2 = aes_isbox[(data[(row+1)%4]>>8)&0xFF];
			a3 = aes_isbox[(data[row%4])&0xFF];

			/* Perform MixColumn iff not last round */
			if (curr_rnd<(rounds-1))
			{
				/* The MDS cofefficients (0x09, 0x0B, 0x0D, 0x0E)
					are quite large compared to encryption; this 
					operation slows decryption down noticeably. */
				xt0 = AES_xtime(a0^a1);
				xt1 = AES_xtime(a1^a2);
				xt2 = AES_xtime(a2^a3);
				xt3 = AES_xtime(a3^a0);
				xt4 = AES_xtime(xt0^xt1);
				xt5 = AES_xtime(xt1^xt2);
				xt6 = AES_xtime(xt4^xt5);

				xt0 ^= a1^a2^a3^xt4^xt6;
				xt1 ^= a0^a2^a3^xt5^xt6;
				xt2 ^= a0^a1^a3^xt4^xt6;
				xt3 ^= a0^a1^a2^xt5^xt6;
				tmp[row-1] = ((xt0<<24)|(xt1<<16)|(xt2<<8)|xt3);
			}
			else
				tmp[row-1] = ((a0<<24)|(a1<<16)|(a2<<8)|a3);
		}
		for (row = 4; row > 0; row--)
			data[row-1] = tmp[row-1] ^ *(--k);
	}
}

void AES_generateSBox(void)
{
//	uint32_t *t;
	uint32_t x, i, y;
	I8U t[256];
	
	aes_sbox = PEDO_get_global_buffer();
	aes_isbox = aes_sbox+256;

	for (i = 0, x = 1; i < 256; i ++)
	{
		t[i] = (I8U)x;
		x ^= (x << 1) ^ ((x >> 7) * 0x11B);
	}

	aes_sbox[0]	= 0x63;
	for (i = 0; i < 255; i ++)
	{
		x = t[255 - i];
		x |= x << 8;
		x ^= (x >> 4) ^ (x >> 5) ^ (x >> 6) ^ (x >> 7);
		y = (x ^ 0x63) & 0xFF;
		aes_sbox[t[i]] = (I8U)y;
	}

	for (i = 0; i < 256;i++)
	{
		aes_isbox[aes_sbox[i]]=i;
	}
}



