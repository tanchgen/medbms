/*
 * bq30z55.c
 *
 *  Created on: 25 дек. 2023 г.
 *      Author: jet
 */

#include "main.h"
#include "hash.h"
#include "bq30z55.h"


const uint8_t bq30zIdKey[16] = {
    0x66, 0x99, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

sBq30z55Dev bq30z55Dev;

// --------------------- ACCESS FUNCTION ----------------------------------
//
eErr bq30zRng( rngM ){
  eErr rc = ERR_OK;

  // TODO: Заполнить 160бит RNG
  return rc;
}


// Расчет SHA-1
eErr sha1( const uint8_t * str, int32_t strsize, uint8_t * hash, uint8_t * hashSize ){
  eErr rc = ERR_OK;

  SHA1ctx_stt sha1Ctx = {
    .mFlags = E_HASH_DEFAULT,
    // 20 byte of output
    .mTagSize = CRL_SHA1_SIZE,
  };

  // Init SHA-1
  rc = SHA1_Init( &sha1Ctx );
  if (rc == ERR_OK ) {
    // Process the message with SHA-1
    rc = SHA1_Append( &sha1Ctx, (const uint8_t *)str, strsize);
    if (rc == ERR_OK) {
      // Output the Digest
      rc = SHA1_Finish( &sha1Ctx, hash, (int32_t *)hashSize );
    }
  }

  return rc;
}


// Расчет HMAC_SHA-1
eErr hmac_sha1( const uint8_t * str, int32_t strsize, uint8_t * hash, uint8_t * hashSize ){
  eErr rc = ERR_OK;

  HMAC_SHA1ctx_stt sha1Ctx = {
    .mFlags = E_HASH_DEFAULT,
    // 20 byte of output
    .mTagSize = CRL_SHA1_SIZE,
  };

  // Init SHA-1
  rc = HMAC_SHA1_Init( &sha1Ctx );
  if (rc == ERR_OK ) {
    // Process the message with SHA-1
    rc = HMAC_SHA1_Append( &sha1Ctx, (const uint8_t *)str, strsize);
    if (rc == ERR_OK) {
      // Output the Digest
      rc = HMAC_SHA1_Finish( &sha1Ctx, hash, (int32_t *)hashSize );
    }
  }

  return rc;
}


// Authentication BQ30Z55
void bq30z55Auth( void ){
  /* 1. Generate 160-bit message M using a random number generator that meets
   *    approved random number generators described in FIPS PUB 140–2.
   * 2. Generate SHA-1 input block B1 of 512 bytes
   *    (total input =128-bit authentication key KD + 160 bit message M + 1 + 159 0s + 100100000).
   * 3. Generate SHA-1 hash HMAC1 using B1.
   * 4. Generate SHA-1 input block B2 of 512 bytes
   *    (total input =128-bit authentication key KD + 160 bit hash * HMAC1 + 1 + 159 0s + 100100000).
   * 5. Generate SHA-1 hash HMAC2 using B2.
   * 6. With no active ManufacturerInput() data waiting, write 160-bit message M
   *    to ManufacturerInput() in the format 0xAABBCCDDEEFFGGHHIIJJKKLLMMNNOOPPQQRRSSTT,
   *    where AA is LSB.
   * 7. Wait 250 ms, then read ManufacturerInput() for HMAC3.
   * 8. Compare host HMAC2 with device HMAC3, it matches, both host and device have
   *    the same key KD and device is authenticated.
   */
  // TODO: Make authentication process
}


// UNSEAL/FULL_ACCESS BQ30Z55
void bq30z55Auth( void ){
  /*
   * 1. Send Unseal (0x0031) or Full Access (0x0032) command to ManufacturerAccess().
   * 2. Read 160-bit message M from ManufacturerInput() in the
   *    format 0xAABBCCDDEEFFGGHHIIJJKKLLMMNNOOPPQQRRSSTT, where AA is LSB.
   * 3. Generate SHA-1 input block B1 of 512 bytes
   *    (total input =128-bit unseal/full access key KD + 160 bit message M + 1 + 159 0s + 100100000).
   * 4. Generate SHA-1 hash HMAC1 using B1. 5. Generate SHA-1 input block B2 of 512 bytes
   *    (total input =128-bit unseal/full access key KD + 160 bit hash HMAC1 + 1 + 159 0s + 100100000).
   * 6. Generate SHA-1 hash HMAC2 using B2.
   * 7. Write 160-bit hash HMAC2 to ManufacturerInput() in the format 0xAABBCCDDEEFFGGHHIIJJKKLLMMNNOOPPQQRRSSTT, where AA is LSB.
   * 8. Device compares hash HMAC2 with internal calculated hash HMAC3. If it matches, device allows UNSEALED/FULL ACCESS mode indicated with the OperationStatus()[SEC1],[SEC0] flags.
   */
  // TODO: Make Unseal/Full access process
}


void bq30z55Init( void ){
  bq30z55Dev.cfgSequence = bq30CfgSeqNull;
  bq30z55Dev.inSequence = bq30InSeqNull;
  bq30z55Dev.cfgFlag = RESET;
}

