/***************************************************************************
 *
 * PURPOSE
 *   RLduino78 framework interrupt header file.
 *
 * TARGET DEVICE
 *   RL78/G13
 *
 * AUTHOR
 *   Renesas Solutions Corp.
 *
 * $Date:: 2012-12-18 17:02:26 +0900#$
 *
 ***************************************************************************
 * Copyright (C) 2012 Renesas Electronics. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * See file LICENSE.txt for further informations on licensing terms.
 ***************************************************************************/
/*
  Modified 13 July 2014 by Nozomu Fujita
*/
/**
 * @file  pgmspace.h
 * @brief RLduino78フレームワーク PGMSPACEヘッダ・ファイル
 */
#ifndef PGMSPACE_H
#define PGMSPACE_H
/***************************************************************************/
/*    Include Header Files                                                 */
/***************************************************************************/


/***************************************************************************/
/*    Macro Definitions                                                    */
/***************************************************************************/
#ifndef PGM_P
#define PGM_P const prog_char *
#endif

#define PROGMEM

#define pgm_read_byte(address_short)    (*(char*)(address_short))
#define pgm_read_byte_near(x)           pgm_read_byte(x)
#define pgm_read_word(address_short)    (*(short int*)(address_short))
#define pgm_read_word_near(x)           pgm_read_word(x)
#define pgm_read_dword(address_long)    (*(long int*)(address_long))

#define PSTR(s) ((const char *)(s))

#define memchr_P memchr
#define memcmp_P memcmp
#define memccpy_P memccpy
#define memcpy_P memcpy
#define memcpy_P memcpy
#define memmem_P memmem
#define memrchr_P memrchr
#define strcat_P strcat
#define strchr_P strchr
#define strchrnul_P strchrnul
#define strcmp_P strcmp
#define strcpy_P strcpy
#define strcasecmp_P strcasecmp
#define strcasestr_P strcasestr
#define strcspn_P strcspn
#define strlcat_P strlcat
#define strlcpy_P strlcpy
#define strlen_P strlen
#define __strlen_P strlen
#define strnlen_P strnlen
#define strncmp_P strncmp
#define strncasecmp_P strncasecmp
#define strncat_P strncat
#define strncpy_P strncpy
#define strpbrk_P strpbrk
#define strrchr_P strrchr
#define strsep_P strsep
#define strspn_P strspn
#define strstr_P strstr
#define strtok_P strtok
#define strtok_rP strtok
#define strlen_PF strlen
#define strnlen_PF strnlen
#define memcpy_PF memcpy
#define strcpy_PF strcpy
#define strncpy_PF strncpy
#define strcat_PF strcat
#define strlcat_PF strlcat
#define strncat_PF strncat
#define strcmp_PF strcmp
#define strncmp_PF strncmp
#define strcasecmp_PF strcasecmp
#define strncasecmp_PF strncasecmp
#define strstr_PF strstr
#define strlcpy_PF strlcpy
#define memcmp_PF memcmp


/***************************************************************************/
/*    Type  Definitions                                                    */
/***************************************************************************/
typedef char prog_char;


/***************************************************************************/
/*    Function prototypes                                                  */
/***************************************************************************/


/***************************************************************************/
/*    Global Variables                                                     */
/***************************************************************************/


/***************************************************************************/
/*    Local Variables                                                      */
/***************************************************************************/


/***************************************************************************/
/*    Global Routines                                                      */
/***************************************************************************/


/***************************************************************************/
/*    Local Routines                                                       */
/***************************************************************************/


/***************************************************************************/
/* End of module                                                           */
/***************************************************************************/
#endif /* PGMSPACE_H */
