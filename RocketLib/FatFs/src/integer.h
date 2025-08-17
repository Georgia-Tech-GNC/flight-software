/*-------------------------------------------*/
/* Integer type definitions for FatFs module */
/*-------------------------------------------*/

#ifndef _FF_INTEGER
#define _FF_INTEGER

#ifdef _WIN32	/* FatFs development platform */

#include <windows.h>
#include <tchar.h>
typedef unsigned __int64 QWORD;


#else			/* Embedded platform */

/* These types MUST be 16-bit or 32-bit */
typedef int				INT;
typedef unsigned int	UINT;
_Static_assert((sizeof(INT) == 2 || sizeof(INT) == 4) && (sizeof(UINT) == 2 || sizeof(UINT) == 4));

/* This type MUST be 8-bit */
typedef unsigned char	BYTE;
_Static_assert(sizeof(BYTE) == 1);

/* These types MUST be 16-bit */
typedef short			SHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;
_Static_assert(sizeof(SHORT) == 2 && sizeof(WORD) == 2 && sizeof(WCHAR) == 2);

/* These types MUST be 32-bit */
typedef long			LONG;
typedef unsigned long	DWORD;
_Static_assert(sizeof(LONG) == 4 && sizeof(DWORD) == 4);

/* This type MUST be 64-bit (Remove this for ANSI C (C89) compatibility) */
typedef unsigned long long QWORD;
_Static_assert(sizeof(QWORD) == 8);

#endif

#endif
