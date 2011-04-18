/*
 *  Access routines for SPARC
 *
 *  COPYRIGHT (c) 2011
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#include <rtems/asm.h>

        .align 4
	.seg "text"
        PUBLIC(sparc_ld8)
        PUBLIC(sparc_ld16)
	PUBLIC(sparc_ld32)
	PUBLIC(sparc_ld64)
	PUBLIC(sparc_st8)
	PUBLIC(sparc_st16)
	PUBLIC(sparc_st32)
	PUBLIC(sparc_st64)
        PUBLIC(sparc_ld_be16)
	PUBLIC(sparc_ld_be32)
	PUBLIC(sparc_st_be16)
	PUBLIC(sparc_st_be32)

SYM(sparc_ld8):
	retl
	 ldub [%o0], %o0

SYM(sparc_ld_be16):
SYM(sparc_ld16):
	retl
	 lduh [%o0], %o0

SYM(sparc_ld_be32):
SYM(sparc_ld32):
	retl
	 ld [%o0], %o0

SYM(sparc_ld_be64):
SYM(sparc_ld64):
	retl
	 ldd [%o0], %o0

SYM(sparc_st8):
	retl
	 stub %o1, [%o0]

SYM(sparc_st_be16):
SYM(sparc_st16):
	retl
	 stuh %o1, [%o0]

SYM(sparc_st_be32):
SYM(sparc_st32):
	retl
	 st %o1, [%o0]

SYM(sparc_st_be64):
SYM(sparc_st64):
	retl
	 std %o1, [%o0]
