/*
 *  Access routines for SPARC
 *
 *  COPYRIGHT (c) 2011
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 */

#include <rtems/asm.h>

	.align 4
	.seg "text"
	PUBLIC(_ld8)
	PUBLIC(_ld16)
	PUBLIC(_ld32)
	PUBLIC(_ld64)
	PUBLIC(_st8)
	PUBLIC(_st16)
	PUBLIC(_st32)
	PUBLIC(_st64)
	PUBLIC(_ld_be16)
	PUBLIC(_ld_be32)
	PUBLIC(_st_be16)
	PUBLIC(_st_be32)

SYM(_ld8):
	retl
	 ldub [%o0], %o0

SYM(_ld_be16):
SYM(_ld16):
	retl
	 lduh [%o0], %o0

SYM(_ld_be32):
SYM(_ld32):
	retl
	 ld [%o0], %o0

SYM(_ld_be64):
SYM(_ld64):
	retl
	 ldd [%o0], %o0

SYM(_st8):
	retl
	 stub %o1, [%o0]

SYM(_st_be16):
SYM(_st16):
	retl
	 stuh %o1, [%o0]

SYM(_st_be32):
SYM(_st32):
	retl
	 st %o1, [%o0]

SYM(_st_be64):
SYM(_st64):
	retl
	 std %o1, [%o0]
