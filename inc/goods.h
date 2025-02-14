#ifndef GOODS_H_INCLUDED
#define GOODS_H_INCLUDED

	// полезные макросы
	#define BV(a) (1UL << (a))
	#define NUMOFARRAY(a)    (sizeof(a) / sizeof(a[0]))

	#define min(a, b) (((a) < (b)) ? (a) : (b))
	#define max(a, b) (((a) > (b)) ? (a) : (b))
//	#define abs(a) ((a) > 0 ? (a) : -(a))
	#define lim(a,b,c) max((a), min((b),(c)))    // a <= b <= c

	#ifndef NULL
		#define NULL   0U
	#endif // NULL


	#if defined __clang__ && __clang__
		// compiler is clang
		//#define asm __asm
	#endif

	#define disable_irq asm volatile ("cpsid i")
	#define enable_irq  asm volatile ("cpsie i")

	#define SWAP16(a) (((a) & 0xff) << 8) | (((a) >> 8) & 0xff)

	#define SWAP32_HALFWORDS(a)  (((a) & 0x0000ffff) << 16) | (((a) >> 16) & 0x0000ffff)


#endif /* GOODS_H_INCLUDED */
