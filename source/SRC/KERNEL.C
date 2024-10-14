/*
	real time micro kernel for H8/300H(AKI-H8 3048F)
		Copyright (C) 1998-1999 by AKIYA
*/

#include <3048f.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "include\kernel.h"
#include "include\machine.h"

/* --- カーネル ---------------------------------------------------------- */

#define STATIC static				/* fixed then no comment */

typedef unsigned char CCR;			/* condition cord register */
#define INT_DIS 0xc0				/* interrupt disable */
#define INT_ENA 0x3f				/* interrupt enable */

STATIC void * stk_top = 0;			/* スタック割り当てアドレス */

#define PRI_MAX 8
#define LNK_RUN 0
#define LNK_DMT 1
#define LNK_SUS 2
#define LNK_WAS 3
#define LNK_WAI 4
#define LNK_PRI 5
#define LNK_NUM (1+4+PRI_MAX)		/* run, dmt, sus, was, wai, pri(1~8) */

STATIC T_CTSK *tsk_lnk[LNK_NUM];	/* 実行中のタスク＆タスクリンク */
#define run_tsk (tsk_lnk[LNK_RUN])	/* 現在実行中のタスク */
#define dmt_lnk (tsk_lnk[LNK_DMT])	/* 休止状態タスクリンク */
#define sus_lnk (tsk_lnk[LNK_SUS])	/* 強制待ち状態タスクリンク */
#define was_lnk (tsk_lnk[LNK_WAS])	/* 二重待ち状態タスクリンク */
#define wai_lnk (tsk_lnk[LNK_WAI])	/* 待ち状態タスクリンク */
#define pri_lnk (&tsk_lnk[LNK_PRI])	/* 優先度１～８タスクリンク */

STATIC T_CSEM *sem_lnk;			/* セマフォリンク */

extern volatile struct st_itu0 *ITUx;		/* 1mSec インターバル用 */
extern int CNT1mS;				/* 1mSec のカウント値 */
extern void *STKini;				/* 割り当てスタック初期値 */
extern void *STKtop;				/* スタック領域先頭 */

/* --- プロトタイプ --- */
/*ER vadd_lnk( T_CTSK **, T_CTSK * );*/

/* --- ＲＴＭＫの初期化 --- */
ER vini_rtmk( ID tskid, T_CTSK *pk_ctsk )
{
	CCR ccr;
	ER er;
	int i;

	if ( stk_top != 0 )
		return E_SYS;		/* now ready */
	for ( i = 0; i < LNK_NUM; i++ )
		tsk_lnk[i] = NULL;
	sem_lnk = NULL;
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	vini_heap( (UB *)(stk_top = STKtop), (UW)STKini-(UW)STKtop );
	if ( (er = cre_tsk( tskid, pk_ctsk )) != E_OK ) {
		stk_top = 0;
		set_ccr( ccr );		/* ccr restore */
		return er;
	}
	if ( (er = sta_tsk( tskid, 0 )) != E_OK ) {
		stk_top = 0;
		set_ccr( ccr );		/* ccr restore */
		return er;
	}
	run_tsk = pk_ctsk;
	/* tick timer setup */
	ITUx->TCR.BYTE = 0xa0;
	ITUx->TIOR.BYTE = 0x88;
	ITUx->GRA = CNT1mS-1;
	ITUx->TSR.BYTE &= 0xf8;
	ITUx->TIER.BYTE = 0xf9;
	ITUx->TCNT = 0;
	if ( ITUx == &ITU0 ) {
		ITU.TSNC.BIT.SYNC0 = 0;
		ITU.TMDR.BIT.PWM0 = 0;
		ITU.TSTR.BIT.STR0 = 1;
	} else if ( ITUx == &ITU1 ) {
		ITU.TSNC.BIT.SYNC1 = 0;
		ITU.TMDR.BIT.PWM1 = 0;
		ITU.TSTR.BIT.STR1 = 1;
	} else if ( ITUx == &ITU2 ) {
		ITU.TSNC.BIT.SYNC2 = 0;
		ITU.TMDR.BIT.PWM2 = 0;
		ITU.TSTR.BIT.STR2 = 1;
	} else if ( (struct st_itu3 *)ITUx == &ITU3 ) {
		ITU.TSNC.BIT.SYNC3 = 0;
		ITU.TMDR.BIT.PWM3 = 0;
		ITU.TSTR.BIT.STR3 = 1;
	} else if ( (struct st_itu3 *)ITUx == &ITU4 ) {
		ITU.TSNC.BIT.SYNC4 = 0;
		ITU.TMDR.BIT.PWM4 = 0;
		ITU.TSTR.BIT.STR4 = 1;
	}
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- タスクをリンクに接続 --- */
STATIC ER vadd_lnk( T_CTSK **lnk, T_CTSK *pk_ctsk )
{
	CCR ccr;

	pk_ctsk->tcb.lnk_t_ctsk = NULL;
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	while ( *lnk != NULL )
		lnk = &((*lnk)->tcb.lnk_t_ctsk);
	*lnk = pk_ctsk;
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- タスクをリンクから外す --- */
STATIC ER vout_lnk( T_CTSK **lnk, T_CTSK *pk_ctsk )
{
	CCR ccr;

	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	while ( (*lnk != NULL) && (*lnk != pk_ctsk) )
		lnk = &((*lnk)->tcb.lnk_t_ctsk);
	if ( *lnk != NULL ) {
		*lnk = pk_ctsk->tcb.lnk_t_ctsk;
		set_ccr( ccr );		/* ccr restore */
		return E_OK;
	}
	set_ccr( ccr );		/* ccr restore */
	return E_SYS;
}

/* --- タスクをリンクからサーチする --- */
STATIC T_CTSK *vsch_lnk( T_CTSK **lnk, ID tskid )
{
	CCR ccr;

	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	while ( *lnk != NULL )
		if ( (*lnk)->tcb.tskid == tskid ) {
			set_ccr( ccr );		/* ccr restore */
			return *lnk;
		} else
			lnk = &((*lnk)->tcb.lnk_t_ctsk);
	set_ccr( ccr );		/* ccr restore */
	return NULL;
}

/* --- タスクを全てのリンクからサーチする --- */
STATIC int vsch_tsk( T_CTSK **pk_ctsk, ID tskid )
{
	CCR ccr;
	int num;

	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	for ( num = 0; num < LNK_NUM; num++ ) {
		if ( (*pk_ctsk = vsch_lnk( &tsk_lnk[num], tskid )) != NULL )
			break;
	}
	set_ccr( ccr );		/* ccr restore */
	return num;
}

/* --- タスクスタートコード --- */
STATIC INT vrun_tsk( int stacd )
{
	and_ccr( INT_ENA );
	return stacd -1;
}

/* --- タスクターミネートコード --- */
STATIC void vter_tsk( int tercd )
{
	and_ccr( INT_ENA );
	if ( run_tsk->tcb.terfnc != NULL )
		(*(run_tsk->tcb.terfnc))();
	ext_tsk();
}

/* --- エラーログ（未実装：無限ループ） --- */
STATIC void verr_log( ER ercd )
{
	while ( ercd );
}

/* --- ヒープ管理（スタック管理に使用） ---------------------------------- */

typedef struct heap_t {
	long size;
	struct heap_t *fwrd;
	struct heap_t *next;
} HEAP_t;

/*
 メモリの先頭アドレスとサイズをもとにヒープ領域を構成する。
 ヒープ管理タグとして先頭の１２バイトを使用する。このとき、
 ヒープ管理タグのサイズを－１バイトとすることにより、空き
 エリアの連結処理を回避する。また、開放時のヒープ管理タグ
 のために取得最小サイズを１２バイトとする。
 ヒープ取得は、メモリの先頭アドレスと取得サイズを指定する。
 ヒープ開放は、メモリの先頭アドレスと取得したアドレスと取
 得サイズを指定する。開放時に誤ったパラメータを指定すると、
 整合性が保てなくなり破錠する。
*/

/* --- ヒープ領域の初期化 --- */
ER vini_heap( UB *mem, UW size )
{
	UB *wrk;
/*	long cnt;	*/
	HEAP_t *top;

	if ( size < sizeof(HEAP_t)*2 )
		return E_PAR;
/*	for ( wrk = mem, cnt = size; cnt--; *wrk++ = 0 );	/* clear */
	top = (HEAP_t *)mem;
	mem += sizeof(HEAP_t);
	top->size = sizeof(HEAP_t)-1;	/* -1 for link block */
	top->fwrd = NULL;
	top->next = (HEAP_t *)mem;
	((HEAP_t *)mem)->size = size - sizeof(HEAP_t);
	((HEAP_t *)mem)->fwrd = top;
	((HEAP_t *)mem)->next = NULL;
	return E_OK;
}

/* --- ヒープメモリの取得 ---
 同一サイズのブロックが存在すれば、そのブロックを割り当てる。
 存在しなければ、取得可能ブロックのリンク最後端で割り当てる。
*/
void *vget_heap( UB *mem, UW size )
{
	CCR ccr;
	HEAP_t *own;
	HEAP_t *wrk;

	if ( size & 1 ) size++;		/* must be evn */
	if ( size < sizeof(HEAP_t) )
		size = sizeof(HEAP_t);
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	for ( own = ((HEAP_t *)mem)->next, wrk = NULL;
		(own != NULL) && (own->size != size); own = own->next )
		if ( own->size >= (size+sizeof(HEAP_t)) )
			wrk = own;
	if ( own != NULL ) {
		(own->fwrd)->next = own->next;
		(own->next)->fwrd = own->fwrd;
	} else if ( wrk != NULL ) {
		wrk->size -= size;
		own = (HEAP_t *)((UB *)wrk + wrk->size);
	}
	set_ccr( ccr );		/* ccr restore */
	return (void *)own;
}

/* --- ヒープメモリの開放 --- */
void vfre_heap( UB *mem, UB *fre, UW size )
{
	CCR ccr;
	HEAP_t *wrk;

	if ( size & 1 ) size++;		/* must be evn */
	if ( size < sizeof(HEAP_t) )
		size = sizeof(HEAP_t);
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	for ( wrk = (HEAP_t *)mem;
		(wrk->next != NULL) && (fre > (UB *)(wrk->next));
			wrk = wrk->next );
	if ( (UB *)wrk+wrk->size == fre ) {
		wrk->size += size;
		if ( wrk->next != NULL )
			if ( (UB *)wrk+wrk->size == (UB *)(wrk->next) ) {
				wrk->size += (wrk->next)->size;
				if ( (wrk->next = (wrk->next)->next) != NULL )
					(wrk->next)->fwrd = wrk;
		}
	} else {
		((HEAP_t *)fre)->size = size;
		((HEAP_t *)fre)->fwrd = wrk;
		if ( (((HEAP_t *)fre)->next = wrk->next) != NULL )
			if ( fre+size == (UB *)(((HEAP_t *)fre)->next) ) {
				((HEAP_t *)fre)->size +=
					(((HEAP_t *)fre)->next)->size;
				((HEAP_t *)fre)->next =
					(((HEAP_t *)fre)->next)->next;
			}
		wrk->next = (HEAP_t *)fre;
	}
	set_ccr( ccr );		/* ccr restore */
}

/* --- システム変数 ------------------------------------------------------ */

STATIC int dsp_flg = 0;	/* dispatch disable flag for run task */
STATIC int dsp_cnt = 0;	/* dispatch disable count for interrupt */

/* --- ディスパッチ ------------------------------------------------------ */

STATIC void vacp_dsp( void )
{
	CCR ccr;
	int i;

	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
/*	if ( dsp_cnt == 0 ) {		bug fx ?
*/	if ( (dsp_cnt == 0) && (dsp_flg == TSS_TSK) ) {
		if ( !setjmp( run_tsk->tcb.ctx.buf ) ) {
			i = 0;
			while ( pri_lnk[i] == NULL )
				if ( ++i == PRI_MAX ) {
					dsp_cnt++;
					and_ccr( INT_ENA );/*interrupt enable*/
					i = 0;
					or_ccr( INT_DIS );/*interrupt disable*/
					if ( dsp_cnt > 0 )	/* + 99/7/23 */
						dsp_cnt--;
				}
			if ( run_tsk->tcb.state == TTS_NON )
				vfre_heap( (UB *)stk_top,
				 (UB *)(run_tsk->tcb.isp-run_tsk->stksz),
				 run_tsk->stksz );
			else if ( run_tsk->tcb.state == TTS_RUN )
				run_tsk->tcb.state = TTS_RDY;
			run_tsk = pri_lnk[i];
			run_tsk->tcb.state = TTS_RUN;
			longjmp( run_tsk->tcb.ctx.buf,
				 run_tsk->tcb.ctx.val.retval );
		}
	}
	set_ccr( ccr );		/* ccr restore */
}

/* --- ディスパッチ禁止［Ｒ］ -------------------------------------------- */

ER dis_dsp( void )
{
	if ( (dsp_cnt != 0) || (dsp_flg > TSS_DDSP) )
		return E_CTX;
	dsp_flg = TSS_DDSP;
	return E_OK;
}

/* --- ディスパッチ許可［Ｒ］ -------------------------------------------- */

ER ena_dsp( void )
{
	CCR ccr;

	if ( (dsp_cnt != 0) || (dsp_flg > TSS_DDSP) )
		return E_CTX;
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( dsp_flg != TSS_TSK ) {
		dsp_flg = TSS_TSK;
		vacp_dsp();
	}
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- 自タスクを起床待ち状態へ移行［Ｒ］ -------------------------------- */

/* #define slp_tsk() tslp_tsk( TMO_FEVR ) */

/* --- 割り込みとディスパッチの禁止［Ｒ］ -------------------------------- */

ER loc_cpu( void )
{
	if ( dsp_cnt != 0 )
		return E_CTX;
	or_ccr( INT_DIS );
	dsp_flg = TSS_LOC;
	return E_OK;
}

/* --- 割り込みとディスパッチの許可［Ｒ］ -------------------------------- */

ER unl_cpu( void )
{
	CCR ccr;

	if ( dsp_cnt != 0 )
		return E_CTX;
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	ccr &= INT_ENA;
	if ( dsp_flg != TSS_TSK ) {
		dsp_flg = TSS_TSK;
		vacp_dsp();
	}
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- 割り込みハンドラから復帰［Ｒ］ ------------------------------------ */

/* #define ret_int() return */
void ret_int( void )	/* こちらへ変更します 99/7/23 */
{
	or_ccr( INT_DIS );
	if ( dsp_cnt > 0 )
		if ( --dsp_cnt == 0 )	/* ディスパッチ禁止カウンタ－１ */
			vacp_dsp();	/* ０になったらディスパッチ */
}

/* --- 他タスクの起床［ＲＮ］ -------------------------------------------- */

ER wup_tsk( ID tskid )
{
	T_CTSK *pk_ctsk;
	CCR ccr;
	int i;

	if ( dsp_cnt == 0 ) {
		if ( tskid <= 0 )
			return E_ID;		/* ID number error */
		if ( run_tsk->tcb.tskid == tskid )
			return E_OBJ;
	}
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( (pk_ctsk = vsch_lnk( &wai_lnk, tskid )) != NULL ) {
		if ( pk_ctsk->tcb.dlytim == 0 ) {
			vout_lnk( &wai_lnk, pk_ctsk );
			pk_ctsk->tcb.state = TTS_RDY;
/*			vadd_lnk( &pri_lnk[run_tsk->tcb.tskpri-1], pk_ctsk );
*/			vadd_lnk( &pri_lnk[pk_ctsk->tcb.tskpri-1], pk_ctsk );
			if ( dsp_cnt == 0 )
				vacp_dsp();
		}
		set_ccr( ccr );			/* ccr restore */
		return E_OK;
	}
	if ( (pk_ctsk = vsch_lnk( &was_lnk, tskid )) != NULL ) {
		vout_lnk( &was_lnk, pk_ctsk );
		pk_ctsk->tcb.state = TTS_SUS;
		vadd_lnk( &sus_lnk, pk_ctsk );
		set_ccr( ccr );			/* ccr restore */
		return E_OK;
	}
	set_ccr( ccr );			/* ccr restore */
	for ( i = 0; i < PRI_MAX; i++ )
		if ( (pk_ctsk = vsch_lnk( &pri_lnk[i], tskid )) != NULL ) {
			if ( pk_ctsk->tcb.wupcnt == LONG_MAX )
				return E_QOVR;
			pk_ctsk->tcb.wupcnt++;
			return E_OK;
		}
	if ( (pk_ctsk = vsch_lnk( &dmt_lnk, tskid )) != NULL )
		return E_OBJ;
	return E_NOEXS;
}

/* --- 自タスク終了 ［Ｓ］ ----------------------------------------------- */

void ext_tsk( void )
{
	if ( (dsp_cnt != 0) || (dsp_flg >= TSS_DDSP) )
		verr_log( E_CTX );	/* not return ! */
	if ( get_ccr() & INT_DIS )
		verr_log( E_CTX );	/* not return ! */
	or_ccr( INT_DIS );
	vout_lnk( &pri_lnk[run_tsk->tcb.tskpri-1], run_tsk );
	run_tsk->tcb.state = TTS_DMT;
	vadd_lnk( &dmt_lnk, run_tsk );
	vacp_dsp();
}

/* --- タスクのレディキュー回転［Ｓ］ ------------------------------------ */

ER rot_rdq( PRI tskpri )
{
	T_CTSK *pk_ctsk;
	CCR ccr;

	if ( (tskpri != TPRI_RUN) && (tskpri < 1) || (tskpri > PRI_MAX) )
		return E_PAR;
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( tskpri == TPRI_RUN )
		pk_ctsk = pri_lnk[(tskpri = run_tsk->tcb.tskpri)-1];
	else
		pk_ctsk = pri_lnk[tskpri-1];
	if ( pk_ctsk != NULL ) {
		vout_lnk( &pri_lnk[tskpri-1], pk_ctsk );
		pk_ctsk->tcb.state = TTS_RDY;
		vadd_lnk( &pri_lnk[tskpri-1], pk_ctsk );
		if ( pri_lnk[tskpri-1] != pk_ctsk )
			if ( dsp_cnt == 0 )
				vacp_dsp();
	}
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- 自タスクのタスクＩＤ参照［Ｓ］ ------------------------------------ */

ER get_tid( ID *p_tskid )
{
	if ( (dsp_cnt != 0) || (dsp_flg > TSS_LOC) )
		return FALSE;
	*p_tskid = run_tsk->tcb.tskid;
	return E_OK;
}

/* --- タスク遅延［Ｓ］ -------------------------------------------------- */

ER dly_tsk( DLYTIME dlytim )
{
	if ( (dsp_cnt != 0) || (dsp_flg >= TSS_DDSP) )
		return E_CTX;
	if ( get_ccr() & INT_DIS )
		return E_CTX;
	if ( dlytim < 0 )
		return E_PAR;
	run_tsk->tcb.ercd = E_OK;
	if ( dlytim ) {
		run_tsk->tcb.dlytim = dlytim;
		or_ccr( INT_DIS );
		vout_lnk( &pri_lnk[run_tsk->tcb.tskpri-1], run_tsk );
		run_tsk->tcb.state = TTS_WAI;
		vadd_lnk( &wai_lnk, run_tsk );
		vacp_dsp();
		and_ccr( INT_ENA );
	}
	return run_tsk->tcb.ercd;
}

/* --- ディレイタイマーの更新と起床（１ｍｓｅｃの割り込みから起動） --- */
#define PK_CTSK_PRI (pk_ctsk->tcb.tskpri)
STATIC void wup_dly_tsk( void )
{
	T_CTSK *pk_ctsk;
	T_CTSK *pk_ctsk_sp;
	CCR ccr;

	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	/* wait */
	pk_ctsk = wai_lnk;
	while ( pk_ctsk != NULL )
		if ( pk_ctsk->tcb.dlytim ) {
			if ( --(pk_ctsk->tcb.dlytim) == 0 ) {
				pk_ctsk_sp = pk_ctsk->tcb.lnk_t_ctsk;
				vout_lnk( &wai_lnk, pk_ctsk );
				pk_ctsk->tcb.state = TTS_RDY;
				vadd_lnk( &pri_lnk[PK_CTSK_PRI-1], pk_ctsk );
				pk_ctsk = pk_ctsk_sp;
			} else
				pk_ctsk = pk_ctsk->tcb.lnk_t_ctsk;
		} else
			pk_ctsk = pk_ctsk->tcb.lnk_t_ctsk;
	/* wait suspend */
	pk_ctsk = was_lnk;
	while ( pk_ctsk != NULL )
		if ( pk_ctsk->tcb.dlytim ) {
			if ( --(pk_ctsk->tcb.dlytim) == 0 ) {
				pk_ctsk_sp = pk_ctsk->tcb.lnk_t_ctsk;
				vout_lnk( &was_lnk, pk_ctsk );
				pk_ctsk->tcb.state = TTS_SUS;
				vadd_lnk( &sus_lnk, pk_ctsk );
				pk_ctsk = pk_ctsk_sp;
			} else
				pk_ctsk = pk_ctsk->tcb.lnk_t_ctsk;
		} else
			pk_ctsk = pk_ctsk->tcb.lnk_t_ctsk;
	set_ccr( ccr );			/* ccr restore */
}

/* --- タスク起動［ＳＮ］ ------------------------------------------------ */

ER sta_tsk( ID tskid, INT stacd )
{
	T_CTSK *pk_ctsk;
	CCR ccr;
	int i;

	if ( dsp_cnt == 0 )
		if ( tskid <= 0 )
			return E_ID;
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( (pk_ctsk = vsch_lnk( &dmt_lnk, tskid )) != NULL ) {
		pk_ctsk->tcb.ctx.val.sp = pk_ctsk->tcb.isp-8;
		*(void **)(pk_ctsk->tcb.ctx.val.sp+4) = pk_ctsk->task;
		pk_ctsk->tcb.ctx.val.ret = (long)vrun_tsk;
		pk_ctsk->tcb.ctx.val.retval = stacd +1;
		vout_lnk( &dmt_lnk, pk_ctsk );
		pk_ctsk->tcb.state = TTS_RDY;
		vadd_lnk( &pri_lnk[pk_ctsk->tcb.tskpri-1], pk_ctsk );
		set_ccr( ccr );			/* ccr restore */
		return E_OK;
	}
	set_ccr( ccr );			/* ccr restore */
	if ( vsch_lnk( &wai_lnk, tskid ) != NULL )
		return E_OBJ;
	if ( vsch_lnk( &was_lnk, tskid ) != NULL )
		return E_OBJ;
	if ( vsch_lnk( &sus_lnk, tskid ) != NULL )
		return E_OBJ;
	for ( i = 0; i < PRI_MAX; i++ )
		if ( vsch_lnk( &pri_lnk[i], tskid ) != NULL )
			return E_OBJ;
	return E_NOEXS;
}

/* --- 他タスク強制終了［ＳＮ］ ------------------------------------------ */

STATIC void ter_tsk_sub( T_CTSK *pk_ctsk )
{
	/*

	ここで、セマフォの待ちリンクを外す（のだけれど、待ちリンクは無い）

	*/
	pk_ctsk->tcb.ctx.val.sp = pk_ctsk->tcb.isp-8;
	*(void **)(pk_ctsk->tcb.ctx.val.sp+4) = pk_ctsk->task;
	pk_ctsk->tcb.ctx.val.ret = (long)vter_tsk;
	pk_ctsk->tcb.ctx.val.retval = 1;
	if ( pk_ctsk != run_tsk )
		pk_ctsk->tcb.state = TTS_RDY;
	vadd_lnk( &pri_lnk[pk_ctsk->tcb.tskpri-1], pk_ctsk );
}

ER ter_tsk( ID tskid )
{
	T_CTSK *pk_ctsk;
	CCR ccr;
	int i;

	if ( dsp_cnt == 0 ) {
		if ( tskid <= 0 )
			return E_ID;
		if ( run_tsk->tcb.tskid == tskid )
			return E_OBJ;
	}
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( vsch_lnk( &dmt_lnk, tskid ) != NULL ) {
		set_ccr( ccr );			/* ccr restore */
		return E_OBJ;
	}
	if ( (pk_ctsk = vsch_lnk( &wai_lnk, tskid )) != NULL ) {
		vout_lnk( &wai_lnk, pk_ctsk );
		ter_tsk_sub( pk_ctsk );
		set_ccr( ccr );			/* ccr restore */
		return E_OK;
	}
	if ( (pk_ctsk = vsch_lnk( &was_lnk, tskid )) != NULL ) {
		vout_lnk( &was_lnk, pk_ctsk );
		ter_tsk_sub( pk_ctsk );
		set_ccr( ccr );			/* ccr restore */
		return E_OK;
	}
	if ( (pk_ctsk = vsch_lnk( &sus_lnk, tskid )) != NULL ) {
		vout_lnk( &sus_lnk, pk_ctsk );
		ter_tsk_sub( pk_ctsk );
		set_ccr( ccr );			/* ccr restore */
		return E_OK;
	}
	for ( i = 0; i < PRI_MAX; i++ )
		if ( (pk_ctsk = vsch_lnk( &pri_lnk[i], tskid )) != NULL ) {
			vout_lnk( &pri_lnk[i], pk_ctsk );
			ter_tsk_sub( pk_ctsk );
			set_ccr( ccr );		/* ccr restore */
			return E_OK;
		}
	set_ccr( ccr );			/* ccr restore */
	return E_NOEXS;
}

/* --- タスクの起床要求を無効化［ＳＮ］ ---------------------------------- */

ER can_wup( INT *p_wupcnt, ID tskid )
{
	T_CTSK *pk_ctsk;
	int lnk;

	if ( ((dsp_cnt == 0) && (tskid < 0)) ||
		((dsp_cnt != 0) && (tskid == TSK_SELF)) )
		return E_ID;
	else if ( tskid == 0 )
		tskid = run_tsk->tcb.tskid;
	if ( (lnk = vsch_tsk( &pk_ctsk, tskid )) >= LNK_NUM ) {
		return E_NOEXS;
	} else if ( lnk == LNK_DMT )
		return E_OBJ;
	*p_wupcnt = pk_ctsk->tcb.wupcnt;
	pk_ctsk->tcb.wupcnt = 0;
	return E_OK;
}

/* --- 自タスク終了と削除 ［Ｅ］ ----------------------------------------- */

void exd_tsk( void )
{
	if ( (dsp_cnt != 0) || (dsp_flg >= TSS_DDSP) )
		verr_log( E_CTX );	/* not return ! */
	if ( get_ccr() & INT_DIS )
		verr_log( E_CTX );	/* not return ! */
	or_ccr( INT_DIS );
	vout_lnk( &pri_lnk[run_tsk->tcb.tskpri-1], run_tsk );
	run_tsk->tcb.state = TTS_NON;
	vacp_dsp();
}

/* --- 自タスクを起床待ち状態へ移行（タイムアウトあり）［Ｅ］ ------------ */

ER tslp_tsk( TMO tmout )
{
	if ( (dsp_cnt != 0) || (dsp_flg >= TSS_DDSP) )
		return E_CTX;
	if ( get_ccr() & INT_DIS )
		return E_CTX;
	if ( tmout < -1 )
		return E_PAR;
	or_ccr( INT_DIS );
	if ( run_tsk->tcb.wupcnt != 0 ) {
		run_tsk->tcb.ercd = E_OK;
		run_tsk->tcb.wupcnt--;
	} else {
		run_tsk->tcb.ercd = E_TMOUT;
		if ( tmout ) {
			run_tsk->tcb.dlytim = tmout < 0 ? 0 : tmout;
			vout_lnk( &pri_lnk[run_tsk->tcb.tskpri-1], run_tsk );
			run_tsk->tcb.state = TTS_WAI;
			vadd_lnk( &wai_lnk, run_tsk );
			vacp_dsp();
		}
	}
	and_ccr( INT_ENA );
	return run_tsk->tcb.ercd;
}

/* --- 割り込みハンドラ復帰とタスク起床［Ｅ］ ---------------------------- */

void ret_wup( ID tskid )
{
	if ( dsp_cnt > 0 )
		wup_tsk( tskid );
	ret_int();
}

/* --- システム状態参照［Ｅ］ -------------------------------------------- */

ER ref_sys( T_RSYS *pk_rsys )
{
	if ( stk_top == 0 )
		return E_SYS;		/* not ready */
	if ( pk_rsys == NULL )
		return E_PAR;
	pk_rsys->sysstat = dsp_cnt != 0 ? TSS_INDP : dsp_flg;
	return E_OK;
}

/* --- タスク生成［ＥＮ］ ------------------------------------------------ */

ER cre_tsk( ID tskid, T_CTSK *pk_ctsk )
{
	CCR ccr;
	void *stk;

	if ( stk_top == 0 )
		return E_SYS;		/* not ready */
	if ( tskid <= 0 )
		return E_ID;		/* ID number error */
	if ( pk_ctsk->task == NULL )
		return E_PAR;		/* task start address error */
	if ( (pk_ctsk->itskpri < 1) || (pk_ctsk->itskpri > PRI_MAX) )
		return E_PAR;		/* priority error */
	/*

	ここで tskid が存在するかチェックする必要あり

	*/
	/* tcb setup */
	pk_ctsk->tcb.tskid = tskid;
	pk_ctsk->tcb.tskpri = pk_ctsk->itskpri;
	pk_ctsk->tcb.dlytim = 0;
	pk_ctsk->tcb.ercd = E_OK;
	pk_ctsk->tcb.wupcnt = 0;
	pk_ctsk->tcb.suscnt = 0;
	pk_ctsk->tcb.terfnc = NULL;
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( (stk = vget_heap( (UB *)stk_top, pk_ctsk->stksz )) == NULL ) {
		set_ccr( ccr );		/* ccr restore */
		return E_PAR;		/* stack overflow */
	}
	pk_ctsk->tcb.isp = (long)stk+pk_ctsk->stksz;
	pk_ctsk->tcb.state = TTS_DMT;
	pk_ctsk->tcb.wstat = 0;
	vadd_lnk( &dmt_lnk, pk_ctsk );
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- セマフォ生成［ＥＮ］ ---------------------------------------------- */

ER cre_sem( ID semid, T_CSEM *pk_csem )
{
	T_CSEM *sem;
	CCR ccr;
	ID *w;
	int i;

	if ( semid <= 0 )
		return semid < -4 ? E_OACV : E_ID;
	/* semafor get flag */
	i = sizeof(pk_csem->flg.semflg)/sizeof(ID);
	if ( pk_csem->maxsem > i ) {
		i = pk_csem->maxsem;
		if ((w = vget_heap((UB *)stk_top,i*sizeof(ID))) == NULL)
			return E_NOMEM;
		pk_csem->flg.psemflg = w;
	} else
		w = pk_csem->flg.semflg;
	while ( --i >= 0 )
		w[i] = 0;
	/* initial */
	pk_csem->semid = semid;
	pk_csem->semcnt = pk_csem->isemcnt;
	pk_csem->lnk_t_csem = NULL;
	/* add semafor link */
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( sem_lnk == NULL )
		sem_lnk = pk_csem;
	else {
		for ( sem = sem_lnk; sem->lnk_t_csem != NULL;
						sem = sem->lnk_t_csem );
		sem->lnk_t_csem = pk_csem;
	}
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- セマフォ削除［ＥＮ］ ---------------------------------------------- */

ER del_sem( ID semid )
{
	T_CSEM *sem;
	T_CSEM *sm2;
	CCR ccr;
	int i;

	if ( semid <= 0 )
		return semid < -4 ? E_OACV : E_ID;
	/* srch & delete semafor link */
	ccr = get_or_ccr( INT_DIS );	/* ccr get & interrupt disable */
	if ( (sem = sem_lnk) == NULL ) {
		set_ccr( ccr );			/* ccr restore */
		return E_NOEXS;
	}
	sm2 = NULL;
	while ( sem->semid != semid ) {
		sm2 = sem;
		if ( (sem = sem->lnk_t_csem) == NULL ) {
			set_ccr( ccr );			/* ccr restore */
			return E_NOEXS;
		}
	}
	if ( sm2 == NULL )
		sem_lnk = NULL;
	else
		sm2->lnk_t_csem = sem->lnk_t_csem;
	i = sizeof(sem->flg.semflg)/sizeof(ID);
	if ( sem->maxsem > i ) {
		i = sem->maxsem;
		vfre_heap((UB *)stk_top,(UB *)sem->flg.psemflg,i*sizeof(ID));
	}
	/*

	ここで、セマフォ待ちタスクに E_DLT を返す

	*/
	set_ccr( ccr );			/* ccr restore */
	return E_OK;
}

/* --- セマフォ資源獲得（タイムアウトあり）［ＥＮ］ ---------------------- */

ER twai_sem( ID semid, TMO tmout )
{
	T_CSEM *sem;
	ID *w;
	int i;

	if ( (dsp_cnt != 0) || (dsp_flg >= TSS_DDSP) )
		return E_CTX;
	if ( get_ccr() & INT_DIS )
		return E_CTX;
	if ( semid <= 0 )
		return semid < -4 ? E_OACV : E_ID;
	if ( tmout < -1 )
		return E_PAR;
	or_ccr( INT_DIS );
	for ( sem = sem_lnk;(sem != NULL) && (sem->semid != semid);
						sem = sem->lnk_t_csem );
	if ( sem == NULL ) {
		and_ccr( INT_ENA );
		return E_NOEXS;
	}
	while ( sem->semcnt == 0 ) {
		if ( tmout == 0 ) {
			and_ccr( INT_ENA );
			return E_TMOUT;
		}
		run_tsk->tcb.dlytim = tmout < 0 ? 0 : tmout;
		vout_lnk( &pri_lnk[run_tsk->tcb.tskpri-1], run_tsk );
		run_tsk->tcb.state = TTS_WAI;
		run_tsk->tcb.wstat = TTW_SEM;
		vadd_lnk( &wai_lnk, run_tsk );
		vacp_dsp();
		/*

		ここで起床状態をチェックする
		E_DLT であれば、直ちにリターン

		*/
	}
	(sem->semcnt)--;
	if ( sem->maxsem > (sizeof(sem->flg.semflg)/sizeof(ID)) )
		w = sem->flg.psemflg;
	else
		w = sem->flg.semflg;
	for ( i = sem->maxsem -1; (i >= 0) && (w[i] != 0); i-- );
	if ( i < 0 )
		verr_log( E_CTX );	/* not return ! */
	w[i] = run_tsk->tcb.tskid;
	and_ccr( INT_ENA );
	return E_OK;
}

/* --- 割り込みハンドラ定義［Ｃ］ ---------------------------------------- */

ER def_int( UINT dintno, T_DINT *pk_dint )
{
/*	void (*fnc)();
*/
	if ( (pk_dint->inthdr != NULL) && (pk_dint->inttbl != NULL) )
		(pk_dint->inttbl)[dintno] = (FP)(0x5a000000l | (int)pk_dint->inthdr);
/*	if ( (fnc = pk_dint->intini) != NULL )
		fnc();
*/	if ( pk_dint->intini != NULL )
		(pk_dint->intini)();
/*	ヘッダーで */
/*	void	(* intini)();	/* 初期化ルーチンアドレス */
/*	と定義すると (pk_dint->intini)(); と記述できるのだが */
	return E_OK;
}

/* --- 例外ハンドラ定義［Ｃ］ -------------------------------------------- */

ER def_exc( UINT exckind, T_DEXC *pk_dexc )
{
	switch ( exckind ) {
	case TER_FNC:
		if ( (dsp_cnt != 0) || (dsp_flg >= TSS_DDSP) )
			return E_CTX;
		run_tsk->tcb.terfnc = pk_dexc->exchdr;
		return E_OK;
	default:
		return E_PAR;
	}
}

/* --- 各種割り込みサポート ---------------------------------------------- */

/* 割り込みサポート */
void vbgn_int( void )
{
	dsp_cnt++;
}

/* １ｍＳｅｃインターバル（ティック）タイマー */

STATIC unsigned int tmr_1ms_h = 0;	/* 16bit */
STATIC unsigned long tmr_1ms = 0;	/* 32bit (total 48bit) */

#pragma interrupt vint_1ms
void vint_1ms( void )
{
	ITUx->TSR.BIT.IMFA = 0;

	dsp_cnt++;	/* 割り込みの入り口でディスパッチ禁止カウンタ＋１ */
/*	vbgn_int();	/* ディスパッチ禁止カウンタ＋１（通常はこちら） */

	if ( ++tmr_1ms == 0 ) tmr_1ms_h++;	/* 48bit countr +1 */
	wup_dly_tsk();

	or_ccr( INT_DIS );
	if (--dsp_cnt == 0)	/* 出口でディスパッチ禁止カウンタ－１ */
		vacp_dsp();	/* ０になったらディスパッチを実行する */
/*	ret_int();	/* 割り込み終了（通常はこちら） */
}

