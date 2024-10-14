/*
	real time micro kernel for H8/300H(AKI-H8 3048F)
		Copyright (C) 1998-1999 by AKIYA
*/

/* --- for rtmk (Real Time Micro Kernel) --------------------------------- */

#include <setjmp.h>

/* --- 全体 -------------------------------------------------------------- */

#define NADR (-1)		/* アドレスやポインタの値が無効 */
#define TRUE 1			/* 真 */
#define FALSE 0			/* 偽 */

#define TA_ASM 0x00		/* アセンブラによるプログラム */
#define TA_HLNG 0x01		/* 高級言語によるプログラム */

#define TMO_POL 0		/* ポーリング */
#define TMO_FEVR (-1)		/* 永久待ち */

typedef char B;			/* 符号付き８ビット整数 */
typedef short H;		/* 符号付き16ビット整数 */
typedef long W;			/* 符号付き32ビット整数 */
typedef unsigned char UB;	/* 符号無し８ビット整数 */
typedef unsigned short UH;	/* 符号無し16ビット整数 */
typedef unsigned long UW;	/* 符号無し32ビット整数 */

typedef UB VB;			/* データタイプが一定しないもの（８ビット） */
typedef UH VH;			/* データタイプが一定しないもの（16ビット） */
typedef UW VW;			/* データタイプが一定しないもの（32ビット） */
typedef void * VP;		/* データタイプが一定しないものへのポインタ */

typedef void (*FP)();		/* プログラムのスタートアドレス一般 */

typedef int INT;		/* 符号付き整数（プロセッサのビット幅） */
typedef unsigned int UINT;	/* 符号無し整数（プロセッサのビット幅） */

typedef short BOOL;		/* ブール値 */
typedef short FN;		/* 機能コード */
typedef short ID;		/* オブジェクトのＩＤ番号 */
typedef short BOOL_ID;		/* ブール値またはＩＤ番号 */
typedef short HND;		/* ハンドラ番号 */
typedef short RNO;		/* ランデブ番号 */
typedef short NODE;		/* ノード番号 */
typedef unsigned short ATR;	/* オブジェクト属性、ハンドラ属性 */
typedef short ER;		/* エラーコード */
typedef short PRI;		/* タスク優先度 */
/* typedef T_MSG ? */		/* メイルボックス用のメッセージパケット */
typedef INT TMO;		/* タイムアウト */
typedef INT DLYTIME;		/* タスク遅延時間を指定するのデータタイプ */

/* --- タスク管理関係 ---------------------------------------------------- */

/* cre_tsk */
typedef struct t_ctsk {
	VP	exinf;		/* 拡張情報 */
	ATR	tskatr;		/* タスク属性 */
	FP	task;		/* タスク起動アドレス */
	PRI	itskpri;	/* タスク起動時優先度 */
	INT	stksz;		/* スタックサイズ */
	/* rtmk work */
	struct {
		ID	tskid;			/* タスクＩＤ */
		PRI	tskpri;			/* 現在の優先度 */
		UINT	state;			/* 現在の状態 */
		UINT	wstat;			/* 待ち要因 */
		ID	waitid;			/* 待ちＩＤ */
		INT	dlytim;			/* ディレイタイム */
		ER	ercd;			/* エラーコード */
		INT	wupcnt;			/* 起床要求キューイング数 */
		INT	suscnt;			/* サスペンドキューイング数 */
		void (*	terfnc)();		/* 終了ハンドラ */
		long	isp;			/* イニシャルスタック */
		struct t_ctsk *lnk_t_ctsk;	/* コンテキストリンク */
		union {				/* コンテキスト */
			jmp_buf buf;
			struct {
				long	sp;
				long	ret;
				long	er1;
				long	er2;
				long	er3;
				long	er4;
				long	er5;
				long	er6;
				int	retval;
			} val;
		} ctx;
	} tcb;
} T_CTSK;

#define TSK_SELF 0		/* 自タスク指定 */

#define TPRI_INI 0		/* タスク起動時の初期優先度を指定 */
#define TPRI_RUN 0		/* その時実行中の最高優先度を指定 */

/* ref_tsk */
typedef struct t_rtsk {
	VP	exinf;		/* 拡張情報 */
	PRI	tskpri;		/* 現在の優先度 */
	UINT	tskstat;	/* タスク状態 */
	/* level X */
	UINT	tskwait;	/* 待ち要因 */
	ID	wid;		/* 待ちオブジェクトＩＤ */
	INT	wupcnt;		/* 起床要求カウント */
	INT	suscnt;		/* ＳＵＳＰＥＮＤ要求カウント */
	ATR	tskatr;		/* タスク属性 */
	FP	task;		/* タスク起動アドレス */
	PRI	itskpri;	/* タスク起動時優先度 */
	INT	stksz;		/* スタックサイズ */
} T_RTSK;

#define TTS_RUN 0x01		/* RUN */
#define TTS_RDY 0x02		/* READY */
#define TTS_WAI 0x04		/* WAIT */
#define TTS_SUS 0x08		/* SUSPEND */
#define TTS_WAS 0x0c		/* WAIT-SUSPEND */
#define TTS_DMT 0x10		/* DORMANT */
#define	TTS_NON	0x00		/* NON-EXISTENT */

#define TTW_SLP 0x0001		/* slp_tsk, tslp_tsk による待ち */
#define TTW_DLY 0x0002		/* dly_tsk による待ち */
#define TTW_NOD 0x0008		/* 接続機能応答待ち */
#define TTW_FLG 0x0010		/* wai_flg, twai_flg による待ち */
#define TTW_SEM 0x0020		/* wai_sem, twai_sem による待ち */
#define TTW_MBX 0x0040		/* rcv_msg, trcv_msg による待ち */
#define TTW_SMBF 0x0080		/* snd_mbf, tsnd_mbf による待ち */
#define TTW_MBF 0x0100		/* rcv_mbf, trcv_mbf による待ち */
#define TTW_CAL 0x0200		/* ランデブ呼出待ち */
#define TTW_ACP 0x0400		/* ランデブ受付待ち */
#define TTW_RDV 0x0800		/* ランデブ終了待ち */
#define TTW_MPL 0x1000		/* get_blk, tget_blk による待ち */
#define TTW_MPF 0x2000		/* get_blf, tget_blf による待ち */

/* --- セマフォ関係 ------------------------------------------------------ */

/* cre_sem */
typedef struct t_csem {
	VP	exinf;		/* 拡張情報 */
	ATR	sematr;		/* セマフォ属性 */
	/* level X */
	INT	isemcnt;	/* セマフォの初期値 */
	INT	maxsem;		/* セマフォの最大値 */
	/* rtmk work */
	ID	semid;		/* セマフォＩＤ */
	INT	semcnt;		/* 現在のセマフォカウント */
	struct t_csem *lnk_t_csem;	/* セマフォリンク */
	union {
	 ID	semflg[sizeof(ID *)/sizeof(ID)];/* セマフォ取得フラグ */
	 ID	*psemflg;			/* セマフォ取得ポインタ */
	} flg;
} T_CSEM;

#define TA_TFIFO 0x00		/* 待ちタスクはＦＩＦＯ */
#define TA_TPRI 0x01		/* 待ちタスクは優先度順 */

/* ref_sem */
typedef struct t_rsem {
	VP	exinf;		/* 拡張情報 */
	BOOL_ID	wtsk;		/* 待ちタスクの有無 */
	INT	semcnt;		/* 現在のセマフォカウント */
} T_RSEM;

/* --- イベントフラグ関係 ------------------------------------------------ */

/* cre_ref */
typedef struct t_cfkg {
	VP	exinf;		/* 拡張情報 */
	ATR	flgatr;		/* イベントフラグ属性 */
	UINT	iflgptn;	/* イベントフラグの初期値 */
} T_CFLG;

#define TA_WSGL 0x00		/* 複数のタスク待ちを許さない */
#define TA_WMUL 0x08		/* 複数のタスク待ちを許す */

#define TWF_ANDW 0x00		/* ＡＮＤ待ち */
#define TWF_ORW 0x02		/* ＯＲ待ち */
#define TWF_CLR 0x01		/* クリア指定 */

/* ref_flg */
typedef struct t_rflg {
	VP	exinf;		/* 拡張情報 */
	BOOL_ID	wtsk;		/* 待ちタスクの有無 */
	UINT	flgptn;		/* イベントフラグのビットパターン */
} T_RFLG;

/* --- メイルボックス関係 ------------------------------------------------ */

/* cre_mbx */
typedef struct t_cmbx {
	VP	exinf;		/* 拡張情報 */
	ATR	mbxatr;		/* メイルボックス属性 */
	INT	bufcnt;		/* リングバッファの大きさ */
} T_CMBX;

#define TA_MFIFO 0x00		/* メッセージはＦＩＦＯ */
#define TA_MPRI 0x01		/* メッセージは優先度順 */

typedef struct t_msg {
	VB	*msgcnt;
} T_MSG;

/* ref_mbx */
typedef struct t_rmbx {
	VP	exinf;		/* 拡張情報 */
	BOOL_ID	wtsk;		/* 待ちタスクの有無 */
	T_MSG*	pk_msg;		/* 次に受信されるメッセージ */
} T_RMBX;

/* --- メッセージバッファ関係 -------------------------------------------- */

/* cre_mbf */
typedef struct t_cmbf {
	VP	exinf;		/* 拡張情報 */
	ATR	mbfatr;		/* メッセージバッファ属性 */
	INT	bufsz;		/* メッセージバッファのサイズ */
	INT	maxmsz;		/* メッセージの最大長 */
} T_CMBF;

#define TMBF_OS (-4)		/* ＯＳのエラーログ用のメッセージバッファ */
#define TMBF_DB (-3)		/* デバッグ用のメッセージバッファ */

/* ref_mbf */
typedef struct t_rmbf {
	VP	exinf;		/* 拡張情報 */
	BOOL_ID	wtsk;		/* 受信待ちタスクの有無 */
	BOOL_ID	stsk;		/* 送信待ちタスクの有無 */
	INT	msgsz;		/* 次に受信されるメッセージのサイズ */
	INT	frbufsz;	/* 空きバッファのサイズ */
} T_RMBF;

/* --- ポート、ランデブ関係 ---------------------------------------------- */

/* cre_por */
typedef struct t_cpor {
	VP	exinf;		/* 拡張情報 */
	ATR	poratr;		/* ポート属性 */
	INT	maxcmsz;	/* 呼出時のメッセージの最大長 */
	INT	maxrmsz;	/* 返答時のメッセージの最大長 */
} T_CPOR;

#define TA_NULL 0		/* 特別な属性を指定しない */

/* ref_por */
typedef struct t_rpor {
	VP	exinf;		/* 拡張情報 */
	BOOL_ID	wtsk;		/* 呼出待ちタスクの有無 */
	BOOL_ID	stsk;		/* 返答待ちタスクの有無 */
} T_RPOR;

/* --- 割り込み管理関係 -------------------------------------------------- */

/* def_int */
typedef struct t_dint {
	ATR	intatr;		/* 割り込みハンドラ属性 */
	FP	inthdr;		/* 割り込みハンドラアドレス */
	/* rtmk work */
	FP	*inttbl;	/* 割り込みベクタテーブル */
	FP	intini;		/* 初期化ルーチンアドレス */
} T_DINT;

/* --- メモリプール関係 -------------------------------------------------- */

/* cre_mpl */
typedef struct t_cmpl {
	VP	exinf;		/* 拡張情報 */
	ATR	mplatr;		/* メモリプール属性 */
	INT	mplsz;		/* メモリプール全体のサイズ */
} T_CMPL;

#define TMPL_OS	(-4)		/* ＯＳ用のメモリプール */

/* ref_mpl */
typedef struct t_rmpl {
	VP	exinf;		/* 拡張情報 */
	BOOL_ID	wtsk;		/* 待ちタスクの有無 */
	INT	frsz;		/* 空き領域の合計サイズ */
	INT	maxsz;		/* 最大の連続空き領域のサイズ */
} T_RMPL;

/* xre_mpf */
typedef struct t_cmpf {
	VP	exinf;		/* 拡張情報 */
	ATR	mpfatr;		/* メモリプール属性 */
	INT	mpfcnt;		/* メモリプール全体のブロック数 */
	INT	blfsz;		/* 固定長メモリブロックサイズ */
} T_CMPF;

/* ref_mpf */
typedef struct t_rmpf {
	VP	exinf;		/* 拡張情報 */
	BOOL_ID	wtsk;		/* 待ちタスクの有無 */
	INT	frbcnt;		/* 空き領域のブロック数 */
} T_RMPF;

/* --- 時間管理関係 ------------------------------------------------------ */

typedef struct t_systime {
	H	utime;		/* 上位16ビット */
	UW	ltime;		/* 下位32ビット */
} SYSTIME, CYCTIME, ALMTIME;

/* def_cyc */
typedef struct t_dcyc {
	VP	exinf;		/* 拡張情報 */
	ATR	cycatr;		/* 周期起動ハンドラ属性 */
	FP	cychdr;		/* 周期起動ハンドラアドレス */
	UINT	cycact;		/* 周期起動ハンドラ活性状態 */
	CYCTIME	cyctim;		/* 周期起動時間間隔 */
} T_DCYC;

#define TCY_OFF 0x00		/* 周期起動ハンドラが起動されない */
#define TCY_ON 0x01		/* 周期起動ハンドラが起動される */
#define TCY_INI 0x02		/* 周期のカウントが初期化される */

/* ref_cyc */
typedef struct t_rcyc {
	VP	exinf;		/* 拡張情報 */
	CYCTIME	lfttim;		/* 次のハンドラ起動までの残り時間 */
	UINT	cycact;		/* 周期起動ハンドラ活性状態 */
} T_RCYC;

/* def_alm */
typedef struct t_dalm {
	VP	exinf;		/* 拡張情報 */
	ATR	almatr;		/* アラームハンドラ属性 */
	FP	almhdr;		/* アラームハンドラアドレス */
	UINT	tmmode;		/* 起動時刻指定モード */
	ALMTIME	almtim;		/* ハンドラ起動時刻 */
} T_DALM;

#define TTM_ABS 0x00		/* 絶対時刻での指定 */
#define TTM_REL 0x01		/* 相対時刻での指定 */

/* ref_arm */
typedef struct t_ralm {
	VP	exinf;		/* 拡張情報 */
	ALMTIME	lfttim;		/* ハンドラ起動までの残り時間 */
} T_RALM;

/* --- システム管理関係 -------------------------------------------------- */

/* get_ver */
typedef struct t_ver {
	UH	maker;		/* メーカー */
	UH	id;		/* 形式番号 */
	UH	spver;		/* 仕様書バージョン */
	UH	prver;		/* 製品バージョン */
	UH	prno[4];	/* 製品管理情報 */
	UH	cpu;		/* ＣＰＵ情報 */
	UH	ver;		/* バリエーション記述子 */
} T_VER;

/* ref_sys */
typedef struct t_rsys {
	INT	sysstat;	/* システム状態 */
} T_RSYS;

#define TSS_TSK 0	/* タスク部実行中でディスパッチを許可した通常の状態 */
#define TSS_DDSP 1	/* タスク部実行中で dis_dsp を実行した後の状態 */
#define TSS_LOC 3	/* タスク部実行中で loc_cpu を実行した後の状態 */
#define TSS_INDP 4	/* タスク独立部 */

/* ref_cfg */
typedef struct t_rcfg {
	VP	vp;
} T_RCFG;

/* def_svc */
typedef struct t_dsvc {
	ATR	svcatr;		/* 拡張ＳＶＣハンドラ属性 */
	FP	svchdr;		/* 拡張ＳＶＣハンドラアドレス */
} T_DSVC;

/* def_exc */
typedef struct t_dexc {
	ATR	excatr;		/* 例外ハンドラ属性 */
	FP	exchdr;		/* 例外ハンドラアドレス */
} T_DEXC;

#define	TER_FNC	1		/* 終了ハンドラ */

/* --- ネットワークサポート関係 ------------------------------------------ */

typedef	NODE srcnode, dstnode, node;

#define TND_SELF 0		/* 自ノードの指定 */
#define TND_OTHE (-1)		/* デフォルトの他ノード指定 */

/* --- エラーコード ------------------------------------------------------ */

#define E_OK	0	/* 正常終了 */
#define E_SYS	(-5)	/* システムエラー */
#define E_NOMEM	(-10)	/* メモリ不足 */
#define E_NOSPT	(-17)	/* 未サポート機能 */
#define E_INOSPT (-18)	/* ITRON/FILE での未サポート機能 */
#define E_RSFN	(-20)	/* 予約機能コード番号 */
#define E_RSATR	(-24)	/* 予約属性 */
#define E_PAR	(-33)	/* パラメータエラー */
#define E_ID	(-35)	/* 不正ＩＤ番号 */
#define E_NOEXS	(-52)	/* オブジェクトが存在していない */
#define E_OBJ	(-63)	/* オブジェクトの状態が不正 */
#define E_MACV	(-65)	/* メモリアクセス不能、メモリアクセス権違反 */
#define E_OACV	(-66)	/* オブジェクトアクセス権違反 */
#define E_CTX	(-69)	/* コンテキストエラー */
#define E_QOVR	(-73)	/* キューイングまたはネストのオーバーフロー */
#define E_DLT	(-81	/* 待ちオブジェクトが削除された */
#define E_TMOUT	(-85)	/* ポーリング失敗またはタイムアウト */
#define E_RLWAI	(-86)	/* 待ち状態強制解除 */

#define EN_NODE	(-113)	/* 対象ノードが存在しない、対象ノードにアクセスできない */
#define EN_OBJNO (-114)	/* 対象ノード側でアクセスできないオブジェクト番号を指定 */
#define EN_PROTO (-115)	/* 対象ノードでサポートされていないプロトコル */
#define EN_RSFN	(-116)	/* 対象ノードでサポートされていないシステムコールや機能 */
#define EN_COMM	(-117)	/* 対象ノードからの応答が無い */
#define EN_RLWAI (-118)	/* 接続機能応答待ち状態の強制解除 */
#define EN_PAR	(-119)	/* 対象ノードや通信パケットでサポートされていない範囲の値をパラメータとして指定した */
#define EN_RPAR	(-120)	/* 対象ノードや通信パケットでサポートされていない範囲の値をリターンパラメータとして返された */
#define EN_CTXID (-121)	/* タスク独立部あるいはディスパッチ禁止状態のタスクから発行されたシステムコールで他ノード上のオブジェクトを指定 */
#define EN_EXEC	(-122)	/* 対象ノードにおける資源不足のため実行できない */
#define EN_NOSPT (-123)	/* サポートされていない接続機能 */

/* --- prototype --------------------------------------------------------- */

ER vini_rtmk( ID, T_CTSK * );		/* rtmk initialize */
ER vini_heap( UB *, UW );		/* heap initialize */
void *vget_heap( UB *, UW );		/* get heap memory */
void vfre_heap( UB *, UB *, UW );	/* free heap memory */
void vbgn_int( void );			/* bigin interrupt */

ER dis_dsp( void );
ER ena_dsp( void );
#define slp_tsk() tslp_tsk( TMO_FEVR )
ER loc_cpu( void );
ER unl_cpu( void );
void ret_int( void );
ER wup_tsk( ID );
void ext_tsk( void );
ER rot_rdq( PRI );
ER get_tid( ID * );
ER dly_tsk( DLYTIME );
ER sta_tsk( ID, INT );
void exd_tsk( void );
ER tslp_tsk( TMO );
void ret_wup( ID );
ER ref_sys( T_RSYS * );
ER cre_tsk( ID, T_CTSK * );
ER def_int( UINT, T_DINT * );
ER def_exc( UINT, T_DEXC * );

