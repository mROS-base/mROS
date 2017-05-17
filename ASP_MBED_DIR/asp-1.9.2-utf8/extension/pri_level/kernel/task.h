/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: task.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		タスク管理モジュール
 */

#ifndef TOPPERS_TASK_H
#define TOPPERS_TASK_H

#include <queue.h>
#include "time_event.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_TSKSTAT
#define LOG_TSKSTAT(p_tcb)
#endif /* LOG_TSKSTAT */

/*
 *  タスク優先度の内部表現・外部表現変換マクロ
 */
#define INT_PRIORITY(x)		((uint_t)((x) - TMIN_TPRI))
#define EXT_TSKPRI(x)		((PRI)(x) + TMIN_TPRI)

/*
 *  タスク状態の内部表現
 *
 *  TCB中のタスク状態では，実行状態（RUNNING）と実行可能状態（READY）
 *  は区別しない．両状態を総称して，実行できる状態（RUNNABLE）と呼ぶ．
 *  二重待ち状態は，(TS_WAITING | TS_SUSPENDED)で表す．TS_WAIT_???は待
 *  ち要因を表し，待ち状態（二重待ち状態を含む）の場合にのみ設定する．
 */
#define TS_DORMANT		0x00U			/* 休止状態 */
#define TS_RUNNABLE		0x01U			/* 実行できる状態 */
#define TS_WAITING		0x02U			/* 待ち状態 */
#define TS_SUSPENDED	0x04U			/* 強制待ち状態 */

#define TS_WAIT_DLY		(0x00U << 3)	/* 時間経過待ち */
#define TS_WAIT_SLP		(0x01U << 3)	/* 起床待ち */
#define TS_WAIT_RDTQ	(0x02U << 3)	/* データキューからの受信待ち */
#define TS_WAIT_RPDQ	(0x03U << 3)	/* 優先度データキューからの受信待ち */
#define TS_WAIT_SEM		(0x04U << 3)	/* セマフォ資源の獲得待ち */
#define TS_WAIT_FLG		(0x05U << 3)	/* イベントフラグ待ち */
#define TS_WAIT_SDTQ	(0x06U << 3)	/* データキューへの送信待ち */
#define TS_WAIT_SPDQ	(0x07U << 3)	/* 優先度データキューへの送信待ち */
#define TS_WAIT_MBX		(0x08U << 3)	/* メールボックスからの受信待ち */
#define TS_WAIT_MPF		(0x09U << 3)	/* 固定長メモリブロックの獲得待ち */

/*
 *  タスク状態判別マクロ
 *
 *  TSTAT_DORMANTはタスクが休止状態であるかどうかを，TSTAT_RUNNABLEは
 *  タスクが実行できる状態であるかどうかを判別する．TSTAT_WAITINGは待
 *  ち状態と二重待ち状態のいずれかであるかどうかを，TSTAT_SUSPENDEDは
 *  強制待ち状態と二重待ち状態のいずれかであるかどうかを判別する．
 */
#define TSTAT_DORMANT(tstat)	((tstat) == TS_DORMANT)
#define TSTAT_RUNNABLE(tstat)	(((tstat) & TS_RUNNABLE) != 0U)
#define TSTAT_WAITING(tstat)	(((tstat) & TS_WAITING) != 0U)
#define TSTAT_SUSPENDED(tstat)	(((tstat) & TS_SUSPENDED) != 0U)

/*
 *  タスク待ち要因判別マクロ
 *
 *  TSTAT_WAIT_SLPはタスクが起床待ちであるかどうかを，TSTAT_WAIT_WOBJ
 *  はタスクが同期・通信オブジェクトに対する待ちであるか（言い換えると，
 *  同期通信オブジェクトの待ちキューにつながれているか）どうかを判別す
 *  る．また，TSTAT_WAIT_WOBJCBはタスクが同期・通信オブジェクトの管理
 *  ブロックの共通部分（WOBJCB）の待ちキューにつながれているかどうかを
 *  判別する．
 *
 *  TSTAT_WAIT_SLPは，任意のタスク状態の中から，タスクが起床待ちである
 *  ことを判別できる．すなわち，TSTAT_WAITINGにより待ち状態であることを
 *  判別せずに，TSTAT_SLPだけを用いて起床待ち状態であることを判別できる．
 *  これを効率的に実現するために，TS_WAIT_SLPの値を，(0x00U << 3)ではな
 *  く(0x01U << 3)としている．そのため，タスクが時間経過待ち状態である
 *  ことを判別するためのTSTAT_WAIT_DLYを，TSTAT_WAIT_SLPと同様の方法で
 *  実現することはできない．
 */
#define TS_WAIT_MASK	(0x0fU << 3)	/* 待ち要因の取出しマスク */

#define TSTAT_WAIT_SLP(tstat)		(((tstat) & TS_WAIT_MASK) == TS_WAIT_SLP)
#define TSTAT_WAIT_WOBJ(tstat)		(((tstat) & TS_WAIT_MASK) >= TS_WAIT_RDTQ)
#define TSTAT_WAIT_WOBJCB(tstat)	(((tstat) & TS_WAIT_MASK) >= TS_WAIT_SEM)

/*
 *  待ち情報ブロック（WINFO）の定義
 *
 *  タスクが待ち状態の間は，TCBおよびそのp_winfoで指されるWINFOを次の
 *  ように設定しなければならない．
 *
 *  (a) TCBのタスク状態を待ち状態（TS_WAITING）にする．その際に，待ち
 *  要因（TS_WAIT_???）も設定する．
 *
 *  (b) タイムアウトを監視するために，タイムイベントブロックを登録する．
 *  登録するタイムイベントブロックは，待ちに入るサービスコール処理関数
 *  のローカル変数として確保し，それへのポインタをWINFOのp_tmevtbに記
 *  憶する．タイムアウトの監視が必要ない場合（永久待ちの場合）には，
 *  p_tmevtbをNULLにする．
 *
 *  同期・通信オブジェクトに対する待ち状態の場合には，標準のWINFOに
 *  p_wobjcbフィールドを追加した構造体（WINFO_WOBJ，wait.hで定義）を使
 *  う．また，以下の(c)～(e)の設定を行う必要がある．同期・通信オブジェ
 *  クトに関係しない待ち（起床待ち，時間経過待ち）の場合には，(c)～(e)
 *  は必要ない．
 *
 *  (c) TCBを待ち対象の同期・通信オブジェクトの待ちキューにつなぐ．待
 *  ちキューにつなぐために，task_queueを使う．
 *
 *  (d) 待ち対象の同期・通信オブジェクトの管理ブロックへのポインタを，
 *  WINFO_WOBJのp_wobjcbに記憶する．
 *
 *  (e) 待ち対象の同期・通信オブジェクトに依存して記憶することが必要な
 *  情報がある場合には，WINFO_WOBJに必要な情報のためのフィールドを追加
 *  した構造体を定義し，WINFO_WOBJの代わりに用いる．
 *
 *  待ち状態を解除する際には，待ち解除したタスクに対する返値をWINFOの
 *  wercdに設定する．wercdが必要なのは待ち解除以降であるのに対して，
 *  p_tmevtbは待ち解除後は必要ないため，メモリ節約のために共用体を使っ
 *  ている．そのため，wercdへエラーコードを設定するのは，タイムイベント
 *  ブロックを登録解除した後にしなければならない．
 */
typedef union waiting_information {
	ER		wercd;			/* 待ち解除時のエラーコード */
	TMEVTB	*p_tmevtb;		/* 待ち状態用のタイムイベントブロック */
} WINFO;

/*
 *  タスク初期化ブロック
 *
 *  タスクに関する情報を，値が変わらないためにROMに置ける部分（タスク
 *  初期化ブロック）と，値が変化するためにRAMに置かなければならない部
 *  分（タスク管理ブロック，TCB）に分離し，TCB内に対応するタスク初期化
 *  ブロックを指すポインタを入れる．タスク初期化ブロック内に対応する
 *  TCBを指すポインタを入れる方法の方が，RAMの節約の観点からは望ましい
 *  が，実行効率が悪くなるために採用していない．他のオブジェクトについ
 *  ても同様に扱う．
 *
 *  タスク初期化ブロックには，DEF_TEXで定義されるタスク例外処理ルーチ
 *  ンに関する情報も含む．
 */
typedef struct task_initialization_block {
	ATR			tskatr;			/* タスク属性 */
	intptr_t	exinf;			/* タスクの拡張情報 */
	TASK		task;			/* タスクの起動番地 */
	uint_t		ipriority;		/* タスクの起動時優先度（内部表現） */

#ifdef USE_TSKINICTXB
	TSKINICTXB	tskinictxb;		/* タスク初期化コンテキストブロック */
#else /* USE_TSKINICTXB */
	SIZE		stksz;			/* スタック領域のサイズ（丸めた値） */
	void		*stk;			/* スタック領域の先頭番地 */
#endif /* USE_TSKINICTXB */

	ATR			texatr;			/* タスク例外処理ルーチン属性 */
	TEXRTN		texrtn;			/* タスク例外処理ルーチンの起動番地 */
} TINIB;

/*
 *  TCB中のフィールドのビット幅の定義
 *
 *  プロセッサによっては，TCB中のフィールドのビット幅でメモリ使用量と
 *  性能がトレードオフになるため，ターゲット依存にフィールドのビット幅
 *  を変更することを許している．
 */
#ifndef TBIT_TCB_PRIORITY
#define	TBIT_TCB_PRIORITY		8		/* priorityフィールドのビット幅 */
#endif /* TBIT_TCB_PRIORITY */

/*
 *  タスク管理ブロック（TCB）
 *
 *  ASPカーネルでは，タスクの起動要求キューイング数の最大値（TMAX_ACTCNT）
 *  と起床要求キューイング数の最大値（TMAX_WUPCNT）は1に固定されている
 *  ため，キューイングされているかどうかの真偽値で表現することができる．
 *  また，強制待ち要求ネスト数の最大値（TMAX_SUSCNT）が1に固定されてい
 *  るので，強制待ち要求ネスト数（suscnt）は必要ない．
 *
 *  TCBのいくつかのフィールドは，特定のタスク状態でのみ有効な値を保持し，
 *  それ以外の場合は値が保証されない（よって，参照してはならない）．各
 *  フィールドが有効な値を保持する条件は次の通り．
 *
 *  ・初期化後は常に有効：
 *  		p_tinib，tstat，actque
 *  ・休止状態以外で有効（休止状態では初期値になっている）：
 *  		priority，wupque，enatex，texptn
 *  ・待ち状態（二重待ち状態を含む）で有効：
 *  		p_winfo
 *  ・実行できる状態と同期・通信オブジェクトに対する待ち状態で有効：
 *  		task_queue
 *  ・実行可能状態，待ち状態，強制待ち状態，二重待ち状態で有効：
 *  		tskctxb
 */
typedef struct task_control_block {
	QUEUE			task_queue;		/* タスクキュー */
	const TINIB		*p_tinib;		/* 初期化ブロックへのポインタ */

#ifdef UINT8_MAX
	uint8_t			tstat;			/* タスク状態（内部表現）*/
#else /* UINT8_MAX */
	BIT_FIELD_UINT	tstat : 8;		/* タスク状態（内部表現）*/
#endif /* UINT8_MAX */
#if defined(UINT8_MAX) && (TBIT_TCB_PRIORITY == 8)
	uint8_t			priority;		/* 現在の優先度（内部表現）*/
#else /* defined(UINT8_MAX) && (TBIT_TCB_PRIORITY == 8) */
	BIT_FIELD_UINT	priority : TBIT_TCB_PRIORITY;
									/* 現在の優先度（内部表現）*/
#endif /* defined(UINT8_MAX) && (TBIT_TCB_PRIORITY == 8) */
	BIT_FIELD_BOOL	actque : 1;		/* 起動要求キューイング */
	BIT_FIELD_BOOL	wupque : 1;		/* 起床要求キューイング */
	BIT_FIELD_BOOL	enatex : 1;		/* タスク例外処理許可状態 */

	TEXPTN			texptn;			/* 保留例外要因 */
	WINFO			*p_winfo;		/* 待ち情報ブロックへのポインタ */
	TSKCTXB			tskctxb;		/* タスクコンテキストブロック */
} TCB;

/*
 *  実行状態のタスク
 *
 *  実行状態のタスク（＝プロセッサがコンテキストを持っているタスク）の
 *  TCBを指すポインタ．実行状態のタスクがない場合はNULLにする．
 *
 *  サービスコールの処理中で，自タスク（サービスコールを呼び出したタス
 *  ク）に関する情報を参照する場合はp_runtskを使う．p_runtskを書き換え
 *  るのは，ディスパッチャ（と初期化処理）のみである．
 */
extern TCB	*p_runtsk;

/*
 *  最高優先順位のタスク
 *
 *  実行できるタスクの中で最高優先順位のタスクのTCBを指すポインタ．実
 *  行できるタスクがない場合はNULLにする．
 *
 *  ディスパッチ禁止状態など，ディスパッチが保留されている間はp_runtsk
 *  と一致しているとは限らない．
 */
extern TCB	*p_schedtsk;

/*
 *  ディスパッチ／タスク例外処理ルーチン起動要求フラグ
 *
 *  割込みハンドラ／CPU例外ハンドラの出口処理に，ディスパッチまたは
 *  タスク例外処理ルーチンの起動を要求することを示すフラグ．
 */
extern bool_t	reqflg;

/*
 *  割込み優先度マスク全解除状態
 *
 *  割込み優先度マスク全解除状態であることを示すフラグ．
 */
extern bool_t	ipmflg;

/*
 *  ディスパッチ禁止状態
 *
 *  ディスパッチ禁止状態であることを示すフラグ．
 */
extern bool_t	disdsp;

/*
 *  タスクディスパッチ可能状態
 *
 *  割込み優先度マスク全解除状態であり，ディスパッチ許可状態である（ディ
 *  スパッチ禁止状態でない）ことを示すフラグ．
 */
extern bool_t	dspflg;

/*
 *  レディキュー
 *
 *  レディキューは，実行できる状態のタスクを管理するためのキューである．
 *  実行状態のタスクも管理しているため，レディ（実行可能）キューという
 *  名称は正確ではないが，レディキューという名称が定着しているため，こ
 *  の名称で呼ぶことにする．
 *
 *  レディキューは，優先度ごとのタスクキューで構成されている．タスクの
 *  TCBは，該当する優先度のキューに登録される．
 */
extern QUEUE	ready_queue[TNUM_TPRI];

/*
 *  レディキューサーチのためのビットマップ
 *
 *  レディキューのサーチを効率よく行うために，優先度ごとのタスクキュー
 *  にタスクが入っているかどうかを示すビットマップを用意している．ビッ
 *  トマップを使うことで，メモリアクセスの回数を減らすことができるが，
 *  ビット操作命令が充実していないプロセッサで，優先度の段階数が少ない
 *  場合には，ビットマップ操作のオーバーヘッドのために，逆に効率が落ち
 *  る可能性もある．
 *
 *  優先度の段階数により，ビットマップを1段階にするか2段階にするかを決
 *  定する．2段階で足りない場合には対応していない．
 */
#define TBIT_PRIMAP		16				/* ビットマップ1つあたりの段階数 */

#if TNUM_TPRI <= TBIT_PRIMAP
#define PRIMAP_LEVEL_1					/* ビットマップを1段階に */
#elif TNUM_TPRI <= TBIT_PRIMAP * TBIT_PRIMAP
#define PRIMAP_LEVEL_2					/* ビットマップを2段階に */
#define TNUM_PRIMAP2	((TNUM_TPRI + TBIT_PRIMAP - 1) / TBIT_PRIMAP)
#else
#error too many task priority levels.
#endif

#ifdef PRIMAP_LEVEL_1
extern uint16_t	ready_primap;					/* 優先度ビットマップ */
#else /* PRIMAP_LEVEL_1 */
extern uint16_t	ready_primap1;					/* 1段目の優先度ビットマップ */
extern uint16_t	ready_primap2[TNUM_PRIMAP2];	/* 2段目の優先度ビットマップ */
#endif /* PRIMAP_LEVEL_1 */

/*
 *  タスクIDの最大値（kernel_cfg.c）
 */
extern const ID	tmax_tskid;

/*
 *  タスク初期化ブロックのエリア（kernel_cfg.c）
 */
extern const TINIB	tinib_table[];

/*
 *  タスク生成順序テーブル（kernel_cfg.c）
 */
extern const ID	torder_table[];

/*
 *  TCBのエリア（kernel_cfg.c）
 */
extern TCB	tcb_table[];

/*
 *  タスクの数
 */
#define tnum_tsk	((uint_t)(tmax_tskid - TMIN_TSKID + 1))

/*
 *  タスクIDからTCBを取り出すためのマクロ
 */
#define INDEX_TSK(tskid)	((uint_t)((tskid) - TMIN_TSKID))
#define get_tcb(tskid)		(&(tcb_table[INDEX_TSK(tskid)]))
#define get_tcb_self(tskid)	((tskid) == TSK_SELF ? p_runtsk : get_tcb(tskid))

/*
 *  TCBからタスクIDを取り出すためのマクロ
 */
#define	TSKID(p_tcb)	((ID)(((p_tcb) - tcb_table) + TMIN_TSKID))

/*
 *  タスク管理モジュールの初期化
 */
extern void	initialize_task(void);

/*
 *  最高優先順位タスクのサーチ
 *
 *  レディキュー中の最高優先順位のタスクをサーチし，そのTCBへのポインタ
 *  を返す．レディキューが空の場合には，この関数を呼び出してはならない．
 */
extern TCB	*search_schedtsk(void);

/*
 *  実行できる状態への遷移
 *
 *  p_tcbで指定されるタスクをレディキューに挿入する．レディキューに挿入
 *  したタスクの優先度が，最高優先順位のタスクの優先度よりも高い場合は，
 *  最高優先順位のタスクを更新し，ディスパッチ許可状態であればtrueを返
 *  す．そうでない場合はfalseを返す．
 */
extern bool_t	make_runnable(TCB *p_tcb);

/*
 *  実行できる状態から他の状態への遷移
 *
 *  p_tcbで指定されるタスクをレディキューから削除する．p_tcbで指定した
 *  タスクが最高優先順位のタスクであった場合には，最高優先順位のタスク
 *  を設定しなおし，ディスパッチ許可状態であればtrueを返す．そうでない
 *  場合はfalseを返す．タスクの状態は更新しない．
 */
extern bool_t	make_non_runnable(TCB *p_tcb);

/*
 *  休止状態への遷移
 *
 *  p_tcbで指定されるタスクの状態を休止状態とする．また，タスクの起動
 *  時に初期化すべき変数の初期化と，タスク起動のためのコンテキストを設
 *  定する．
 */
extern void	make_dormant(TCB *p_tcb);

/*
 *  休止状態から実行できる状態への遷移
 *
 *  p_tcbで指定されるタスクの状態を休止状態から実行できる状態とする．
 *  実行できる状態に遷移したタスクへのディスパッチが必要な場合はtrue，
 *  そうでない場合はfalseを返す．
 */
extern bool_t	make_active(TCB *p_tcb);

/*
 *  タスクの優先度の変更
 *
 *  p_tcbで指定されるタスクの優先度をnewpri（内部表現）に変更する．また，
 *  必要な場合には最高優先順位のタスクを更新し，ディスパッチ許可状態で
 *  あればtrueを返す．そうでない場合はfalseを返す．
 */
extern bool_t	change_priority(TCB *p_tcb, uint_t newpri);

/*
 *  レディキューの回転
 *
 *  レディキュー中の，priで指定される優先度のタスクキューを回転させる．
 *  また，必要な場合には最高優先順位のタスクを変更し，ディスパッチが保
 *  留されていなければtrueを返す．そうでない場合はfalseを返す．
 */
extern bool_t	rotate_ready_queue(uint_t pri);

/*
 *  タスク例外処理ルーチンの呼出し
 *
 *  タスク例外処理ルーチンを呼び出す．呼び出す前に，実行状態のタスクの
 *  保留例外要因をクリアし，タスク例外処理禁止状態にし，CPUロックを解
 *  除する．
 *
 *  タスク例外処理ルーチンから戻ると，まずCPUロック状態に戻し，その間
 *  に保留例外要因が0でなくなっていれば，再びタスク例外処理ルーチンを
 *  呼び出す．保留例外要因が0の場合には，例外処理許可状態にして関数か
 *  らリターンする．
 *
 *  この関数は，実行状態のタスクが，タスク例外処理許可状態（enatexが
 *  true）で，保留例外要因が0でない（texptnが0でない）場合に呼び出すこ
 *  とを想定している．この関数は，CPUロック状態で呼び出さなければなら
 *  ない．
 */
extern void	call_texrtn(void);

/*
 *  タスク例外処理ルーチンの起動
 *
 *  実行状態のタスクがタスク例外処理ルーチンの起動条件を満たしていれば，
 *  タスク例外処理ルーチンを呼び出す．CPU例外処理ルーチンを呼び出す時
 *  は，一時的にCPUロックを解除する．
 *
 *  この関数は，ディスパッチャや割込みハンドラ／CPU例外ハンドラの出口
 *  処理から呼び出されることを想定している．この関数は，CPUロック状態
 *  で呼び出さなければならない．
 *
 *  実行効率を上げるために，この関数をターゲット依存部で記述してもよい．
 *  その場合には，OMIT_CALLTEXをマクロ定義する．
 */
extern void	calltex(void);

#endif /* TOPPERS_TASK_H */
