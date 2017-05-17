/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: itron.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		ITRON仕様共通規定のデータ型・定数・マクロ
 *
 *  このヘッダファイルは，ITRON仕様共通規定のデータ型・定数・マクロの中
 *  で，TOPPERS共通ヘッダファイルに含まれないものの定義を含む．ITRON仕
 *  様との互換性を必要とするアプリケーションがインクルードすることを想
 *  定している．
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておく．これにより，マクロ定義以外を
 *  除くようになっている．
 */

#ifndef TOPPERS_ITRON_H
#define TOPPERS_ITRON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  TOPPERS共通ヘッダファイル
 */
#include "t_stddef.h"

/*
 *  ITRON仕様共通データ型
 */
#ifndef TOPPERS_MACRO_ONLY

#ifdef INT8_MAX
typedef	int8_t			B;			/* 符号付き8ビット整数 */
#endif /* INT8_MAX */

#ifdef UINT8_MAX
typedef	uint8_t			UB;			/* 符号無し8ビット整数 */
typedef	uint8_t			VB;			/* 型が定まらない8ビットの値 */
#endif /* UINT8_MAX */

typedef	int16_t			H;			/* 符号付き16ビット整数 */
typedef	uint16_t		UH;			/* 符号無し16ビット整数 */
typedef	uint16_t		VH;			/* 型が定まらない16ビットの値 */

typedef	int32_t			W;			/* 符号付き32ビット整数 */
typedef	uint32_t		UW;			/* 符号無し32ビット整数 */
typedef	uint32_t		VW;			/* 型が定まらない32ビットの値 */

#ifdef INT64_MAX
typedef	int64_t			D;			/* 符号付き64ビット整数 */
#endif /* INT64_MAX */

#ifdef UINT64_MAX
typedef	uint64_t		UD;			/* 符号無し64ビット整数 */
typedef	uint64_t		VD;			/* 型が定まらない64ビットの値 */
#endif /* UINT64_MAX */

typedef	void			*VP;		/* 型が定まらないものへのポインタ */

typedef int_t			INT;		/* 自然なサイズの符号付き整数 */
typedef uint_t			UINT;		/* 自然なサイズの符号無し整数 */

typedef bool_t			BOOL;		/* 真偽値 */

typedef	intptr_t		VP_INT;		/* ポインタまたは符号付き整数 */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ITRON仕様共通定数
 */
#define	TRUE		true			/* 真 */
#define	FALSE		false			/* 偽 */

/*
 *  オブジェクト属性の定義
 */
#define TA_HLNG			UINT_C(0x00)	/* 高級言語用インタフェース */
#define TA_TFIFO		UINT_C(0x00)	/* タスクの待ち行列をFIFO順に */
#define TA_MFIFO		UINT_C(0x00)	/* メッセージキューをFIFO順に */
#define TA_WSGL			UINT_C(0x00)	/* 待ちタスクは1つのみ */

/*
 *  ネスト回数の最大値
 */
#define TMAX_SUSCNT		UINT_C(1)		/* 強制待ち要求ネスト数の最大値 */

/*
 *  強制待ち状態からの強制再開
 */
#define frsm_tsk(tskid)		rsm_tsk(tskid)

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_ITRON_H */
