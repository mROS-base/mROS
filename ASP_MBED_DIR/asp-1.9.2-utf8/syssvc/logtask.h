/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2008 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: logtask.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		システムログタスク
 */

#ifndef TOPPERS_LOGTASK_H
#define TOPPERS_LOGTASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "target_syssvc.h"

/*
 *  システムログタスク関連の定数のデフォルト値の定義
 */ 
#ifndef LOGTASK_PRIORITY
#define LOGTASK_PRIORITY	4		/* 初期優先度 */
#endif /* LOGTASK_PRIORITY */

#ifndef LOGTASK_STACK_SIZE
#define LOGTASK_STACK_SIZE	1024	/* スタック領域のサイズ */
#endif /* LOGTASK_STACK_SIZE */

#ifndef LOGTASK_PORTID
#define LOGTASK_PORTID		1		/* システムログ用のシリアルポート番号 */
#endif /* LOGTASK_PORTID */

#ifndef LOGTASK_INTERVAL
#define LOGTASK_INTERVAL	10U		/* システムログタスクの動作間隔（ミリ秒）*/
#endif /* LOGTASK_INTERVAL */

#ifndef LOGTASK_FLUSH_WAIT
#define LOGTASK_FLUSH_WAIT	1U		/* フラッシュ待ちの単位時間（ミリ秒）*/
#endif /* LOGTASK_FLUSH_WAIT */

/*
 *  システムログ出力の待ち合わせ
 *
 *  ログバッファ中のログの数がcount以下になるまで待つ．countが0の場合に
 *  は，シリアルバッファが空になるのも待つ．
 */
extern ER	logtask_flush(uint_t count) throw();

/*
 *  システムログタスクの本体
 */
extern void	logtask_main(intptr_t exinf) throw();

/*
 *  システムログタスクの終了処理
 */
extern void	logtask_terminate(intptr_t exinf) throw();

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_LOGTASK_H */
