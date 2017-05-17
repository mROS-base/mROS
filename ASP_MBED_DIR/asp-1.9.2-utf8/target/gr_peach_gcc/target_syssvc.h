/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: target_syssvc.h 2758 2016-03-10 15:15:26Z ertl-honda $
 */

/*
 *  システムサービスのターゲット依存部（GR-PEACH用）
 *
 *  システムサービスのターゲット依存部のインクルードファイル．このファ
 *  イルの内容は，コンポーネント記述ファイルに記述され，このファイルは
 *  無くなる見込み．
 */

#ifndef TOPPERS_TARGET_SYSSVC_H
#define TOPPERS_TARGET_SYSSVC_H

#include "gr_peach.h"

/*
 *  トレースログに関する設定
 */
#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"
#endif /* TOPPERS_ENABLE_TRACE */

/*
 *  起動メッセージのターゲットシステム名
 */
#define TARGET_NAME    "GR-PEACH(RZA1H Cortex-A9)"

/*
 *  システムログの低レベル出力のための文字出力
 *
 *  ターゲット依存の方法で，文字cを表示/出力/保存する．
 */
extern void	target_fput_log(char c);

/*
 *  ログタスクで使用するポート
 */
#define LOGTASK_PORTID  USE_SIO_PORTID

/*
 *  ボーレート設定
 */
#define UART1_BPS_SETTING  115200
#define UART2_BPS_SETTING  115200
#define UART3_BPS_SETTING  115200
#define UART4_BPS_SETTING  115200
#define UART5_BPS_SETTING  115200
#define UART6_BPS_SETTING  115200
#define UART7_BPS_SETTING  115200
#define UART8_BPS_SETTING  115200

/*
 *  サポートするシリアルポートの数
 */
#ifndef TNUM_PORT
#define TNUM_PORT       4
#endif /* TNUM_PORT */

/*
 *  システムログタスク関連の定数の定義
 *
 *  デフォルト値の通り．
 */

#endif /* TOPPERS_TARGET_SYSSVC_H */
