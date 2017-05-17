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
 *  @(#) $Id: strerror.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		エラーメッセージ文字列を返す関数
 */

#include <t_stddef.h>
#include <t_stdlib.h>

const char *
itron_strerror(ER ercd)
{
	switch (MERCD(ercd)) {
	case E_OK:
		return("E_OK");

	case E_SYS:
		return("E_SYS");
	case E_NOSPT:
		return("E_NOSPT");
	case E_RSFN:
		return("E_RSFN");
	case E_RSATR:
		return("E_RSATR");

	case E_PAR:
		return("E_PAR");
	case E_ID:
		return("E_ID");

	case E_CTX:
		return("E_CTX");
	case E_MACV:
		return("E_MACV");
	case E_OACV:
		return("E_OACV");
	case E_ILUSE:
		return("E_ILUSE");

	case E_NOMEM:
		return("E_NOMEM");
	case E_NOID:
		return("E_NOID");
	case E_NORES:
		return("E_NORES");

	case E_OBJ:
		return("E_OBJ");
	case E_NOEXS:
		return("E_NOEXS");
	case E_QOVR:
		return("E_QOVR");

	case E_RLWAI:
		return("E_RLWAI");
	case E_TMOUT:
		return("E_TMOUT");
	case E_DLT:
		return("E_DLT");
	case E_CLS:
		return("E_CLS");

	case E_WBLK:
		return("E_WBLK");
	case E_BOVR:
		return("E_BOVR");

	default:
		return("unknown error");
	}
}
