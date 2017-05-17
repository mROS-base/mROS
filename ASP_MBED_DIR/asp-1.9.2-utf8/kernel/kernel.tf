$ ======================================================================
$ 
$   TOPPERS/ASP Kernel
$       Toyohashi Open Platform for Embedded Real-Time Systems/
$       Advanced Standard Profile Kernel
$ 
$   Copyright (C) 2007 by TAKAGI Nobuhisa
$   Copyright (C) 2007-2014 by Embedded and Real-Time Systems Laboratory
$               Graduate School of Information Science, Nagoya Univ., JAPAN
$  
$   上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
$   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
$   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
$   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
$       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
$       スコード中に含まれていること．
$   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
$       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
$       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
$       の無保証規定を掲載すること．
$   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
$       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
$       と．
$     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
$         作権表示，この利用条件および下記の無保証規定を掲載すること．
$     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
$         報告すること．
$   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
$       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
$       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
$       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
$       免責すること．
$  
$   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
$   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
$   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
$   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
$   の責任を負わない．
$ 
$   $Id: kernel.tf 2728 2015-12-30 01:46:11Z ertl-honda $
$  
$ =====================================================================

$ =====================================================================
$ AID_YYYの処理
$ =====================================================================

$num_atskid = 0$
$FOREACH i ATSK.ORDER_LIST$
$	// notskが負の場合（E_PAR）
	$IF ATSK.NOTSK[i] < 0$
		$ERROR ATSK.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "notsk", ATSK.NOTSK[i], "AID_TSK")$$END$
	$END$
	$num_atskid = num_atskid + ATSK.NOTSK[i]$
$END$
$num_tskid = LENGTH(TSK.ID_LIST) + num_atskid$

$num_asemid = 0$
$FOREACH i ASEM.ORDER_LIST$
$	// nosemが負の場合（E_PAR）
	$IF ASEM.NOSEM[i] < 0$
		$ERROR ASEM.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "nosem", ASEM.NOSEM[i], "AID_SEM")$$END$
	$END$
	$num_asemid = num_asemid + ASEM.NOSEM[i]$
$END$
$num_semid = LENGTH(SEM.ID_LIST) + num_asemid$

$num_aflgid = 0$
$FOREACH i AFLG.ORDER_LIST$
$	// noflgが負の場合（E_PAR）
	$IF AFLG.NOFLG[i] < 0$
		$ERROR AFLG.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "noflg", AFLG.NOFLG[i], "AID_FLG")$$END$
	$END$
	$num_aflgid = num_aflgid + AFLG.NOFLG[i]$
$END$
$num_flgid = LENGTH(FLG.ID_LIST) + num_aflgid$

$num_adtqid = 0$
$FOREACH i ADTQ.ORDER_LIST$
$	// nodtqが負の場合（E_PAR）
	$IF ADTQ.NODTQ[i] < 0$
		$ERROR ADTQ.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "nodtq", ADTQ.NODTQ[i], "AID_DTQ")$$END$
	$END$
	$num_adtqid = num_adtqid + ADTQ.NODTQ[i]$
$END$
$num_dtqid = LENGTH(DTQ.ID_LIST) + num_adtqid$

$num_apdqid = 0$
$FOREACH i APDQ.ORDER_LIST$
$	// nopdqが負の場合（E_PAR）
	$IF APDQ.NOPDQ[i] < 0$
		$ERROR APDQ.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "nopdq", APDQ.NOPDQ[i], "AID_PDQ")$$END$
	$END$
	$num_apdqid = num_apdqid + APDQ.NOPDQ[i]$
$END$
$num_pdqid = LENGTH(PDQ.ID_LIST) + num_apdqid$

$num_ambxid = 0$
$FOREACH i AMBX.ORDER_LIST$
$	// nombxが負の場合（E_PAR）
	$IF AMBX.NOMBX[i] < 0$
		$ERROR AMBX.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "nombx", AMBX.NOMBX[i], "AID_MBX")$$END$
	$END$
	$num_ambxid = num_ambxid + AMBX.NOMBX[i]$
$END$
$num_mbxid = LENGTH(MBX.ID_LIST) + num_ambxid$

$num_amtxid = 0$
$FOREACH i AMTX.ORDER_LIST$
$	// nomtxが負の場合（E_PAR）
	$IF AMTX.NOMTX[i] < 0$
		$ERROR AMTX.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "nomtx", AMTX.NOMTX[i], "AID_MTX")$$END$
	$END$
	$num_amtxid = num_amtxid + AMTX.NOMTX[i]$
$END$
$num_mtxid = LENGTH(MTX.ID_LIST) + num_amtxid$

$num_ampfid = 0$
$FOREACH i AMPF.ORDER_LIST$
$	// nompfが負の場合（E_PAR）
	$IF AMPF.NOMPF[i] < 0$
		$ERROR AMPF.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "nompf", AMPF.NOMPF[i], "AID_MPF")$$END$
	$END$
	$num_ampfid = num_ampfid + AMPF.NOMPF[i]$
$END$
$num_mpfid = LENGTH(MPF.ID_LIST) + num_ampfid$

$num_acycid = 0$
$FOREACH i ACYC.ORDER_LIST$
$	// nocycが負の場合（E_PAR）
	$IF ACYC.NOCYC[i] < 0$
		$ERROR ACYC.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "nocyc", ACYC.NOCYC[i], "AID_CYC")$$END$
	$END$
	$num_acycid = num_acycid + ACYC.NOCYC[i]$
$END$
$num_cycid = LENGTH(CYC.ID_LIST) + num_acycid$

$num_aalmid = 0$
$FOREACH i AALM.ORDER_LIST$
$	// noalmが負の場合（E_PAR）
	$IF AALM.NOALM[i] < 0$
		$ERROR AALM.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "noalm", AALM.NOALM[i], "AID_ALM")$$END$
	$END$
	$num_aalmid = num_aalmid + AALM.NOALM[i]$
$END$
$num_almid = LENGTH(ALM.ID_LIST) + num_aalmid$

$num_aisrid = 0$
$FOREACH i AISR.ORDER_LIST$
$	// noisrが負の場合（E_PAR）
	$IF AISR.NOISR[i] < 0$
		$ERROR AISR.TEXT_LINE[i]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "noisr", AISR.NOISR[i], "AID_ISR")$$END$
	$END$
	$num_aisrid = num_aisrid + AISR.NOISR[i]$
$END$
$num_isrid = num_aisrid$
$num_isr = LENGTH(ISR.ORDER_LIST) + num_aisrid$

$ =====================================================================
$ kernel_cfg.hの生成
$ =====================================================================

$FILE "kernel_cfg.h"$
/* kernel_cfg.h */$NL$
#ifndef TOPPERS_KERNEL_CFG_H$NL$
#define TOPPERS_KERNEL_CFG_H$NL$
$NL$
#define TNUM_TSKID	$num_tskid$$NL$
#define TNUM_SEMID	$num_semid$$NL$
#define TNUM_FLGID	$num_flgid$$NL$
#define TNUM_DTQID	$num_dtqid$$NL$
#define TNUM_PDQID	$num_pdqid$$NL$
#define TNUM_MBXID	$num_mbxid$$NL$
#define TNUM_MTXID	$num_mtxid$$NL$
#define TNUM_MPFID	$num_mpfid$$NL$
#define TNUM_CYCID	$num_cycid$$NL$
#define TNUM_ALMID	$num_almid$$NL$
#define TNUM_ISRID	$num_isrid$$NL$
$NL$
$FOREACH id TSK.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id SEM.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id FLG.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id DTQ.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id PDQ.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id MBX.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id MTX.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id MPF.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id CYC.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$FOREACH id ALM.ID_LIST$
	#define $id$	$+id$$NL$
$END$
$NL$
#endif /* TOPPERS_KERNEL_CFG_H */$NL$

$ =====================================================================
$ kernel_cfg.cの生成
$ =====================================================================

$FILE "kernel_cfg.c"$
/* kernel_cfg.c */$NL$
#include "kernel/kernel_int.h"$NL$
#include "kernel_cfg.h"$NL$
$NL$
#if TKERNEL_PRID != 0x07u$NL$
#error The kernel does not match this configuration file.$NL$
#endif$NL$
$NL$

$ 
$  インクルードディレクティブ（#include）
$ 
/*$NL$
$SPC$*  Include Directives (#include)$NL$
$SPC$*/$NL$
$NL$
$INCLUDES$
$NL$

$ 
$  オブジェクトのID番号を保持する変数
$ 
$IF USE_EXTERNAL_ID$
	/*$NL$
	$SPC$*  Variables for Object ID$NL$
	$SPC$*/$NL$
	$NL$
	$FOREACH id TSK.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id SEM.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id FLG.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id DTQ.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id PDQ.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id MBX.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id MTX.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$	
	$FOREACH id MPF.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id CYC.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
	$FOREACH id ALM.ID_LIST$
		const ID $id$_id$SPC$=$SPC$$+id$;$NL$
	$END$
$END$

$ 
$  タスク
$ 
/*$NL$
$SPC$*  Task Management Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成されたタスクが1個以上存在することのチェック
$IF !LENGTH(TSK.ID_LIST)$
	$ERROR$$FORMAT(_("no task is registered"))$$END$
$END$

$ 静的に生成されたタスクの数
#define TNUM_STSKID	$LENGTH(TSK.ID_LIST)$$NL$
$NL$

$ タスクID番号の最大値
const ID _kernel_tmax_tskid = (TMIN_TSKID + TNUM_TSKID - 1);$NL$
const ID _kernel_tmax_stskid = (TMIN_TSKID + TNUM_STSKID - 1);$NL$
$NL$

$ エラーチェック
$FOREACH tskid TSK.ID_LIST$
$	// tskatrが（［TA_ACT］）でない場合（E_RSATR）
	$IF (TSK.TSKATR[tskid] & ~(TA_ACT|TARGET_TSKATR)) != 0$
		$ERROR TSK.TEXT_LINE[tskid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "tskatr", TSK.TSKATR[tskid], tskid, "CRE_TSK")$$END$
	$END$

$	// (TMIN_TPRI <= itskpri && itskpri <= TMAX_TPRI)でない場合（E_PAR）
	$IF !(TMIN_TPRI <= TSK.ITSKPRI[tskid] && TSK.ITSKPRI[tskid] <= TMAX_TPRI)$
		$ERROR TSK.TEXT_LINE[tskid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "itskpri", TSK.ITSKPRI[tskid], tskid, "CRE_TSK")$$END$
	$END$

$ 	// texatrが（TA_NULL）でない場合（E_RSATR）
	$IF LENGTH(TSK.TEXATR[tskid]) && TSK.TEXATR[tskid] != 0$
		$ERROR DEF_TEX.TEXT_LINE[tskid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "texatr", TSK.TEXATR[tskid], tskid, "DEF_TEX")$$END$
	$END$
$END$

$ スタック領域の生成とそれに関するエラーチェック
$FOREACH tskid TSK.ID_LIST$
$	// stkszが0以下か，ターゲット定義の最小値（TARGET_MIN_STKSZ）よりも
$	// 小さい場合（E_PAR）
	$IF TSK.STKSZ[tskid] <= 0 || (TARGET_MIN_STKSZ
									&& TSK.STKSZ[tskid] < TARGET_MIN_STKSZ)$
		$ERROR TSK.TEXT_LINE[tskid]$E_PAR: $FORMAT(_("%1% `%2%\' of `%3%\' in %4% is too small"), "stksz", TSK.STKSZ[tskid], tskid, "CRE_TSK")$$END$
	$END$

$ 	// stkszがスタック領域のサイズとして正しくない場合（E_PAR）
	$IF !EQ(TSK.STK[tskid], "NULL") && CHECK_STKSZ_ALIGN
							&& (TSK.STKSZ[tskid] & (CHECK_STKSZ_ALIGN - 1))$
		$ERROR TSK.TEXT_LINE[tskid]$E_PAR: $FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"), "stksz", TSK.STKSZ[tskid], tskid, "CRE_TSK")$$END$
	$END$

	$IF EQ(TSK.STK[tskid],"NULL")$
		static STK_T _kernel_stack_$tskid$[COUNT_STK_T($TSK.STKSZ[tskid]$)];$NL$
		$TSK.TINIB_STKSZ[tskid] = FORMAT("ROUND_STK_T(%1%)", TSK.STKSZ[tskid])$
		$TSK.TINIB_STK[tskid] = CONCAT("_kernel_stack_", tskid)$
	$ELSE$
		$TSK.TINIB_STKSZ[tskid] = TSK.STKSZ[tskid]$
		$TSK.TINIB_STK[tskid] = FORMAT("(void *)(%1%)", TSK.STK[tskid])$
	$END$
$END$
$NL$

$ タスク初期化ブロックの生成（タスクは1個以上存在する）
const TINIB _kernel_tinib_table[TNUM_STSKID] = {$NL$
$JOINEACH tskid TSK.ID_LIST ",\n"$
$	// タスク属性，拡張情報，起動番地，起動時優先度
	$TAB${
	$SPC$($TSK.TSKATR[tskid]$), (intptr_t)($TSK.EXINF[tskid]$),
	$SPC$((TASK)($TSK.TASK[tskid]$)), INT_PRIORITY($TSK.ITSKPRI[tskid]$),

$	// タスク初期化コンテキストブロック，スタック領域
	$IF USE_TSKINICTXB$
		$GENERATE_TSKINICTXB(tskid)$
	$ELSE$
		$SPC$$TSK.TINIB_STKSZ[tskid]$, $TSK.TINIB_STK[tskid]$,
	$END$

$	// タスク例外処理ルーチンの属性と起動番地
	$SPC$($ALT(TSK.TEXATR[tskid],"TA_NULL")$), ($ALT(TSK.TEXRTN[tskid],"NULL")$) }
$END$$NL$
};$NL$
$NL$

$ 動的生成タスク用のタスク初期化ブロックの生成
$IF num_atskid > 0$
	TINIB _kernel_atinib_table[$num_atskid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(TINIB, _kernel_atinib_table);$NL$
$END$$NL$

$ タスク管理ブロックの生成
TCB _kernel_tcb_table[TNUM_TSKID];$NL$
$NL$

$ タスク生成順序テーブルの生成
const ID _kernel_torder_table[TNUM_STSKID] = {$NL$
$TAB$$JOINEACH tskid TSK.ORDER_LIST ", "$$tskid$$END$$NL$
};$NL$
$NL$

$ 
$  セマフォ
$ 
/*$NL$
$SPC$*  Semaphore Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成されたセマフォの数
#define TNUM_SSEMID	$LENGTH(SEM.ID_LIST)$$NL$
$NL$

$ セマフォID番号の最大値
const ID _kernel_tmax_semid = (TMIN_SEMID + TNUM_SEMID - 1);$NL$
const ID _kernel_tmax_ssemid = (TMIN_SEMID + TNUM_SSEMID - 1);$NL$
$NL$

$ セマフォ初期化ブロックの生成
$IF LENGTH(SEM.ID_LIST)$
	const SEMINIB _kernel_seminib_table[TNUM_SSEMID] = {$NL$
	$JOINEACH semid SEM.ID_LIST ",\n"$
$		// sematrが（［TA_TPRI］）でない場合（E_RSATR）
		$IF (SEM.SEMATR[semid] & ~TA_TPRI) != 0$
			$ERROR SEM.TEXT_LINE[semid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "sematr", SEM.SEMATR[semid], semid, "CRE_SEM")$$END$
		$END$

$		// (0 <= isemcnt && isemcnt <= maxsem)でない場合（E_PAR）
		$IF !(0 <= SEM.ISEMCNT[semid] && SEM.ISEMCNT[semid] <= SEM.MAXSEM[semid])$
			$ERROR SEM.TEXT_LINE[semid]$E_PAR: $FORMAT(_("too large %1% `%2%\' of `%3%\' in %4%"), "isemcnt", SEM.ISEMCNT[semid], semid, "CRE_SEM")$$END$
		$END$

$		// (1 <= maxsem && maxsem <= TMAX_MAXSEM)でない場合（E_PAR）
		$IF !(1 <= SEM.MAXSEM[semid] && SEM.MAXSEM[semid] <= TMAX_MAXSEM)$
			$ERROR SEM.TEXT_LINE[semid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "maxsem", SEM.MAXSEM[semid], semid, "CRE_SEM")$$END$
		$END$

$		// セマフォ初期化ブロック
		$TAB${ ($SEM.SEMATR[semid]$), ($SEM.ISEMCNT[semid]$), ($SEM.MAXSEM[semid]$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const SEMINIB, _kernel_seminib_table);$NL$
$END$$NL$

$ 動的生成セマフォ用のセマフォ初期化ブロックの生成
$IF num_asemid > 0$
	SEMINIB _kernel_aseminib_table[$num_asemid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(SEMINIB, _kernel_aseminib_table);$NL$
$END$$NL$

$ セマフォ管理ブロックの生成
$IF num_semid > 0$
	SEMCB _kernel_semcb_table[TNUM_SEMID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(SEMCB, _kernel_semcb_table);$NL$
$END$$NL$

$ 
$  イベントフラグ
$ 
/*$NL$
$SPC$*  Eventflag Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成されたイベントフラグの数
#define TNUM_SFLGID	$LENGTH(FLG.ID_LIST)$$NL$
$NL$

$ イベントフラグID番号の最大値
const ID _kernel_tmax_flgid = (TMIN_FLGID + TNUM_FLGID - 1);$NL$
const ID _kernel_tmax_sflgid = (TMIN_FLGID + TNUM_SFLGID - 1);$NL$
$NL$

$ イベントフラグ初期化ブロックの生成
$IF LENGTH(FLG.ID_LIST)$
	const FLGINIB _kernel_flginib_table[TNUM_SFLGID] = {$NL$
	$JOINEACH flgid FLG.ID_LIST ",\n"$
$		// flgatrが（［TA_TPRI］｜［TA_WMUL］｜［TA_CLR］）でない場合（E_RSATR）
		$IF (FLG.FLGATR[flgid] & ~(TA_TPRI|TA_WMUL|TA_CLR)) != 0$
			$ERROR FLG.TEXT_LINE[flgid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "flgatr", FLG.FLGATR[flgid], flgid, "CRE_FLG")$$END$
		$END$

$		// iflgptnがFLGPTNに格納できない場合（E_PAR）
		$IF (FLG.IFLGPTN[flgid] & ~((1 << TBIT_FLGPTN) - 1)) != 0$
			$ERROR FLG.TEXT_LINE[flgid]$E_PAR: $FORMAT(_("too large %1% `%2%\' of `%3%\' in %4%"), "iflgptn", FLG.IFLGPTN[flgid], flgid, "CRE_FLG")$$END$
		$END$

$		// イベントフラグ初期化ブロック
		$TAB${ ($FLG.FLGATR[flgid]$), ($FLG.IFLGPTN[flgid]$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const FLGINIB, _kernel_flginib_table);$NL$
$END$$NL$

$ 動的生成イベントフラグ用のイベントフラグ初期化ブロックの生成
$IF num_aflgid > 0$
	FLGINIB _kernel_aflginib_table[$num_aflgid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(FLGINIB, _kernel_aflginib_table);$NL$
$END$$NL$

$ イベントフラグ管理ブロックの生成
$IF num_flgid > 0$
	FLGCB _kernel_flgcb_table[TNUM_FLGID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(FLGCB, _kernel_flgcb_table);$NL$
$END$$NL$

$ 
$  データキュー
$ 
/*$NL$
$SPC$*  Dataqueue Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成されたデータキューの数
#define TNUM_SDTQID	$LENGTH(DTQ.ID_LIST)$$NL$
$NL$

$ データキューID番号の最大値
const ID _kernel_tmax_dtqid = (TMIN_DTQID + TNUM_DTQID - 1);$NL$
const ID _kernel_tmax_sdtqid = (TMIN_DTQID + TNUM_SDTQID - 1);$NL$
$NL$

$IF LENGTH(DTQ.ID_LIST)$
	$FOREACH dtqid DTQ.ID_LIST$
$		// dtqatrが（［TA_TPRI］）でない場合（E_RSATR）
		$IF (DTQ.DTQATR[dtqid] & ~TA_TPRI) != 0$
			$ERROR DTQ.TEXT_LINE[dtqid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "dtqatr", DTQ.DTQATR[dtqid], dtqid, "CRE_DTQ")$$END$
		$END$

$		// dtqcntが負の場合（E_PAR）
		$IF DTQ.DTQCNT[dtqid] < 0$
			$ERROR DTQ.TEXT_LINE[dtqid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "dtqcnt", DTQ.DTQCNT[dtqid], dtqid, "CRE_DTQ")$$END$
		$END$

$		// dtqmbがNULLでない場合（E_NOSPT）
		$IF !EQ(DTQ.DTQMB[dtqid], "NULL")$
			$ERROR DTQ.TEXT_LINE[dtqid]$E_NOSPT: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "dtqmb", DTQ.DTQMB[dtqid], dtqid, "CRE_DTQ")$$END$
		$END$

$		// データキュー管理領域
		$IF DTQ.DTQCNT[dtqid]$
			static DTQMB _kernel_dtqmb_$dtqid$[$DTQ.DTQCNT[dtqid]$];$NL$
		$END$
	$END$

$	// データキュー初期化ブロックの生成
	const DTQINIB _kernel_dtqinib_table[TNUM_SDTQID] = {$NL$
	$JOINEACH dtqid DTQ.ID_LIST ",\n"$
		$TAB${ ($DTQ.DTQATR[dtqid]$), ($DTQ.DTQCNT[dtqid]$), $IF DTQ.DTQCNT[dtqid]$(_kernel_dtqmb_$dtqid$)$ELSE$NULL$END$ }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const DTQINIB, _kernel_dtqinib_table);$NL$
$END$$NL$

$ 動的生成データキュー用のデータキュー初期化ブロックの生成
$IF num_adtqid > 0$
	DTQINIB _kernel_adtqinib_table[$num_adtqid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(DTQINIB, _kernel_adtqinib_table);$NL$
$END$$NL$

$ データキュー管理ブロックの生成
$IF num_dtqid > 0$
	DTQCB _kernel_dtqcb_table[TNUM_DTQID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(DTQCB, _kernel_dtqcb_table);$NL$
$END$$NL$

$ 
$  優先度データキュー
$ 
/*$NL$
$SPC$*  Priority Dataqueue Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成された優先度データキューの数
#define TNUM_SPDQID	$LENGTH(PDQ.ID_LIST)$$NL$
$NL$

$ 優先度データキューID番号の最大値
const ID _kernel_tmax_pdqid = (TMIN_PDQID + TNUM_PDQID - 1);$NL$
const ID _kernel_tmax_spdqid = (TMIN_PDQID + TNUM_SPDQID - 1);$NL$
$NL$

$IF LENGTH(PDQ.ID_LIST)$
	$FOREACH pdqid PDQ.ID_LIST$
$		// pdqatrが（［TA_TPRI］）でない場合（E_RSATR）
		$IF (PDQ.PDQATR[pdqid] & ~TA_TPRI) != 0$
			$ERROR PDQ.TEXT_LINE[pdqid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "pdqatr", PDQ.PDQATR[pdqid], pdqid, "CRE_PDQ")$$END$
		$END$

$		// pdqcntが負の場合（E_PAR）
		$IF PDQ.PDQCNT[pdqid] < 0$
			$ERROR PDQ.TEXT_LINE[pdqid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "pdqcnt", PDQ.PDQCNT[pdqid], pdqid, "CRE_PDQ")$$END$
		$END$

$		// (TMIN_DPRI <= maxdpri && maxdpri <= TMAX_DPRI)でない場合（E_PAR）
		$IF !(TMIN_DPRI <= PDQ.MAXDPRI[pdqid] && PDQ.MAXDPRI[pdqid] <= TMAX_DPRI)$
			$ERROR PDQ.TEXT_LINE[pdqid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "maxdpri", PDQ.MAXDPRI[pdqid], pdqid, "CRE_PDQ")$$END$
		$END$

$		// pdqmbがNULLでない場合（E_NOSPT）
		$IF !EQ(PDQ.PDQMB[pdqid], "NULL")$
			$ERROR PDQ.TEXT_LINE[pdqid]$E_NOSPT: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "pdqmb", PDQ.PDQMB[pdqid], pdqid, "CRE_PDQ")$$END$
		$END$

$		// 優先度データキュー管理領域
		$IF PDQ.PDQCNT[pdqid]$
			static PDQMB _kernel_pdqmb_$pdqid$[$PDQ.PDQCNT[pdqid]$];$NL$
		$END$
	$END$

$	// 優先度データキュー初期化ブロックの生成
	const PDQINIB _kernel_pdqinib_table[TNUM_SPDQID] = {$NL$
	$JOINEACH pdqid PDQ.ID_LIST ",\n"$
		$TAB${ ($PDQ.PDQATR[pdqid]$), ($PDQ.PDQCNT[pdqid]$), ($PDQ.MAXDPRI[pdqid]$), $IF PDQ.PDQCNT[pdqid]$(_kernel_pdqmb_$pdqid$)$ELSE$NULL$END$ }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const PDQINIB, _kernel_pdqinib_table);$NL$
$END$$NL$

$ 動的生成優先度データキュー用の優先度データキュー初期化ブロックの生成
$IF num_apdqid > 0$
	PDQINIB _kernel_apdqinib_table[$num_apdqid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(PDQINIB, _kernel_apdqinib_table);$NL$
$END$$NL$

$ 優先度データキュー管理ブロックの生成
$IF num_pdqid > 0$
	PDQCB _kernel_pdqcb_table[TNUM_PDQID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(PDQCB, _kernel_pdqcb_table);$NL$
$END$$NL$

$ 
$  メールボックス
$ 
/*$NL$
$SPC$*  Mailbox Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成されたメールボックスの数
#define TNUM_SMBXID	$LENGTH(MBX.ID_LIST)$$NL$
$NL$

$ メールボックスID番号の最大値
const ID _kernel_tmax_mbxid = (TMIN_MBXID + TNUM_MBXID - 1);$NL$
const ID _kernel_tmax_smbxid = (TMIN_MBXID + TNUM_SMBXID - 1);$NL$
$NL$

$ メールボックス初期化ブロックの生成
$IF LENGTH(MBX.ID_LIST)$
	const MBXINIB _kernel_mbxinib_table[TNUM_SMBXID] = {$NL$
	$JOINEACH mbxid MBX.ID_LIST ",\n"$
$		// mbxatrが（［TA_TPRI］｜［TA_MPRI］）でない場合（E_RSATR）
		$IF (MBX.MBXATR[mbxid] & ~(TA_TPRI|TA_MPRI)) != 0$
			$ERROR MBX.TEXT_LINE[mbxid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "mbxatr", MBX.MBXATR[mbxid], mbxid, "CRE_MBX")$$END$
		$END$

$		// (TMIN_MPRI <= maxmpri && maxmpri <= TMAX_MPRI)でない場合（E_PAR）
		$IF !(TMIN_MPRI <= MBX.MAXMPRI[mbxid] && MBX.MAXMPRI[mbxid] <= TMAX_MPRI)$
			$ERROR MBX.TEXT_LINE[mbxid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "maxmpri", MBX.MAXMPRI[mbxid], mbxid, "CRE_MBX")$$END$
		$END$

$		// mprihdがNULLでない場合（E_NOSPT）
		$IF !EQ(MBX.MPRIHD[mbxid], "NULL")$
			$ERROR MBX.TEXT_LINE[mbxid]$E_NOSPT: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "mprihd", MBX.MPRIHD[mbxid], mbxid, "CRE_MBX")$$END$
		$END$

$		// メールボックス初期化ブロック
		$TAB${ ($MBX.MBXATR[mbxid]$), ($MBX.MAXMPRI[mbxid]$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const MBXINIB, _kernel_mbxinib_table);$NL$
$END$$NL$

$ 動的生成メールボックス用のメールボックス初期化ブロックの生成
$IF num_ambxid > 0$
	MBXINIB _kernel_ambxinib_table[$num_ambxid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(MBXINIB, _kernel_ambxinib_table);$NL$
$END$$NL$

$ メールボックス管理ブロックの生成
$IF num_mbxid > 0$
	MBXCB _kernel_mbxcb_table[TNUM_MBXID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(MBXCB, _kernel_mbxcb_table);$NL$
$END$$NL$

$ 
$  ミューテックス
$ 
/*$NL$
$SPC$*  Mutex Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成されたミューテックスの数
#define TNUM_SMTXID	$LENGTH(MTX.ID_LIST)$$NL$
$NL$

$ ミューテックスID番号の最大値
const ID _kernel_tmax_mtxid = (TMIN_MTXID + TNUM_MTXID - 1);$NL$
const ID _kernel_tmax_smtxid = (TMIN_MTXID + TNUM_SMTXID - 1);$NL$
$NL$

$ ミューテックス初期化ブロックの生成
$IF LENGTH(MTX.ID_LIST)$
	const MTXINIB _kernel_mtxinib_table[TNUM_MTXID] = {$NL$
	$JOINEACH mtxid MTX.ID_LIST ",\n"$
$		// mtxatrが（［TA_TPRI｜TA_CEILING］）でない場合（E_RSATR）
		$IF !(MTX.MTXATR[mtxid] == 0 || MTX.MTXATR[mtxid] == TA_TPRI || MTX.MTXATR[mtxid] == TA_CEILING)$
			$ERROR MTX.TEXT_LINE[mtxid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "mtxatr", MTX.MTXATR[mtxid], mtxid, "CRE_MTX")$$END$
		$END$

$		// ceilpriが未指定の場合は0と見なす
		$IF !LENGTH(MTX.CEILPRI[mtxid])$
			$MTX.CEILPRI[mtxid] = 0$
		$END$
$		// (TMIN_TPRI <= ceilpri && ceilpri <= TMAX_TPRI)でない場合（E_PAR）
		$IF MTX.MTXATR[mtxid] == TA_CEILING && (MTX.CEILPRI[mtxid] < TMIN_TPRI || TMAX_TPRI < MTX.CEILPRI[mtxid])$
			$ERROR MTX.TEXT_LINE[mtxid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "ceilpri", MTX.CEILPRI[mtxid], mtxid, "CRE_MTX")$$END$
		$END$

$		// ミューテックス初期化ブロック
		$TAB${ ($MTX.MTXATR[mtxid]$), INT_PRIORITY($MTX.CEILPRI[mtxid]$) }
	$END$$NL$
	};$NL$
	$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const MTXINIB, _kernel_mtxinib_table);$NL$
$END$$NL$

$ 動的生成ミューテックス用のミューテックス初期化ブロックの生成
$IF num_amtxid > 0$
	MTXINIB _kernel_amtxinib_table[$num_amtxid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(MTXINIB, _kernel_amtxinib_table);$NL$
$END$$NL$

$ ミューテックス管理ブロックの生成
$IF num_mtxid > 0$
	MTXCB _kernel_mtxcb_table[TNUM_MTXID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(MTXCB, _kernel_mtxcb_table);$NL$
$END$$NL$

$ 
$  固定長メモリプール
$ 
/*$NL$
$SPC$*  Fixed-sized Memorypool Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成された固定長メモリプールの数
#define TNUM_SMPFID	$LENGTH(MPF.ID_LIST)$$NL$
$NL$

$ 固定長メモリプールID番号の最大値
const ID _kernel_tmax_mpfid = (TMIN_MPFID + TNUM_MPFID - 1);$NL$
const ID _kernel_tmax_smpfid = (TMIN_MPFID + TNUM_SMPFID - 1);$NL$
$NL$

$IF LENGTH(MPF.ID_LIST)$
	$FOREACH mpfid MPF.ID_LIST$
$		// mpfatrが（［TA_TPRI］）でない場合（E_RSATR）
		$IF (MPF.MPFATR[mpfid] & ~TA_TPRI) != 0$
			$ERROR MPF.TEXT_LINE[mpfid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "mpfatr", MPF.MPFATR[mpfid], mpfid, "CRE_MPF")$$END$
		$END$

$		// blkcntが0以下の場合（E_PAR）
		$IF MPF.BLKCNT[mpfid] <= 0$
			$ERROR MPF.TEXT_LINE[mpfid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "blkcnt", MPF.BLKCNT[mpfid], mpfid, "CRE_MPF")$$END$
		$END$

$		// blkszが0以下の場合（E_PAR）
		$IF MPF.BLKSZ[mpfid] <= 0$
			$ERROR MPF.TEXT_LINE[mpfid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "blksz", MPF.BLKSZ[mpfid], mpfid, "CRE_MPF")$$END$
		$END$

$		// 固定長メモリプール領域
		$IF EQ(MPF.MPF[mpfid], "NULL")$
			static MPF_T _kernel_mpf_$mpfid$[($MPF.BLKCNT[mpfid]$) * COUNT_MPF_T($MPF.BLKSZ[mpfid]$)];$NL$
		$END$

$		// mpfmbがNULLでない場合（E_NOSPT）
		$IF !EQ(MPF.MPFMB[mpfid], "NULL")$
			$ERROR MPF.TEXT_LINE[mpfid]$E_NOSPT: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "mpfmb", MPF.MPFMB[mpfid], mpfid, "CRE_MPF")$$END$
		$END$

$		// 固定長メモリプール管理領域
		static MPFMB _kernel_mpfmb_$mpfid$[$MPF.BLKCNT[mpfid]$];$NL$
	$END$

$	// 固定長メモリプール初期化ブロックの生成
	const MPFINIB _kernel_mpfinib_table[TNUM_SMPFID] = {$NL$
	$JOINEACH mpfid MPF.ID_LIST ",\n"$
		$TAB${ ($MPF.MPFATR[mpfid]$), ($MPF.BLKCNT[mpfid]$), ROUND_MPF_T($MPF.BLKSZ[mpfid]$), $IF EQ(MPF.MPF[mpfid],"NULL")$(_kernel_mpf_$mpfid$)$ELSE$(void *)($MPF.MPF[mpfid]$)$END$, (_kernel_mpfmb_$mpfid$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const MPFINIB, _kernel_mpfinib_table);$NL$
$END$$NL$

$ 動的生成固定長メモリプール用の固定長メモリプール初期化ブロックの生成
$IF num_ampfid > 0$
	MPFINIB _kernel_ampfinib_table[$num_ampfid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(MPFINIB, _kernel_ampfinib_table);$NL$
$END$$NL$

$ 固定長メモリプール管理ブロックの生成
$IF num_mpfid > 0$
	MPFCB _kernel_mpfcb_table[TNUM_MPFID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(MPFCB, _kernel_mpfcb_table);$NL$
$END$$NL$

$ 
$  周期ハンドラ
$ 
/*$NL$
$SPC$*  Cyclic Handler Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成された周期ハンドラの数
#define TNUM_SCYCID	$LENGTH(CYC.ID_LIST)$$NL$
$NL$

$ 周期ハンドラID番号の最大値
const ID _kernel_tmax_cycid = (TMIN_CYCID + TNUM_CYCID - 1);$NL$
const ID _kernel_tmax_scycid = (TMIN_CYCID + TNUM_SCYCID - 1);$NL$
$NL$

$ 周期ハンドラ初期化テーブルの生成
$IF LENGTH(CYC.ID_LIST)$
	const CYCINIB _kernel_cycinib_table[TNUM_SCYCID] = {$NL$
	$JOINEACH cycid CYC.ID_LIST ",\n"$
$		// cycatrが（［TA_STA］）でない場合（E_RSATR）
		$IF (CYC.CYCATR[cycid] & ~TA_STA) != 0$
			$ERROR CYC.TEXT_LINE[cycid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "cycatr", CYC.CYCATR[cycid], cycid, "CRE_CYC")$$END$
		$END$

$		// (0 < cyctim && cyctim <= TMAX_RELTIM)でない場合（E_PAR）
		$IF !(0 < CYC.CYCTIM[cycid] && CYC.CYCTIM[cycid] <= TMAX_RELTIM)$
			$ERROR CYC.TEXT_LINE[cycid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "cyctim", CYC.CYCTIM[cycid], cycid, "CRE_CYC")$$END$
		$END$

$		// (0 <= cycphs && cycphs <= TMAX_RELTIM)でない場合（E_PAR）
		$IF !(0 <= CYC.CYCPHS[cycid] && CYC.CYCPHS[cycid] <= TMAX_RELTIM)$
			$ERROR CYC.TEXT_LINE[cycid]$E_PAR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "cycphs", CYC.CYCPHS[cycid], cycid, "CRE_CYC")$$END$
		$END$

$		// 警告：cycatrにTA_STAが設定されていて，(cycphs == 0)の場合
		$IF (CYC.CYCATR[cycid] & TA_STA) != 0 && CYC.CYCPHS[cycid] == 0$
			$WARNING CYC.TEXT_LINE[cycid]$$FORMAT(_("%1% is not recommended when %2% is set to %3% in %4%"), "cycphs==0", "TA_STA", "cycatr", "CRE_CYC")$$END$
		$END$

$		// 周期ハンドラ初期化ブロック
		$TAB${ ($CYC.CYCATR[cycid]$), (intptr_t)($CYC.EXINF[cycid]$), ($CYC.CYCHDR[cycid]$), ($CYC.CYCTIM[cycid]$), ($CYC.CYCPHS[cycid]$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const CYCINIB, _kernel_cycinib_table);$NL$
$END$$NL$

$ 動的生成周期ハンドラ用の周期ハンドラ初期化ブロックの生成
$IF num_acycid > 0$
	CYCINIB _kernel_acycinib_table[$num_acycid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(CYCINIB, _kernel_acycinib_table);$NL$
$END$$NL$

$ 周期ハンドラ管理ブロックの生成
$IF num_cycid > 0$
	CYCCB _kernel_cyccb_table[TNUM_CYCID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(CYCCB, _kernel_cyccb_table);$NL$
$END$$NL$

$ 
$  アラームハンドラ
$ 
/*$NL$
$SPC$*  Alarm Handler Functions$NL$
$SPC$*/$NL$
$NL$

$ 静的に生成されたアラームハンドラの数
#define TNUM_SALMID	$LENGTH(ALM.ID_LIST)$$NL$
$NL$

$ アラームハンドラID番号の最大値
const ID _kernel_tmax_almid = (TMIN_ALMID + TNUM_ALMID - 1);$NL$
const ID _kernel_tmax_salmid = (TMIN_ALMID + TNUM_SALMID - 1);$NL$
$NL$

$ アラームハンドラ初期化ブロックの生成
$IF LENGTH(ALM.ID_LIST)$
	const ALMINIB _kernel_alminib_table[TNUM_SALMID] = {$NL$
	$JOINEACH almid ALM.ID_LIST ",\n"$
$		// almatrが（TA_NULL）でない場合（E_RSATR）
		$IF ALM.ALMATR[almid] != 0$
			$ERROR ALM.TEXT_LINE[almid]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of `%3%\' in %4%"), "almatr", ALM.ALMATR[almid], almid, "CRE_ALM")$$END$
		$END$

$		// アラームハンドラ初期化ブロック
		$TAB${ ($ALM.ALMATR[almid]$), (intptr_t)($ALM.EXINF[almid]$), ($ALM.ALMHDR[almid]$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const ALMINIB, _kernel_alminib_table);$NL$
$END$$NL$

$ 動的生成アラームハンドラ用のアラームハンドラ初期化ブロックの生成
$IF num_aalmid > 0$
	ALMINIB _kernel_aalminib_table[$num_aalmid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(ALMINIB, _kernel_aalminib_table);$NL$
$END$$NL$

$ アラームハンドラ管理ブロックの生成
$IF num_almid > 0$
	ALMCB _kernel_almcb_table[TNUM_ALMID];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(ALMCB, _kernel_almcb_table);$NL$
$END$$NL$

$ 
$  割込み管理機能
$ 
/*$NL$
$SPC$*  Interrupt Management Functions$NL$
$SPC$*/$NL$
$NL$

$ 割込み番号と割込みハンドラ番号の変換テーブルの作成
$IF LENGTH(INTNO_ATTISR_VALID) != LENGTH(INHNO_ATTISR_VALID)$
	$ERROR$length of `INTNO_ATTISR_VALID' is different from length of `INHNO_ATTISR_VALID'$END$
$END$
$i = 0$
$FOREACH intno INTNO_ATTISR_VALID$
	$inhno = AT(INHNO_ATTISR_VALID, i)$
	$INHNO[intno] = inhno$
	$INTNO[inhno] = intno$
	$i = i + 1$
$END$

$ 割込み要求ラインに関するエラーチェック
$i = 0$
$FOREACH intno INT.ORDER_LIST$
$	// intnoがCFG_INTに対する割込み番号として正しくない場合（E_PAR）
	$IF !LENGTH(FIND(INTNO_CFGINT_VALID, INT.INTNO[intno]))$
		$ERROR INT.TEXT_LINE[intno]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "intno", INT.INTNO[intno], "CFG_INT")$$END$
	$END$

$	// intnoがCFG_INTによって設定済みの場合（E_OBJ）
	$j = 0$
	$FOREACH intno2 INT.ORDER_LIST$
		$IF INT.INTNO[intno] == INT.INTNO[intno2] && j < i$
			$ERROR INT.TEXT_LINE[intno]$E_OBJ: $FORMAT(_("%1% `%2%\' in %3% is duplicated"), "intno", INT.INTNO[intno], "CFG_INT")$$END$
		$END$
		$j = j + 1$
	$END$

$	// intatrが（［TA_ENAINT］｜［TA_EDGE］）でない場合（E_RSATR）
	$IF (INT.INTATR[intno] & ~(TA_ENAINT|TA_EDGE|TARGET_INTATR)) != 0$
		$ERROR INT.TEXT_LINE[intno]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of %3% `%4%\' in %5%"), "intatr", INT.INTATR[intno], "intno", INT.INTNO[intno], "CFG_INT")$$END$
	$END$

$	// intpriがCFG_INTに対する割込み優先度として正しくない場合（E_PAR）
	$IF !LENGTH(FIND(INTPRI_CFGINT_VALID, INT.INTPRI[intno]))$
		$ERROR INT.TEXT_LINE[intno]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "intpri", INT.INTPRI[intno], "CFG_INT")$$END$
	$END$

$	// カーネル管理に固定されているintnoに対して，intpriにTMIN_INTPRI
$	// よりも小さい値が指定された場合（E_OBJ）
	$IF LENGTH(FIND(INTNO_FIX_KERNEL, intno))$
		$IF INT.INTPRI[intno] < TMIN_INTPRI$
			$ERROR INT.TEXT_LINE[intno]$E_OBJ: $FORMAT(_("%1% `%2%\' must not have higher priority than %3%"), "intno", INT.INTNO[intno], "TMIN_INTPRI")$$END$
		$END$
	$END$

$	// カーネル管理外に固定されているintnoに対して，intpriにTMIN_INTPRI
$	// よりも小さい値が指定されなかった場合（E_OBJ）
	$IF LENGTH(FIND(INTNO_FIX_NONKERNEL, intno))$
		$IF INT.INTPRI[intno] >= TMIN_INTPRI$
			$ERROR INT.TEXT_LINE[intno]$E_OBJ: $FORMAT(_("%1% `%2%\' must have higher priority than %3%"), "intno", INT.INTNO[intno], "TMIN_INTPRI")$$END$
		$END$
	$END$
	$i = i + 1$
$END$

$ 割込みハンドラに関するエラーチェック
$i = 0$
$FOREACH inhno INH.ORDER_LIST$
$	// inhnoがDEF_INHに対する割込みハンドラ番号として正しくない場合（E_PAR）
	$IF !LENGTH(FIND(INHNO_DEFINH_VALID, INH.INHNO[inhno]))$
		$ERROR INH.TEXT_LINE[inhno]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "inhno", INH.INHNO[inhno], "DEF_INH")$$END$
	$END$

$	// inhnoがDEF_INHによって設定済みの場合（E_OBJ）
	$j = 0$
	$FOREACH inhno2 INH.ORDER_LIST$
		$IF INH.INHNO[inhno] == INH.INHNO[inhno2] && j < i$
			$ERROR INH.TEXT_LINE[inhno]$E_OBJ: $FORMAT(_("%1% `%2%\' in %3% is duplicated"), "inhno", INH.INHNO[inhno], "DEF_INH")$$END$
		$END$
		$j = j + 1$
	$END$

$	// inhatrが（TA_NULL）でない場合（E_RSATR）
	$IF (INH.INHATR[inhno] & ~TARGET_INHATR) != 0$
		$ERROR INH.TEXT_LINE[inhno]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of %3% `%4%\' in %5%"), "inhatr", INH.INHATR[inhno], "inhno", INH.INHNO[inhno], "DEF_INH")$$END$
	$END$

$	// カーネル管理に固定されているinhnoに対して，inhatrにTA_NONKERNEL
$	//　が指定されている場合（E_RSATR）
	$IF LENGTH(FIND(INHNO_FIX_KERNEL, inhno))$
		$IF (INH.INHATR[inhno] & TA_NONKERNEL) != 0$
			$ERROR INH.TEXT_LINE[inhno]$E_RSATR: $FORMAT(_("%1% `%2%\' must not be non-kernel interrupt"), "inhno", INH.INHNO[inhno])$$END$
		$END$
	$END$

$	// カーネル管理外に固定されているinhnoに対して，inhatrにTA_NONKERNEL
$	// が指定されていない場合（E_RSATR）
	$IF LENGTH(FIND(INHNO_FIX_NONKERNEL, inhno))$
		$IF (INH.INHATR[inhno] & TA_NONKERNEL) == 0$
			$ERROR INH.TEXT_LINE[inhno]$E_RSATR: $FORMAT(_("%1% `%2%\' must be non-kernel interrupt"), "inhno", INH.INHNO[inhno])$$END$
		$END$
	$END$

	$IF LENGTH(INTNO[INH.INHNO[inhno]])$
		$intno = INTNO[INH.INHNO[inhno]]$
$		// inhnoに対応するintnoに対するCFG_INTがない場合（E_OBJ）
		$IF !LENGTH(INT.INTNO[intno])$
			$ERROR INH.TEXT_LINE[inhno]$E_OBJ: $FORMAT(_("%1% `%2%\' corresponding to %3% `%4%\' is not configured with %5%"), "intno", INT.INTNO[intno], "inhno", INH.INHNO[inhno], "CFG_INT")$$END$
		$ELSE$
			$IF (INH.INHATR[inhno] & TA_NONKERNEL) == 0$
$				// inhatrにTA_NONKERNELが指定されておらず，inhnoに対応
$				// するintnoに対してCFG_INTで設定された割込み優先度が
$				// TMIN_INTPRIよりも小さい場合（E_OBJ）
				$IF INT.INTPRI[intno] < TMIN_INTPRI$
					$ERROR INT.TEXT_LINE[intno]$E_OBJ: $FORMAT(_("%1% `%2%\' configured for %3% `%4%\' is higher than %5%"), "intpri", INT.INTPRI[intno], "inhno", INH.INHNO[inhno], "TMIN_INTPRI")$$END$
				$END$
			$ELSE$
$				// inhatrにTA_NONKERNELが指定されており，inhnoに対応
$				// するintnoに対してCFG_INTで設定された割込み優先度が
$				// TMIN_INTPRI以上である場合（E_OBJ）
				$IF INT.INTPRI[intno] >= TMIN_INTPRI$
					$ERROR INT.TEXT_LINE[intno]$E_OBJ: $FORMAT(_("%1% `%2%\' configured for %3% `%4%\' is lower than or equal to %5%"), "intpri", INT.INTPRI[intno], "inhno", INH.INHNO[inhno], "TMIN_INTPRI")$$END$
				$END$
			$END$
		$END$
	$END$
	$i = i + 1$
$END$

$ 割込みサービスルーチン（ISR）に関するエラーチェック
$FOREACH order ISR.ORDER_LIST$
$	// isratrが（TA_NULL）でない場合（E_RSATR）
	$IF (ISR.ISRATR[order] & ~TARGET_ISRATR) != 0$
		$ERROR ISR.TEXT_LINE[order]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "isratr", ISR.ISRATR[order], "ATT_ISR")$$END$
	$END$

$	// intnoがATT_ISRに対する割込み番号として正しくない場合（E_PAR）
	$IF !LENGTH(FIND(INTNO_ATTISR_VALID, ISR.INTNO[order]))$
		$ERROR ISR.TEXT_LINE[order]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "intno", ISR.INTNO[order], "ATT_ISR")$$END$
	$END$

$	// (TMIN_ISRPRI <= isrpri && isrpri <= TMAX_ISRPRI)でない場合（E_PAR）
	$IF !(TMIN_ISRPRI <= ISR.ISRPRI[order] && ISR.ISRPRI[order] <= TMAX_ISRPRI)$
		$ERROR ISR.TEXT_LINE[order]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "isrpri", ISR.ISRPRI[order], "ATT_ISR")$$END$
	$END$
$END$

$FOREACH intno INTNO_ATTISR_VALID$
	$inhno = INHNO[intno]$

$	// 割込み番号intnoに対して登録されたISRのリストの作成
	$isr_order_list = {}$
	$FOREACH order ISR.ORDER_LIST$
		$IF ISR.INTNO[order] == intno$
			$isr_order_list = APPEND(isr_order_list, order)$
			$order_for_error = order$
		$END$
	$END$

$	// 割込み番号intnoに対して登録されたISRが存在する場合
	$IF LENGTH(isr_order_list) > 0$
$		// intnoに対応するinhnoに対してDEF_INHがある場合（E_OBJ）
		$IF LENGTH(INH.INHNO[inhno])$
			$ERROR ISR.TEXT_LINE[order_for_error]$E_OBJ: $FORMAT(_("%1% `%2%\' in %3% is duplicated with %4% `%5%\'"), "intno", ISR.INTNO[order_for_error], "ATT_ISR", "inhno", INH.INHNO[inhno])$$END$
		$END$

$		// intnoに対するCFG_INTがない場合（E_OBJ）
		$IF !LENGTH(INT.INTNO[intno])$
			$ERROR ISR.TEXT_LINE[order_for_error]$E_OBJ: $FORMAT(_("%1% `%2%\' is not configured with %3%"), "intno", ISR.INTNO[order_for_error], "CFG_INT")$$END$
		$ELSE$
$			// intnoに対してCFG_INTで設定された割込み優先度がTMIN_INTPRI
$			// よりも小さい場合（E_OBJ）
			$IF INT.INTPRI[intno] < TMIN_INTPRI$
				$ERROR INT.TEXT_LINE[intno]$E_OBJ: $FORMAT(_("%1% `%2%\' configured for %3% `%4%\' is higher than %5%"), "intpri", INT.INTPRI[intno], "intno", ISR.INTNO[order_for_error], "TMIN_INTPRI")$$END$
			$END$
		$END$
	$END$
$END$

$ 割込みサービスルーチン（ISR）管理のデータ構造
$intno_isr_list = {}$
$FOREACH intno INTNO_ATTISR_VALID$
	$inhno = INHNO[intno]$
	$IF LENGTH(INT.INTNO[intno]) && !LENGTH(INH.INHNO[inhno])$
		$intno_isr_list = APPEND(intno_isr_list, intno)$
	$END$
$END$

$INTNO_ISR = {}$
$i = 0$
$FOREACH intno SORT(intno_isr_list, "INT.INTNO")$
	$INTNO_ISR = APPEND(INTNO_ISR, intno)$
	$ISR_QUEUE_HEADER[intno] = FORMAT("&(_kernel_isr_queue_table[%d])", i)$
	$i = i + 1$
$END$

const uint_t _kernel_tnum_isr_queue = $LENGTH(INTNO_ISR)$;$NL$
$NL$

$IF LENGTH(INTNO_ISR)$
	const ISR_ENTRY _kernel_isr_queue_list[$LENGTH(INTNO_ISR)$] = {$NL$
	$JOINEACH intno INTNO_ISR ",\n"$
		$TAB${ $intno$, $ISR_QUEUE_HEADER[intno]$ }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(QUEUE, _kernel_isr_queue_table);$NL$
$END$$NL$

$IF LENGTH(INTNO_ISR)$
	QUEUE _kernel_isr_queue_table[$LENGTH(INTNO_ISR)$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(QUEUE, _kernel_isr_queue_table);$NL$
$END$$NL$

$ 割込みサービスルーチン（ISR）呼出しのための割込みハンドラの生成
$FOREACH intno INTNO_ISR$
	$inhno = INHNO[intno]$

$	// DEF_INH(inhno, { TA_NULL, _kernel_inthdr_<intno> } );
	$INH.INHNO[inhno] = inhno$
	$INH.INHATR[inhno] = VALUE("TA_NULL", 0)$
	$INH.INTHDR[inhno] = CONCAT("_kernel_inthdr_", intno)$
	$INH.ORDER_LIST = APPEND(INH.ORDER_LIST, inhno)$

$	// ISR用の割込みハンドラ
	void$NL$
	_kernel_inthdr_$intno$(void)$NL$
	{$NL$
	$TAB$i_begin_int($intno$);$NL$
	$TAB$_kernel_call_isr($ISR_QUEUE_HEADER[intno]$);$NL$
	$TAB$i_end_int($intno$);$NL$
	}$NL$
	$NL$
$END$

$ 割込みサービスルーチンの数
#define TNUM_SISR	$LENGTH(ISR.ORDER_LIST)$$NL$
#define TNUM_ISR	$LENGTH(ISR.ORDER_LIST) + num_aisrid$$NL$
$NL$

$ 割込みサービスルーチンID番号の最大値
const ID _kernel_tmax_isrid = (TMIN_ISRID + TNUM_ISRID - 1);$NL$
const uint_t _kernel_tnum_sisr = TNUM_SISR;$NL$
$NL$

$ 割込みサービスルーチン初期化ブロックの生成
$IF LENGTH(ISR.ORDER_LIST)$
	const ISRINIB _kernel_sisrinib_table[TNUM_SISR] = {$NL$
	$JOINEACH order ISR.ORDER_LIST ",\n"$
		$TAB${ ($ISR.ISRATR[order]$), ($ISR.EXINF[order]$), ($ISR.INTNO[order]$), ($ISR_QUEUE_HEADER[ISR.INTNO[order]]$), ($ISR.ISR[order]$), ($ISR.ISRPRI[order]$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const ISRINIB, _kernel_sisrinib_table);$NL$
$END$
$NL$

$ 動的生成割込みサービスルーチン用の割込みサービスルーチン初期化ブロッ
$ クの生成
$IF num_aisrid > 0$
	ISRINIB _kernel_aisrinib_table[$num_aisrid$];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(ISRINIB, _kernel_aisrinib_table);$NL$
$END$
$NL$

$ 割込みサービスルーチン管理ブロックの生成
$IF num_isr > 0$
	ISRCB _kernel_isrcb_table[TNUM_ISR];$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(ISRCB, _kernel_isrcb_table);$NL$
$END$
$NL$

$ 
$  割込み管理機能のための標準的な初期化情報の生成
$ 
$ 割込みハンドラの初期化に必要な情報
$IF !OMIT_INITIALIZE_INTERRUPT || ALT(USE_INHINIB_TABLE,0)$

$ 割込みハンドラ数
#define TNUM_INHNO	$LENGTH(INH.ORDER_LIST)$$NL$
const uint_t _kernel_tnum_inhno = TNUM_INHNO;$NL$
$NL$
$FOREACH inhno INH.ORDER_LIST$
	$IF (INH.INHATR[inhno] & TA_NONKERNEL) == 0$
		INTHDR_ENTRY($INH.INHNO[inhno]$, $+INH.INHNO[inhno]$, $INH.INTHDR[inhno]$)$NL$
	$END$
$END$
$NL$

$ 割込みハンドラ初期化テーブル
$IF LENGTH(INH.ORDER_LIST)$
	const INHINIB _kernel_inhinib_table[TNUM_INHNO] = {$NL$
	$JOINEACH inhno INH.ORDER_LIST ",\n"$
		$IF (INH.INHATR[inhno] & TA_NONKERNEL) == 0$
			$TAB${ ($INH.INHNO[inhno]$), ($INH.INHATR[inhno]$), (FP)(INT_ENTRY($INH.INHNO[inhno]$, $INH.INTHDR[inhno]$)) }
		$ELSE$
			$TAB${ ($INH.INHNO[inhno]$), ($INH.INHATR[inhno]$), (FP)($INH.INTHDR[inhno]$) }
		$END$
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const INHINIB, _kernel_inhinib_table);$NL$
$END$$NL$
$END$

$ 割込み要求ラインの初期化に必要な情報
$IF !OMIT_INITIALIZE_INTERRUPT || ALT(USE_INTINIB_TABLE,0)$

$ 割込み要求ライン数
#define TNUM_INTNO	$LENGTH(INT.ORDER_LIST)$$NL$
const uint_t _kernel_tnum_intno = TNUM_INTNO;$NL$
$NL$

$ 割込み要求ライン初期化テーブル
$IF LENGTH(INT.ORDER_LIST)$
	const INTINIB _kernel_intinib_table[TNUM_INTNO] = {$NL$
	$JOINEACH intno INT.ORDER_LIST ",\n"$
		$TAB${ ($INT.INTNO[intno]$), ($INT.INTATR[intno]$), ($INT.INTPRI[intno]$) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const INTINIB, _kernel_intinib_table);$NL$
$END$$NL$
$END$

$ 
$  CPU例外管理機能
$ 
/*$NL$
$SPC$*  CPU Exception Management Functions$NL$
$SPC$*/$NL$
$NL$

$ CPU例外ハンドラに関するエラーチェック
$i = 0$
$FOREACH excno EXC.ORDER_LIST$
$	// excnoがDEF_EXCに対するCPU例外ハンドラ番号として正しくない場合（E_PAR）
	$IF !LENGTH(FIND(EXCNO_DEFEXC_VALID, EXC.EXCNO[excno]))$
		$ERROR EXC.TEXT_LINE[excno]$E_PAR: $FORMAT(_("illegal %1% `%2%\' in %3%"), "excno", EXC.EXCNO[excno], "DEF_EXC")$$END$
	$END$

$	// excnoがDEF_EXCによって設定済みの場合（E_OBJ）
	$j = 0$
	$FOREACH excno2 EXC.ORDER_LIST$
		$IF EXC.EXCNO[excno] == EXC.EXCNO[excno2] && j < i$
			$ERROR EXC.TEXT_LINE[excno]$E_OBJ: $FORMAT(_("%1% `%2%\' in %3% is duplicated"), "excno", EXC.EXCNO[excno], "DEF_EXC")$$END$
		$END$
		$j = j + 1$
	$END$

$	// excatrが（TA_NULL）でない場合（E_RSATR）
	$IF (EXC.EXCATR[excno] & ~TARGET_EXCATR) != 0$
		$ERROR EXC.TEXT_LINE[excno]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of %3% `%4%\' in %5%"), "excatr", EXC.EXCATR[excno], "excno", EXC.EXCNO[excno], "DEF_EXC")$$END$
	$END$
	$i = i + 1$
$END$

$ CPU例外ハンドラのための標準的な初期化情報の生成
$IF !OMIT_INITIALIZE_EXCEPTION$

$ CPU例外ハンドラ数
#define TNUM_EXCNO	$LENGTH(EXC.ORDER_LIST)$$NL$
const uint_t _kernel_tnum_excno = TNUM_EXCNO;$NL$
$NL$
$FOREACH excno EXC.ORDER_LIST$
	EXCHDR_ENTRY($EXC.EXCNO[excno]$, $+EXC.EXCNO[excno]$, $EXC.EXCHDR[excno]$)$NL$
$END$
$NL$

$ CPU例外ハンドラ初期化テーブル
$IF LENGTH(EXC.ORDER_LIST)$
	const EXCINIB _kernel_excinib_table[TNUM_EXCNO] = {$NL$
	$JOINEACH excno EXC.ORDER_LIST ",\n"$
		$TAB${ ($EXC.EXCNO[excno]$), ($EXC.EXCATR[excno]$), (FP)(EXC_ENTRY($EXC.EXCNO[excno]$, $EXC.EXCHDR[excno]$)) }
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const EXCINIB, _kernel_excinib_table);$NL$
$END$$NL$
$END$

$ 
$  非タスクコンテキスト用のスタック領域
$ 
/*$NL$
$SPC$*  Stack Area for Non-task Context$NL$
$SPC$*/$NL$
$NL$

$IF !LENGTH(ICS.ORDER_LIST)$
$	// DEF_ICSがない場合のデフォルト値の設定
	#ifdef DEFAULT_ISTK$NL$
	$NL$
	#define TOPPERS_ISTKSZ		DEFAULT_ISTKSZ$NL$
	#define TOPPERS_ISTK		DEFAULT_ISTK$NL$
	$NL$
	#else /* DEAULT_ISTK */$NL$
	$NL$
	static STK_T				_kernel_istack[COUNT_STK_T(DEFAULT_ISTKSZ)];$NL$
	#define TOPPERS_ISTKSZ		ROUND_STK_T(DEFAULT_ISTKSZ)$NL$
	#define TOPPERS_ISTK		_kernel_istack$NL$
	$NL$
	#endif /* DEAULT_ISTK */$NL$
$ELSE$

$	// 静的API「DEF_ICS」が複数ある（E_OBJ）
	$IF LENGTH(ICS.ORDER_LIST) > 1$
		$ERROR$E_OBJ: $FORMAT(_("too many %1%"), "DEF_ICS")$$END$
	$END$

$	// istkszが0以下か，ターゲット定義の最小値（TARGET_MIN_ISTKSZ）よりも
$	// 小さい場合（E_PAR）
	$IF ICS.ISTKSZ[1] <= 0 || (TARGET_MIN_ISTKSZ
									&& ICS.ISTKSZ[1] < TARGET_MIN_ISTKSZ)$
		$ERROR ICS.TEXT_LINE[1]$E_PAR: $FORMAT(_("%1% `%2%\' in %3% is too small"), "istksz", ICS.ISTKSZ[1], "DEF_ICS")$$END$
	$END$

$ 	// istkszがスタック領域のサイズとして正しくない場合（E_PAR）
	$IF !EQ(ICS.ISTK[1], "NULL") && CHECK_STKSZ_ALIGN
							&& (ICS.ISTKSZ[1] & (CHECK_STKSZ_ALIGN - 1))$
		$ERROR ICS.TEXT_LINE[1]$E_PAR: $FORMAT(_("%1% `%2%\' in %3% is not aligned"), "istksz", ICS.ISTKSZ[1], "DEF_ICS")$$END$
	$END$

	$IF EQ(ICS.ISTK[1], "NULL")$
$		// スタック領域の自動割付け
		static STK_T				_kernel_istack[COUNT_STK_T($ICS.ISTKSZ[1]$)];$NL$
		#define TOPPERS_ISTKSZ		ROUND_STK_T($ICS.ISTKSZ[1]$)$NL$
		#define TOPPERS_ISTK		_kernel_istack$NL$
	$ELSE$
		#define TOPPERS_ISTKSZ		($ICS.ISTKSZ[1]$)$NL$
		#define TOPPERS_ISTK		(void *)($ICS.ISTK[1]$)$NL$
	$END$
$END$
$NL$

$ 非タスクコンテキスト用のスタック領域
const SIZE		_kernel_istksz = TOPPERS_ISTKSZ;$NL$
STK_T *const	_kernel_istk = TOPPERS_ISTK;$NL$
$NL$
#ifdef TOPPERS_ISTKPT$NL$
STK_T *const	_kernel_istkpt = TOPPERS_ISTKPT(TOPPERS_ISTK, TOPPERS_ISTKSZ);$NL$
#endif /* TOPPERS_ISTKPT */$NL$
$NL$

$ 
$  カーネルが割り付けるメモリ領域
$ 
/*$NL$
$SPC$*  Memory Area Allocated by Kernel$NL$
$SPC$*/$NL$
$NL$

$IF !LENGTH(KMM.ORDER_LIST)$
$	// DEF_KMMがない場合のデフォルト値の設定
	#define TOPPERS_KMMSZ		0$NL$
	#define TOPPERS_KMM			NULL$NL$
$ELSE$
$	// 静的API「DEF_KMM」が複数ある（E_OBJ）
	$IF LENGTH(KMM.ORDER_LIST) > 1$
		$ERROR$E_OBJ: $FORMAT(_("too many %1%"), "DEF_KMM")$$END$
	$END$

$	// kmmszが0以下の場合（E_PAR）
	$IF KMM.KMMSZ[1] <= 0$
		$ERROR KMM.TEXT_LINE[1]$E_PAR: $FORMAT(_("%1% `%2%\' is zero in %3%"), "kmmsz", KMM.KMMSZ[1], "DEF_KMM")$$END$
	$END$

$ 	// kmmszがカーネルが割り付けるメモリ領域のサイズとして正しくない場合（E_PAR）
	$IF !EQ(KMM.KMM[1], "NULL") && CHECK_MB_ALIGN
							&& (KMM.KMMSZ[1] & (CHECK_MB_ALIGN - 1))$
		$ERROR KMM.TEXT_LINE[1]$E_PAR: $FORMAT(_("%1% `%2%\' in %3% is not aligned"), "kmmsz", KMM.KMMSZ[1], "DEF_KMM")$$END$
	$END$

	$IF EQ(KMM.KMM[1], "NULL")$
$		// カーネルが割り付けるメモリ領域の自動割付け
		static MB_T					_kernel_memory[TOPPERS_COUNT_SZ($KMM.KMMSZ[1]$, sizeof(MB_T))];$NL$
		#define TOPPERS_KMMSZ		TOPPERS_ROUND_SZ($KMM.KMMSZ[1]$, sizeof(MB_T))$NL$
		#define TOPPERS_KMM			_kernel_memory$NL$
	$ELSE$
		#define TOPPERS_KMMSZ		($KMM.KMMSZ[1]$)$NL$
		#define TOPPERS_KMM			(void *)($KMM.KMM[1]$)$NL$
	$END$
$END$
$NL$

$ カーネルが割り付けるメモリ領域
const SIZE		_kernel_kmmsz = TOPPERS_KMMSZ;$NL$
MB_T *const		_kernel_kmm = TOPPERS_KMM;$NL$
$NL$

$ 
$  タイムイベント管理
$ 
/*$NL$
$SPC$*  Time Event Management$NL$
$SPC$*/$NL$
$NL$
TMEVTN   _kernel_tmevt_heap[TNUM_TSKID + TNUM_CYCID + TNUM_ALMID];$NL$
$NL$

$ 
$  各モジュールの初期化関数
$ 
/*$NL$
$SPC$*  Module Initialization Function$NL$
$SPC$*/$NL$
$NL$
void$NL$
_kernel_initialize_object(void)$NL$
{$NL$
$TAB$_kernel_initialize_task();$NL$
$IF num_semid$$TAB$_kernel_initialize_semaphore();$NL$$END$
$IF num_flgid$$TAB$_kernel_initialize_eventflag();$NL$$END$
$IF num_dtqid$$TAB$_kernel_initialize_dataqueue();$NL$$END$
$IF num_pdqid$$TAB$_kernel_initialize_pridataq();$NL$$END$
$IF num_mbxid$$TAB$_kernel_initialize_mailbox();$NL$$END$
$IF num_mtxid$$TAB$_kernel_initialize_mutex();$NL$$END$
$IF num_mpfid$$TAB$_kernel_initialize_mempfix();$NL$$END$
$IF num_cycid$$TAB$_kernel_initialize_cyclic();$NL$$END$
$IF num_almid$$TAB$_kernel_initialize_alarm();$NL$$END$
$TAB$_kernel_initialize_interrupt();$NL$
$IF num_isr$$TAB$_kernel_initialize_isr();$NL$$END$
$TAB$_kernel_initialize_exception();$NL$
}$NL$
$NL$

$ 
$  初期化ルーチンの実行関数
$ 
/*$NL$
$SPC$*  Initialization Routine$NL$
$SPC$*/$NL$
$NL$
void$NL$
_kernel_call_inirtn(void)$NL$
{$NL$
$FOREACH order INI.ORDER_LIST$
$ 	// iniatrが（TA_NULL）でない場合（E_RSATR）
	$IF INI.INIATR[order] != 0$
		$ERROR INI.TEXT_LINE[order]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of %3% `%4%\' in %5%"), "iniatr", INI.INIATR[order], "inirtn", INI.INIRTN[order], "ATT_INI")$$END$
	$END$
	$TAB$((INIRTN)($INI.INIRTN[order]$))((intptr_t)($INI.EXINF[order]$));$NL$
$END$
}$NL$
$NL$

$ 
$  終了処理ルーチンの実行関数
$ 
/*$NL$
$SPC$*  Termination Routine$NL$
$SPC$*/$NL$
$NL$
void$NL$
_kernel_call_terrtn(void)$NL$
{$NL$
$FOREACH rorder TER.RORDER_LIST$
$ 	// teratrが（TA_NULL）でない場合（E_RSATR）
	$IF TER.TERATR[rorder] != 0$
		$ERROR TER.TEXT_LINE[rorder]$E_RSATR: $FORMAT(_("illegal %1% `%2%\' of %3% `%4%\' in %5%"), "teratr", TER.TERATR[rorder], "terrtn", TER.TERRTN[rorder], "ATT_TER")$$END$
	$END$
	$TAB$((TERRTN)($TER.TERRTN[rorder]$))((intptr_t)($TER.EXINF[rorder]$));$NL$
$END$
}$NL$
$NL$
