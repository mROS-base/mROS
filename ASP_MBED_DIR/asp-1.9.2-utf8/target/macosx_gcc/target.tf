$ 
$ 		パス2のターゲット依存テンプレート（Mac OS X用）
$ 

$ 
$  使用できる割込み番号とそれに対応する割込みハンドラ番号
$ 
$  割込み番号と割込みハンドラ番号は，シグナル番号に一致させる．1～31の
$  シグナル番号の内，SIGKILL（＝9），SIGSTOP（＝17），SIGUSR2（＝31）以
$  外が使用できる．
$ 
$INTNO_VALID = { 1, 2,..., 8; 10, 11,..., 16; 18, 19,..., 30 }$
$INHNO_VALID = INTNO_VALID$

$ 
$  ATT_ISRで使用できる割込み番号とそれに対応する割込みハンドラ番号
$ 
$INTNO_ATTISR_VALID = INTNO_VALID$
$INHNO_ATTISR_VALID = INHNO_VALID$

$ 
$  DEF_INT／DEF_EXCで使用できる割込みハンドラ番号／CPU例外ハンドラ番号
$ 
$INHNO_DEFINH_VALID = INHNO_VALID$
$EXCNO_DEFEXC_VALID = INHNO_VALID$

$ 
$  CFG_INTで使用できる割込み番号と割込み優先度
$ 
$  割込み優先度は，-1～-7が使用できる．-7はNMIと扱う．
$ 
$INTNO_CFGINT_VALID = INTNO_VALID$
$INTPRI_CFGINT_VALID = { -1, -2,..., -7 }$

$
$  スタック領域の確保関数
$
$  Intelプロセッサでは，スタックポインタを16バイト境界にアラインさせる
$  必要がある．
$
$FUNCTION ALLOC_STACK$
	static STK_T $ARGV[1]$[COUNT_STK_T($ARGV[2]$)]
						$SPC$__attribute__((aligned(16)));$NL$
	$RESULT = FORMAT("ROUND_STK_T(%1%)", ARGV[2])$
$END$

$ 
$  標準テンプレートファイルのインクルード
$ 
$INCLUDE "kernel/kernel.tf"$

/*$NL$
$SPC$*  Target-dependent Definitions (Mac OS X)$NL$
$SPC$*/$NL$
$NL$

$ 
$  マスクできないシグナルとカーネルが使うシグナルに関する設定
$ 
$  SIGKILL（マスク不可）
$  SIGSTOP（マスク不可）
$  SIGUSR2（カーネルが利用）
$ 
$INT.INTPRI[SIGKILL] = -7$
$INT.INTPRI[SIGSTOP] = -7$
$INT.INTPRI[SIGUSR2] = -7$

$ 
$   CFG_INTのターゲット依存のエラーチェック
$ 
$FOREACH intno INT.ORDER_LIST$
	$IF (INT.INTATR[intno] & TA_EDGE) == 0$
		$ERROR INT.TEXT_LINE[intno]$E_RSATR: $FORMAT("Level trigger is not supported for intno `%1%\' in CFG_INT", INT.INTNO[intno])$$END$
	$END$
$END$

$ 
$  CPU例外ハンドラに関する処理
$ 
$FOREACH excno EXC.ORDER_LIST$
$	// 割込みハンドラとの重複チェック
	$IF LENGTH(INH.INHNO[excno])$
		$ERROR EXC.TEXT_LINE[excno]$E_OBJ: $FORMAT(_("%1% `%2%\' in %3% is duplicated with %4% `%5%\'"), "excno", EXC.EXCNO[excno], "DEF_EXC", "inhno", INH.INHNO[excno])$$END$
	$END$

$	// CPU例外ハンドラを登録したシグナルに関する設定
	$IF LENGTH(INTNO[excno])$
		$INT.INTPRI[INTNO[excno]] = -7$
	$END$
$END$

$ 
$  デバッグ用にマスクしないシグナルに関する設定
$ 
$  SIGINT（デバッグ用）
$  SIGBUS（デバッグ用）
$  SIGSEGV（デバッグ用）
$ 
$IF !LENGTH(INT.INTPRI[SIGINT])$
	$INT.INTPRI[SIGINT] = -7$
$END$
$IF !LENGTH(INT.INTPRI[SIGBUS])$
	$INT.INTPRI[SIGBUS] = -7$
$END$
$IF !LENGTH(INT.INTPRI[SIGSEGV])$
	$INT.INTPRI[SIGSEGV] = -7$
$END$

$ 
$  割込みハンドラの初期化に必要な情報
$ 

$ 割込みハンドラ数
#define TNUM_INHNO	$LENGTH(INH.ORDER_LIST)$$NL$
const uint_t _kernel_tnum_inhno = TNUM_INHNO;$NL$
$NL$
$FOREACH inhno INH.ORDER_LIST$
	$IF (INH.INHATR[inhno] & TA_NONKERNEL) == 0$
		INTHDR_ENTRY($INH.INHNO[inhno]$, $INH.INTHDR[inhno]$, $INT.INTPRI[INTNO[inhno]]$)$NL$
	$END$
$END$
$NL$

$ 割込みハンドラ初期化テーブル
$IF LENGTH(INH.ORDER_LIST)$
	const INHINIB _kernel_inhinib_table[TNUM_INHNO] = {$NL$
	$JOINEACH inhno INH.ORDER_LIST ",\n"$
		$IF (INH.INHATR[inhno] & TA_NONKERNEL) == 0$
			$TAB${ ($INH.INHNO[inhno]$), ($INH.INHATR[inhno]$), (FP)(INT_ENTRY($INH.INHNO[inhno]$, $INH.INTHDR[inhno]$)), ($INT.INTPRI[INTNO[INH.INHNO[inhno]]]$) }
		$ELSE$
			$TAB${ ($INH.INHNO[inhno]$), ($INH.INHATR[inhno]$), (FP)($INH.INTHDR[inhno]$), ($INT.INTPRI[INTNO[INH.INHNO[inhno]]]$) }
		$END$
	$END$$NL$
	};$NL$
$ELSE$
	TOPPERS_EMPTY_LABEL(const INHINIB, _kernel_inhinib_table);$NL$
$END$
$NL$

$ 
$  割込み優先度毎のそれ以下の割込み要求をマスクするための情報のテーブル
$ 
const sigset_t _kernel_sigmask_table[8] = {$NL$
$FOREACH intpri { 0, -1,..., -6 }$
	$intmask = 0$
	$FOREACH intno { 1, 2, ..., 31 }$
		$IF ALT(INT.INTPRI[intno], 0) >= intpri$
			$intmask = intmask | (1 << (intno - 1))$
		$END$
	$END$
	$TAB$UINT32_C($FORMAT("0x%08x", intmask)$),$NL$
$END$
$TAB$UINT32_C($FORMAT("0x%08x", intmask)$)$NL$
};$NL$

$ 
$  割込み要求禁止フラグ実現のための変数の初期値
$ 
$sigmask_disint_init = 0$
$FOREACH intno INT.ORDER_LIST$
	$IF (INT.INTATR[intno] & TA_ENAINT) == 0$
		$sigmask_disint_init = sigmask_disint_init | (1 << (INT.INTNO[intno] - 1))$
	$END$
$END$
$NL$
const sigset_t	_kernel_sigmask_disint_init = 
		UINT32_C($FORMAT("0x%08x", sigmask_disint_init)$);$NL$
