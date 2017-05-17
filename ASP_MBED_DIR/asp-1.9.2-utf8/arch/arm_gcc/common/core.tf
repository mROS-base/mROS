$ 
$     パス2のコア依存テンプレート（ARM用）
$

$
$  有効なCPU例外ハンドラ番号
$
$EXCNO_VALID = { 1,2,3,4,6 }$

$
$  DEF_EXCで使用できるCPU例外ハンドラ番号
$
$EXCNO_DEFEXC_VALID = EXCNO_VALID$

$
$  標準テンプレートファイルのインクルード
$
$INCLUDE "kernel/kernel.tf"$

$
$  例外ハンドラテーブル
$
$FILE "kernel_cfg.c"$
$NL$
const FP _kernel_exch_tbl[TNUM_EXCH] = {$NL$
$FOREACH excno {0,1,...,6}$ 
	$IF LENGTH(EXC.EXCNO[excno])$
		$TAB$(FP)($EXC.EXCHDR[excno]$),
	$ELSE$
		$TAB$(FP)(_kernel_default_exc_handler),
	$END$
	$SPC$$FORMAT("/* %d */", +excno)$$NL$
$END$
$NL$};$NL$
$NL$

