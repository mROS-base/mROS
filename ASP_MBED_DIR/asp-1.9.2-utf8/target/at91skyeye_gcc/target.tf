$ 
$     パス2のターゲット依存テンプレート（AT91SKYEYE用）
$ 


$ 
$  有効な割込み番号，割込みハンドラ番号，CPU例外ハンドラ番号
$  0,19～31は使用出来ないが，テーブルを作成するために必要
$INTNO_VALID = { 0,1,...,31 }$
$INHNO_VALID = INTNO_VALID$

$ 
$  ATT_ISRで使用できる割込み番号とそれに対応する割込みハンドラ番号
$ 
$INTNO_ATTISR_VALID = { 1,2,...,18 }$
$INHNO_ATTISR_VALID = { 1,2,...,18 }$

$ 
$  DEF_INTで使用できる割込みハンドラ番号
$ 
$INHNO_DEFINH_VALID = { 1,2,...,18 }$

$ 
$  CFG_INTで使用できる割込み番号と割込み優先度
$ 
$INTNO_CFGINT_VALID  = { 1,2,...,18 }$
$INTPRI_CFGINT_VALID = { -1,-2,...,-7 }$

$ 
$  割込み属性中のターゲット依存に用いるビット
$ 
$TARGET_INTATR = TA_HIGHLEVEL | TA_POSEDGE | TA_LOWLEVEL$

$ 
$  コア依存テンプレートのインクルード（ARM用）
$ 
$INCLUDE"arm_gcc/common/core.tf"$

$ 
$  割込み優先度テーブル
$ 
$FILE "kernel_cfg.c"$
$NL$
const PRI _kernel_inh_ipm_tbl[TNUM_INH] = {$NL$
$FOREACH inhno INHNO_VALID$ 
	$IF LENGTH(INH.INHNO[inhno])$
	  $TAB$$INT.INTPRI[inhno]$,
	$ELSE$
	  $TAB$0,
	$END$
	$SPC$$FORMAT("/* %d */", +inhno)$$NL$
$END$
$NL$};$NL$
$NL$

$ 
$  割込みマスクテーブル
$ 
const uint32_t _kernel_ipm_mask_tbl[8]={$NL$
$FOREACH intpri { 0,-1,...,-7 }$
 $intmask = 0$
 $FOREACH intno (INT.ID_LIST)$
  $IF INT.INTPRI[intno] >= intpri $
	$intmask = intmask | (1 << (INT.INTNO[intno]))$
  $END$
 $END$
 $TAB$UINT32_C($FORMAT("0x%08x", intmask)$),/* Priority $+intpri$ */$NL$
$END$
$NL$
};$NL$


$ 
$  割込みハンドラテーブル
$ 
$NL$
const FP _kernel_inh_tbl[TNUM_INH] = {$NL$
$FOREACH inhno INHNO_VALID$ 
	$IF LENGTH(INH.INHNO[inhno])$
		$TAB$(FP)($INH.INTHDR[inhno]$),
	$ELSE$
		$TAB$(FP)(_kernel_default_int_handler),
	$END$
	$SPC$$FORMAT("/* %d */", +inhno)$$NL$
$END$
$NL$};$NL$
$NL$
