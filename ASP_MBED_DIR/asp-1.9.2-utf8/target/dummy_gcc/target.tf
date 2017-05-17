$ 
$ 		パス2のターゲット依存テンプレート（ダミーターゲット用）
$ 

$ 
$  使用できる割込み番号とそれに対応する割込みハンドラ番号
$ 
$INTNO_VALID = { 0, 1,..., 31 }$
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
$INTNO_CFGINT_VALID = INTNO_VALID$
$INTPRI_CFGINT_VALID = { -1, -2,..., -7 }$

$ 
$  標準テンプレートファイルのインクルード
$ 
$INCLUDE "kernel/kernel.tf"$
