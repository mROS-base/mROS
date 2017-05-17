$ 
$ 		オフセットファイル生成用テンプレートファイル（M68040用）
$ 

$ 
$  標準テンプレートファイルのインクルード
$ 
$INCLUDE "kernel/genoffset.tf"$

$ 
$  オフセット値のマクロ定義の生成
$ 
$DEFINE("TCB_p_tinib", offsetof_TCB_p_tinib)$
$DEFINE("TCB_texptn", offsetof_TCB_texptn)$
$DEFINE("TCB_msp", offsetof_TCB_msp)$
$DEFINE("TCB_pc", offsetof_TCB_pc)$

$DEFINE("TINIB_exinf", offsetof_TINIB_exinf)$
$DEFINE("TINIB_task", offsetof_TINIB_task)$

$ 
$  ビットオフセット値等のマクロ定義の生成
$ 
$DEFINE_BIT("TCB_enatex", sizeof_TCB, "B")$
