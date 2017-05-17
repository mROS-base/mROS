$
$       オフセットファイル生成用テンプレートファイル（ARM用）
$

$
$  標準テンプレートファイルのインクルード
$
$INCLUDE "kernel/genoffset.tf"$

$
$  フィールドのオフセットの定義の生成
$
$DEFINE("TCB_p_tinib", offsetof_TCB_p_tinib)$
$DEFINE("TCB_texptn", offsetof_TCB_texptn)$
$DEFINE("TCB_sp", offsetof_TCB_sp)$
$DEFINE("TCB_pc", offsetof_TCB_pc)$
$DEFINE("TINIB_exinf", offsetof_TINIB_exinf)$
$DEFINE("TINIB_task", offsetof_TINIB_task)$

$
$  ビットフィールドのオフセットとビット位置の定義の生成
$
$DEFINE_BIT("TCB_enatex", sizeof_TCB, "B")$
