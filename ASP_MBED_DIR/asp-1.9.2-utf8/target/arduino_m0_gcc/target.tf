$ 
$     パス2のターゲット依存テンプレート（ARDUINO_M0用）
$ 

$FUNCTION VECTOR_ATTRIBUTE$
	__attribute__ ((section(".isr_vector"))) $NL$
$END$


$ 
$  コア依存のテンプレートファイルのインクルード
$ 
$INCLUDE"arm_m_gcc/common/core.tf"$
