$ ======================================================================
$ 
$   TOPPERS/ASP Kernel
$       Toyohashi Open Platform for Embedded Real-Time Systems/
$       Advanced Standard Profile Kernel
$ 
$   Copyright (C) 2011 by Embedded and Real-Time Systems Laboratory
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
$   $Id: genoffset.tf 2728 2015-12-30 01:46:11Z ertl-honda $
$ 
$ =====================================================================

$ 
$  #defineディレクティブの生成
$ 
$FUNCTION DEFINE$
	#define $ARGV[1]$$TAB$$FORMAT("%d", +ARGV[2])$$NL$
$END$

$ 
$  ビットのサーチ
$ 
$FUNCTION SEARCH_BIT$
	$_val = ARGV[1]$
	$FOREACH _val_bit RANGE(0,7)$
		$IF (_val & 1) != 0$
			$RESULT = _val_bit$
		$END$
		$_val = _val >> 1$
	$END$
$END$

$ 
$  ビットフィールドのオフセットとビット位置の定義の生成
$ 
$FUNCTION DEFINE_BIT$
	$label = ARGV[1]$
	$struct_size = ARGV[2]$
	$output_size = ARGV[3]$

	$top = SYMBOL(label)$
	$IF !LENGTH(top)$
		$ERROR$$FORMAT("label %1% not found", label)$$END$
	$ELSE$
		$val = 0$
		$FOREACH i RANGE(0,struct_size-1)$
			$tmp_val = PEEK(top + i, 1)$
			$IF val == 0 && tmp_val != 0$
				$val = tmp_val$
				$offset = i$
			$END$
		$END$

		$IF val == 0$
			$ERROR$$FORMAT("bit not found in %1%", ARGV[1])$$END$
		$ELSE$
			$val_bit = SEARCH_BIT(val)$
			$RESULT = {}$
			$IF EQ(output_size, "W")$
				$IF SIL_ENDIAN_BIG$
					$val_bit = val_bit + 24 - ((offset & 0x03) << 3)$
				$ELSE$
					$val_bit = val_bit + ((offset & 0x03) << 3)$
				$END$
				$offset = offset & ~0x03$
			$ELSE$$IF EQ(output_size, "H")$
				$IF SIL_ENDIAN_BIG$
					$val_bit = val_bit + 8 - ((offset & 0x01) << 3)$
				$ELSE$
					$val_bit = val_bit + ((offset & 0x01) << 3)$
				$END$
				$offset = offset & ~0x01$
			$END$$END$
		$END$

		#define $label$$TAB$$FORMAT("%d", +offset)$$NL$
		#define $label$_bit$TAB$$FORMAT("%d", +val_bit)$$NL$
		#define $label$_mask$TAB$$FORMAT("0x%x", 1 << val_bit)$$NL$
	$END$
$END$

$ 
$  バイト配置のチェック
$ 
$FUNCTION MAGIC_CHECK$
	$size = ARGV[1]$
	$check = ARGV[2]$

	$label = FORMAT("MAGIC_%d", +size)$
	$top = SYMBOL(label)$
	$IF !LENGTH(top)$
		$ERROR$$FORMAT("label %1% not found", label)$$END$
	$ELSE$
		$FOREACH offset RANGE(1,size)$
			$IF SIL_ENDIAN_BIG$
				$val = PEEK(top + offset - 1, 1)$
			$ELSE$
				$val = PEEK(top + size - offset, 1)$
			$END$
			$IF val != AT(check, offset - 1)$
				$ERROR$$FORMAT("value check of %1% failed", label)$$END$
			$END$
		$END$
	$END$
$END$

$MAGIC_CHECK(1, { 0x12 })$
$MAGIC_CHECK(2, { 0x12, 0x34 })$
$MAGIC_CHECK(4, { 0x12, 0x34, 0x56, 0x78 })$

$ 
$  ファイルヘッダの生成
$ 
$FILE "offset.h"$
/* offset.h */$NL$
$NL$
