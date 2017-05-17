/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2015-2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: core_config_v6m.h 2744 2016-01-11 01:44:30Z ertl-honda $
 */

/*
 *		割込み処理モデル（ARMv6-M用）
 *
 *  このインクルードファイルは，core_config.h（または，そこからインク
 *  ルードされるファイル）のみからインクルードされる．他のファイルから
 *  直接インクルードしてはならない．
 */

#ifndef TOPPERS_CORE_INTMODEL_V6M_H
#define TOPPERS_CORE_INTMODEL_V6M_H

/*
 *  ターゲット依存のオブジェクト属性
 */
#define TARGET_INHATR  TA_NONKERNEL /* ターゲット定義の割込みハンドラ属性 */

/*
 *  割込み優先度マスクの外部表現と内部表現の変換
 *
 *  アセンブリ言語のソースファイルからインクルードする場合のために，
 *  CASTを使用
 *   外部表現 : TMIN_INTPRI  ～       0  
 *   内部表現 :      0       ～  -TMIN_INTPRI
 */
#define EXT_IPM(iipm)   (CAST(PRI,iipm + TMIN_INTPRI))     /* 内部表現を外部表現に */
#define INT_IPM(ipm)    (CAST(uint8_t, ipm - TMIN_INTPRI)) /* 外部表現を内部表現に */

/*
 *  割込み優先度マスクをNVICの優先度に変換
 */
#define INT_NVIC_PRI(ipm)    (((1 << TBITW_IPRI) - CAST(uint8_t, -(ipm)))  << (8 - TBITW_IPRI))

/*
 *  TIPM_ENAALL（割込み優先度マスク全解除）の内部表現
 *
 */
#define IIPM_ENAALL  (-TMIN_INTPRI)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  割込み要求禁止フラグの実現のための変数
 */
extern uint32_t ief;			/* IRQの割込み要求許可フラグの状態 */
extern uint8_t  ief_systick;	/* SysTickの割込み要求許可フラグの状態 */

/*
 *  割込み優先度マスク実現のための変数
 */
extern uint8_t iipm;		/* 現在の割込み優先度マスクの値 */ 

/*
 *  割込み優先度マスク実現のための変数（kernel_cfg.c）
 */
extern const uint32_t iipm_enable_irq_tbl[];
extern const uint8_t iipm_enable_systic_tbl[];

/*
 *  CPUロック状態への移行
 *
 */
Inline void
x_lock_cpu(void)
{
	set_primask();
	/* クリティカルセクションの前後でメモリが書き換わる可能性がある */
	ARM_MEMORY_CHANGED;    
}

#define t_lock_cpu()    x_lock_cpu()
#define i_lock_cpu()    x_lock_cpu()

/*
 *  CPUロック状態の解除
 *
 */
Inline void
x_unlock_cpu(void)
{
	/* クリティカルセクションの前後でメモリが書き換わる可能性がある */
	ARM_MEMORY_CHANGED;
	clear_primask();
}

#define t_unlock_cpu()    x_unlock_cpu()
#define i_unlock_cpu()    x_unlock_cpu()

/*
 *  CPUロック状態の参照
 */
Inline bool_t
x_sense_lock(void)
{
	return(read_primask() == 0x1u);
}

#define t_sense_lock()    x_sense_lock()
#define i_sense_lock()    x_sense_lock()

/*
 *  chg_ipmで有効な割込み優先度の範囲の判定
 *
 *  TMIN_INTPRIの値によらず，chg_ipmでは，-(1 << TBITW_IPRI)～TIPM_ENAALL（＝0）
 *  の範囲に設定できることとする（ターゲット定義の拡張）．
 *  割込み優先度のビット幅(TBITW_IPRI)が 2 の場合は，-4 ～ 0 が指定可能である．
 *   
 */
#define VALID_INTPRI_CHGIPM(intpri) \
				((-((1 << TBITW_IPRI) - 1) <= (intpri) && (intpri) <= TIPM_ENAALL))

/*
 * （モデル上の）割込み優先度マスクの設定
 *
 */
Inline void
x_set_ipm(PRI intpri)
{
	uint32_t tmp;
	iipm = INT_IPM(intpri);

	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	if ((iipm_enable_systic_tbl[iipm] & ief_systick) ==  0x01) {
		tmp |= SYSTIC_TICINT;
	}else{
		tmp &= ~SYSTIC_TICINT;
	}
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);

	/* 一旦全割込み禁止 */
	sil_wrw_mem((void *)NVIC_CLRENA0, 0xffffffff);
	sil_wrw_mem((void *)NVIC_SETENA0, (iipm_enable_systic_tbl[iipm] & ief));
	SCS_SYNC;
}

#define t_set_ipm(intpri)    x_set_ipm(intpri)
#define i_set_ipm(intpri)    x_set_ipm(intpri)

/*
 * （モデル上の）割込み優先度マスクの参照
 *
 */
Inline PRI
x_get_ipm(void)
{
	return EXT_IPM(iipm);
}

#define t_get_ipm()    x_get_ipm()
#define i_get_ipm()    x_get_ipm()

/*
 *  割込み要求禁止フラグ
 */

/*
 *  割込み属性が設定されているかを判別するための変数（kernel_cfg.c）
 */
extern const uint32_t	bitpat_cfgint[];

/*
 *  割込み要求禁止フラグのセット
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．  
 */
Inline bool_t
x_disable_int(INTNO intno)
{
	uint32_t tmp;

	/*
	 *  割込み属性が設定されていない場合
	 */
	if ((bitpat_cfgint[intno >> 5] & (1 << (intno & 0x1f))) == 0x00) {
		return(false);
	}

	if (intno == IRQNO_SYSTICK) {
		tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
		tmp &= ~SYSTIC_TICINT;
		sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
		ief_systick &= ~0x01;
	}else {
		tmp = intno - 16;
		sil_wrw_mem((void *)(NVIC_CLRENA0), (1 << (tmp & 0x1f)));
		ief &= ~(1 << (tmp & 0x1f));
	}
	SCS_SYNC;

	return(true);
}

#define t_disable_int(intno) x_disable_int(intno)
#define i_disable_int(intno) x_disable_int(intno)

/*
 *  割込み要求禁止フラグの解除
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */
Inline bool_t
x_enable_int(INTNO intno)
{
	uint32_t tmp;

	/*
	 *  割込み属性が設定されていない場合
	 */
	if ((bitpat_cfgint[intno >> 5] & (1 << (intno & 0x1f))) == 0x00) {
		return(false);
	}

	if (intno == IRQNO_SYSTICK) {
		ief_systick |= 0x01;
		if ((iipm_enable_systic_tbl[iipm] & ief_systick) ==  0x01) {
			tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
			tmp |= SYSTIC_TICINT;;
			sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
		}
	}else {
		tmp = intno - 16;
		ief |= (1 << (tmp & 0x1f));
		if ((iipm_enable_irq_tbl[iipm] & (1 << (tmp & 0x1f))) != 0) {
			sil_wrw_mem((void *)(NVIC_SETENA0), (1 << (tmp & 0x1f)));
		}
	}

	return(true);
}

#define t_enable_int(intno) x_enable_int(intno)
#define i_enable_int(intno) x_enable_int(intno)

/*
 *  PendSVCハンドラ（core_support.S）
 */
extern void pendsvc_handler(void);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_CORE_INTMODEL_V6M_H */
