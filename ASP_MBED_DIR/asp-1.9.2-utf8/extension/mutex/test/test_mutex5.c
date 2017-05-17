/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2007-2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_mutex5.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		ミューテックスのテスト(5)
 *
 * 【テストの目的】
 *
 *  優先度上限ミューテックスに対して，ミューテックスの再初期化処理を網
 *  羅的にテストする．ただし，change_priorityとmutex_calc_priorityの内
 *  容には踏み込まない．
 *
 * 【テスト項目】
 *
 *	(A) ミューテックスの初期化（ini_mtx）に伴うミューテックスをロックし
 *		ていたタスク（実行できる状態）の優先度変更
 *		(A-1) 初期化したミューテックスの上限優先度が，ミューテックスを
 *			  ロックしていたタスク（実行できる状態）の現在優先度と同じ
 *			  で，ミューテックスのロック解除で優先度が下がるべき場合に，
 *			  当該タスクの優先度が適切に下げられること．また，同じ優先
 *			  度内での優先順位が最高になること
 *		(A-2) 初期化したミューテックスの上限優先度が，ミューテックスを
 *			  ロックしていたタスク（実行できる状態）の現在優先度と同じ
 *			  で，ミューテックスのロック解除で優先度が変わるべきでない
 *			  場合に，当該タスクの優先度が変わらないこと．また，同じ優
 *			  先度内での優先順位が変わらないこと
 *		(A-3) 初期化したミューテックスの上限優先度が，ミューテックスを
 *			  ロックしていたタスク（実行できる状態）の現在優先度よりも
 *			  低い場合に，当該タスクの優先度が変わらないこと．また，同
 *			  じ優先度内での優先順位が変わらないこと
 *		(A-4) (A-1)の結果，タスクディスパッチが起こること．
 *	(B) ミューテックスの初期化（ini_mtx）に伴うミューテックスをロックし
 *		ていたタスク（待ち状態）の優先度変更
 *		(B-1) 初期化したミューテックスの上限優先度が，ミューテックスを
 *			  ロックしていたタスク（待ち状態）の現在優先度と同じで，
 *			  ミューテックスのロック解除で優先度が下がるべき場合に，当
 *			  該タスクの優先度が適切に下げられること．また，同じ優先度
 *			  内での順序が最後になること
 *		(B-2) 初期化したミューテックスの上限優先度が，ミューテックスを
 *			  ロックしていたタスク（待ち状態）の現在優先度と同じで，
 *			  ミューテックスのロック解除で優先度が変わるべきでない場合
 *			  に，当該タスクの優先度が変わらないこと．また，同じ優先度
 *			  内での順序が変わらないこと
 *		(B-3) 初期化したミューテックスの上限優先度が，ミューテックスを
 *			  ロックしていたタスク（待ち状態）の現在優先度よりも低い場
 *			  合に，当該タスクの優先度が変わらないこと．また，同じ優先
 *			  度内での順序が変わらないこと
 *	(C) ミューテックスの初期化（ini_mtx）に伴うミューテックスを待ってい
 *		たタスクの状態変化
 *		(C-1) ミューテックスを待っていたタスクが待ち解除されること．
 *		(C-2) (C-1)の結果，タスクディスパッチが起こること．
 *
 * 【テスト項目の実現方法】
 *
 *	(A-1)
 *		低優先度タスク（TASK1）が，中優先度上限ミューテックス（MTX1）を
 *		ロックした状態で，高優先度タスク（TASK5）からMTX1を再初期化し，
 *		TASK1の優先度が低優先度に下がることを確認する．また，実行可能状
 *		態の低優先度タスク（TASK2）よりも，優先順位が高くなることを確認
 *		する．　
 *	(A-2)
 *		低優先度タスク（TASK1）が，中優先度上限ミューテックスを2つ
 *		（MTX1，MTX2）をロックした状態で，高優先度タスク（TASK5）から
 *		MTX1を再初期化し，TASK1の優先度が変化しないことを確認する．また，
 *		実行可能状態の中優先度タスクを2つ（TASK3，TASK4）を用意しておき，
 *		優先順位が変わらないことを確認する．
 *	(A-3)
 *		低優先度タスク（TASK1）が，中優先度上限ミューテックス（MTX2）と
 *		低優先度上限ミューテックス（MTX3）をロックした状態で，高優先度
 *		タスク（TASK5）からMTX3を再初期化し，TASK1の優先度が変化しない
 *		ことを確認する．また，実行可能状態の中優先度タスクを2つ（TASK3，
 *		TASK4）を用意しておき，優先順位が変わらないことを確認する．
 *	(A-4)
 *		低優先度タスク（TASK1）が，中優先度上限ミューテックス（MTX2）を
 *		ロックした状態で，TASK1からMTX2を再初期化し，TASK1の優先度が低
 *		優先度に下がり，実行可能状態の中優先度タスク（TASK4）に切り換わ
 *		ることを確認する．また，実行可能状態の低優先度タスク（TASK2）よ
 *		りも，優先順位が高くなることを確認する．　
 *	(B-1)
 *		低優先度タスク（TASK1）が，中優先度上限ミューテックス（MTX1）を
 *		ロックした状態で，別のミューテックス（MTX4）待ち状態とし，高優
 *		先度タスク（TASK5）からMTX1を再初期化し，TASK1の優先度が低優先
 *		度に下がることを確認する．また，MTX4待ち状態の低優先度タスク
 *		（TASK2）よりも，待ち行列中での順序が後になることを確認する．
 *	(B-2)
 *		低優先度タスク（TASK1）が，中優先度上限ミューテックスを2つ
 *		（MTX1，MTX2）をロックした状態で，別のミューテックス（MTX4）待
 *		ち状態とし，高優先度タスク（TASK5）からMTX1を再初期化し，TASK1
 *		の優先度が変化しないことを確認する．また，MTX4待ち状態の中優先
 *		度タスクを2つ（TASK3，TASK4）を用意しておき，待ち行列中での順序
 *		が変わらないことを確認する．
 *	(B-3)
 *		低優先度タスク（TASK1）が，中優先度上限ミューテックス（MTX2）と
 *		低優先度上限ミューテックス（MTX3）をロックした状態で，別のミュー
 *		テックス（MTX4）待ち状態とし，高優先度タスク（TASK5）からMTX3を
 *		再初期化し，TASK1の優先度が変化しないことを確認する．また，
 *		MTX4待ち状態の中優先度タスクを2つ（TASK3，TASK4）を用意しておき，
 *		待ち行列中での順序が変わらないことを確認する．
 *	(C-1)
 *		低優先度タスク（TASK1）が中優先度上限ミューテックス（MTX1）をロッ
 *		クし，中優先度タスク（TASK3）がMTX1を待っている状態で，高優先度
 *		タスク（TASK5）からMTX1を再初期化し，TASK3が待ち解除されること
 *		を確認する．また，実行可能状態の中優先度タスク（TASK4）よりも，
 *		TASK3の方が優先順位が低くなることを確認する．　
 *	(C-2)
 *		低優先度タスク（TASK1）が中優先度上限ミューテックス（MTX1）をロッ
 *		クし，中優先度タスク（TASK3）がMTX1を待ち，TASK1を起床待ち状態
 *		とした状態で，別の低優先度タスク（TASK2）からMTX1を再初期化し，
 *		TASK3が待ち解除され，TASK3に切り換わることを確認する．
 *
 * 【使用リソース】
 *
 *	TASK1: 低優先度タスク，メインタスク，最初から起動
 *	TASK2: 低優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 中優先度タスク
 *	TASK5: 高優先度タスク
 *	MTX1: ミューテックス（TA_CEILING属性，上限は中優先度）
 *	MTX2: ミューテックス（TA_CEILING属性，上限は中優先度）
 *	MTX3: ミューテックス（TA_CEILING属性，上限は低優先度）
 *	MTX4: ミューテックス（TA_CEILING属性，上限は高優先度）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：低）==
 *		call(set_bit_func(bit_mutex))
 *	1:	loc_mtx(MTX1)
 *	2:	act_tsk(TASK2)
 *	3:	act_tsk(TASK5)
 *	//		高：TASK5，中：TASK1，低：TASK2，MTX1：TASK1
 *	== TASK5（優先度：高）==
 *	4:	ini_mtx(MTX1)			... (A-1)
 *	//		高：TASK5，低：TASK1→TASK2
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == LOW_PRIORITY)
 *	5:	slp_tsk()
 *	//		低：TASK1→TASK2
 *
 *	== TASK1（続き）==
 *	6:	sus_tsk(TASK2)
 *	7:	loc_mtx(MTX1)
 *		loc_mtx(MTX2)
 *	8:	act_tsk(TASK3)
 *	//		中：TASK1→TASK3，MTX1：TASK1，MTX2：TASK1
 *	9:	dis_dsp()
 *	10:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK3→TASK1，MTX1：TASK1，MTX2：TASK1
 *	11:	act_tsk(TASK4)
 *	//		中：TASK3→TASK1→TASK4，MTX1：TASK1，MTX2：TASK1
 *	12:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK3→TASK1→TASK4，MTX1：TASK1，MTX2：TASK1
 *	13:	ena_dsp()
 *	== TASK5（続き）==
 *	14:	ini_mtx(MTX1)			... (A-2)
 *	//		高：TASK5，中：TASK3→TASK1→TASK4，MTX2：TASK1
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	15:	slp_tsk()
 *	//		中：TASK3→TASK1→TASK4，MTX2：TASK1
 *	== TASK3（優先度：中）==
 *	16:	slp_tsk()
 *	//		中：TASK1→TASK4，MTX2：TASK1
 *	== TASK1（続き）==
 *	17:	slp_tsk()
 *	//		中：TASK4，MTX2：TASK1
 *	== TASK4（優先度：中）==
 *	18:	wup_tsk(TASK1)
 *	//		中：TASK4→TASK1，MTX2：TASK1
 *	19:	slp_tsk()
 *	//		中：TASK1，MTX2：TASK1
 *
 *	== TASK1（続き）==
 *	20:	loc_mtx(MTX3)
 *	//		中：TASK1，MTX2：TASK1，MTX3：TASK1
 *	21:	wup_tsk(TASK3)
 *	//		中：TASK1→TASK3，MTX2：TASK1，MTX3：TASK1
 *	22:	dis_dsp()
 *	23:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK3→TASK1，MTX2：TASK1，MTX3：TASK1
 *	24:	wup_tsk(TASK4)
 *	//		中：TASK3→TASK1→TASK4，MTX2：TASK1，MTX3：TASK1
 *	25:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK3→TASK1→TASK4，MTX2：TASK1，MTX3：TASK1
 *	26:	ena_dsp()
 *	== TASK5（続き）==
 *	27:	ini_mtx(MTX3)			... (A-3)
 *	//		高：TASK5，中：TASK3→TASK1→TASK4，MTX2：TASK1
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	28:	slp_tsk()
 *	//		中：TASK3→TASK1→TASK4，MTX2：TASK1
 *	== TASK3（続き）==
 *	29:	slp_tsk()
 *	//		中：TASK1→TASK4，MTX2：TASK1
 *
 *	== TASK1（続き）==
 *	30:	slp_tsk()
 *	//		中：TASK4，MTX2：TASK1
 *	== TASK4（続き）==
 *	31:	wup_tsk(TASK1)
 *	//		中：TASK4→TASK1，MTX2：TASK1
 *	32:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK1→TASK4，MTX2：TASK1
 *	== TASK1（続き）==
 *	33:	rsm_tsk(TASK2)
 *	//		中：TASK1→TASK4，低：TASK2，MTX2：TASK1
 *	34:	ini_mtx(MTX2)			... (A-4)
 *	//		中：TASK4，低：TASK1→TASK2
 *	== TASK4（続き）==
 *	35:	get_pri(TASK1, &tskpri)
 *		assert(tskpri == LOW_PRIORITY)
 *	36:	slp_tsk()
 *	//		低：TASK1→TASK2
 *
 *	== TASK1（続き）==
 *	37:	wup_tsk(TASK5)
 *	== TASK5（続き）==
 *	38:	loc_mtx(MTX4)
 *	39:	tslp_tsk(10) -> E_TMOUT
 *	//		低：TASK1→TASK2，MTX4：TASK5
 *	== TASK1（続き）==
 *	40:	loc_mtx(MTX1)
 *	//		中：TASK1，低：TASK2，MTX1：TASK1，MTX4：TASK5
 *	41:	loc_mtx(MTX4)
 *	//		低：TASK2，MTX1：TASK1，MTX4：TASK5→TASK1
 *	== TASK2（優先度：低）==
 *	42:	loc_mtx(MTX4)
 *	//		MTX1：TASK1，MTX4：TASK5→TASK1→TASK2
 *	//		タイムアウト待ち
 *	//		高：TASK5，MTX1：TASK1，MTX4：TASK5→TASK1→TASK2
 *	== TASK5（続き）==
 *	43:	ini_mtx(MTX1)			... (B-1)
 *	//		高：TASK5，MTX4：TASK5→TASK2→TASK1
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == LOW_PRIORITY)
 *	44:	unl_mtx(MTX4)
 *	//		高：TASK5→TASK2，MTX4：TASK2→TASK1
 *	45:	slp_tsk()
 *	//		高：TASK2，MTX4：TASK2→TASK1
 *	== TASK2（続き）==
 *	46:	unl_mtx(MTX4)
 *	//		高：TASK1，低：TASK2，MTX4：TASK1
 *	== TASK1（続き）==
 *	47:	unl_mtx(MTX4)
 *	//		低：TASK1→TASK2
 *
 *	48:	wup_tsk(TASK5)
 *	== TASK5（続き）==
 *	49:	loc_mtx(MTX4)
 *	50:	slp_tsk()
 *	//		低：TASK1→TASK2，MTX4：TASK5
 *	== TASK1（続き）==
 *	51:	wup_tsk(TASK3)
 *	//		中：TASK3，低：TASK1→TASK2，MTX4：TASK5
 *	== TASK3（続き）==
 *	52:	loc_mtx(MTX4)
 *	//		低：TASK1→TASK2，MTX4：TASK5→TASK3
 *	== TASK1（続き）==
 *	53:	loc_mtx(MTX1)
 *		loc_mtx(MTX2)
 *	//		中：TASK1，低：TASK2，MTX1：TASK1，MTX2：TASK1，MTX4：TASK5→TASK3
 *	54:	loc_mtx(MTX4)
 *	//		低：TASK2，MTX1：TASK1，MTX2：TASK1，MTX4：TASK5→TASK3→TASK1
 *	== TASK2（続き）==
 *	55:	wup_tsk(TASK4)
 *	== TASK4（続き）==
 *	56:	loc_mtx(MTX4)
 *	//		低：TASK2，MTX1：TASK1，MTX2：TASK1，
 *	//								MTX4：TASK5→TASK3→TASK1→TASK4
 *	== TASK2（続き）==
 *	57:	wup_tsk(TASK5)
 *	//		高：TASK5，低：TASK2，MTX1：TASK1，MTX2：TASK1，
 *	//								MTX4：TASK5→TASK3→TASK1→TASK4
 *	== TASK5（続き）==
 *	58:	ini_mtx(MTX1)			... (B-2)
 *	//		高：TASK5，低：TASK2，MTX2：TASK1，MTX4：TASK5→TASK3→TASK1→TASK4
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	59:	unl_mtx(MTX4)
 *	//		高：TASK5→TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK3→TASK1→TASK4
 *	60:	slp_tsk()
 *	//		高：TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK3→TASK1→TASK4
 *	== TASK3（続き）==
 *	61:	unl_mtx(MTX4)
 *	//		高：TASK1，中：TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK1→TASK4
 *	== TASK1（続き）==
 *	62:	unl_mtx(MTX4)
 *	//		高：TASK4，中：TASK1→TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK4
 *	== TASK4（続き）==
 *	63:	unl_mtx(MTX4)
 *	//		中：TASK4→TASK1→TASK3，低：TASK2，MTX2：TASK1
 *	64:	slp_tsk()
 *	//		中：TASK1→TASK3，低：TASK2，MTX2：TASK1
 *	== TASK1（続き）==
 *	65:	slp_tsk()
 *	//		中：TASK3，低：TASK2，MTX2：TASK1
 *	== TASK3（続き）==
 *	66:	slp_tsk()
 *	//		低：TASK2，MTX2：TASK1
 *
 *	== TASK2（続き）==
 *	67: wup_tsk(TASK5)
 *	== TASK5（続き）==
 *	68:	loc_mtx(MTX4)
 *	69:	slp_tsk()
 *	//		低：TASK2，MTX4：TASK5
 *	== TASK2（続き）==
 *	70:	wup_tsk(TASK3)
 *	== TASK3（続き）==
 *	71:	loc_mtx(MTX4)
 *	//		低：TASK2，MTX4：TASK5→TASK3
 *	== TASK2（続き）==
 *	72:	wup_tsk(TASK1)
 *	//		中：TASK1，低：TASK2，MTX4：TASK5
 *	== TASK1（続き）==
 *	73:	loc_mtx(MTX3)
 *	//		中：TASK1，低：TASK2，MTX2：TASK1，MTX3：TASK1，MTX4：TASK5→TASK3
 *	74:	loc_mtx(MTX4)
 *	//		低：TASK2，MTX2：TASK1，MTX3：TASK1，MTX4：TASK5→TASK3→TASK1
 *	== TASK2（続き）==
 *	75:	wup_tsk(TASK4)
 *	== TASK4（続き）==
 *	76:	loc_mtx(MTX4)
 *	//		低：TASK2，MTX2：TASK1，MTX3：TASK1，
 *	//								MTX4：TASK5→TASK3→TASK1→TASK4
 *	== TASK2（続き）==
 *	77:	wup_tsk(TASK5)
 *	//		高：TASK5，低：TASK2，MTX2：TASK1，MTX3：TASK1，
 *	//								MTX4：TASK5→TASK3→TASK1→TASK4
 *	== TASK5（続き）==
 *	78:	ini_mtx(MTX3)			... (B-3)
 *	//		高：TASK5，低：TASK2，MTX2：TASK1，MTX4：TASK5→TASK3→TASK1→TASK4
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	79:	unl_mtx(MTX4)
 *	//		高：TASK5→TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK3→TASK1→TASK4
 *	80:	slp_tsk()
 *	//		高：TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK3→TASK1→TASK4
 *	== TASK3（続き）==
 *	81:	unl_mtx(MTX4)
 *	//		高：TASK1，中：TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK1→TASK4
 *	== TASK1（続き）==
 *	82:	unl_mtx(MTX4)
 *	//		高：TASK4，中：TASK1→TASK3，低：TASK2，MTX2：TASK1，MTX4：TASK4
 *	== TASK4（続き）==
 *	83:	unl_mtx(MTX4)
 *	//		中：TASK4→TASK1→TASK3，低：TASK2，MTX2：TASK1
 *	84:	slp_tsk()
 *	//		中：TASK1→TASK3，低：TASK2，MTX2：TASK1
 *	== TASK1（続き）==
 *	85:	unl_mtx(MTX2)
 *	//		中：TASK3，低：TASK1→TASK2
 *	== TASK3（続き）==
 *	86:	slp_tsk()
 *	//		低：TASK1→TASK2
 *
 *	== TASK1（続き）==
 *	87:	sus_tsk(TASK2)
 *	//		低：TASK1
 *	88:	loc_mtx(MTX1)
 *	//		中：TASK1，MTX1：TASK1
 *	89:	wup_tsk(TASK3)
 *	//		中：TASK1→TASK3，MTX1：TASK1
 *	90:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK3→TASK1，MTX1：TASK1
 *	== TASK3（続き）==
 *	91:	loc_mtx(MTX1) -> E_DLT
 *	//		中：TASK1，MTX1：TASK1→TASK3
 *	== TASK1（続き）==
 *	92:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK1，MTX1：TASK1→TASK3
 *	== TASK5（続き）==
 *	93:	wup_tsk(TASK4)
 *	//		高：TASK5，中：TASK1→TASK4，MTX1：TASK1→TASK3
 *	94:	ini_mtx(MTX1)			... (C-1)
 *	//		高：TASK5，中：TASK4→TASK3，低：TASK1
 *	95:	ext_tsk() -> noreturn
 *	//		中：TASK4→TASK3，低：TASK1
 *	== TASK4（続き）==
 *	96:	ext_tsk() -> noreturn
 *	//		中：TASK3，低：TASK1
 *	== TASK3（続き）==
 *	97:	slp_tsk()
 *	//		低：TASK1
 *
 *	== TASK1（続き）==
 *	98:	loc_mtx(MTX1)
 *	//		中：TASK1，MTX1：TASK1
 *	99:	wup_tsk(TASK3)
 *	//		中：TASK1→TASK3，MTX1：TASK1
 * 100:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK3→TASK1，MTX1：TASK1
 *	== TASK3（続き）==
 * 101:	loc_mtx(MTX1) -> E_DLT
 *	//		中：TASK1，MTX1：TASK1→TASK3
 *	== TASK1（続き）==
 * 102:	rsm_tsk(TASK2)
 *	//		中：TASK1，低：TASK2，MTX1：TASK1→TASK3
 * 103:	slp_tsk()
 *	//		低：TASK2，MTX1：TASK1→TASK3
 *	== TASK2（続き）==
 * 104:	ini_mtx(MTX1)			... (C-2)
 *	//		中：TASK3，低：TASK2
 *	== TASK3（続き）==
 * 105:	ext_tsk() -> noreturn
 *	//		低：TASK2
 *	== TASK2（続き）==
 * 106:	ter_tsk(TASK1)
 * 107:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_mutex5.h"

extern ER	bit_mutex(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	set_bit_func(bit_mutex);

	check_point(1);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(3);
	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = sus_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = loc_mtx(MTX3);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(23);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = rsm_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(34);
	ercd = ini_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(37);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(41);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(47);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(48);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(51);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(53);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(54);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(62);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(65);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(73);
	ercd = loc_mtx(MTX3);
	check_ercd(ercd, E_OK);

	check_point(74);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(82);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(85);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(87);
	ercd = sus_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(88);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(89);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(90);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(92);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(98);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(99);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(100);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(102);
	ercd = rsm_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(103);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(42);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(46);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(55);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(57);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(67);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(70);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(72);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(75);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(77);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(104);
	ercd = ini_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(106);
	ercd = ter_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_finish(107);
	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(16);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(52);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(61);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(66);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(71);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(81);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(86);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(91);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_DLT);

	check_point(97);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(101);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_DLT);

	check_point(105);
	ercd = ext_tsk();

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;

	check_point(18);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == LOW_PRIORITY);

	check_point(36);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(56);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(63);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(64);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(76);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(83);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(84);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(96);
	ercd = ext_tsk();

	check_point(0);
}

void
task5(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;

	check_point(4);
	ercd = ini_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == LOW_PRIORITY);

	check_point(5);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = ini_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(15);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = ini_mtx(MTX3);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(28);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(39);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(43);
	ercd = ini_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == LOW_PRIORITY);

	check_point(44);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(45);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(49);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(50);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(58);
	ercd = ini_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(59);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(60);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(68);
	ercd = loc_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(69);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(78);
	ercd = ini_mtx(MTX3);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(79);
	ercd = unl_mtx(MTX4);
	check_ercd(ercd, E_OK);

	check_point(80);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(93);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(94);
	ercd = ini_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(95);
	ercd = ext_tsk();

	check_point(0);
}
