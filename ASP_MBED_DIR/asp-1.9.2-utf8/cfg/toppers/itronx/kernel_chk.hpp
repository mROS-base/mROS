/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007 by TAKAGI Nobuhisa
 * 
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
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
 */
#undef  BEGIN_KERNEL_CHK
#undef  END_KERNEL_CHK
#undef  KERNEL_CHK_SIZEOF
#undef  KERNEL_CHK_OFFSETOF
#undef  KERNEL_CHK_VALUEOF

#ifdef IMPLEMENT_KERNEL_CHK

#define BEGIN_KERNEL_CHK            ofile_ << "\t0x12345678,\n\tTKERNEL_PRID,\n"  \
                                              "\tCHAR_BIT,\n" \
                                              "\tCHAR_MAX,\n" \
                                              "\tSCHAR_MAX,\n" \
                                              "\tSHRT_MAX,\n" \
                                              "\tINT_MAX,\n" \
                                              "\tLONG_MAX,\n"
#define END_KERNEL_CHK              ;
#define KERNEL_CHK_SIZEOF( sym )    KERNEL_CHK_SIZEOF_( sym )
#define KERNEL_CHK_SIZEOF_( sym )   "\tsizeof(" #sym "),\n"
#define KERNEL_CHK_OFFSETOF( type, member )  "\toffsetof(" #type ", " #member "),\n"
#define KERNEL_CHK_VALUEOF( expr )  "\t" #expr ",\n"

#else

#define BEGIN_KERNEL_CHK            enum { magic_number, TKERNEL_PRID, \
                                           valueof_CHAR_BIT, \
                                           valueof_CHAR_MAX, \
                                           valueof_SCHAR_MAX, \
                                           valueof_SHRT_MAX, \
                                           valueof_INT_MAX, \
                                           valueof_LONG_MAX,
#define END_KERNEL_CHK              kernel_chk_table_size };
#define KERNEL_CHK_SIZEOF( sym )    sizeof_##sym,
#define KERNEL_CHK_OFFSETOF( type, member )  offsetof_##type##_##member,
#define KERNEL_CHK_VALUEOF( expr )  valueof_##expr,

#endif
