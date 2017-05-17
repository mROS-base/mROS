/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2004 by Witz Corporation, JAPAN    
 *  Copyright (C) 2005-2008 by TAKAGI Nobuhisa
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

/*!
 *  \file   toppers/config.hpp
 *  \brief  処理系に依存する設定
 */
#ifndef	TOPPERS_CONFIG_HPP_
#define	TOPPERS_CONFIG_HPP_

#ifdef _MSC_VER

# define for			if(0);else for

# pragma warning( disable: 4127 )
# pragma warning( disable: 4239 )
# pragma warning( disable: 4505 )
# pragma warning( disable: 4511 )
# pragma warning( disable: 4512 )
# pragma warning( disable: 4630 )
# pragma warning( disable: 4701 )
# pragma warning( disable: 4786 )
# pragma warning( disable: 4819 )
# pragma warning( disable: 4996 )

#define _SCL_SECURE_NO_WARNINGS   1

#endif	// _MSC_VER

#ifdef __BORLANDC__

#pragma warn -8026
#pragma warn -8027

#endif  // __BORLANDC__

#ifdef __GNUC__

# if __GNUC__ > 4 || ( __GNUC__ == 4 && __GNUC_MINOR__ >= 1 )
#   if __GNUC__ == 4 && __GNUC_MINOR__ < 3
//#     define TOPPERS_HAS_TR1_LIBRARY	1
#   endif
# endif

# define TOPPERS_HAS_ICONV	1

#endif	// __GNUC__

#if defined(unix) || defined(__unix) || defined(__unix__)

# define TOPPERS_HAS_ICONV	1

#endif	// UNIX

#endif	// ! TOPPERS_CONFIG_HPP_
