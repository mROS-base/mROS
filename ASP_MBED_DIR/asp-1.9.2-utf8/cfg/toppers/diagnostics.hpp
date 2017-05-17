/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2012 by TAKAGI Nobuhisa
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
 *  \file   toppers/diagnostics.hpp
 *  \brief  診断処理に関する宣言定義
 */
#ifndef TOPPERS_DIAGNOSTICS_HPP_
#define TOPPERS_DIAGNOSTICS_HPP_

#include <stdexcept>
#include "toppers/debug.hpp"
#include "toppers/gettext.hpp"
#include <boost/format.hpp>

namespace toppers
{

  struct text_line;

  class diagnostics_error : public std::runtime_error
  {
  public:
    diagnostics_error( std::string const& msg ) : std::runtime_error( msg ) {}
  };

  class normal_exit {};

  int get_error_count();
  int increment_error_count();
  void set_program_name( char const* name );
  std::string const& get_program_name();
  int set_error_abort_threshold( int thresh );
  void warning( char const* msg );
  void warning( text_line const& line, char const* msg );
  void error( char const* msg );
  void error( text_line const& line, char const* msg );
  void fatal( char const* msg );
  void fatal( text_line const& line, char const* msg );
  void set_error_location( char const* msg );
  char const* get_error_location();

  template < typename T1 >
    inline void warning( char const* str, T1 const& arg1 )
  {
    warning( ( boost::format( str ) % arg1 ).str().c_str() );
  }

  template < typename T1, typename T2 >
    inline void warning( char const* str, T1 const& arg1, T2 const& arg2 )
  {
    warning( ( boost::format( str ) % arg1 % arg2 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3 >
    inline void warning( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3 )
  {
    warning( ( boost::format( str ) % arg1 % arg2 % arg3 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4 >
    inline void warning( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4 )
  {
    warning( ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 ).str().c_str() );
  }

  template < typename T1 >
    inline void warning( text_line const& line, char const* str, T1 const& arg1 )
  {
    warning( line, ( boost::format( str ) % arg1 ).str().c_str() );
  }

  template < typename T1, typename T2 >
    inline void warning( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2 )
  {
    warning( line, ( boost::format( str ) % arg1 % arg2 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3 >
    inline void warning( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3 )
  {
    warning( line, ( boost::format( str ) % arg1 % arg2 % arg3 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4 >
    inline void warning( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4 )
  {
    warning( line, ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4, typename T5 >
    inline void warning( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4, T5 const& arg5 )
  {
    warning( line, ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 % arg5 ).str().c_str() );
  }

  template < typename T1 >
    inline void error( char const* str, T1 const& arg1 )
  {
    error( ( boost::format( str ) % arg1 ).str().c_str() );
  }

  template < typename T1, typename T2 >
    inline void error( char const* str, T1 const& arg1, T2 const& arg2 )
  {
    error( ( boost::format( str ) % arg1 % arg2 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3 >
    inline void error( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3 )
  {
    error( ( boost::format( str ) % arg1 % arg2 % arg3 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4 >
    inline void error( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4 )
  {
    error( ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4, typename T5 >
    inline void error( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4, T5 const& arg5 )
  {
    error( ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 % arg5 ).str().c_str() );
  }

  template < typename T1 >
    inline void error( text_line const& line, char const* str, T1 const& arg1 )
  {
    error( line, ( boost::format( str ) % arg1 ).str().c_str() );
  }

  template < typename T1, typename T2 >
    inline void error( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2 )
  {
    error( line, ( boost::format( str ) % arg1 % arg2 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3 >
    inline void error( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3 )
  {
    error( line, ( boost::format( str ) % arg1 % arg2 % arg3 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4 >
    inline void error( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4 )
  {
    error( line, ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4, typename T5 >
    inline void error( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4, T5 const& arg5 )
  {
    error( line, ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 % arg5 ).str().c_str() );
  }

  template < typename T1 >
    inline void fatal( char const* str, T1 const& arg1 )
  {
    fatal( ( boost::format( str ) % arg1 ).str().c_str() );
  }

  template < typename T1, typename T2 >
    inline void fatal( char const* str, T1 const& arg1, T2 const& arg2 )
  {
    fatal( ( boost::format( str ) % arg1 % arg2 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3 >
    inline void fatal( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3 )
  {
    fatal( ( boost::format( str ) % arg1 % arg2 % arg3 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4 >
    inline void fatal( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4 )
  {
    fatal( ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4, typename T5 >
    inline void fatal( char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4, T5 const& arg5 )
  {
    fatal( ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 % arg5 ).str().c_str() );
  }

  template < typename T1 >
    inline void fatal( text_line const& line, char const* str, T1 const& arg1 )
  {
    fatal( line, ( boost::format( str ) % arg1 ).str().c_str() );
  }

  template < typename T1, typename T2 >
    inline void fatal( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2 )
  {
    fatal( line, ( boost::format( str ) % arg1 % arg2 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3 >
    inline void fatal( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3 )
  {
    fatal( line, ( boost::format( str ) % arg1 % arg2 % arg3 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4 >
    inline void fatal( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4 )
  {
    fatal( line, ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 ).str().c_str() );
  }

  template < typename T1, typename T2, typename T3, typename T4, typename T5 >
    inline void fatal( text_line const& line, char const* str, T1 const& arg1, T2 const& arg2, T3 const& arg3, T4 const& arg4, T5 const& arg5 )
  {
    fatal( line, ( boost::format( str ) % arg1 % arg2 % arg3 % arg4 % arg5 ).str().c_str() );
  }

  inline void exit()
  {
    throw normal_exit();
  }

#undef  _
#define _( str )  ::toppers::gettext( str ).c_str()

}

#endif  // ! TOPPERS_DIAGNOSTICS_HPP_

