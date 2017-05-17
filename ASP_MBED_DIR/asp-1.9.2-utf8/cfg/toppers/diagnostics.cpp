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
#include <cstdio>
#include <string>
#include "toppers/diagnostics.hpp"
#include "toppers/text_line.hpp"

namespace toppers
{
  namespace
  {
    int error_abort_threshold = 100;
    int error_count = 0;
    std::string program_name( "(unknown)" );
    std::string error_location;
  }

  int get_error_count()
  {
    return error_count;
  }

  int increment_error_count()
  {
    return ++error_count;
  }

  void set_program_name( char const* name )
  {
    program_name = name;
  }

  std::string const& get_program_name()
  {
    return program_name;
  }

  int set_error_abort_threshold( int thresh )
  {
    if ( thresh < 1 )
    {
      return -1;
    }
    int previous = error_abort_threshold;
    error_abort_threshold = thresh;
    return previous;
  }

  void warning( const char* msg )
  {
    if ( !error_location.empty() ) fprintf( stderr, "In function `%s`:\n", error_location.c_str() );
    fprintf( stderr, "%s: %s: %s\n", program_name.c_str(), _( "warning" ), msg );
    error_location.clear();
  }
  void warning( text_line const& line, const char* msg )
  {
    if ( !error_location.empty() ) fprintf( stderr, "In function `%s`:\n", error_location.c_str() );
    fprintf( stderr, "%s:%s:%ld: %s: %s\n", program_name.c_str(), line.file.c_str(), line.line, _( "warning" ), msg );
    error_location.clear();
  }
  void error( const char* msg )
  {
    if ( !error_location.empty() ) fprintf( stderr, "In function `%s`:\n", error_location.c_str() );
    fprintf( stderr, "%s: %s: %s\n", program_name.c_str(), _( "error" ), msg );
    ++error_count;
    if ( error_abort_threshold <= error_count )
    {
      error_location.clear();
      throw diagnostics_error( _( "too many errors" ) );
    }
  }
  void error( text_line const& line, const char* msg )
  {
    if ( !error_location.empty() ) fprintf( stderr, "In function `%s`:\n", error_location.c_str() );
    fprintf( stderr, "%s:%s:%ld: %s: %s\n", program_name.c_str(), line.file.c_str(), line.line, _( "error" ), msg );
    ++error_count;
    if ( error_abort_threshold <= error_count )
    {
      error_location.clear();
      throw diagnostics_error( _( "too many errors" ) );
    }
  }
  void fatal( const char* msg )
  {
    if ( !error_location.empty() ) fprintf( stderr, "In function `%s`:\n", error_location.c_str() );
    fprintf( stderr, "%s: %s: %s\n", program_name.c_str(), _( "error" ), msg );
    error_location.clear();
    throw diagnostics_error( _( "fatal error" ) );
  }
  void fatal( text_line const& line, const char* msg )
  {
    if ( !error_location.empty() ) fprintf( stderr, "In function `%s`:\n", error_location.c_str() );
    fprintf( stderr, "%s:%s:%ld: %s: %s\n", program_name.c_str(), line.file.c_str(), line.line, _( "error" ), msg );
    error_location.clear();
    throw diagnostics_error( _( "fatal error" ) );
  }

  void set_error_location( char const* msg )
  {
    error_location = msg;
  }

  char const* get_error_location()
  {
    return error_location.c_str();
  }
}
