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
#include <cstdlib>
#include <cctype>
#include <cerrno>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>
#include "toppers/macro_processor.hpp"
#include "toppers/diagnostics.hpp"
#include "toppers/gettext.hpp"
#include "toppers/cpp.hpp"
#include <boost/format.hpp>
#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/xpressive/xpressive.hpp>
#include <boost/algorithm/string.hpp> 

namespace toppers
{
  namespace
  {
    typedef macro_processor::element element;
    typedef macro_processor::var_t var_t;
    typedef macro_processor::context context;

    inline std::tr1::int64_t get_i( var_t const& var, context const* p_ctx )
    {
      return macro_processor::to_integer( var, p_ctx );
    }
    inline std::string get_s( var_t const& var, context const* p_ctx )
    {
      return macro_processor::to_string( var, p_ctx );
    }

  }

  /*!
    *  \brief  引数の個数チェック
    *  \param[in]  line    行番号情報
    *  \param[in]  arity   引数の個数
    *  \param[in]  valid   期待している引数の個数
    *  \param[in]  function_name 組み込み関数名
    */
  bool macro_processor::check_arity( text_line const& line, std::size_t arity, std::size_t valid, char const* function_name )
  {
    bool result = false;
    if ( arity < valid )
    {
      error( line, _( "too few arguments for `%1%\'" ), function_name );
    }
    else if ( arity > valid )
    {
      error( line, _( "too many arguments for `%1%\'" ), function_name );
    }
    else
    {
      result = true;
    }
    return result;
  }

  /*!
   *  \brief  変数ダンプのための<<演算子
   *  \param[in,out]  ostr  出力ストリーム
   *  \param[in]      arg   変数を参照するためのペア
   *  \return         ostrを返す
   *  \note 現在の実装では、arg.second は使用していない。
   */
  std::ostream& operator<<( std::ostream& ostr, std::pair< var_t const*, context const* > const& arg )
  {
    for ( var_t::const_iterator iter( arg.first->begin() ), last( arg.first->end() ); iter != last; ++iter )
    {
      if ( !iter->s.empty() )
      {
        ostr << iter->s;
      }
      else if ( iter->i )
      {
        ostr << iter->i.get();
      }
      if ( boost::next( iter ) != last )
      {
        ostr << ",";
      }
    }
    return ostr;
  }

  /*!
   *  \brief  順序リストの長さ
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数として指定した順序付きリストの要素数を返す。
   *  第1マクロ実引数が順序付きリストでない場合は1を返す。また、第1マクロ実引数が無効な変数の場合は0を返す。
   */
  var_t bf_length( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "LENGTH" ) )
    {
      std::tr1::int64_t size = arg_list.front().size();
      e.i = size;
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  文字列の一致判定
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数と第2マクロ実引数を文字列として比較し、一致する場合は真を、そうでなければ偽を返す。
   */
  var_t bf_eq( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "EQ" ) )
    {
      e.i = get_s( arg_list[ 0 ], p_ctx ) == get_s( arg_list[ 1 ], p_ctx );
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  代替値
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数が無効な変数の場合は第2実引数を返す。その他は第1実引数を返す。
   */
  var_t bf_alt( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "ALT" ) )
    {
      if ( !arg_list[0].empty() )
      {
        return arg_list[ 0 ];
      }
      else
      {
        return arg_list[ 1 ];                
      }
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  順序リストの整列
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数として与えた順序付きリストの各要素を、第2マクロ実引数の添え字とした場合の変数を評価し、
   *  その評価結果に基づき昇順に整列する。
   *
   *  \example
   *  $FOO[1] = 20$
   *  $FOO[2] = 10$
   *  $FOO[3] = 30$
   *  $SORT({ 1,2,3 }, "FOO")$
   *  → { 2,1,3 }
   *  \endexample
   */
  var_t bf_sort( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    var_t result;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "SORT" ) )
    {
      var_t list( arg_list[ 0 ] );
      std::string field( get_s( arg_list[ 1 ], p_ctx ) );
      std::vector< std::pair< element, std::tr1::int64_t > > temp;

      for ( var_t::const_iterator iter( list.begin() ), last( list.end() ); iter != last; ++iter )
      {
        std::tr1::int64_t order = iter->i.get();
        std::string name( ( boost::format( "%s[%d]" ) % field % order ).str() );
        std::map< std::string, var_t >::const_iterator m_iter( p_ctx->var_map.find( name ) );
        if ( m_iter == p_ctx->var_map.end() )
        {
          return var_t();
        }
        if ( !m_iter->second.empty() )
        {
          temp.push_back( std::make_pair( m_iter->second.front(), order ) );
        }
      }

      std::stable_sort( temp.begin(), temp.end() );

      for ( std::vector< std::pair< element, std::tr1::int64_t > >::const_iterator iter( temp.begin() ), last( temp.end() );
            iter != last;
            ++iter )
      {
        element e;
        e.i = iter->second;
        result.push_back( e );
      }
    }
    return result;
  }

  /*!
   *  \brief  環境変数の取得
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した環境変数の値を返す。
   */
  var_t bf_environ( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "ENVIRON" ) )
    {
      std::string name = get_s( arg_list[ 0 ], p_ctx );
      char const* env = std::getenv( name.c_str() );
      if ( env == 0 )
      {
        return var_t();
      }
      e.s = env;
      errno = 0;
      char* endptr;
      if ( std::tr1::int64_t value = std::strtol( env, &endptr, 0 ) )
      {
        if ( *endptr == '\0' && errno == 0 )
        {
          e.i = value;
        }
      }
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  値の生成
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数をテキスト、第2マクロ実引数を数値として、値を生成する。
   */
  var_t bf_value( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "VALUE" ) )
    {
      if ( !arg_list[0].empty() )
      {
        e.s = get_s( arg_list[ 0 ], p_ctx );
      }
      if ( !arg_list[1].empty() )
      {
        e.i = get_i( arg_list[ 1 ], p_ctx );
      }
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  文字列の連結
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数と第2マクロ実引数を連結して新しい文字列を生成する。
   */
  var_t bf_concat( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "CONCAT" ) )
    {
      e.s = get_s( arg_list[ 0 ], p_ctx ) + get_s( arg_list[ 1 ], p_ctx );
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  順序リストの終端に要素を追加
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数と第2マクロ実引数以降を連結して新しい順序付きリストを生成する。
   */
  var_t bf_append( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    var_t result;
    var_t::size_type const n = arg_list.size();
    if ( n < 2 )
    {
      error( line, _( "too few arguments for `%1%\'" ), "APPEND" );
        }
        else
        {
          result = arg_list[ 0 ];
      for ( var_t::size_type i = 1; i < n; i++)
            {
              result.insert( result.end(), arg_list[ i ].begin(), arg_list[ i ].end() );
            }
        }
    return result;
  }

  /*!
   *  \brief  順序リストの指定要素の参照
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した順序リストの、第2マクロ実引数で指定した要素を返す。
   */
  var_t bf_at( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "AT" ) )
    {
      try
      {
        e = arg_list[ 0 ].at( static_cast< std::vector< var_t >::size_type >( get_i( arg_list[1], p_ctx ) ) );
      }
      catch ( std::out_of_range& )
      {
        // 添え字が不正
        // 特に何もしない → この時点で e が空値であることを期待
      }
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  テキストの翻訳
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した文字列を翻訳する。
   */
  var_t bf_gettext( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "GETTEXT" ) )
    {
      std::string message = get_s( arg_list[ 0 ], p_ctx );
      e.s = gettext( message );
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  マクロ実引数の書式化
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した初期化文字列によって、第2マクロ実引数以降を書式化する。
   *  書式化文字列は、%nが使えないことを除き、printf関数のスーパーセットである。
   *  正確な仕様は、boost::formatを参照のこと。
   */
  var_t bf_format( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    std::size_t arity = arg_list.size();
    if ( arity < 1 )
    {
      error( line, _( "too few arguments for `%1%\'" ), "FORMAT" );
    }
    std::string format_str = get_s( arg_list[ 0 ], p_ctx );
#if 0
    std::string debug_str = format_str;
    if ( debug_str == "0x%08x" )
      toppers::trace("%s", debug_str.c_str() );
#endif
    boost::format fmt( format_str );
    try
    {
      for ( std::size_t i = 1; i < arity; i++ )
      {
        std::pair< var_t const*, context const* > arg( &arg_list[i], p_ctx );
        fmt % arg;
      }
      e.s = fmt.str();
    }
    catch ( ... )
    {
      error( line, _( "illegal argument value in `%1%\'" ), "FORMAT" );
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  順序付きリスト内の探索
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した順序付きリストに含まれる第2マクロ実引数で指定した値に等しい要素を、
   *  先頭から順に探索する。
   *  等しい要素が見つかればその要素へのインデックスを、そうでなければ空値を返す。
   */
  var_t bf_find( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "FIND" ) )
    {
      var_t list( arg_list[ 0 ] );

      if ( !arg_list[ 1 ].empty() )
      {
        element key( arg_list[ 1 ].front() );

        if ( !key.i )   // 整数値が設定されていなければ...
        {
          std::string value( key.s );

          for ( var_t::const_iterator iter( list.begin() ), last( list.end() ); iter != last; ++iter )
          {
            if ( iter->s == value ) // 発見！
            {
              e.i = iter - list.begin();  // iter は RandomAccessIterator
              return var_t( 1, e );
            }
          }
        }
        else
        {
          std::tr1::int64_t value( key.i.get() );

          for ( var_t::const_iterator iter( list.begin() ), last( list.end() ); iter != last; ++iter )
          {
            if ( iter->i && iter->i.get() == value ) // 発見！
            {
              e.i = iter - list.begin();  // iter は RandomAccessIterator
              return var_t( 1, e );
            }
          }
        }
      }
    }
    return var_t();
  }

  /*!
   *  \brief  範囲指定による順序付きリスト
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で最初の値を、第2マクロ実引数で最後の値を指定する。
   *  { 最初の値, 最初の値 + 1, ... 最後の値 }
   *  となる順序付きリストを生成する。
   *  引数が正しくない場合は空値を返す。
   */
  var_t bf_range( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    var_t result;
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "RANGE" ) )
    {
      std::tr1::int64_t arg1( get_i( arg_list[ 0 ], p_ctx ) );
      std::tr1::int64_t arg2( get_i( arg_list[ 1 ], p_ctx ) );

      for ( ; arg1 <= arg2; ++arg1 )
      {
        element e;
        e.i = arg1;
        result.push_back( e );
      }
    }
    return result;
  }

  /*!
   *  \brief  全変数のダンプ
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  マクロ実引数を指定した場合、その文字列属性で指定したファイルにダンプした文字列を追記する。
   *  ファイル名として、"stdout"を指定した場合は標準出力、"stderr"を指定した場合は標準エラーに出力する。
   *  ファイル名を省略した場合は"stderr"を指定したものとして振舞う。
   */
  var_t bf_dump( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    std::size_t arity = arg_list.size();

    if ( arity > 1 )
    {
      error( line, _( "too many arguments for `%1%\'" ), "DUMP" );
    }

    std::string dump_str;

    // 全変数をなめる
    for ( std::map< std::string, var_t >::const_iterator iter( p_ctx->var_map.begin() ), last( p_ctx->var_map.end() );
          iter != last;
          ++iter )
    {
      dump_str += "$" + iter->first + "$ = { ";
      if ( !iter->second.empty() )
      {
        // 各変数の全要素
        for ( var_t::const_iterator iter2( iter->second.begin() ), last2( iter->second.end() );
              iter2 != last2;
              ++iter2 )
        {
          dump_str += "\"" + iter2->s + "\"(";
          if ( iter2->i ) // 値属性があれば...
          {
            dump_str += boost::lexical_cast< std::string >( *iter2->i );
          }
          dump_str += "), ";
        }
      }
      dump_str += " }\n";
    }

    std::string filename( "stderr" );
    if ( arity == 1 )
    {
      filename = get_s( arg_list[ 0 ], p_ctx );
    }
    if ( filename == "stdout" )
    {
      std::fputs( dump_str.c_str(), stdout );
      std::fflush( stdout );
    }
    else if ( filename == "stderr" )
    {
      std::fputs( dump_str.c_str(), stderr );
      std::fflush( stderr );
    }
    else
    {
      std::FILE* stream = std::fopen( filename.c_str(), "a" );
      if ( stream != 0 )
      {
        std::fputs( dump_str.c_str(), stream );
        std::fclose( stream );
      }
    }
    element e;
    return var_t( 1, e );
  }

  /*!
   *  \brief  変数のトレース
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した変数の内容をトレースする。
   *  第2マクロ実引数を指定した場合、その文字列属性で指定したファイルにトレース内容を追記する。
   *  ファイル名として、"stdout"を指定した場合は標準出力、"stderr"を指定した場合は標準エラーに出力する。
   *  ファイル名を省略した場合は"stderr"を指定したものとして振舞う。
   */
  var_t bf_trace( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    std::size_t arity = arg_list.size();

    if ( arity < 1 )
    {
      error( line, _( "too few arguments for `%1%\'" ), "TRACE" );
    }
    else if ( arity > 2 )
    {
      error( line, _( "too many arguments for `%1%\'" ), "TRACE" );
    }

    var_t value( arg_list[ 0 ] );

    std::string trace_str = "{ ";
    for ( var_t::const_iterator iter( value.begin() ), last( value.end() );
          iter != last;
          ++iter )
    {
      trace_str += "\"" + iter->s + "\"(";
      if ( iter->i ) // 値属性があれば...
      {
        trace_str += boost::lexical_cast< std::string >( *iter->i ) + " as integer";
      }
      else if ( !iter->v.empty() )
      {
        trace_str += "\"" + boost::lexical_cast< std::string >( iter->v ) + "\" as string";
      }
      trace_str += "), ";
    }
    trace_str += " }\n";

    std::string filename( "stderr" );
    if ( arity == 2 )
    {
      filename = get_s( arg_list[ 1 ], p_ctx );
    }
    if ( filename == "stdout" )
    {
      std::fputs( trace_str.c_str(), stdout );
      std::fflush( stdout );
    }
    else if ( filename == "stderr" )
    {
      std::fputs( trace_str.c_str(), stderr );
      std::fflush( stderr );
    }
    else
    {
      std::FILE* stream = std::fopen( filename.c_str(), "a" );
      if ( stream != 0 )
      {
        std::fputs( trace_str.c_str(), stream );
        std::fclose( stream );
      }
    }

    element e;
    return var_t( 1, e );
  }

  /*!
   *  \brief  文字列のエスケープ
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   */
  var_t bf_escstr( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "ESCSTR" ) )
    {
      std::string str( get_s( arg_list[ 0 ], p_ctx ) );
      e.s = quote_string( str );
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  文字列のエスケープ解除
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   */
  var_t bf_unescstr( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "UNESCSTR" ) )
    {
      std::string str( get_s( arg_list[ 0 ], p_ctx ) );
      if ( !str.empty() )
      {
        e.s = expand_quote( str );
      }
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  関数の呼び出し
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   */
  var_t bf_call( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    return macro_processor::call_user_function( line, arg_list, p_ctx );
  }

  namespace
  {
    struct bf_functor : std::binary_function< element const&, element const&, bool >
    {
    public:
      bf_functor( text_line const& line, std::string const& func_name, context* p_ctx )
        : line_( line ), func_name_( func_name ), p_ctx_( p_ctx )
      {
      }
      bool operator()( element const& lhs, element const& rhs )
      {
        std::vector< var_t > arg_list;
        arg_list.reserve( 3 );

        element e;
        e.s = func_name_;
        arg_list.push_back( var_t( 1, e ) );
        arg_list.push_back( var_t( 1, lhs ) );
        arg_list.push_back( var_t( 1, rhs ) );
        int arg1 = static_cast< int >( *lhs.i );
        int arg2 = static_cast< int >( *rhs.i );

        var_t r = bf_call( line_, arg_list, p_ctx_ );
        bool result = 0;
        if ( !r.empty() )
        {
          int retval = static_cast< int >( *r.front().i );
          result = ( *r.front().i < 0 );
        }
        return result;
      }
    private:
      std::string func_name_;
      context* p_ctx_;
      text_line line_;
    };
  }

  /*!
   *  \brief  ソート
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   */
  var_t bf_lsort( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "LSORT" ) )
    {
      var_t temp( arg_list[ 0 ] );
      std::string compare( arg_list[ 1 ].front().s );
      std::stable_sort( temp.begin(), temp.end(), bf_functor( line, compare, p_ctx ) );
      return temp;
    }
    element e;
    return var_t( 1, e );
  }

  /*!
   *  \brief  関数かどうかの判別
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   */
  var_t bf_isfunction( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "ISFUNCTION" ) )
    {
      std::string func_name( get_s( arg_list[ 0 ], p_ctx ) );
      if ( p_ctx->func_map.find( func_name ) != p_ctx->func_map.end() )
      {
        e.i = 1;
      }
      else
      {
        e.i = 0;
      }
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  順序付きリストの並びを逆にする
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   */
  var_t bf_reverse( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    var_t result;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "REVERSE" ) )
    {
      result = arg_list[ 0 ];
      std::reverse(result.begin(), result.end());
    }
    return result;
  }

  /*! 
   *  \brief  正規表現を用いた置換 
   *  \param[in]  line      行番号 
   *  \param[in]  arg_list  マクロ実引数リスト 
   *  \param[in]  p_ctx     マクロコンテキスト 
   *  \retval     マクロ返却値 
   *  第1マクロ実引数で指定した文字列のうち、第2マクロ実引数で指定した正規表現にマッチする箇所を第3マクロ実引数の内容で置換する。
   *  正規表現はECMAScript互換とする。
   */ 
   var_t bf_regex_replace( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx ) 
   { 
     element e; 
     if ( macro_processor::check_arity( line, arg_list.size(), 3, "REGEX_REPLACE" ) ) 
     {
       e.s = boost::xpressive::regex_replace( get_s( arg_list[ 0 ], p_ctx ), 
                                              boost::xpressive::sregex::compile( get_s( arg_list[ 1 ], p_ctx ) ), 
                                              get_s( arg_list[ 2 ], p_ctx ));
     } 
     return var_t( 1, e ); 
   }

  /*!
   *  \brief  文字列から整数への変換
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した文字列を整数値に変換する。
   *  第2マクロ実引数を指定した場合、それを基数とみなして変換を行う。
   *  第2マクロ実引数に0を指定した場合、接頭辞に応じて、8進、10進、16進を判別する。
   *  第2マクロ実引数に1を指定した場合、接頭辞に応じて、8進、10進、16進を判別する。
   */
  var_t bf_atoi( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    std::size_t arity = arg_list.size();

    if ( arity < 1 )
    {
      error( line, _( "too few arguments for `%1%\'" ), "ATOI" );
    }
    else if ( arity > 2 )
    {
      error( line, _( "too many arguments for `%1%\'" ), "ATOI" );
    }

    std::string str( get_s( arg_list[ 0 ], p_ctx ) );
    int radix = 10;

    if ( arity == 2 )
    {
      std::tr1::int64_t t = get_i( arg_list[ 1 ], p_ctx );
      if ( t < 0 || 36 < t )
      {
        error( line, _( "illegal_radix `%1%\' in function `%2%\'" ), radix, "ATOI" );
      }
      radix = static_cast< int >( t );
    }
    if ( radix == 1 )
    {
      std::string::size_type const pos = str.find_first_of( "0123456789" );
      if ( pos != std::string::npos && str[ pos ] == '0' )
      {
        char c = str[ pos + 1 ];
        if ( c != 'x' && c != 'X' )
        {
          radix = 10;
        }
      }
    }

    element e;
    char* endptr;
    errno = 0;
    using namespace std;
    e.i = strtoll( str.c_str(), &endptr, static_cast< int >( radix ) );
    if ( errno != 0 || *endptr != '\0')
    {
      error( line, _( "conversion error in function `%1%\'" ), "ATOI" );
    }

    return var_t( 1, e ); 
  }

  /*!
   *  \brief  大文字への変換
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した文字列中の小文字を大文字に変換します。
   */
  var_t bf_toupper( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "TOUPPER" ) ) 
    {
      std::string str = get_s( arg_list[ 0 ], p_ctx );
      for ( std::string::iterator first( str.begin() ), last( str.end() ); first != last; ++first )
      {
        char c = *first;
        *first = std::toupper( c );
      }
      e.s = str;
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  小文字への変換
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した文字列中の大文字を小文字に変換します。
   */
  var_t bf_tolower( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "TOLOWER" ) ) 
    {
      std::string str = get_s( arg_list[ 0 ], p_ctx );
      for ( std::string::iterator first( str.begin() ), last( str.end() ); first != last; ++first )
      {
        char c = *first;
        *first = std::tolower( c );
      }
      e.s = str;
    }
    return var_t( 1, e );
  }

  /*!  
   *  \brief  文字列の分割 
   *  \param[in]  line      行番号  
   *  \param[in]  arg_list  マクロ実引数リスト  
   *  \param[in]  p_ctx     マクロコンテキスト  
   *  \retval     マクロ返却値  
   *  第1マクロ実引数で指定した文字列のうち、第2マクロ実引数で指定した文字種separatorの文字で分割し，新しい順序付きリストを生成する。 
   */  
  var_t bf_split( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx ) 
  { 
    var_t result; 
    if ( macro_processor::check_arity( line, arg_list.size(), 2, "SPLIT" ) ) 
    { 
      std::list<std::string> split_results; 
      std::string heystack = get_s( arg_list[ 0 ], p_ctx ); 
      boost::split(split_results, heystack, boost::is_any_of( get_s( arg_list[ 1 ], p_ctx ) ) );  

      std::list<std::string>::iterator it = split_results.begin(); 
      while ( it != split_results.end() ) 
      { 
        element e; 
        e.s = *it; 
        result.push_back( e ); 
        ++it; 
      } 
    } 
    return result; 
  } 

  /*!
   *  \brief  配列の全削除
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  第1マクロ実引数で指定した名前の配列を全削除する。
   */
  var_t bf_clean( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 1, "CLEAN" ) ) 
    {
      std::string name = get_s( arg_list[ 0 ], p_ctx ) + "[";
      for ( std::map< std::string, var_t >::iterator it = p_ctx->var_map.lower_bound( name );
            it != p_ctx->var_map.end();
            ++it )
      {
        if ( std::strncmp( it->first.c_str(), name.c_str(), name.size() ) != 0 )
          break;
        it->second = var_t();
      }
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  マクロプロセッサの終了
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  マクロプロセッサを終了する。
   */
  var_t bf_die( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    if ( macro_processor::check_arity( line, arg_list.size(), 0, "DIE" ) ) 
    {
      throw macro_processor::die_terminate();
    }
    return var_t( 1, e );
  }

  /*!
   *  \brief  何もしない組み込み関数
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *  この組み込み関数は何も行わない。また、マクロ実引数のチェックも行わない。
   *  NOOP関数は常に "" を返す。
   *  \note       空値を返さないのは、$NOOP()$のような使い方をしたときでも不正な参照が起こらないようにするため。
   */
  var_t bf_noop( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    element e;
    return var_t( 1, e );
  }

  macro_processor::func_t const macro_processor::builtin_function_table[] =
  {
    { "LENGTH", bf_length },
    { "EQ", bf_eq },
    { "ALT", bf_alt },
    { "SORT", bf_sort },
    { "ENVIRON", bf_environ },
    { "VALUE", bf_value },
    { "CONCAT", bf_concat },
    { "APPEND", bf_append },
    { "AT", bf_at },
    { "GETTEXT", bf_gettext },
    { "_", bf_gettext },  // GETTEXTのシノニム
    { "FORMAT", bf_format },
    { "FIND", bf_find },
    { "RANGE", bf_range },
    { "DUMP", bf_dump },
    { "TRACE", bf_trace },
    { "ESCSTR", bf_escstr },
    { "UNESCSTR", bf_unescstr },
    { "CALL", bf_call },
    { "LSORT", bf_lsort },
    { "ISFUNCTION", bf_isfunction },
    { "REVERSE", bf_reverse },
    { "REGEX_REPLACE", bf_regex_replace }, 
    { "ATOI", bf_atoi },
    { "TOLOWER", bf_tolower },
    { "TOUPPER", bf_toupper },
    { "SPLIT", bf_split },
    { "CLEAN", bf_clean },
    { "DIE", bf_die },
    { "NOOP", bf_noop },
    { "", 0 },
  };

}
