/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
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
 *  \file   toppers/misc.hpp
 *  \brief  雑多なライブラリのための宣言定義
 */
#ifndef TOPPERS_MISC_HPP_
#define TOPPERS_MISC_HPP_

#include <stdexcept>
#include <locale>
#include <iosfwd>
#include <string>
#include <algorithm>
#include <cctype>
#include <cwchar>
#include <cwctype>
#include <cstdlib>
#include "toppers/codeset.hpp"
#include <boost/scoped_array.hpp>

#if defined(_MSC_VER) || defined(__MINGW32__)
#include <mbstring.h>
#endif

// workaround
#include <ctype.h>
#include <wctype.h>

#define TOPPERS_STRINGIFY( s )  TOPPERS_STRINGIFY_( s )
#define TOPPERS_STRINGIFY_( s ) # s

namespace toppers
{

  /*!
   *  \class  conversion_error misc.hpp "toppers/misc.hpp"
   *  \brief  変換エラー例外クラス
   */
  class conversion_error : public std::runtime_error
  {
  public:
    /*!
     *  \brief  コンストラクタ
     *  \param  what  例外原因文字列
     */
    explicit conversion_error( std::string const& what ) : std::runtime_error( what ) {}
  };

  /*!
   *  \brief  シングルバイト文字から CharT 型（多くは wchar_t 型）文字への変換
   *  \param  ch  シングルバイト文字
   *  \return CharT 型文字を返す
   */
  template < typename CharT >
    inline CharT widen( char ch )
  {
    return std::use_facet< std::ctype< CharT > >( std::locale() ).widen( ch );
  }

  template <>
    inline char widen< char >( char ch )
  {
    return ch;
  }

  template <>
    inline unsigned char widen< unsigned char >( char ch )
  {
    return static_cast< unsigned char >( ch );
  }

  template <>
    inline wchar_t widen< wchar_t >( char ch )
  {
#if defined( __MINGW32__ )
    wchar_t wc = wchar_t( -1 );
    if ( std::mbtowc( &wc, &ch, 1 ) < 0 )
    {
      return WEOF;
    }
    return wc;
#elif defined( __CYGWIN__ )
    return static_cast< wchar_t >( static_cast< unsigned char >( ch ) );
#else
    return static_cast< wchar_t >( std::btowc( static_cast< unsigned char >( ch ) ) );
#endif
  }

  /*!
   *  \brief  シングルバイト文字列から CharT 型文字列への変換
   *  \param  str   シングルバイト文字列
   *  \return CharT 型文字列を返す
   */
  template < typename CharT >
    std::basic_string< CharT > const widen( std::string const& str );

  template <>
    inline std::basic_string< char > const widen( std::string const& str )
  {
    return str;
  }

  template <>
    inline std::basic_string< wchar_t > const widen( std::string const& str )
  {
    boost::scoped_array< wchar_t > t( new wchar_t[str.size()+1] );
    if ( std::mbstowcs( t.get(), str.c_str(), str.size() ) == size_t( -1 ) )
    {
      static conversion_error x( "in function widen" );
      throw x;
    }
    return t.get();
  }

#undef  tolower

  /*!
   *  \brief  小文字への変換
   *  \param  ch    変換対象の文字
   *  \return ch が大文字であれば対応する小文字を、それ以外は ch を返す
   */
  inline char tolower( char ch )
  {
    return static_cast< char >( std::tolower( static_cast< unsigned char >( ch ) ) );
  }

  /*!
   *  \brief  文字列を小文字に変換
   *  \param  str   変換対象の文字
   *  \return str に含まれる大文字を小文字に変換した文字列を返す
   */
  inline std::string const tolower( std::string str )
  {
    char ( *f )( char ) = &tolower;
    std::transform( str.begin(), str.end(), str.begin(), f );
    return str;
  }

  /*!
   *  \brief  ワイド文字列を小文字に変換
   *  \param  str   変換対象の文字
   *  \return str に含まれる大文字を小文字に変換したワイド文字列を返す
   */
  template < class Traits, class Allocator >
    std::basic_string< wchar_t, Traits, Allocator > const tolower( std::basic_string< wchar_t, Traits, Allocator > str )
  {
  	wint_t ( *f )( wint_t ) = &towlower;
    std::transform( str.begin(), str.end(), str.begin(), f );
    return str;
  }

#undef  toupper

  /*!
   *  \brief  大文字への変換
   *  \param  ch    変換対象の文字
   *  \return ch が小文字であれば対応する大文字を、それ以外は ch を返す
   */
  inline char toupper( char ch )
  {
    return static_cast< char >( std::toupper( static_cast< unsigned char >( ch ) ) );
  }

  /*!
   *  \brief  文字列を大文字に変換
   *  \param  str   変換対象の文字
   *  \return str に含まれる小文字を大文字に変換した文字列を返す
   */
  inline std::string const toupper( std::string str )
  {
    char ( *f )( char ) = &toupper;
    std::transform( str.begin(), str.end(), str.begin(), f );
    return str;
  }

  /*!
   *  \brief  ワイド文字列を大文字に変換
   *  \param  str   変換対象の文字
   *  \return str に含まれる小文字を大文字に変換したワイド文字列を返す
   */
  template < class Traits, class Allocator >
    std::basic_string< wchar_t, Traits, Allocator > const toupper( std::basic_string< wchar_t, Traits, Allocator > str )
  {
  	wint_t ( *f )( wint_t ) = &towupper;
    std::transform( str.begin(), str.end(), str.begin(), f );
    return str;
  }

#undef  isspace

  /*!
   *  \brief  空白類の判別
   *  
   */
  inline bool isspace( char ch )
  {
    return std::isspace( static_cast< unsigned char >( ch ) ) != 0;
  }

  /*!
   *  \brief  指定文字で区切られたリスト出力
   *  \param  first   出力する先頭要素位置
   *  \param  last    出力する終端要素位置+1
   *  \param  ostr    出力ストリーム
   *  \param  pred    各要素を受け取り出力値を返す述語
   *  \param  delim   区切文字
   *
   *  この関数は区間 [first, last) の各要素を pred に渡して得られる値を delim
   *  で区切って ostr に出力します。\n
   *  終端の要素の後には delim は出力されず、要素と要素の間にのみ delim が
   *  出力されます。
   */
  template < class InputIterator, typename CharT, class Traits, class Pred >
    void output_list( InputIterator first, InputIterator last, std::basic_ostream< CharT, Traits >& ostr, Pred pred, CharT const* delim = 0 )
  {
    if ( delim == 0 )
    {
      static CharT const null_delim[] = { 0 };
      delim = null_delim;
    }

    for ( bool f = false; first != last; ++first )
    {
      if ( f )
      {
        ostr << delim;
      }
      ostr << pred( *first );
      f = true;
    }
  }

  template < typename CharT, class Traits, class Allocator >
    std::basic_string< CharT, Traits, Allocator > trim( std::basic_string< CharT, Traits, Allocator > const& str, std::basic_string< CharT, Traits, Allocator > const& ws )
  {
    if ( str.empty() || ws.empty() )
    {
      return str;
    }
    typename std::basic_string< CharT, Traits, Allocator >::size_type first = str.find_first_not_of( ws, 0 );
    for ( typename std::basic_string< CharT, Traits, Allocator >::size_type i = str.size() - 1; i >= 0; i-- )
    {
      if ( ws.find( str[ i ] ) == ws.npos )
      {
        return str.substr( first, i );
      }
    }
    return str.substr( first );
  }

  inline std::string dir_delimiter_to_slash( std::string const& str )
  {
#if defined(_MSC_VER) || defined(__MINGW32__)
    std::string result;
    result.reserve( str.size() );

    unsigned char const* s1 = reinterpret_cast< unsigned char const* >( str.c_str() );
    while ( unsigned char const* s2 = _mbschr( s1, '\\' ) )
    {
      result.append( s1, s2 );
      result.push_back( '/' );
      s1 = s2 + 1;
    }
    return result + reinterpret_cast< char const* >( s1 );
#else
    return str;
#endif
  }

}

#endif  // ! TOPPERS_MISC_HPP_
