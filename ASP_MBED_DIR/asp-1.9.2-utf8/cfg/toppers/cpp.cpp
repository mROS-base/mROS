/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2011 by TAKAGI Nobuhisa
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
#include <cctype>
#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include "toppers/cpp.hpp"

namespace toppers
{

  /*!
   *  \brief  二重引用符で囲まれた文字列を展開する。
   */
  std::string expand_quote( std::string const& str )
  {
    // boost-1.35.x以上対策
    std::string::size_type n = str.find_first_not_of( " \t\r\n" );
    std::string quoted( str, n );

    if ( quoted.size() < 2 || quoted[0] != '"' || quoted[quoted.size() - 1] != '"' )
    {
      throw std::invalid_argument( "argument is not quoted" );
    }
    std::string result;
    for ( std::string::const_iterator iter( quoted.begin() + 1 ), last( quoted.end() - 1 );
          iter != last;
          ++iter )
    {
      if ( *iter == '\\' )
      {
        if ( ++iter == last )
        {
          throw std::invalid_argument( "argument is not quoted" );
        }
        char c = *iter;
        switch ( c )
        {
        case 'a':
          c = '\a';
          break;
        case 'b':
          c = '\b';
          break;
        case 'f':
          c = '\f';
          break;
        case 'n':
          c = '\n';
          break;
        case 'r':
          c = '\r';
          break;
        case 't':
          c = '\t';
          break;
        case 'v':
          c = '\v';
          break;
        case 'x':
          if ( std::isxdigit( static_cast< unsigned char >( *iter ) ) )
          {
            c = 0;
            for ( std::string::const_iterator bound( iter + 2 );
                  iter != bound && std::isxdigit( static_cast< unsigned char >( *iter ) );
                  ++iter )
            {
              c <<= 4;
              int t = std::tolower( static_cast< unsigned char >( *iter ) );
              static char const xdigits[] = "0123456789abcdef";
              c += std::strchr( xdigits, t ) - xdigits;
            }
          }
          break;
        default:
          if ( '0' <= c && c <= '7' ) // '\ooo'
          {
            c = 0;
            for ( std::string::const_iterator bound( iter + 3 );
                  iter != bound && ( '0' <= *iter && *iter <= '7' );
                  ++iter )
            {
              c = ( c << 3 ) + *iter - '0';
            }
          }
          // 国際文字名（\uhhhh, \Uhhhhhhhh）未対応
          // 二文字表記（<:等）未対応
          // 三文字表記（??/等）未対応
          break;
        }
        result.push_back( c );
      }
      else
      {
        result.push_back( *iter );
      }
    }
    return result;
  }

  /*!
   *  \brief  文字列で二重引用符で囲む
   */
  std::string quote_string( std::string const& str )
  {
    std::string result;
    result.reserve( str.size() + 2 );

    result.push_back( '"' );  // open

    for ( std::string::const_iterator iter( str.begin() ), last( str.end() );
          iter != last;
          ++iter )
    {
      switch ( char c = *iter )
      {
      case '\'':
        result += "\\\'";
        break;
      case '\"':
        result += "\\\"";
        break;
      case '\0':
        result += "\\0";
        break;
      case '\a':
        result += "\\a";
        break;
      case '\b':
        result += "\\b";
        break;
      case '\f':
        result += "\\f";
        break;
      case '\n':
        result += "\\n";
        break;
      case '\r':
        result += "\\r";
        break;
      case '\t':
        result += "\\t";
        break;
      case '\v':
        result += "\\v";
        break;
      case '\\':
        // SJIS未対応
        result.push_back( c );
        result.push_back( c );
        break;
      default:
        result.push_back( c );
        break;
      }
    }

    result.push_back( '"' );  // close

    return result;
  }

}
