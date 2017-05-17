/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2005-2010 by TAKAGI Nobuhisa
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
 *  \file   toppers/c_parser.hpp
 *  \brief  一般的なパーサーの宣言定義
 */
#ifndef TOPPERS_PARSER_HPP_
#define TOPPERS_PARSER_HPP_

#include "toppers/codeset.hpp"
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_actor.hpp>
#include <boost/spirit/include/classic_utility.hpp>
#include <boost/spirit/include/classic_dynamic.hpp>

namespace toppers
{

  namespace detail
  {

    struct c_integer_suffix_parse_functor
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& ) const
      {
        using namespace boost::spirit::classic;
        int length =
                  as_lower_d
                  [
                      ch_p( 'u' ) >> !( str_p( "ll" ) | 'l' | "i8" | "i16" | "i32" | "i64" )
                    | ( str_p( "ll" ) | 'l' | "i8" | "i16" | "i32" | "i64" ) >> !ch_p( 'u' )
                  ].parse( scan ).length();
        return length;
      }
    };

    template < typename T >
    struct c_integer_constant_parse_functor
    {
      typedef T result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        result_t x = T( 0 );
        int length =
                  (
                    if_p( '0' ) // 16進または8進
                    [
                      !(
                          ( ch_p( 'x' ) | 'X' ) >> uint_parser< T, 16 >()[ assign_a( x ) ]
                        | uint_parser< T, 8 >()[ assign_a( x ) ]
                      )
                    ]
                    .else_p // 10進
                    [
                      uint_parser< T, 10 >()[ assign_a( x ) ]
                    ]
                  ).parse( scan ).length();
        result = x;
        return length;
      }
    };

    template < typename T >
    struct c_integer_parse_functor
    {
      typedef T result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        static functor_parser< c_integer_constant_parse_functor< T > > const c_int_const_p;
        static functor_parser< c_integer_suffix_parse_functor > const c_int_suffix_p;
        bool negative = false;
        result_t x;
        int length =
                  (
                    !sign_p[ assign_a( negative ) ] >> lexeme_d[ c_int_const_p[ assign_a( x ) ] >> !c_int_suffix_p ]
                  ).parse( scan ).length();
        result = ( negative ? -x : x );
        return length;
      }
    };

    template < int CodeSet = -1 > struct mbchar_parse_functor;

    template <>
    struct mbchar_parse_functor< ascii >
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& ) const
      {
        return boost::spirit::classic::range_p( '\x01', '\x7f' ).parse( scan ).length();
      }
    };

    template <>
    struct mbchar_parse_functor< shift_jis >
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& ) const
      {
        using namespace boost::spirit::classic;
        int length =
                (
                    range_p( '\x01', '\x7f' ) // 半角英数字記号
                  | range_p( '\xa1', '\xdf' ) // 半角カタカナ
                  | ( chset<>( "\x81-\x9f\xe0-\xef" ) >> chset<>( "\x40-\x7e\x80-\xfc" ) )  // 全角
                ).parse( scan ).length();
        return length;
      }
    };

    template <>
    struct mbchar_parse_functor< euc_jp >
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& ) const
      {
        using namespace boost::spirit::classic;
        int length =
                (
                    range_p( '\x01', '\x7f' ) // 半角英数字記号
                  | ( ch_p( '\x8e' ) >> range_p( '\xa1', '\xdf' ) )   // 半角カタカナ
                  | ( !ch_p( '\x8f' ) >> range_p( '\xa1', '\xf0' ) >> range_p( '\xa1', '\xf0' ) ) // 全角
                ).parse( scan ).length();
        return length;
      }
    };

    template <>
    struct mbchar_parse_functor< utf8 >
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& ) const
      {
        using namespace boost::spirit::classic;
        int length =
                (
                    range_p( '\x01', '\x7f' ) // 1バイト
                  | ( range_p( '\xc0', '\xdf' ) >> range_p( '\x80', '\xbf' ) )  // 2バイト
                  | ( range_p( '\xe0', '\xef' ) >> repeat_p( 2 )[ range_p( '\x80', '\xbf' ) ] ) // 3バイト
                  | ( range_p( '\xf0', '\xf7' ) >> repeat_p( 3 )[ range_p( '\x80', '\xbf' ) ] ) // 4バイト
                  | ( range_p( '\xf8', '\xfb' ) >> repeat_p( 4 )[ range_p( '\x80', '\xbf' ) ] ) // 5バイト
                  | ( range_p( '\xfc', '\xfd' ) >> repeat_p( 5 )[ range_p( '\x80', '\xbf' ) ] ) // 6バイト
                ).parse( scan ).length();
        return length;
      }
    };

    template <>
    struct mbchar_parse_functor< -1 >
    {
      typedef boost::spirit::classic::nil_t result_t;
      explicit mbchar_parse_functor( codeset_t codeset = ascii ) : codeset_( codeset ) {}
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        int length;
        switch ( codeset_ )
        {
        case ascii:
          {
            static mbchar_parse_functor< ascii > f;
            length = f( scan, result );
          }
          break;
        case shift_jis:
          {
            static mbchar_parse_functor< shift_jis > f;
            length = f( scan, result );
          }
          break;
        case euc_jp:
          {
            static mbchar_parse_functor< euc_jp > f;
            length = f( scan, result );
          }
          break;
        case utf8:
          {
            static mbchar_parse_functor< utf8 > f;
            length = f( scan, result );
          }
          break;
        default:
          length = -1;
          break;
        }
        return length;
      }
      codeset_t codeset_;
    };

    struct ucn_parse_functor
    {
      typedef long result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        result_t x;
        int length =
                (
                  lexeme_d
                  [
                      ch_p( '\\' ) >>
                      (
                          ( 'U' >> int_parser< long, 16, 8, 8 >()[ assign_a( x ) ] )  // \Uhhhhhhhh形式
                        | ( 'u' >> int_parser< long, 16, 4, 4 >()[ assign_a( x ) ] )  // \uhhhh形式
                      )
                  ]
                ).parse( scan ).length();
        if ( ( x < 0xa0 && !( x == 0x24 || x == 0x40 || x == 0x60 ) )
          || ( 0xd800 <= x && x <= 0xdfff ) 
          || 0x10ffff < x ) // 国際文字名に使えない値（JIS X3010:2003 6.4.3）
        {
          x = -1;
        }
        result = x;
        return length;
      }
    };

    template < int CodeSet = -1 > struct c_strlit_parse_functor;

    template < int CodeSet >
    struct c_strlit_parse_functor
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        static functor_parser< detail::mbchar_parse_functor< CodeSet > > const mbchar_p;
        static functor_parser< detail::ucn_parse_functor > const ucn_p;
        int length =
                (
                  confix_p( '\"', *( "\\\"" | mbchar_p - '\\' | ucn_p | c_escape_ch_p ), '\"' )
                ).parse( scan ).length();
        return length;
      }
    };

    template <>
    struct c_strlit_parse_functor< -1 >
    {
      typedef boost::spirit::classic::nil_t result_t;
      explicit c_strlit_parse_functor( codeset_t codeset = ascii ) : codeset_( codeset ) {}
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        mbchar_parse_functor<> const functor( codeset_ );
        functor_parser< detail::mbchar_parse_functor<> > const mbchar_p( functor );
        static functor_parser< detail::ucn_parse_functor > const ucn_p;
        int length =
                (
                  confix_p( '\"', *( "\\\"" | ( mbchar_p - ch_p( '\\' ) ) | ucn_p | c_escape_ch_p ), '\"' )
                ).parse( scan ).length();
        return length;
      }
      codeset_t codeset_;
    };

    template < int CodeSet = -1 > struct c_chlit_parse_functor;

    template < int CodeSet >
    struct c_chlit_parse_functor
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        static functor_parser< detail::mbchar_parse_functor< CodeSet > > const mbchar_p;
        static functor_parser< detail::ucn_parse_functor > const ucn_p;
        int length =
                (
                  confix_p( '\'', +( "\\\'" | mbchar_p - '\\' | ucn_p | c_escape_ch_p ), '\'' )
                ).parse( scan ).length();
        return length;
      }
    };

    template <>
    struct c_chlit_parse_functor< -1 >
    {
      typedef boost::spirit::classic::nil_t result_t;
      explicit c_chlit_parse_functor( codeset_t codeset = ascii ) : codeset_( codeset ) {}
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        mbchar_parse_functor<> const functor( codeset_ );
        functor_parser< detail::mbchar_parse_functor<> > const mbchar_p( functor );
        static functor_parser< detail::ucn_parse_functor > const ucn_p;
        int length =
                (
                  confix_p( '\'', +( "\\\'" | mbchar_p - '\\' | ucn_p | c_escape_ch_p ), '\'' )
                ).parse( scan ).length();
        return length;
      }
      codeset_t codeset_;
    };

    extern char const* const c_keywords[];
    extern char const* const c_plus_plus_keywords[];

    struct c_identifier_parse_functor
    {
      typedef boost::spirit::classic::nil_t result_t;
      template < class Scanner >
      int operator()( Scanner scan, result_t& result ) const
      {
        using namespace boost::spirit::classic;
        static functor_parser< detail::ucn_parse_functor > const ucn_p;
        int length;
        typename Scanner::iterator_t const first( scan.first );

        if ( ucn_ )
        {
          length =
                (
                  lexeme_d
                  [
                    ( alpha_p | '_' | ucn_p ) >>
                   *( alnum_p | '_' | ucn_p )
                  ]
                ).parse( scan ).length();
        }
        else
        {
          length =
                (
                  lexeme_d
                  [
                     ( alpha_p | '_' ) >>
                    *( alnum_p | '_' )
                  ]
                ).parse( scan ).length();
        }
        std::string token( first, scan.first );

        for ( int i = 0; c_keywords[i] != 0; i++ )
        {
          if ( token == c_keywords[i] )
          {
            length = -1;
            break;
          }
        }
        if ( c_plus_plus_ )
        {
          for ( int i = 0; c_plus_plus_keywords[i] != 0; i++ )
          {
            if ( token == c_plus_plus_keywords[i] )
            {
              length = -1;
              break;
            }
          }
        }
        return length;
      }
      explicit c_identifier_parse_functor( bool ucn = false, bool c_plus_plus = false )
        : ucn_( ucn ), c_plus_plus_( c_plus_plus )
      {
      }

      bool ucn_;
      bool c_plus_plus_;
    };

  }

  /*!
   *  \brief  C言語形式の整数定数パーサー
   *
   *  boost::spirit::int_parserとの違いは、接頭辞に応じて8進数や16進数として解釈
   *  する点です。また型を特定するための接尾辞も受け入れます。
   */
  template < typename T >
  inline boost::spirit::classic::functor_parser< detail::c_integer_parse_functor< T > > const c_int_parser()
  {
    return boost::spirit::classic::functor_parser< detail::c_integer_parse_functor< T > >();
  }

  extern boost::spirit::classic::functor_parser< detail::c_integer_parse_functor< int > > const c_int_p;
  extern boost::spirit::classic::functor_parser< detail::c_integer_parse_functor< unsigned int > > const c_uint_p;

  /*!
   *  \brief  マルチバイト文字パーサー
   *
   *  テンプレート引数 CodeSet で指定した文字コードを解析します。
   *  ascii 以外を指定した場合でも、ASCIIの範囲（0x7f以下）にも合致します。
   *  0x00 は文字列の終端と区別できないため、合致対象にはなりません。
   */
  template < int CodeSet >
  inline boost::spirit::classic::functor_parser< detail::mbchar_parse_functor< CodeSet > > const mbchar_parser()
  {
    return boost::spirit::classic::functor_parser< detail::mbchar_parse_functor< CodeSet > >();
  }
  inline boost::spirit::classic::functor_parser< detail::mbchar_parse_functor<> > const mbchar_parser( codeset_t codeset )
  {
    return boost::spirit::classic::functor_parser< detail::mbchar_parse_functor<> >( detail::mbchar_parse_functor<>( codeset ) );
  }

  extern boost::spirit::classic::functor_parser< detail::mbchar_parse_functor< ascii > > const ascii_p;
  extern boost::spirit::classic::functor_parser< detail::mbchar_parse_functor< shift_jis > > const shift_jis_p;
  extern boost::spirit::classic::functor_parser< detail::mbchar_parse_functor< euc_jp > > const euc_jp_p;
  extern boost::spirit::classic::functor_parser< detail::mbchar_parse_functor< utf8 > > const utf8_p;

  /*!
   *  \brief  国際文字名パーサー
   *
   *  \\uまたは\\Uで始まる国際文字名（Universal Character Name）を解析します。
   *  \\uhhhhまたは\\Uhhhhhhhh（hは16進数字）に合致します。なお、これらの形式に
   *  合致している場合でも、一部の値は国際文字名として使用できません。
   */
  inline boost::spirit::classic::functor_parser< detail::ucn_parse_functor > const ucn_parser()
  {
    return boost::spirit::classic::functor_parser< detail::ucn_parse_functor >();
  }

  extern boost::spirit::classic::functor_parser< detail::ucn_parse_functor > const ucn_p;

  /*!
   *  \brief  C言語形式の文字列定数パーサー
   *
   *  二重引用符で囲まれたC言語形式の文字列を解析します。
   *  文字列の文字コードは CodeSet で指定したものになります。
   */
  template < int CodeSet >
  inline boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor< CodeSet > > const c_strlit_parser()
  {
    return boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor< CodeSet > >();
  }

  typedef boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor<> > c_strlit_parser_t;

  /*!
   *  \brief  C言語形式の文字列定数パーサー
   *  \param  codeset 文字列に使用されている文字コード
   *
   *  二重引用符で囲まれたC言語形式の文字列を解析します。
   */
  inline boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor<> > const c_strlit_parser( codeset_t codeset )
  {
    return boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor<> >( detail::c_strlit_parse_functor<>( codeset ) );
  }

  extern boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor< ascii > > const ascii_str_p;
  extern boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor< shift_jis > > const shift_jis_str_p;
  extern boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor< euc_jp > > const euc_jp_str_p;
  extern boost::spirit::classic::functor_parser< detail::c_strlit_parse_functor< utf8 > > const utf8_str_p;

  /*!
   *  \brief  C言語形式の文字定数パーサー
   *
   *  単引用符で囲まれたC言語形式の文字定数を解析します。
   *  文字コードは CodeSet で指定したものになります。
   */
  template < int CodeSet >
  inline boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor< CodeSet > > const c_chlit_parser()
  {
    return boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor< CodeSet > >();
  }

  typedef boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor<> > c_chlit_parser_t;

  /*!
   *  \brief  C言語形式の文字定数パーサー
   *  \param  codeset 文字コード
   *
   *  単引用符で囲まれたC言語形式の文字定数を解析します。
   */
  inline boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor<> > const c_chlit_parser( codeset_t codeset )
  {
    return boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor<> >( detail::c_chlit_parse_functor<>( codeset ) );
  }

  extern boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor< ascii > > const ascii_ch_p;
  extern boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor< shift_jis > > const shift_jis_ch_p;
  extern boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor< euc_jp > > const euc_jp_ch_p;
  extern boost::spirit::classic::functor_parser< detail::c_chlit_parse_functor< utf8 > > const utf8_ch_p;

  typedef boost::spirit::classic::functor_parser< detail::c_identifier_parse_functor > c_ident_parser_t;

  /*!
   *  \brief  C言語形式の識別子パーサー
   *  \param  ucn         国際文字を許容する場合は true を指定
   *  \param  c_plus_plus C++の予約語を禁止する場合は true を指定
   *
   *  C言語の識別子として使用できる文字列の解析を行います。
   *  識別子は下線または英文字で始まり、下線または英数字が続くのが原則ですが、
   *  C99以降では国際文字名も使用できるため、引数 ucn に true を指定することで、
   *  国際文字名対応が可能になります。
   *
   *  \note 翻訳限界および予約識別子の判別は行っていません。
   */
  inline boost::spirit::classic::functor_parser< detail::c_identifier_parse_functor > const c_ident_parser( bool ucn = false, bool c_plus_plus = false )
  {
    return boost::spirit::classic::functor_parser< detail::c_identifier_parse_functor >( detail::c_identifier_parse_functor( ucn, c_plus_plus ) );
  }

  extern boost::spirit::classic::functor_parser< detail::c_identifier_parse_functor > const c_ident_p;
  extern boost::spirit::classic::functor_parser< detail::c_identifier_parse_functor > const c99_ident_p;
  extern boost::spirit::classic::functor_parser< detail::c_identifier_parse_functor > const c_plus_plus_ident_p;

}

#endif  // ! TOPPERS_PARSER_HPP_
