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
 *  \file   toppers/itronx/static_api_parser.hpp
 *  \brief  静的APIの文法に関する宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  struct static_api_parser;
 *  struct static_api_parser::error_handler;
 *  struct static_api_parser::definition< Scanner >;
 *  \endcode
 */
#ifndef TOPPERS_ITRONX_STATIC_API_PARSER_HPP_
#define TOPPERS_ITRONX_STATIC_API_PARSER_HPP_

#include "toppers/diagnostics.hpp"
#include "toppers/c_expr.hpp"
#include <boost/spirit/include/classic_error_handling.hpp>

namespace toppers
{
  namespace itronx
  {

    /*!
     *  \struct static_api_parser static_api_parser.hpp "toppers/itronx/static_api_parser.hpp"
     *  \brief  静的APIの構文解析クラス
     */
    struct static_api_parser : boost::spirit::classic::grammar< static_api_parser >
    {
      enum rule_id_t
      {
        id_top = 1, id_api_name, id_parameter_list, id_parameter, id_packet, id_cexpr
      };
      enum expected_t
      {
        open_paren_expected, close_paren_expected, open_brace_expected, close_brace_expected,
        comma_expected, semicolon_expected
      };

      /*!
       *  \struct error_handler static_api_parser.hpp "toppers/itronx/static_api_parser.hpp"
       *  \brief  静的APIの構文解析におけるエラー処理ファンクタ
       */
      struct error_handler
      {
        template < class Scanner, class Error >
          boost::spirit::classic::error_status<> operator()( Scanner const& scan, Error const& error ) const
        {
          typename Error::iterator_t iter( error.where );
          std::string str;
          text_line ln;
          if ( iter != scan.last )
          {
            while ( *iter != '\0' && *iter != '\n' )
            {
              ++iter;
            }
            str = '\"' + std::string( error.where, iter ) + '\"';
            ln = get_text_line( error.where );
          }
          else
          {
            str = "\"end of file\"";
            ln = get_text_line( iter - 1 );
          }

          switch ( error.descriptor )
          {
          case open_paren_expected:
            toppers::fatal( ln, _( "missing `%1%\' before %2%" ), '(', str );
            break;
          case close_paren_expected:
            toppers::fatal( ln, _( "missing `%1%\' before %2%" ), ')', str );
            break;
          case close_brace_expected:
            toppers::fatal( ln, _( "missing `%1%\' before %2%" ), '}', str );
            break;
          case semicolon_expected:
            toppers::fatal( ln, _( "missing `%1%\' before %2%" ), ';', str );
            break;
          }
          return  boost::spirit::classic::error_status<>( boost::spirit::classic::error_status<>::fail );
        }
      };

      /*!
       *  \struct definition static_api_parser.hpp "toppers/itronx/static_api_parser.hpp" 
       *  \brief  静的APIの構文解析における文法定義
       */
      template < class Scanner >
        struct definition
      {
        typedef boost::spirit::classic::rule< Scanner, boost::spirit::classic::dynamic_parser_tag > rule_t;
        typedef boost::spirit::classic::guard< expected_t > guard_t;
        typedef boost::spirit::classic::assertion< expected_t > assertion_t;

        c_strlit_parser_t const c_strlit_p;
        c_ident_parser_t const c_ident_p;

        rule_t  top, api_name, parameter_list, parameter, packet, cexpr;
        guard_t guard_api, guard_packet;
        assertion_t expect_open_paren,
                    expect_close_paren,
                    expect_close_brace,
                    expect_comma, expect_semicolon;

        /*!
         *  \brief  コンストラクタ
         *  \param  self  構文解析クラス（文法クラス）への参照
         */
        definition( static_api_parser const& self )
          : c_strlit_p( c_strlit_parser( self.cexpr_p_.codeset_ ) ),
            c_ident_p( c_ident_parser( self.cexpr_p_.ucn_, self.cexpr_p_.c_plus_plus_ ) ),
            expect_open_paren( open_paren_expected ),
            expect_close_paren( close_paren_expected ),
            expect_close_brace( close_brace_expected ),
            expect_comma( comma_expected ),
            expect_semicolon( semicolon_expected )
        {
          using namespace boost::spirit::classic;
          set_id();
          top =
              guard_api
              (
                api_name >> 
                  str_p( "(" ) >>
                  !parameter_list >>
                  expect_close_paren( str_p( ")" ) ) >>
                  expect_semicolon( ch_p( ';' ) )
              )
              [
                error_handler()
              ];
          api_name =
              c_ident_p[ push_back_a( self.tokens_ ) ];
          parameter_list =
              !( parameter % ',' );
          parameter =
              packet | cexpr;
          packet =
              guard_packet
              (
                str_p( "{" )[ push_back_a( self.tokens_ ) ] >>
                parameter_list >>
                expect_close_brace( str_p( "}" )[ push_back_a( self.tokens_ ) ] ) 
              )
              [
                error_handler()
              ];
          cexpr =
              self.cexpr_p_[ push_back_a( self.tokens_ ) ];
        }
        void set_id()
        {
          top.set_id( id_top );
          api_name.set_id( id_api_name );
          parameter_list.set_id( id_parameter_list );
          parameter.set_id( id_parameter );
          packet.set_id( id_packet );
          cexpr.set_id( id_cexpr );
        }
        rule_t const& start() const { return top; }
      };
      /*!
       *  \brief  コンストラクタ
       *  \param  tokens  静的APIの構成トークンの格納先
       *  \param  cexpr_p C言語の定数式構文解析関数オブジェクト
       */
      explicit static_api_parser( std::vector< std::string >& tokens, c_const_expr_parser const& cexpr_p )
        : tokens_( tokens ), cexpr_p_( cexpr_p )
      {
      }

      std::vector< std::string >& tokens_;
      c_const_expr_parser const& cexpr_p_;
    };

  }
}

#endif  // ! TOPPERS_ITRONX_STATIC_API_PARSER_HPP_
