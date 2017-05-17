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
 *  \file   toppers/c_pp_line.hpp
 *  \brief  \#line指令に関する宣言定義
 */
#ifndef TOPPERS_C_PP_LINE_HPP_
#define TOPPERS_C_PP_LINE_HPP_

#include "toppers/text_line.hpp"
#include "toppers/c_parser.hpp"
#include "toppers/workaround.hpp"
#include <vector>
#include <functional>

namespace toppers
{

  namespace detail
  {

    //! \#line 指令の構文解析
    struct c_pp_line_parser : boost::spirit::grammar< c_pp_line_parser >
    {
      template < class Scanner >
      struct definition
      {
        typedef boost::spirit::rule< Scanner > rule_t;
        rule_t r;
        definition( c_pp_line_parser const& self )
        {
          using namespace boost::spirit;
          r = (
                '#' >> lexeme_d[ str_p( "line" ) >> space_p >> uint_p[ assign_a( self.line_ ) ] ] >>
                c_strlit_parser( self.codeset_ )[ assign_a( self.file_ ) ]
              )
            | (
                '#' >>
                uint_p[ assign_a( self.line_ ) ] >>
                c_strlit_parser( self.codeset_ )[ assign_a( self.file_ ) ] >>
                *anychar_p
              );
        }
        rule_t const& start() const { return r; }
      };

      c_pp_line_parser( long& line, std::string& file, codeset_t codeset = ascii )
        : line_( line ), file_( file ), codeset_( codeset )
      {
      }

      long& line_;
      std::string& file_;
      codeset_t codeset_;
    };

    //! \#pragma 指令の構文解析
    struct c_pp_pragma_parser : boost::spirit::grammar< c_pp_pragma_parser >
    {
      template < class Scanner >
      struct definition
      {
        typedef boost::spirit::rule< Scanner > rule_t;
        rule_t r;
        definition( c_pp_pragma_parser const& self )
        {
          using namespace boost::spirit;
          r = '#' >> lexeme_d[ str_p( "pragma" ) >> space_p >> ( +anychar_p )[ assign_a( self.parameter_ ) ] ];
        }
        rule_t const& start() const { return r; }
      };

      c_pp_pragma_parser( std::string& parameter ) : parameter_( parameter )
      {
      }
      std::string& parameter_;
    };

  }

  /*!
   *  \class  c_pp_line c_pp_line.hpp "toppers/c_pp_line.hpp"
   *  \brief  \#line指令を処理させるためのファンクタクラス
   *
   *  このクラスは basic_text クラスと組み合わせて使用します。
   */
  template < class Container >
  class c_pp_line : public std::binary_function< Container, line_buf, void >
  {
  public:
    typedef Container conatiner;

    /*!
     *  \brief  コンストラクタ
     *  \param  codeset 文字コード指定
     */
    explicit c_pp_line( codeset_t codeset = ascii )
      : codeset_( codeset ), pragmas_( new std::vector< line_buf > )
    {
    }
    /*!
     *  \brief  括弧演算子
     *  \param  cont  line_buf を要素とするコンテナ
     *  \param  buf   1行バッファ
     */
    void operator()( conatiner& cont, line_buf& buf )
    {
      using namespace boost::spirit;
      long line;
      std::string file;
      detail::c_pp_line_parser c_pp_line_p( line, file, codeset_ );

      if ( parse( buf.buf.begin(), buf.buf.end(), c_pp_line_p, space_p ).full ) // #line指令の処理
      {
        buf.line.line = line;
        assert( file.size() >= 2 );
        buf.line.file = file.substr( 1, file.size()-2 );
      }
      else
      {
        std::string param;
        detail::c_pp_pragma_parser c_pp_pragma_p( param );

        if ( parse( buf.buf.begin(), buf.buf.end(), c_pp_pragma_p, space_p ).full ) // #pragma指令の処理
        {
          line_buf t( buf );
          t.buf = param;
          pragmas_->push_back( t );
        }
        else
        {
          std::string::size_type pos = buf.buf.find_first_not_of( " \t" );
          if ( pos == std::string::npos || buf.buf[pos] != '#' )
          {
            cont.push_back( buf );
          }
          ++buf.line.line;
        }
      }
      buf.buf.clear();
    }
    /*!
     *  \brief  \#pragma指令リストの取得
     *  \return \#pragma指令リストを返す
     */
    std::vector< line_buf > const& pragmas() const { return *pragmas_; }
  private:
    codeset_t codeset_;
    std::tr1::shared_ptr< std::vector< line_buf > > pragmas_;
  };

}

#endif  // ! TOPPERS_C_PP_LINE_HPP_
