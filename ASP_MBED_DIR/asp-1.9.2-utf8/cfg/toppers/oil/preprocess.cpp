/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2008 by TAKAGI Nobuhisa
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
#include <cstring>
#include <algorithm>
#include <iostream>
#include <iterator>
#include "toppers/io.hpp"
#include "toppers/cpp.hpp"
#include "toppers/global.hpp"
#include "toppers/diagnostics.hpp"
#include "toppers/oil/preprocess.hpp"
#include <boost/spirit/include/classic_spirit.hpp>

namespace toppers
{
  namespace oil
  {

    /*!
     *  \brief  静的API `INCLUDE'の展開
     *  \param[in]  in      入力ファイルの内容
     *  \param[out] out     出力ファイルの内容
     *  \param[in]  codeset 文字コード
     *  \param[out] dependencies  依存関係の格納先（NULLの場合は格納しない）
     *  \param[out] onces   #pragma onceが記述されていたファイル名の格納先（NULLの場合は格納しない）
     *
     *  `INCLUDE' の引数は、μITRON 仕様のものとは異なり、二重引用符で囲まずに #include 指令と
     *  同じ形式を用いる。
     */
    void expand_include( text const& in, text& out, codeset_t codeset, std::set< std::string >* dependencies, std::set< std::string >* onces )
    {
      typedef text::container::const_iterator const_row_iterator;
      typedef std::string::size_type size_type;
      size_type const npos = std::string::npos;
      const_row_iterator first( in.begin().get_row() ), last( in.end().get_row() );

      out.set_line( first->line.file, first->line.line );
      for ( const_row_iterator iter( first ); iter != last; ++iter )
      {
        std::string const& buf = iter->buf;
        char state = 0;

        for ( size_type i = 0, n = buf.size(); i != n; ++i )
        {
          char c = buf[i];
          if ( c == '\'' || c == '\"' )
          {
            if ( state == 0 )
            {
              state = c;    // 文字（列）リテラル開始
            }
            else if ( buf[i - 1] != '\\' ) //  \' または \" ではない
            {
              state = 0;    // 文字（列）リテラル終了
            }
            else if ( ( codeset == shift_jis ) && ( i >= 2 ) && is_lead< shift_jis >( buf[i - 2] ) )
            {
              state = 0;
            }
            out.push_back( c );
          }
          else if ( state != 0 )  // 文字（列）リテラル内
          {
            out.push_back( c );
          }
          else
          {
            using namespace boost::spirit::classic;
            std::string headername;
            if ( iter == last )
            {
              return;
            }
            text::const_iterator iter2( iter, i );
            if ( iter2 == in.end() )
            {
              break;
            }
            parse_info< text::const_iterator > info
              = parse( iter2, in.end(),
				( str_p( "#include" ) >> *space_p
                            >> '\"' >> ( +( anychar_p - '\"' ) )[ assign( headername ) ] >> '\"'
                           ) );  // 行番号がずれるので、スキップパーサは使用しない
            if ( info.hit )
            {
              std::vector< std::string > include_paths = get_global< std::vector< std::string > >( "include-path" );
              std::string hname = search_include_file( include_paths.begin(), include_paths.end(), headername );
              if ( hname.empty() )  // ヘッダ名が見つからない
              {
                fatal( iter->line, _( "cannot open file `%1%\'" ), headername );
              }
              else if ( onces == 0 || onces->find( hname ) == onces->end() )
              {
                if ( dependencies != 0 )
                {
                  dependencies->insert( hname );
                }
                out.push_back( ' ' ); // ダミーを挿入しておかないと行番号がずれる
                preprocess( hname, out, codeset, dependencies, onces );  // ヘッダ名で指定されたファイルに対して前処理を再帰的に行う
                iter = info.stop.get_row();
                i = info.stop.get_col() - 1;
                if ( iter != last )
                {
                  out.set_line( iter->line.file, iter->line.line );
                }
              }
            }
            else
            {
              if ( iter2.get_col() == 0 )
              {
                info = parse( iter2, in.end(),
                              ( *( space_p - eol_p ) >>
                                  ch_p( '#' ) >> *( space_p - eol_p ) >> "pragma" >> *( space_p - eol_p )
                                    >> "once"
                                    >> *( space_p - eol_p ) >> eol_p ) );
                if ( info.hit )
                {
                  if ( onces != 0 )
                  {
                    onces->insert( iter2.line().file );
                  }
                  out.push_back( ' ' );
                  iter = info.stop.get_row();
                  if ( iter != last )
                  {
                    out.set_line( iter->line.file, iter->line.line );
                  }
                  --iter;   // インクリメントされるのでいったん戻す。
                  break;
                }
              }
              if ( parse( iter2, in.end(),
				  ( str_p( "#include" ) >> *space_p >> '\"' >> +( anychar_p - ')' ) >> '\"') ).hit )
              {
                warning( iter->line, _( "probably, %1% argument of `%2%\' is illegal" ), _( "1st" ), "INCLUDE" );
              }
              out.push_back( c );
            }
          }
        }
      }
    }

    /*!
     *  \brief  コンフィギュレーションファイルの前処理
     *  \param[in]  file    コンフィギュレーションファイル名
     *  \param[out] result  前処理後の内容
     *  \param[in]  codeset 文字コード
     *  \param[out] dependencies  依存関係の格納先（NULLの場合は格納しない）
     *  \param[out] onces   #pragma onceが記述されていたファイル名の格納先（NULLの場合は格納しない）
     */
    void preprocess( std::string const& file, text& result, codeset_t codeset, std::set< std::string >* dependencies, std::set< std::string >* onces )
    {
      std::string buf;
      read( file, buf );
      if ( buf.empty() )
      {
        return;
      }
      text txt;
      txt.set_line( file, 1 );
      remove_comment( buf.begin(), buf.end(), std::back_inserter( txt ), codeset );
      expand_include( txt, result, codeset, dependencies, onces );
    }

  }
}
