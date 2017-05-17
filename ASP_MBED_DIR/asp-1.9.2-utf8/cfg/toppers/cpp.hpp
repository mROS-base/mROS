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
 *  \file   toppers/cpp.hpp
 *  \brief  Cプリプロセッサ代替機能に関する宣言定義
 */
#ifndef TOPPERS_CPP_HPP_
#define TOPPERS_CPP_HPP_

#include "toppers/codeset.hpp"
#include "toppers/diagnostics.hpp"
#include <boost/utility.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>

namespace toppers
{

  /*!
   *  \brief  コメントの除去
   *  \param[in]  first     入力元の先頭位置
   *  \param[in]  last      入力元の終端位置の次
   *  \param[out] result    結果の出力先
   *  \param[in]  codeset   文字コード
   *  \return     処理完了後の result の値
   *
   *  CスタイルのブロックコメントとC++スタイルの行コメントを取り除きます。
   *  行コメントは、行末の \\ があっても次の行には続きません。
   */
  template < class ForwardIterator, class OutputIterator >
    OutputIterator remove_comment( ForwardIterator first, ForwardIterator last, OutputIterator result, codeset_t codeset = ascii )
  {
    enum { none, line_comment, block_comment, single_quote, double_quote } state = none;
    char prev = '\0';

    for ( ; first != last; ++first, ++result )
    {
      char c = *first;
      ForwardIterator next_iter = boost::next( first );
      char next = ( next_iter != last ? *next_iter : '\0' );

      switch ( state )
      {
      case line_comment:
        if ( c == '\n' )
        {
          state = none;
          *result = c;
        }
        break;
      case block_comment:
        if ( c == '*' && next == '/' )
        {
          state = none;
          ++first;
        }
        else if ( c == '\n' )
        {
          *result = c;
        }
        break;
      case single_quote:
        if ( c == '\\'
          && !( codeset == shift_jis && is_lead< shift_jis >( prev ) )
          && next == '\'' )   // '〜\'〜 の場合
        {
          *result++ = *first++;
        }
        else if ( c == '\'' )
        {
          state = none;
        }
        *result = *first;
        break;
      case double_quote:
        if ( c == '\\'
          && !( codeset == shift_jis && is_lead< shift_jis >( prev ) )
          && next == '\"' )   // "〜\"〜 の場合
        {
          *result++ = *first++;
        }
        else if ( c == '\"' )
        {
          state = none;
        }
        *result = *first;
        break;
      default:
        switch ( c )
        {
        case '/':   // コメント開始の検出
          if ( next == '*' )
          {
            state = block_comment;
            ++first;
          }
          else if ( next == '/' )
          {
            state = line_comment;
            ++first;
          }
          else
          {
            *result = *first;
          }
          break;
        case '\'':
          state = single_quote;
          *result = c;
          break;
        case '\"':
          state = double_quote;
          *result = c;
          break;
        default:
          *result = c;
          break;
        }
      }
      prev = *first;
    }
    switch ( state )
    {
    case line_comment:
    case block_comment:
      error( "unterminated comment" );
      break;
    }
    return result;
  }

  /*!
   *  \brief  インクルードパスの探索
   *  \param[in]  first       探索対象ディレクトリ列の先頭位置
   *  \param[in]  last        探索対象ディレクトリ列の終端位置の次
   *  \param[in]  headername  探索対象のヘッダ名
   *  \return     探索に成功すればヘッダへの相対またはフルパスを返す。失敗した場合は空文字列を返す。
   */
  template < class InputIterator >
    std::string search_include_file( InputIterator first, InputIterator last, std::string const& headername )
  {
    namespace fs = boost::filesystem;
//    fs::path filename( headername, fs::native );
    fs::path filename( headername );  // filesystem3対応

    if ( fs::exists( filename ) && !fs::is_directory( filename ) )
    {
      return headername;
    }
    while ( first != last )
    {
//      fs::path pathname = fs::path( *first, fs::native )/filename;
      fs::path pathname = fs::path( *first )/filename;  // filesystem3対応
      if ( fs::exists( pathname ) && !fs::is_directory( pathname ) )
      {
//        return pathname.native_file_string();
        return pathname.string();  // filesysytem3対応
      }
      ++first;
    }
    return std::string();
  }

  std::string expand_quote( std::string const& str );
  std::string quote_string( std::string const& str );

}

#endif  // ! TOPPERS_CPP_HPP_
