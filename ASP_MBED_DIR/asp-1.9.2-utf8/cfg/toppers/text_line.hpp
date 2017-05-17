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
 *  \file   toppers/text_line.hpp
 *  \brief  テキストデータの行に関する宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  struct text_line;
 *  struct basic_line_buf< CharT, Traits, Allocator >;
 *  \endcode
 */
#ifndef TOPPERS_TEXTLINE_HPP_
#define TOPPERS_TEXTLINE_HPP_

#include <iosfwd>
#include <string>
#include <algorithm>
#include "toppers/config.hpp"

namespace toppers
{

  /*!
   *  \struct text_line text_line.hpp "toppers/text_line.hpp"
   *  \brief  ファイル名と行番号の保持クラス
   */
  struct text_line
  {
    std::string file;   //!< ファイル名
    long line;          //!< 行番号（1〜）

    /*!
     *  \brief  デフォルトコンストラクタ
     */
    text_line()
      : line( 0 )
    {
    }
    /*!
     *  \brief  コンストラクタ
     *  \param  filename  ファイル名
     *  \param  lineno    行番号
     */
    explicit text_line( std::string const& filename, long lineno = 0 )
      : file( filename ), line( lineno )
    {
    }
    /*!
     *  \brief  コンストラクタ
     *  \param  filename  ファイル名
     *  \param  lineno    行番号
     */
    explicit text_line( char const* filename, long lineno = 0 )
      : file( filename ), line( lineno )
    {
    }
    /*!
     *  \brief  オブジェクトの交換
     *  \param  other   交換対象のオブジェクト
     */
    void swap( text_line& other ) throw()
    {
      std::swap( line, other.line );
      file.swap( other.file );
    }
  };

  /*!
   *  \struct line_buf text_line.hpp "toppers/text_line.hpp"
   *  \brief  1 行テキストの管理クラス
   */
  template
  <
    typename CharT,
    class Traits = std::char_traits< CharT >,
    class Allocator = std::allocator< CharT >
  >
  struct basic_line_buf
  {
    typedef std::basic_string< CharT, Traits, Allocator > string_type;

    string_type buf; //!< 行データを表す文字列
    text_line line;  //!< 行番号情報

    /*!
     *  \brief  デフォルトコンストラクタ
     */
    basic_line_buf()
    {
    }
    /*!
     *  \brief  コンストラクタ
     *  \param  line  行番号情報
     */
    basic_line_buf( text_line const& line )
      : line( line )
    {
    }
    /*!
     *  \brief  コンストラクタ
     *  \param  line  行番号情報
     *  \param  data  行データ
     */
    basic_line_buf( text_line const& line, string_type const& data )
      : buf( data ), line( line )
    {
    }
  };

  //! char 版の 1 行テキスト管理クラス
  typedef basic_line_buf< char > line_buf;

  /*!
   *  \brief  イテレータが保持する行番号情報の取得
   *  \param  iter  イテレータ
   *  \return 行番号情報への参照を返す
   *
   *  iter で指定したイテレータが行番号情報を保持しているなら、その行番号情報を返します。
   *  行番号情報を保持していない場合はダミーオブジェクトへの参照を返します。
   */
  template < class Iterator >
  inline text_line const& get_text_line( Iterator iter )
  {
    static text_line dummy;
    return dummy;
  }

}

#endif  // ! TOPPERS_TEXTLINE_HPP_
