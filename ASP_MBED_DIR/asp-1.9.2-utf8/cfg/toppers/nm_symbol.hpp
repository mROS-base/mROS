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
 *  \file   toppers/nm_symbol.hpp
 *  \brief  "nm"コマンドで得られるシンボルテーブルを扱うための宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class nm_symbol;
 *  \endcode
 */
#ifndef TOPPERS_NM_SYMBOL_HPP_
#define TOPPERS_NM_SYMBOL_HPP_

#include "toppers/config.hpp"
#include <iosfwd>
#include <string>
#include <map>

namespace toppers
{

  /*!
   *  \class  nm_symbol nm_symbol.hpp "toppers/nm_symbol.hpp"
   *  \brief  `nm'コマンドで得られるシンボルテーブルを扱うためのクラス
   *
   *  \sa nm_symbol::entry
   */
  class nm_symbol
  {
  public:
    /*!
     *  \struct entry nm_symbol.hpp "toppers/nm_symbol.hpp"
     *  \brief  アドレスとタイプ情報を格納する構造体
     *
     *  `nm'コマンドの出力結果が、
     *  \code
     *  01234567 T foo
     *  \endcode
     *  となった場合、0x01234567が address に、'T'が type に格納されます。
     */
    struct entry
    {
      unsigned long address;
      int type;
    };

    /*!
     *  \brief  デフォルトコンストラクタ
     */
    nm_symbol() {}
    /*!
     *  \brief  コンストラクタ
     *  \param  istr  入力ストリーム
     */
    explicit nm_symbol( std::istream& istr ) { load( istr ); }

    void load( std::istream& istr );
    entry const find( std::string const& symbol ) const; 
  private:
    std::map< std::string, entry > symbol_map_;
  };

}

#endif  // ! TOPPERS_NM_SYMBOL_HPP_
