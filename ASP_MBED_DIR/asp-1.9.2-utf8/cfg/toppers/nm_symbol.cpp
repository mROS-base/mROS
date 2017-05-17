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

/*
 *  toppers/nm_symbol.cpp
 */
#include "toppers/nm_symbol.hpp"
#include <istream>
#include <sstream>
#include <cctype>

namespace toppers
{

  /*!
   *  \brief  シンボルテーブルのロード
   *  \param  istr  入力ストリーム
   */
  void nm_symbol::load( std::istream& istr )
  {
    while ( istr )
    {
      std::string buf;
      std::getline( istr, buf );
      std::istringstream isstr( buf );

      unsigned long address = 0;
      char type;
      std::string symbol;
      if ( !buf.empty() && !std::isspace( static_cast< unsigned char >( buf[0] ) ) )
      {
        isstr >> std::hex >> address;
      }
      isstr >> type >> symbol;

      if ( !symbol.empty() )
      {
        entry e;
        e.address = address;
        e.type = static_cast< unsigned char >( type );
        symbol_map_[symbol] = e;
      }
    }
  }

  /*!
   *  \brief  シンボルの探索
   *  \param  symbol  探索するシンボル文字列
   *  \return シンボルに対応するエントリを返す
   */
  nm_symbol::entry const nm_symbol::find( std::string const& symbol ) const
  {
    std::map< std::string, entry >::const_iterator iter( symbol_map_.find( symbol ) );
    entry e = { 0, -1 };
    if ( iter != symbol_map_.end() )
    {
      e = iter->second;
    }
    else
    {
      iter = symbol_map_.find( "_" + symbol );  // 識別子に'_'が付加される場合に対応
      if ( iter != symbol_map_.end() )
      {
        e = iter->second;
      }
    }
    return e;
  }

}
