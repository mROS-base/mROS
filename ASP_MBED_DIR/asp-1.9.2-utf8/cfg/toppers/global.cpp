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
#include <new>
#include <map>
#include "toppers/global.hpp"

namespace toppers
{

  /*!
   *  \brief  グローバルオブジェクトへのアクセッサ
   *  \param[in]  key   オブジェクト名
   *  \return     key で指定したオブジェクトへの参照
   *
   *  グローバルオブジェクトへのアクセスを一元管理する。
   *  この関数を介してグローバルアクセスを行うことにより、通常の静的オブジェクトが持つ動的初期化の
   *  順序に関する問題が解消される。
   *  また、内部的に管理しているオブジェクトは、プログラム終了時にデストラクタが呼ばれることはない。
   *  （厳密にいえばメモリリークにあたるが、弊害はとくにない）
   */
  boost::any& global( std::string const& key )
  {
    static std::map< std::string, boost::any >* gvm = 0;
    if ( gvm == 0 )
    {
      union max_aligner
      {
        long long ll;
        double d;
        long double ld;
        void* pv;
        void (*pfn)();
      };
      const std::size_t size = ( sizeof( std::map< std::string, boost::any > ) + sizeof( max_aligner ) - 1 ) / sizeof( max_aligner );
      static max_aligner gvm_storage[ size ];
      gvm = new ( gvm_storage ) std::map< std::string, boost::any >;  // デストラクタは呼ばれない
    }
    return ( *gvm )[key];
  }

}
