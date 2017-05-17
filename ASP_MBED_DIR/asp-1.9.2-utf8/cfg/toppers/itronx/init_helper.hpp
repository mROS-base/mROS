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
/*!
 *  \file   toppers/itronx/init_helper.hpp
 *  \brief  カーネルまたはモジュールの初期化処理ヘルパー関数の宣言・定義
 */
#ifndef TOPPERS_ITRONX_INIT_HELPER_HPP_
#define TOPPERS_ITRONX_INIT_HELPER_HPP_

#include <string>
#include <map>
#include <iostream>
#include "toppers/global.hpp"
#include "toppers/itronx/static_api.hpp"
#include "toppers/itronx/asp/factory.hpp"

namespace toppers
{
  namespace itronx
  {

    /*!
     *  \brief      カーネルまたはモジュールの初期化処理ヘルパー関数
     *  \param[in]  module_name   モジュール名
     *  \param[in]  info_table    静的APIに関する情報テーブル
     *  \param[in]  table_count   info_table の要素数
     *  retval      true          初期化成功
     *  retval      false         初期化失敗
     */
    template < class Factory >
      bool init_helper( std::string const& module_name, static_api::info const* info_table, std::size_t table_count )
    {
      try
      {
        std::map< std::string, static_api::info > temp_map;
        for ( std::size_t i = 0; ( i < table_count ) && ( info_table[ i ].type != 0 ); ++i )
        {
          temp_map[ info_table[ i ].api_name ] = info_table[ i ];
        }
        static std::map< std::string, static_api::info > const info_map( temp_map );
        global( module_name + "_static_api_info_map" ) = static_cast< void const* >( &info_map );	// GCCバグ対策

        static Factory factory;
        global( module_name + "_factory" ) = static_cast< itronx::factory* >( &factory );
      }
      catch ( ... )
      {
        std::cerr << "module `" << module_name << "\' is failed to initialize" << std::endl;
        return false;
      }
      return true;
    }

  }
}

#endif  // ! TOPPERS_ITRONX_INIT_HELPER_HPP_
