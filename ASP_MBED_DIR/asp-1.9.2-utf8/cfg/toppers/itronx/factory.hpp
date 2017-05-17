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
 *  \file   toppers/itronx/factory.hpp
 *  \brief  カーネルまたはモジュールに応じた処理オブジェクト生成に関する宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class factory;
 *  \endcode
 */
#ifndef TOPPERS_ITRONX_FACTORY_HPP_
#define TOPPERS_ITRONX_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>
#include "toppers/macro_processor.hpp"
#include "toppers/itronx/static_api.hpp"
#include "toppers/itronx/cfg1_out.hpp"
#include "toppers/itronx/checker.hpp"
#include "toppers/itronx/component.hpp"

namespace toppers
{

  namespace itronx
  {

    /*!
     *  \class  factory factory.hpp "toppers/itronx/factory.hpp"
     *  \brief  カーネルまたはモジュールに応じた処理オブジェクト生成クラス
     */
    class factory
    {
    public:
      typedef itronx::cfg1_out cfg1_out;
      typedef itronx::checker checker;
      typedef std::map< std::string, static_api::info > cfg_info;
      typedef itronx::component component;
      static bool const is_itronx = true;

      explicit factory( std::string const& kernel );
      virtual ~factory();
      std::map< std::string, static_api::info > const* get_static_api_info_map() const;
      cfg1_out::cfg1_def_table const* get_cfg1_def_table() const;
      std::auto_ptr< cfg1_out > create_cfg1_out( std::string const& filename ) const
      {
        return do_create_cfg1_out( filename );
      }
      std::auto_ptr< checker > create_checker() const
      {
        return do_create_checker();
      }
      std::auto_ptr< macro_processor > create_macro_processor( cfg1_out const& cfg1out ) const;
      std::auto_ptr< macro_processor > create_macro_processor( cfg1_out const& cfg1out, cfg1_out::static_api_map const& api_map ) const
      {
        return do_create_macro_processor( cfg1out, api_map );
      }
      std::auto_ptr< macro_processor > create_macro_processor( cfg1_out const& cfg1out, std::auto_ptr< component >& component_ptr ) const
      {
        std::auto_ptr< macro_processor > mproc( do_create_macro_processor( cfg1out, cfg1out.get_static_api_array() ) );
        component_ptr.reset( new component( mproc.get() ) );
        return mproc;
      }
      void swap( factory& other ) { do_swap( other ); }

      cfg_info const& get_cfg_info() const
      {
        return *get_static_api_info_map();
      }
    protected:
      virtual void do_swap( factory& other );
      virtual std::auto_ptr< macro_processor > do_create_macro_processor( cfg1_out const& cfg1out, cfg1_out::static_api_map const& api_map ) const;
      virtual std::auto_ptr< macro_processor > do_create_macro_processor( cfg1_out const& cfg1out, std::vector< static_api > const& api_array ) const;
    private:
      virtual std::auto_ptr< cfg1_out > do_create_cfg1_out( std::string const& filename ) const;
      virtual std::auto_ptr< checker > do_create_checker() const;

      std::string kernel_;
    };

  }
}

#endif  // ! TOPPERS_ITRONX_FACTORY_HPP_
