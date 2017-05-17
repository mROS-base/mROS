/* 
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *  
 *  Copyright (C) 2007-2012 by TAKAGI Nobuhisa
 *  Copyright (C) 2010 by Meika Sugimoto
 *  Copyright (C) 2011-2012 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2012 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2012 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2011-2012 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2012 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2012 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2012 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2012 by TOSHIBA CORPOTATION, JAPAN
 *  Copyright (C) 2011-2012 by Witz Corporation, JAPAN
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
 *  $Id: factory.hpp 133 2012-12-18 04:30:30Z ertl-hiro $
 */

/*!
 *  \file   toppers/xml/factory.hpp
 *  \brief  カーネルまたはモジュールに応じた処理オブジェクト生成に関する宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class factory;
 *  \endcode
 */
#ifndef TOPPERS_XML_FACTORY_HPP_
#define TOPPERS_XML_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>
#include "toppers/macro_processor.hpp"
#include "toppers/xml/cfg1_out.hpp"
#include "toppers/xml/checker.hpp"

namespace toppers
{

  namespace xml
  {

    /*!
     *  \class  factory factory.hpp "toppers/xml/factory.hpp"
     *  \brief  カーネルまたはモジュールに応じた処理オブジェクト生成クラス
     */
    class factory
    {
    public:
      typedef xml::cfg1_out cfg1_out;
      typedef xml::checker checker;
      typedef std::map< std::string, toppers::xml::info > cfg_info;
      typedef struct {} component;  // ダミー
      static bool const is_itronx = false;

      explicit factory( std::string const& kernel );
      virtual ~factory();
      std::map< std::string, toppers::xml::info > const* get_container_info_map() const;
      cfg1_out::cfg1_def_table const* get_cfg1_def_table() const;
      std::auto_ptr< cfg1_out > create_cfg1_out( std::string const& filename ) const
      {
        return do_create_cfg1_out( filename );
      }
      std::auto_ptr< checker > create_checker() const
      {
        return do_create_checker();
      }
      std::auto_ptr< macro_processor > create_macro_processor( cfg1_out const& cfg1out ) const
      {
        cfg1_out::xml_obj_map xml_map( cfg1out.merge( *get_container_info_map() ) );
        return do_create_macro_processor( cfg1out, xml_map );
      }
      std::auto_ptr< macro_processor > create_macro_processor( cfg1_out const& cfg1out, cfg1_out::xml_obj_map const& xml_map ) const
      {
        return do_create_macro_processor( cfg1out, xml_map );
      }
      std::auto_ptr< macro_processor > create_macro_processor( cfg1_out const& cfg1out, std::auto_ptr< component >& cmponent_ptr ) const
      {
        error( _( "with-software-components is not supported." ) );
        return std::auto_ptr< macro_processor >();
      }
      void swap( factory& other ) { do_swap( other ); }

      cfg_info const& get_cfg_info() const
      {
        return *get_container_info_map();
      }
    protected:
      virtual void do_swap( factory& other );
      virtual std::auto_ptr< macro_processor > do_create_macro_processor( cfg1_out const& cfg1out, cfg1_out::xml_obj_map const& xml_map ) const;
    private:
      virtual std::auto_ptr< cfg1_out > do_create_cfg1_out( std::string const& filename ) const;
      virtual std::auto_ptr< checker > do_create_checker() const;

      std::string kernel_;
    };

  }
}

#endif  /* TOPPERS_XML_FACTORY_HPP_ */
