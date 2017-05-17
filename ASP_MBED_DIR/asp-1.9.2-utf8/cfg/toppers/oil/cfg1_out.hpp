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
 *  \file   toppers/oil/cfg1_out.hpp
 *  \brief  cfg1_out.c/.srec ファイルを扱うための宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class cfg1_out;
 *  \endcode
 */
#ifndef TOPPERS_OIL_CFG1_OUT_HPP_
#define TOPPERS_OIL_CFG1_OUT_HPP_

#include <string>
#include <vector>
#include <map>
#include "toppers/workaround.hpp"
#include "toppers/codeset.hpp"
#include "toppers/oil/oil_object.hpp"

namespace toppers
{

  class macro_processor;
  class s_record;
  class nm_symbol;

  namespace oil
  {
	  using namespace toppers::oil::oil_definition;

    /*!
     *  \class  cfg1_out  cfg1_out.hpp  "toppers/oil/cfg1_out.hpp"
     *  \brief  cfg1_out.c/.srec ファイル管理クラス
     */
    class cfg1_out
    {
    public:
      struct cfg1_def_t
      {
        bool is_signed;
        std::string name;
        std::string expression;
        std::string value1;
        std::string value2;
      };
      typedef std::map< std::string, std::vector< object_definition* > > cfg_obj_map;
      typedef cfg_obj_map cfg_element_map;
      typedef std::vector< cfg1_def_t > cfg1_def_table;

      explicit cfg1_out( std::string const& filename, cfg1_def_table const* def_table = 0 );
      cfg1_out( cfg1_out const& other );
      virtual ~cfg1_out();
      cfg1_out& operator = ( cfg1_out const& other )
      {
        cfg1_out t( other );
        swap( t );
        return *this;
      }

      void load_cfg( std::string const& input_file, codeset_t codeset, std::vector<std::string> const& obj_info );
      void generate( char const* type = 0 ) const;
      std::string const& get_includes() const;

      void load_srec();
      std::tr1::shared_ptr< s_record > get_srec() const;
      std::tr1::shared_ptr< nm_symbol > get_syms() const;
      cfg1_def_table const* get_def_table() const;
      cfg_obj_map merge(void) const;
      bool is_little_endian() const;

      void swap( cfg1_out& other )
      {
        implementation* t = pimpl_;
        pimpl_ = other.pimpl_;
        other.pimpl_ = t;
      }
      void swap(cfg_obj_map& other)
      {

      }

      static std::tr1::int32_t make_signed( std::tr1::uint32_t value )
      {
        std::tr1::int32_t result;
        if ( ( value >> 31 ) != 0 )
        {
          result = -static_cast< std::tr1::int32_t >( ( value ^ 0xffffffff ) + 1 ); // 2の補数表現にしか対応しない
        }
        else
        {
          result = static_cast< std::tr1::int32_t >( value );
        }
        return result;
      }

      static void load_id_input_file( std::map< std::string, std::pair< long, bool > >& id_map );

    protected:
      struct implementation;
      explicit cfg1_out( implementation* pimpl ) : pimpl_( pimpl ) {}
    private:
      implementation* pimpl_;
    };

  }
}

#endif  // ! TOPPERS_OIL_CFG1_OUT_HPP_
