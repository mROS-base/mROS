/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
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
#include <fstream>
#include <map>
#include "toppers/workaround.hpp"
#include "toppers/s_record.hpp"
#include "toppers/nm_symbol.hpp"
#include "toppers/misc.hpp"
#include "toppers/diagnostics.hpp"
#include "toppers/oil/checker.hpp"
#include "toppers/oil/cfg1_out.hpp"

namespace toppers
{
  namespace oil
  {

    struct checker::implementation
    {
    protected:
      std::tr1::shared_ptr< s_record > srec_;
      std::tr1::shared_ptr< nm_symbol > syms_;

      virtual ~implementation() {}
      virtual bool do_check( cfg1_out& cfg1out, std::map< std::string, std::size_t > const& count_map )
      {
        return true;
      }
      virtual implementation* do_clone() const
      {
        return new implementation( *this );
      }

      friend class checker;
    };

    checker::checker()
      : pimpl_( new implementation )
    {
    }

    /*!
     *  \brief  コピーコンストラクタ
     *  \param[in]  コピー元
     */
    checker::checker( checker const& other )
      : pimpl_( other.pimpl_->do_clone() )
    {
    }

    //! デストラクタ
    checker::~checker()
    {
      delete pimpl_;
      pimpl_ = 0;
    }

    /*!
     *  \brief  ROMイメージのロード
     *  \param[in]  srec_file   Sレコード形式のファイル名
     *  \param[in]  nm_file     nmコマンドが出力したシンボルテーブルのファイル名
     */
    void checker::load_rom_image( std::string const& srec_file, std::string const& nm_file )
    {
      std::ifstream ifs_srec( srec_file.c_str() );
      if ( !ifs_srec.is_open() )
      {
        fatal( _( "cannot open file `%1%\'" ), srec_file );
      }
      std::tr1::shared_ptr< s_record > srec( new s_record( ifs_srec ) );

      std::ifstream ifs_nm( nm_file.c_str() );
      if ( !ifs_nm.is_open() )
      {
        fatal( _( "cannot open file `%1%\'" ), nm_file );
      }
      std::tr1::shared_ptr< nm_symbol > syms( new nm_symbol( ifs_nm ) );

      pimpl_->srec_.swap( srec );
      pimpl_->syms_.swap( syms );
    }

    /*!
     *  \brief  チェックの実施
     *  \param[in]  cfg1_out情報
     */
    bool checker::check( cfg1_out& cfg1out ) const
    {
      cfg1_out::cfg_obj_map obj_map( cfg1out.merge() );

      std::map< std::string, std::size_t > count_map;
      for ( cfg1_out::cfg_obj_map::const_iterator iter( obj_map.begin() ), last( obj_map.end() );
            iter != last;
            ++iter )
      {
        std::size_t count = 0;
        for ( std::vector< object_definition* >::const_iterator iter2( iter->second.begin() ), last2( iter->second.end() );
              iter2 != last2;
              ++iter2 )
        {
          // if ( !iter2->get_info()->slave )
          {
            ++count;
          }
        }
        count_map[ iter->first ] = count;
      }

      return pimpl_->do_check( cfg1out, count_map );
    }

    /*!
     *  \brief  シンボル情報の検索
     *  \param[in]  symbol  検索するシンボル名
     *  \return     検索結果
     *
     *  検索に失敗した場合には、返却値の.typeフィールドに-1が設定される。
     */
    nm_symbol::entry checker::find( std::string const& symbol ) const
    {
      return pimpl_->syms_->find( symbol );
    }

    /*!
     *  \brief  指定アドレスの整数値を取得
     *  \param[in]  address   アドレス
     *  \param[in]  size      整数値を構成するバイト数
     *  \param[in]  little_endian   バイトオーダーがリトルエンディアンならtrueを指定する
     *  \return     取得した整数値を符号無しで返す。
     */
    std::tr1::uintmax_t checker::get( std::size_t address, std::size_t size, bool little_endian ) const
    {
      return pimpl_->srec_->get_value( address, size, little_endian );
    }

  }
}
