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
#ifndef TOPPERS_OIL_CHECKER_HPP_
#define TOPPERS_OIL_CHECKER_HPP_

#include <cstddef>
#include <string>
#include "toppers/workaround.hpp"
#include "toppers/nm_symbol.hpp"

namespace toppers
{
  namespace oil
  {

    class cfg1_out;

    /*!
     *  \class  checker checker.hpp "toppers/oil/checker.hpp"
     *  \brief  全体リンク後のパラメータチェック処理クラス
     */
    class checker
    {
    public:
      checker();
      checker( checker const& other );
      virtual ~checker();
      checker& operator=( checker const& other )
      {
        checker t( other );
        swap( t );
        return *this;
      }

      void load_rom_image( std::string const& srec_file, std::string const& nm_file );
      bool check( cfg1_out& cfg1out ) const;
      nm_symbol::entry find( std::string const& symbol ) const;
      std::tr1::uintmax_t get( std::size_t address, std::size_t size, bool little_endian ) const;

      void swap( checker& other )
      {
        implementation* t = pimpl_;
        pimpl_ = other.pimpl_;
        other.pimpl_ = t;
      }
    protected:
      struct implementation;
      explicit checker( implementation* pimpl ) : pimpl_( pimpl ) {}
    private:
      implementation* pimpl_;
    };

  }
}

#endif  // ! TOPPERS_OIL_CHECKER_HPP_
