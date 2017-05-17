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
 *  \file   toppers/s_record.hpp
 *  \brief  モトローラSレコードを扱うための宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class s_record;
 *  struct s_record::record;
 *  class s_record::format_error;
 *  class s_record::checksum_error;
 *  \endcode
 */
#ifndef TOPPERS_S_RECORD_HPP_
#define TOPPERS_S_RECORD_HPP_

#include "toppers/config.hpp"
#include "toppers/workaround.hpp"
#include <iosfwd>
#include <string>
#include <vector>
#include <utility>
#include <stdexcept>

namespace toppers
{

  /*!
   *  \class  s_record s_record.hpp "toppers/s_recored.hpp"
   *  \brief  モトローラSレコードを扱うためのクラス
   *
   *  \sa s_record::record, s_record::format_error, s_record::checksum_error
   */
  class s_record
  {
  public:
    typedef std::pair< unsigned long, std::vector< unsigned char > > value_type;
    typedef std::vector< value_type >::size_type size_type;

    /*!
     *  \struct record s_record.hpp "toppers/s_record.hpp"
     *  \brief  Sレコードの１行レコード情報を格納するための構造体
     */
    struct record
    {
      int type;
      int length;
      unsigned long address;
      std::vector< unsigned char > data;
      int checksum;
    };

    /*!
     *  \class  data_error s_record.hpp "toppers/s_record.hpp"
     *  \brief  Sレコードのデータエラー例外クラス
     */
    class data_error : public std::runtime_error
    {
    public:
      data_error() : std::runtime_error( "S-record data error" ) {}
    };
    /*!
     *  \class  format_error s_record.hpp "toppers/s_record.hpp"
     *  \brief  Sレコードの書式エラー例外クラス
     */
    class format_error : public std::runtime_error
    {
    public:
      format_error() : std::runtime_error( "S-record format error" ) {}
    };
    /*!
     *  \class  checksum_error s_record.hpp "toppers/s_record.hpp"
     *  \brief  Sレコードのチェックサムエラー例外クラス
     */
    class checksum_error : public std::runtime_error
    {
    public:
      checksum_error() : std::runtime_error( "S-record checksum error" ) {}
    };

    /*!
     *  \brief  デフォルトコンストラクタ
     */
    s_record() : cache_( data_.end() ) {}
    /*!
     *  \brief  コンストラクタ
     *  \param  istr  入力ストリーム
     *
     *  s_record クラスの生成と同時にデータのロードを行います。
     */
    explicit s_record( std::istream& istr ) { load( istr ); }
    /*!
     *  \brief  デストラクタ
     */
    virtual ~s_record() {}

    void load( std::istream& istr );
    int operator[]( size_type address ) const;
    boost::uintmax_t get_value( std::size_t base, std::size_t size, bool little_endian ) const;
    unsigned long lower_bound() const;
    unsigned long upper_bound() const;
  protected:
    static record const read_record( std::string const& rec_buf );
    static int xdigit_to_int( int ch );
  private:
    std::vector< value_type > data_;
    mutable std::vector< value_type >::const_iterator cache_;
  };

}

#endif  // ! TOPPERS_S_RECORD_HPP_
