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
 *  \file   toppers/output_file.hpp
 *  \brief  出力ファイル管理を扱うための宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class output_file;
 *  \endcode
 */
#ifndef TOPPERS_OUTPUT_FILE_HPP_
#define TOPPERS_OUTPUT_FILE_HPP_

#include <ostream>
#include <string>
#include "toppers/config.hpp"

namespace toppers
{

  /*!
   *  \class  output_file output_file.hpp "toppers/output_file.hpp"
   *  \brief  出力ファイル管理クラス
   */
  class output_file
  {
  public:
    output_file() : omode_( static_cast< std::ios_base::openmode >( 0 ) ) {}
    output_file( std::string const& filename, std::ios_base::openmode omode )
      : filename_( filename ), omode_( omode ), enable_( true ) {}
    virtual ~output_file() {}

    std::ostream& ostr() const;
    std::string const& file_name() const { return filename_; }
    bool is_enable() const { return enable_; }
    void enable( bool flag = true ) { enable_ = flag; }
    void push_back( char c )
    {
      if ( enable_ )
      {
        ostr() << c;
      }
    }

    static std::string path_name( std::string const& filename );
    static std::string get_file_data( std::string const& filename );
    static bool set_file_data( std::string const& filename, std::string const& data );
    static bool clear_file_data( std::string const& filename );
    static void save();
  private:
    std::string filename_;
    std::ios_base::openmode omode_;
    bool enable_;
  };

  template < typename T >
    inline output_file const& operator << ( output_file const& ofile, T const& value )
  {
    if ( ofile.is_enable() )
    {
      ofile.ostr() << value;
    }
    return ofile;
  }

}

#endif  // ! TOPPERS_OUTPUT_FILE_HPP_
