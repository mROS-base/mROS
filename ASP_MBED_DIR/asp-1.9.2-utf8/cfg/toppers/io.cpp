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
#include <fstream>
#include <algorithm>
#include <iterator>
#include "toppers/io.hpp"
#include "toppers/diagnostics.hpp"
#include <boost/format.hpp>

namespace toppers
{

  void read( std::string const& filename, std::string& buf, std::ios_base::openmode omode )
  {
    std::ifstream ifs( filename.c_str(), std::ios_base::in | omode );
    if ( !ifs.is_open() )
    {
      throw io_error( str( boost::format( _( "cannot open file `%1%\'" ) ) % filename ) );
    }
    std::string t;
    char c;
    while ( ifs.get( c ) )
    {
      t.push_back( c );
    }
    if ( ifs.bad() )
    {
      throw io_error( _( "I/O error" ) );
    }
    buf.swap( t );
  }

  void write( std::string const& filename, std::string const& buf, std::ios_base::openmode omode )
  {
    std::ofstream ofs( filename.c_str(), std::ios_base::out | omode );
    if ( !ofs.is_open() )
    {
      throw io_error( str( boost::format( _( "cannot open file `%1%\'" ) ) % filename ) );
    }
    std::copy( buf.begin(), buf.end(), std::ostream_iterator< char >( ofs ) );
    if ( ofs.bad() )
    {
      throw io_error( _( "I/O error" ) );
    }
  }

}
