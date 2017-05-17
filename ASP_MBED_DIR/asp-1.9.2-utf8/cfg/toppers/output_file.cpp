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
#include <map>
#include <iostream>
#include <sstream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "toppers/workaround.hpp"
#include "toppers/output_file.hpp"
#include "toppers/io.hpp"
#include "toppers/global.hpp"
#include "toppers/diagnostics.hpp"

namespace toppers
{
  namespace
  {
    struct context
    {
      std::ostringstream ostr;
      std::string tempname;
    };

    typedef std::map< std::string, std::tr1::shared_ptr< context > > context_map;

    inline context_map& get_context_map()
    {
      static context_map m;
      return m;
    }
  }

  /*!
   *  \brief  出力ストリームの参照
   *  \return 出力ストリーム
   *
   *  設定されているファイルに出力するための出力ストリームを返す。
   *  既に出力ストリームが生成されている場合はその参照を返し、そうでなければ新たに出力ストリームを生成する。
   *
   *  ファイル名として、"stdout"が設定されていれば標準出力、"stderr"が設定されていれば標準エラー出力になる。
   */
  std::ostream& output_file::ostr() const
  {
    if ( filename_.empty() || filename_ == "stdout" )
    {
      return std::cout;
    }
    else if ( filename_ == "stderr" )
    {
      return std::cerr;
    }

    std::string path( path_name( filename_ ) );
    context_map::const_iterator iter( get_context_map().find( path ) );
    if ( iter == get_context_map().end() )
    {
      std::tr1::shared_ptr< context > ctx( new context );
      get_context_map().insert( std::make_pair( path, ctx ) );
      return ctx->ostr;
    }
    return iter->second->ostr;
  }

  /*!
   *  \brief  メモリ上に格納されている内容を実際にファイルに出力する。
   *
   *  context_map としてメモリ上に格納された内容を、それぞれのファイルに対して出力する。
   *  出力するにあたり、まず元のファイルを .org を付けたファイル名に変更する。
   *  ここで、以前にあった .org は破壊される。
   *  ファイルを順に書き込み、途中でエラーが発生した場合は、.org を元のファイル名に戻す。
   *  途中でひとつでもエラーが発生した場合は、全ファイルについて .org を元に戻す。
   *  以前にあった .org は復活しない。
   */
  void output_file::save()
  {
    namespace fs = boost::filesystem;
    std::vector< std::string > saved_files;

    try
    {
      for ( context_map::const_iterator iter( get_context_map().begin() ), last( get_context_map().end() );
            iter != last;
            ++iter )
      {
        if ( iter->first != "" )
        {
//          fs::path filename( iter->first, fs::native );
          fs::path filename( iter->first );  // filesystem3対応
//          fs::path backup( iter->first + ".org", fs::native );
          fs::path backup( iter->first + ".org" );  // filesystem3対応
          bool existed = fs::exists( filename );

          try
          {
//            std::string file( filename.native_file_string() );
            std::string file( filename.string() );  // filesystem3対応
            if ( existed )
            {
              fs::rename( filename, backup );
            }
            write( file, iter->second->ostr.str() );  // ここで書き込む
            saved_files.push_back( file );
            fs::remove( backup );
          }
          catch ( ... )
          {
            if ( existed )
            {
              fs::remove( filename );
              fs::rename( backup, filename ); 
            }
            throw;
          }
        }
      }
    }
    catch ( ... )
    {
      // すでに書き込んだファイルを元に戻す。
      for ( std::vector< std::string >::const_iterator iter( saved_files.begin() ), last( saved_files.end() );
            iter != last;
            ++iter )
      {
//        fs::path filename( *iter, fs::native );
        fs::path filename( *iter );  // filesystem3対応
        fs::remove( filename );
//        fs::path backup( *iter + ".org", fs::native );
        fs::path backup( *iter + ".org" );  // filesystem3対応
        if ( fs::exists( backup ) )
        {
          fs::rename( backup, filename );
        }
      }
      throw;
    }
  }

  /*!
   *  \brief  指定したファイル名に対応したパス名を返す。
   *  \param[in]  filename  ファイル名
   *  \return     パス名
   *
   *  --output-directory オプションで指定した出力先ディレクトリを反映したパス名を生成する。
   *  ただし、下記のファイル名の場合はそのままの文字列を返す。
   *  - "stdout"
   *  - "stderr"
   *  - ""
   */
  std::string output_file::path_name( std::string const& filename )
  {
    if ( filename == "stdin" || filename == "stderr" || filename == "" )
    {
      return filename;
    }
    namespace fs = boost::filesystem;
    boost::any output_directory = global( "output-directory" );
    if ( output_directory.empty() )
    {
      return filename;
    }
    fs::path dir( get_global_string( "output-directory" )  );  // filesystem3対応
    return ( dir/filename ).string();  // filesystem3対応
  }

  /*!
   *  \brief  指定ファイルへの書き込みデータを参照
   *  \param[in]  filename  出力先のファイル名
   *  \return     書き込みデータ
   */
  std::string output_file::get_file_data( std::string const& filename )
  {
    context_map::const_iterator iter( get_context_map().find( path_name( filename ) ) ), last( get_context_map().end() );
    if ( iter != last )
    {
      return iter->second->ostr.str();
    }
    return "";
  }

  /*!
   *  \brief  指定ファイルへの書き込みデータを設定
   *  \param[in]  filename  出力先のファイル名
   *  \param[in]  data      書き込みデータ
   *  \retval     true      設定成功
   *  \retval     false     設定失敗
   */
  bool output_file::set_file_data( std::string const& filename, std::string const& data )
  {
    output_file ofile( filename, std::ios_base::out );
    dynamic_cast< std::ostringstream& >( ofile.ostr() ).str( data );
    return true;
  }

  /*!
   *  \brief  メモリ上に格納されている書き込みようデータを消去
   *  \param[in]  filename  出力先のファイル名
   *  \retval     true      データの消去成功
   *  \retval     false     指定したファイルへの出力データが存在しない
   */
  bool output_file::clear_file_data( std::string const& filename )
  {
    context_map::const_iterator iter( get_context_map().find( path_name( filename ) ) ), last( get_context_map().end() );
    if ( iter != last )
    {
      iter->second->ostr.str( "" );
      return true;
    }
    return false;
  }

}
