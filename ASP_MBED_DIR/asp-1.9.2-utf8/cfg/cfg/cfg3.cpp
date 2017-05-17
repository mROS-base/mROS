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
#include <fstream>
#include <sstream>
#include "toppers/diagnostics.hpp"
#include "toppers/s_record.hpp"
#include "toppers/macro_processor.hpp"
#include "toppers/itronx/component.hpp"
#include "toppers/itronx/component.hpp"
#include "cfg.hpp"
#include <boost/spirit/include/classic.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

namespace
{
  using toppers::text_line;
  typedef toppers::macro_processor::element element;
  typedef toppers::macro_processor::var_t var_t;
  typedef toppers::macro_processor::context context;

  /*!
   *  \brief  シンボルに対応するアドレスの取得
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   */
  template < class Checker >
  var_t bf_symbol( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    using namespace toppers;
    using namespace toppers::itronx;

    if ( macro_processor::check_arity( line, arg_list.size(), 1, "SYMBOL" ) )
    {
      std::string symbol( macro_processor::to_string( arg_list[0], p_ctx ) );
      std::tr1::shared_ptr< Checker > chk = get_global< std::tr1::shared_ptr< Checker > >( "checker" );
      nm_symbol::entry entry = chk->find( symbol );
      if ( entry.type >= 0 )
      {
        element e;
        e.i = entry.address;
        return var_t( 1, e );
      }
    }
    return var_t();
  }

  // VMA アドレス解決用テーブル
  std::vector< std::pair< std::tr1::int64_t, std::vector< unsigned char > > > vma_table;

  /*!
   *  \brief  指定したアドレスに格納されている値の取得
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *
   *  第1引数にアドレスを、第2引数に読み込むバイト数を指定します。
   */
  template < class Checker >
  var_t bf_peek( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    using namespace toppers;
    using namespace toppers::itronx;

    if ( macro_processor::check_arity( line, arg_list.size(), 2, "PEEK" ) )
    {
      std::size_t address = static_cast< std::size_t >( macro_processor::to_integer( arg_list[0], p_ctx ) );
      std::size_t size = static_cast< std::size_t >( macro_processor::to_integer( arg_list[1], p_ctx ) );
      std::tr1::shared_ptr< Checker > chk = get_global< std::tr1::shared_ptr< Checker > >( "checker" );

      std::map< std::string, var_t >::const_iterator le_iter( p_ctx->var_map.find( "LITTLE_ENDIAN" ) );
      if ( le_iter != p_ctx->var_map.end() )
      {
        bool little_endian = !!( *le_iter->second.front().i );
        long pos = -1;
        long base = 0;
        element e;

        for ( std::size_t i = 0, n = vma_table.size(); i < n; i++ )
        {
          if ( vma_table[i].first <= address && address < vma_table[i].first + vma_table[i].second.size() )
          {
            pos = i;
            base = static_cast< long >( address - vma_table[i].first );
          }
        }
        if ( pos >= 0 )  // VMA から読み取る
        {
          std::tr1::uint64_t value = 0;
          if ( little_endian )
          {
            for ( long j = static_cast< long >( size-1 ); j >= 0; j-- )
            {
              int t = vma_table[ pos ].second[ base + j ];
              if ( t < 0 )
              {
                return var_t();
              }
              value = ( value << 8 ) | ( t & 0xff );
            }
          }
          else
          {
            for ( std::size_t j = 0; j < size; j++ )
            {
              int t = vma_table[ pos ].second[ base + j ];
              if ( t < 0 )
              {
                return var_t();
              }
              value = ( value << 8 ) | ( t & 0xff );
            }
          }
          e.i = value;
        }
        else  // VMA ではないので、Sレコードから読み取る
        {
          e.i = chk->get( address, size, !!little_endian );
        }
        return var_t( 1, e );
      }
    }
    return var_t();
  }

  /*!
   *  \brief  メモリブロックの転送
   *  \param[in]  line      行番号
   *  \param[in]  arg_list  マクロ実引数リスト
   *  \param[in]  p_ctx     マクロコンテキスト
   *  \retval     マクロ返却値
   *
   *  第1引数に転送元アドレス、第2引数に転送先アドレス、第3引数に転送するバイト数を指定します。
   *  指定したコピー元アドレスからコピー先アドレスへ、指定バイト数のメモリブロックを転送します。
   *  この関数は、LMAからVMAへのアドレス変換を目的として使用することを想定しています。
   *
   *  \attention  この組み込み関数は、LMAからVMAへのアドレス変換を想定しているため、頻繁に転送を
   *              繰り返すような状況には対応していません（メモリ不足が発生します）。
   */
  template < class Checker >
  var_t bf_bcopy( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
  {
    using namespace toppers;
    using namespace toppers::itronx;

    if ( macro_processor::check_arity( line, arg_list.size(), 3, "BCOPY" ) )
    {
      std::size_t src = static_cast< std::size_t >( macro_processor::to_integer( arg_list[0], p_ctx ) );
      std::size_t dst = static_cast< std::size_t >( macro_processor::to_integer( arg_list[1], p_ctx ) );
      std::size_t size = static_cast< std::size_t >( macro_processor::to_integer( arg_list[2], p_ctx ) );
      std::tr1::shared_ptr< Checker > chk = get_global< std::tr1::shared_ptr< Checker > >( "checker" );

      std::pair< std::tr1::int64_t, std::vector< unsigned char > > block;
      block.first = dst;
      block.second.reserve( size );
      for ( std::tr1::int64_t i = 0; i < size; i++ )
      {
        block.second.push_back( static_cast< unsigned char >( chk->get( static_cast< std::size_t >( src + i ), 1, true ) ) );
      }
      vma_table.push_back( block );
    }
    return var_t();
  }

}

namespace
{
  template < class Factory>
  inline bool cfg3_main_implementation( std::string const& kernel )
  {
    using namespace toppers;
    typedef typename Factory::cfg1_out Cfg1_out;

    Factory factory( kernel );
    global( "factory" ) = &factory;

    // *.cfgとcfg1_out.srecの読み込み
    std::string input_file;
    try
    {
      get_global( "input-file", input_file );
    }
    catch ( boost::bad_any_cast& )
    {
      fatal( _( "no input files" ) );
    }
    std::string cfg1_out_name;
    get_global( "cfg1_out", cfg1_out_name );
    std::auto_ptr< Cfg1_out > cfg1_out( factory.create_cfg1_out( cfg1_out_name ) );

    codeset_t codeset = get_global< codeset_t >( "codeset" );
    cfg1_out->load_cfg( input_file, codeset, factory.get_cfg_info() );
    cfg1_out->load_srec();

    std::auto_ptr< typename Factory::checker > p_checker( factory.create_checker() );
    std::tr1::shared_ptr< typename Factory::checker > chk( p_checker );
    global( "checker" ) = chk;
    std::string rom_image( get_global_string( "rom-image" ) );
    std::string symbol_table( get_global_string( "symbol-table" ) );
    chk->load_rom_image( rom_image, symbol_table );

    // テンプレートファイル
    boost::any template_file( global( "template-file" ) );
    if ( template_file.empty() )
    {
      // テンプレートファイルが指定されていなければ最低限のチェックのみ（後方互換性のため）
      // パラメータチェック
      if ( !chk->check( *cfg1_out ) )
      {
        return false;
      }
    }
    else
    {
      namespace fs = boost::filesystem;

      // テンプレート処理
      std::auto_ptr< macro_processor > mproc;
      std::auto_ptr< typename Factory::component > component_ptr;

      if ( get_global_bool( "with-software-components" ) )
      {
        mproc = factory.create_macro_processor( *cfg1_out, component_ptr );
      }
      else
      {
        mproc = factory.create_macro_processor( *cfg1_out );
      }

      // ↓ 追加組み込み関数の登録
      toppers::macro_processor::func_t func_info = {};
      func_info.name = "SYMBOL";
      func_info.f = &bf_symbol< typename Factory::checker >;
      mproc->add_builtin_function( func_info );

      func_info.name = "PEEK";
      func_info.f = &bf_peek< typename Factory::checker >;
      mproc->add_builtin_function( func_info );

      func_info.name = "BCOPY";
      func_info.f = &bf_bcopy< typename Factory::checker >;
      mproc->add_builtin_function( func_info );
      // ↑ 追加組み込み関数の登録

      fs::path cfg_dir( get_global_string( "cfg-directory" ) );  // filesystem3対応
      std::vector< std::string > include_paths = get_global< std::vector< std::string > >( "include-path" );
      include_paths.push_back( cfg_dir.empty() ? "." : cfg_dir.string() );  // filesystem3対応

      toppers::text in_text;
      toppers::text pp_text;
      std::string file_name( boost::any_cast< std::string& >( template_file ) );

      in_text.set_line( file_name, 1 );
      std::ifstream ifs( file_name.c_str() );
      if ( !ifs.is_open() )
      {
        fatal( _( "`%1%` can not be found." ), file_name );
      }

      in_text.append( ifs );
      macro_processor::preprocess( in_text, pp_text );
      mproc->evaluate( pp_text );

      if ( get_error_count() > 0 )
      {
        return false;
      }
      // 出力ファイルがあるかどうか分からないが、一応セーブする。
      output_file::save();
    }

    // パス4以降からも流用されるため、現在処理中のパスを調べる。
    int pass = get_global< int >( "pass" );
    int max_pass = get_global< int >( "max-pass" );
    if ( max_pass == pass ) // 最終段階のパスが成功したときに"check complete"メッセージを出す。
    {
      std::cerr << _( "check complete" ) << std::endl;
    }

    return true;
  }
}

bool cfg3_main()
{
  std::string kernel;
  toppers::get_global( "kernel", kernel );

  if ( toppers::get_global_bool( "oil" ) )
  {
    return cfg3_main_implementation< toppers::oil::factory >( kernel );
  }
#ifdef  HAS_CFG_XML
  else if ( toppers::get_global_bool( "xml" ) )
  {
    return cfg3_main_implementation< toppers::xml::factory >( kernel );
  }
#endif
  else
  {
    return cfg3_main_implementation< toppers::itronx::factory >( kernel );
  }
}
