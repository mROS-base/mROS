/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2012 by TAKAGI Nobuhisa
 *  Copyright (C) 2010 by Meika Sugimoto
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
#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <stack>
#include "toppers/text.hpp"
#include "toppers/diagnostics.hpp"
#include "toppers/c_expr.hpp"
#include "toppers/global.hpp"
#include "toppers/macro_processor.hpp"
#include "toppers/s_record.hpp"
#include "toppers/nm_symbol.hpp"
#include "toppers/oil/cfg1_out.hpp"
#include "toppers/oil/preprocess.hpp"
#include "toppers/oil/configuration_manager.hpp"
#include <boost/spirit/include/classic_spirit.hpp>
#include <boost/filesystem/path.hpp>

using namespace toppers::oil::oil_definition;
using namespace toppers::configuration_manager;

namespace toppers
{
  namespace oil
  {
    namespace
    {
      struct block_t
      {
        std::string type;
        std::string id;
        text_line line;
      };
    }

    //! cfg_out クラスの実装詳細
    struct cfg1_out::implementation
    {
    protected:
      output_file ofile_;
      oil_def *oil_def_array;
      std::string cfg1_out_list_;
      std::string includes_;
      std::vector< block_t > block_table_;

      std::tr1::shared_ptr< s_record > srec_;
      std::tr1::shared_ptr< nm_symbol > syms_;
      bool little_endian_;
      std::size_t base_;
      cfg1_def_table const* def_table_;
      config_manage oil_configuration;

      implementation( std::string const& filename, std::ios_base::openmode omode, cfg1_def_table const* def_table = 0 )
        : ofile_( filename, omode ), little_endian_( true ), base_( 1 ), def_table_( def_table )
      {
      }
      virtual ~implementation()
      {
      }
      virtual void do_load_cfg( std::string const& input_file, codeset_t codeset,  std::vector<std::string> const& obj_info );
      virtual void do_generate_includes() const
      {
      }
      virtual void do_generate_cfg1_def() const
      {
        ofile_ << "const uint32_t TOPPERS_cfg_magic_number = 0x12345678;\n"
                  "const uint32_t TOPPERS_cfg_sizeof_signed_t = sizeof(signed_t);\n"
                  "const uint32_t TOPPERS_cfg_sizeof_pointer = sizeof(const volatile void*);\n"
                  "const unsigned_t TOPPERS_cfg_CHAR_BIT = ((unsigned char)~0u == 0xff ? 8 : 16);\n"  // CHAR_BITが8または16ビットであることを仮定
                  "const unsigned_t TOPPERS_cfg_CHAR_MAX = ((char)-1 < 0 ? (char)((unsigned char)~0u >> 1) : (unsigned char)~0u);\n"
                  "const unsigned_t TOPPERS_cfg_CHAR_MIN = ((char)-1 < 0 ? -((unsigned char)~0u >> 1) - 1 : 0);\n"
                  "const unsigned_t TOPPERS_cfg_SCHAR_MAX = (signed char)((unsigned char)~0u >> 1);\n"
                  "const unsigned_t TOPPERS_cfg_SHRT_MAX = (short)((unsigned short)~0u >> 1);\n"
                  "const unsigned_t TOPPERS_cfg_INT_MAX = (int)(~0u >> 1);\n"
                  "const unsigned_t TOPPERS_cfg_LONG_MAX = (long)(~0ul >> 1);\n"
                  "\n";

        if ( def_table_ != 0 )	// 「値取得シンボルテーブル」
        {
          for ( cfg1_def_table::const_iterator iter( def_table_->begin() ), last( def_table_->end() );
                iter != last;
                ++iter )
          {
            // 式の最初に # があれば、それはマクロ定義の判定
            // ★注意★ #@ で始まる書式は廃止 2010/07/23
            bool is_pp = ( iter->expression[ 0 ] == '#' );
            if ( !iter->value1.empty() || !iter->value2.empty() ) // （CSVの）4番目または5番目の値があれば...
            {
              is_pp = true;
            }

            std::string type = ( iter->is_signed ? "signed_t" : "unsigned_t" );
            std::string definition = "const " + type + " ";
            definition += "TOPPERS_cfg_" + iter->name;
            if ( is_pp )
            {
              std::string expression = iter->expression.substr( iter->expression[ 0 ] == '#' ? 1 : 0 );
              std::string value1 = ( !iter->value1.empty() ? iter->value1 : "1" );
              std::string value2 = ( !iter->value2.empty() ? iter->value2 : "0" );
              definition +=
                          " = \n"
                          "#if " + expression + "\n"
                          "(" + value1 + ");\n"
                          "#else\n"
                          "(" + value2 + ");\n"
                          "#endif\n";
            }
            else if ( iter->expression[ 0 ] == '@' )  // '@'で始まればアドレス
            {
              definition = "const volatile void* const TOPPERS_cfg_" + iter->name + " = (" + ( iter->expression.c_str() + 1 ) + ");\n";
            }
            else
            {
              definition +=
                          " = ( " + type + " )" + iter->expression + ";\n";
            }
            ofile_ << definition;
          }
        }
      }
      virtual void do_assign_params();
      virtual implementation* do_clone() const
      {
        return new implementation( *this );
      }

      void preprocess( std::string const& input_file, codeset_t codeset, text& txt );

      friend class cfg1_out;
    };


    /*!
     *  \brief  コンストラクタ
     *  \param[in]  filename    cfg1_out.c または cfg1_out.srec 若しくはそれらの代替名
     *  \param[in]  def_table   cfg1_out.c 生成用の定義テーブル
     */
    cfg1_out::cfg1_out( std::string const& filename, cfg1_def_table const* def_table )
      : pimpl_( new implementation( filename, std::ios_base::out, def_table ) )
    {
    }

    /*!
     *  \brief  コピーコンストラクタ
     *  \param[in]  other   コピー元
     */
    cfg1_out::cfg1_out( cfg1_out const& other )
      : pimpl_( other.pimpl_->do_clone() )
    {
    }

    //! デストラクタ
    cfg1_out::~cfg1_out()
    {
      delete pimpl_;
      pimpl_ = 0;
    }

    /*!
     *  \brief  システムコンフィギュレーションファイルのロード
     *  \param[in]  input_file  入力ファイル名
     *  \param[in]  codeset     文字コード
     *  \param[in]  obj_info    オブジェクト情報の連想配列
     */
    void cfg1_out::load_cfg( std::string const& input_file, codeset_t codeset, 
        std::vector<std::string> const& obj_info )
    {
      return pimpl_->do_load_cfg( input_file, codeset, obj_info );
    }

    //! 前処理
    void cfg1_out::implementation::preprocess( std::string const& input_file, codeset_t codeset, text& txt )
    {
      boost::any print_depend = global( "print-dependencies" );
      if ( !print_depend.empty() )
      {
        std::set< std::string > depend, onces;
        oil::preprocess( input_file, txt, codeset, &depend, &onces );

        // 依存関係の出力（GNU makeに適した形式）
        std::string target_file = boost::any_cast< std::string& >( print_depend );
        std::cout << target_file << ": " << input_file << ' ';
        std::copy( depend.begin(), depend.end(), std::ostream_iterator< std::string >( std::cout, " " ) );
        std::cout << std::endl;
        exit();
      }
      else
      {
        std::set< std::string > onces;
        oil::preprocess( input_file, txt, codeset, 0, &onces );
      }
    }

    /*!
     *  \brief  システムコンフィギュレーションファイルのロード処理の実体
     *  \param[in]  input_file  入力ファイル名
     *  \param[in]  codeset     文字コード
     *  \param[in]  obj_info    オブジェクト情報の連想配列
     */
    void cfg1_out::implementation::do_load_cfg( std::string const& input_file, 
                codeset_t codeset, std::vector<std::string> const& obj_info )
    {

      text txt;
      // システムコンフィギュレーションファイルの解析
      std::ostringstream oss, includes_oss;

      preprocess( input_file, codeset, txt );

      // OIL情報の読み出し
      if(oil_configuration.read_configuration(&txt , obj_info) == true)
      {
        oil_def_array = oil_configuration.get_obj_def();
        oil_configuration.validate_and_assign_default_configuration();
      }

      // データメンバへの反映
      std::string cfg1_list_temp( oss.str() );
      std::string includes_temp( includes_oss.str() );

      cfg1_out_list_.swap( cfg1_list_temp );
      includes_.swap( includes_temp );
    }

    /*!
     *  \brief  cfg1_out.c の内容生成
     *  \param[in]  type  配列 cfg1_out[] の要素型。空ポインタの場合は uint32_t として扱われる。
     */
    void cfg1_out::generate( char const* type ) const
    {
      if ( type == 0 )
      {
        type = "uint32_t";
      }
      pimpl_->ofile_ << "/* cfg1_out.c */\n"
                        "#define TOPPERS_CFG1_OUT  1\n" 
                        "#include \"osek_kernel.h\"\n"
                        "#include \"target_cfg1_out.h\"\n";
      pimpl_->do_generate_includes();
      pimpl_->ofile_ << pimpl_->includes_ << '\n';

      // int128_tは故意に無視
      // int128_tに揃えると処理が重くなりすぎるため
      if ( get_global< int >( "atk" ) < 2 )
      {
        pimpl_->ofile_ << "\n#ifdef INT64_MAX\n"
                          "  typedef int64_t signed_t;\n"
                          "  typedef uint64_t unsigned_t;\n"
                          "#else\n"
                          "  typedef int32_t signed_t;\n"
                          "  typedef uint32_t unsigned_t;\n"
                          "#endif\n";
      }
      else
      {
        pimpl_->ofile_ << "\n#ifdef INT64_MAX\n"
                          "  typedef sint64 signed_t;\n"
                          "  typedef uint64 unsigned_t;\n"
                          "#else\n"
                          "  typedef sint32 signed_t;\n"
                          "  typedef uint32 unsigned_t;\n"
                          "#endif\n";
      }

      pimpl_->ofile_ << "\n#include \"target_cfg1_out.h\"\n\n";

      pimpl_->do_generate_cfg1_def();
      pimpl_->ofile_ << pimpl_->cfg1_out_list_ << '\n';
    }

    /*!
     *  \brief  #include指令の並びを取得する
     *  \return #include指令の並び
     *
     *  このメンバ関数は、
     *  \code
     *  #include <...>
     *  #include "..."
     *  \endcode
     *  といった#include指令の並びを文字列として返す。
     */
    std::string const& cfg1_out::get_includes() const
    {
      return pimpl_->includes_;
    }

    /*!
     *  \brief  Sレコードのロード
     */
    void cfg1_out::load_srec()
    {
      std::ifstream srec_ifs( ( pimpl_->ofile_.file_name() + ".srec" ).c_str() );
      if ( !srec_ifs.is_open() )
      {
        fatal( _( "cannot open file `%1%\'" ), pimpl_->ofile_.file_name() + ".srec" );
      }
      pimpl_->srec_ = std::tr1::shared_ptr< s_record >( new s_record( srec_ifs ) );

      std::ifstream syms_ifs( ( pimpl_->ofile_.file_name() + ".syms" ).c_str() );
      if ( !syms_ifs.is_open() )
      {
        fatal( _( "cannot open file `%1%\'" ), pimpl_->ofile_.file_name() + ".syms" );
      }
      pimpl_->syms_ = std::tr1::shared_ptr< nm_symbol >( new nm_symbol( syms_ifs ) );

      nm_symbol::entry nm_entry = pimpl_->syms_->find( "TOPPERS_cfg_magic_number" );
      if ( nm_entry.type < 0 )
      {
        fatal( _( "magic number is not found in `%1%\'" ), ( pimpl_->ofile_.file_name() + ".srec/.syms" ) );
      }
      unsigned long magic[ 4 ];
      magic[ 0 ] = ( *pimpl_->srec_ )[ nm_entry.address + 0 ];
      magic[ 1 ] = ( *pimpl_->srec_ )[ nm_entry.address + 1 ];
      magic[ 2 ] = ( *pimpl_->srec_ )[ nm_entry.address + 2 ];
      magic[ 3 ] = ( *pimpl_->srec_ )[ nm_entry.address + 3 ];
      unsigned long magic_number = ( magic[ 0 ] << 24 ) | ( magic[ 1 ] << 16 ) | ( magic[ 2 ] << 8 ) | magic[ 3 ];
      if ( magic_number == 0x12345678 )
      {
        pimpl_->little_endian_ = false;
      }
      else if ( magic_number == 0x78563412 )
      {
        pimpl_->little_endian_ = true;
      }
      else
      {
        fatal( _( "magic number is not found in `%1%\'" ), ( pimpl_->ofile_.file_name() + ".srec/.syms" ) );
      }
      pimpl_->do_assign_params();
    }

    /*!
     *  \brief  "cfg1_out.srec" から読み取った情報の参照
     */
    std::tr1::shared_ptr< s_record > cfg1_out::get_srec() const
    {
      return pimpl_->srec_;
    }

    /*!
     *  \brief  "cfg1_out.syms" から読み取った情報の参照
     */
    std::tr1::shared_ptr< nm_symbol > cfg1_out::get_syms() const
    {
      return pimpl_->syms_;
    }

    cfg1_out::cfg1_def_table const* cfg1_out::get_def_table() const
    {
      return pimpl_->def_table_;
    }

    /*!
     *  \brief  オブジェクトごとにOILオブジェクト情報をまとめる
     *  \return 静的API情報
     *
     *  この関数は、"tsk"や"sem"といった種別をキーとして、その種別に分類される静的API情報の連想配列を生成する。
     */
    cfg1_out::cfg_obj_map cfg1_out::merge() const
    {
      cfg_obj_map result;
      result = oil_definition::merge(pimpl_->oil_def_array , result);
      return result;
    }

    /*!
     *  \brief  リトルエンディアンかどうかの判定
     *  \retval     true  リトルエンディアン
     *  \retval     false ビッグエンディアン
     *  \attention  load_srec 呼び出し前は正しい結果を得られない。
     */
    bool cfg1_out::is_little_endian() const
    {
      return pimpl_->little_endian_;
    }

    /*!
     *  \brief  静的APIのパラメータにSレコードから取得した値を代入する
     */
    void cfg1_out::implementation::do_assign_params()
    {
    }

    /*!
     *  \brief  --id-input-fileオプションで指定したファイルの読み込み
     *  \id_map 読み込んだデータの格納先
     */
    void cfg1_out::load_id_input_file( std::map< std::string, std::pair< long, bool > >& id_map )
    {
      std::string id_input_file( get_global_string( "id-input-file" ) );
      if ( id_input_file.empty() )
      {
        return;
      }

      std::ifstream ifs( id_input_file.c_str() );
      while ( ifs )
      {
        std::string linebuf;
        std::getline( ifs, linebuf );
        if ( ifs.bad() )
        {
          fatal( _( "I/O error" ) );
        }
        if ( linebuf.empty() || linebuf == "\r" )
        {
          break;
        }

        std::istringstream iss( linebuf );
        std::string name;
        iss >> name;
        if ( iss.fail() )
        {
          fatal( _( "id file `%1%\' is invalid" ), id_input_file );
        }

        long value;
        iss >> value;
        if ( iss.fail() )
        {
          fatal( _( "id file `%1%\' is invalid" ), id_input_file );
        }

        if ( id_map.find( name ) != id_map.end() )
        {
          fatal( _( "`%1%\' is duplicated" ), name );
        }
        else
        {
          id_map[ name ] = std::make_pair( value, false );
        }
      }
    }
  }
}
