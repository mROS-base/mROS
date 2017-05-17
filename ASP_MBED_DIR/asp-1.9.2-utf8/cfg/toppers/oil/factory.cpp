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
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/spirit/include/classic_spirit.hpp>
#include "toppers/misc.hpp"
#include "toppers/global.hpp"
#include "toppers/csv.hpp"
#include "toppers/nm_symbol.hpp"
#include "toppers/s_record.hpp"
#include "toppers/diagnostics.hpp"
#include "toppers/macro_processor.hpp"
#include "toppers/io.hpp"
#include "toppers/cpp.hpp"
#include "toppers/oil/factory.hpp"
#include "toppers/oil/cfg1_out.hpp"

namespace toppers
{
  namespace oil
  {
    namespace
    {

    object_definition* find_object(string name , cfg1_out::cfg_obj_map const& obj_def_map)
    {
          // modified by takuya 110823
      //std::map< std::string, std::vector<object_definition*>>::const_iterator p;
      std::map< std::string, std::vector<object_definition*> >::const_iterator p;
      std::vector<object_definition*>::const_iterator q;
    
      // 名前が一致するオブジェクトを検索
      for(p = obj_def_map.begin() ; p != obj_def_map.end() ; p++)
      {
        for(q = (*p).second.begin() ; q != (*p).second.end() ; q++)
        {
          if((*q)->get_name() == name)
          {
            return (*q);
          }
        }
      }    
      return NULL;
    }

      // カーネルオブジェクト生成・定義用静的APIの各パラメータをマクロプロセッサの変数として設定する。
      void set_object_vars( cfg1_out::cfg_obj_map const& obj_def_map, macro_processor& mproc )
      {
        typedef macro_processor::element element;
        typedef macro_processor::var_t var_t;
        std::map< std::string, var_t > order_list_map , obj_parameter;
        std::map< std::string, long > id_map;

    using namespace toppers::oil::oil_definition;
        // modified by takuya 110823
    //std::map< std::string, std::vector<object_definition*>>::const_iterator p;
    std::map< std::string, std::vector<object_definition*> >::const_iterator p;
    std::vector<object_definition*>::const_iterator q;
    std::vector<object_parameter_def*>::const_iterator r;

        for ( p = obj_def_map.begin() ; p != obj_def_map.end() ; p++)
        {
      element e;
      var_t t;
      string name;
      long obj_id;
      long param_id;
      
      // パラメータの値代入
      for(q = (*p).second.begin() ; q != (*p).second.end() ; q++)
      {
        std::map< std::string, var_t > obj_parameter;
        std::map< std::string, var_t >::iterator s;
        obj_id = (*q)->get_id();
        name = (*p).first;
        std::map<std::string , long> id_count;
        std::map<std::string , long>::iterator id_iter;

        // 値の取り出し
        e.i = (*q)->get_id();
        e.s = (*q)->get_name();
        // 出現順リスト用の情報作成
        order_list_map[ name ].push_back(e);
        // オブジェクト自身の値代入
              mproc.set_var( toppers::toupper(name), obj_id, var_t( 1, e ) );
        
        // オブジェクトメンバの値代入
        for(r = (*q)->get_params()->begin() ; r != (*q)->get_params()->end() ; r++)
        {
          name = (*p).first+ string(".") + (*r)->get_parameter_name();
          e.s = (*r)->get_value();

          if(e.s == string(""))
          {
            continue;
          }

          // メンバのIDを検索
          id_iter = id_count.find(name);
          if(id_iter == id_count.end())
          {
            id_count[name] = 0;
            param_id = 0;
          }
          else
          {
            param_id = (*id_iter).second + 1;
            id_count[name] = (*id_iter).second + 1;
          }

          if((*r)->get_type() == oil::TYPE_UINT)
          {
            string value_str(((*r)->get_value()));
            try
            {
              e.i = boost::lexical_cast<uint64_t>((*r)->get_value());
            }
            catch( std::exception& exception)
            {
              uint64_t temp;
              // 16進の場合があるので変換
              /// きれいじゃないのでなんとかしたい
              if(value_str.find("0x") == 0)
              {
                                // modified by takuya 110823
                //sscanf_s(value_str.c_str() , "0x%I64x" , &temp);
                sscanf(value_str.c_str() , "0x%llx" , &temp);
                e.i = temp;
              }
              else if(value_str.find("0X") == 0)
              {
                                // modified by takuya 110823
                //sscanf_s(value_str.c_str() , "0X%I64x" , &temp);
                sscanf(value_str.c_str() , "0X%llx" , &temp);
                e.i = temp;
              }
              else
              {
                // キャストに失敗したら0にしておく
                e.i = 0;
              }

            }
          }
          else if((*r)->get_type() == oil::TYPE_INT)
          {
            string value_str(((*r)->get_value()));
            try
            {
              e.i = boost::lexical_cast<int64_t>(value_str);
            }
            catch(std::exception& exception)
            {
              int64_t temp;
              // 16進の場合があるので変換
              /// きれいじゃないのでなんとかしたい
              if(value_str.find("0x") == 0)
              {
                                // modified by takuya 110823
                //sscanf_s(value_str.c_str() , "0x%I64x" , &temp);
                sscanf(value_str.c_str() , "0x%llx" , &temp);
                e.i = temp;
              }
              else if(value_str.find("0X") == 0)
              {
                                // modified by takuya 110823
                //sscanf_s(value_str.c_str() , "0X%I64x" , &temp);
                sscanf(value_str.c_str() , "0X%llx" , &temp);
                e.i = temp;
              }
              else
              {
                // キャストに失敗したら0にしておく
                e.i = 0;
              }
            }
          }
          else if((*r)->get_type() == oil::TYPE_REF)
          {
            object_definition *obj;
            string refefence_obj_type;

            // オブジェクトIDの探索
            e.i = 0;
            obj = find_object((*r)->get_value() , obj_def_map);
            if(obj != NULL)
            {
              e.i = obj->get_id();
            }
          }
          else
          {
            e.i = 0;
          }
          obj_parameter[name].push_back(e);
        }

        for(s = obj_parameter.begin() ; s != obj_parameter.end() ; s++)
        {
          mproc.set_var((*s).first , obj_id , (*s).second );
        }
      }

    }
    // 順序リストの作成
        for ( std::map< std::string, var_t >::const_iterator iter( order_list_map.begin() ), last( order_list_map.end() );
              iter != last;
              ++iter )
        {
          // 出現順リスト $OBJ.ORDER_LIST$ -- ID番号の並び
          mproc.set_var( toppers::toupper( iter->first + ".order_list" ), iter->second );
          var_t rorder_list( iter->second );

          // 逆順リスト $OBJ.RORDER_LIST$ -- ID番号の並び
          std::reverse( rorder_list.begin(), rorder_list.end() );
          mproc.set_var( toppers::toupper( iter->first + ".rorder_list" ), rorder_list );

          // ID番号リスト $OBJ.ID_LIST$ -- ID番号の並び
          var_t id_list( iter->second );
          std::sort( id_list.begin(), id_list.end() );
          mproc.set_var( toppers::toupper( iter->first + ".id_list" ), id_list );
        }
    }

      // カーネルオブジェクト生成・定義用静的APIの各パラメータをマクロプロセッサの変数として設定する。
      void set_object_vars( std::vector< object_definition* > const& obj_array, macro_processor& mproc )
      {
        typedef macro_processor::element element;
        typedef macro_processor::var_t var_t;
        long order = 1;
        var_t order_list;

        for ( std::vector< object_definition* >::const_iterator v_iter( obj_array.begin() ), v_last( obj_array.end() );
              v_iter != v_last;
              ++v_iter )
        {
          var_t params;
          var_t args;

#if 0
          // 静的APIが出現した行番号
          element e;
          // e.s = v_iter->line().file;
          // e.i = v_iter->line().line;
          // mproc.set_var( "API.TEXT_LINE", order, var_t( 1, e ) );

          // 静的API名
          e.s = v_iter->get_name();
          e.i = boost::none;
          mproc.set_var( "OBJ.NAME", order, var_t( 1, e ) );

          // オブジェクトタイプ（"TSK", "SEM", ...）
          e.s = toppers::toupper( v_iter->object_type );
          mproc.set_var( "OBJ.TYPE", order, var_t( 1 , e ) );

          // 各パラメータ
          for ( static_api::const_iterator api_iter( v_iter->begin() ), api_last( v_iter->end() );
                api_iter != api_last;
                ++api_iter )
          {
            std::string name( toppers::toupper( ( boost::format( "%s.%s" ) % info->type % ( api_iter->symbol.c_str() + 1 ) ).str() ) );
            // 末尾の ? を除去
            if ( *name.rbegin() == '\?' ) 
            {
              name.resize( name.size() - 1 );
            }
            // 末尾の ... を除去 & order を付加
            if ( name.size() > 3 && name.substr( name.size() - 3 ) == "..." )
            {
              name.resize( name.size() - 3 );
              name += boost::lexical_cast< std::string >( api_iter->order );
            }

            element e;
            e.s = api_iter->text; // ソースの字面
            if ( api_iter->symbol[0] != '&' )   // 一般定数式パラメータは値が特定できない
            {
              if ( api_iter->symbol[0] == '$' )  // 文字列定数式パラメータ
              {
                e.v = api_iter->string; // 展開後の文字列
              }
              else
              {
                e.i = api_iter->value;
              }
            }
            args.push_back( e );

            e.s = name;
            e.i = boost::none;
            params.push_back( e );

            if ( api_iter->symbol[0] == '%' )
            {
              continue;
            }
          }
          mproc.set_var( "API.ARGS", order, args );
          mproc.set_var( "API.PARAMS", order, params );
          e.s.clear();
          e.i = order;
          order_list.push_back( e );
#endif
          ++order;
        }
        mproc.set_var( "API.ORDER_LIST", order_list );

        element external_id;
        external_id.i = get_global_bool( "external-id" );
        mproc.set_var( "USE_EXTERNAL_ID", var_t( 1, external_id ) );
      }

      // プラットフォーム・コンパイラ依存の値をマクロプロセッサの変数として設定する。
      void set_platform_vars( cfg1_out const& cfg1out, macro_processor& mproc )
      {
        typedef macro_processor::element element;
        typedef macro_processor::var_t var_t;

        cfg1_out::cfg1_def_table const* def_table = cfg1out.get_def_table();
        std::size_t sizeof_signed_t;
        std::size_t sizeof_pointer;

        static cfg1_out::cfg1_def_t const limit_defs[] =
        {
          { false, "TOPPERS_cfg_CHAR_BIT",  "CHAR_BIT" },
          { false, "TOPPERS_cfg_CHAR_MAX",  "CHAR_MAX" },
          { true, "TOPPERS_cfg_CHAR_MIN",  "CHAR_MIN" },
          { false, "TOPPERS_cfg_SCHAR_MAX", "SCHAR_MAX" },  // 本来は符号付きだが、負になることはない
          { false, "TOPPERS_cfg_SHRT_MAX",  "SHRT_MAX" },   // 本来は符号付きだが、負になることはない
          { false, "TOPPERS_cfg_INT_MAX",   "INT_MAX" },    // 本来は符号付きだが、負になることはない
          { false, "TOPPERS_cfg_LONG_MAX",  "LONG_MAX" },   // 本来は符号付きだが、負になることはない
        };

        nm_symbol::entry nm_entry = cfg1out.get_syms()->find( "TOPPERS_cfg_sizeof_signed_t" );
        sizeof_signed_t = static_cast< std::size_t >( cfg1out.get_srec()->get_value( nm_entry.address, 4, cfg1out.is_little_endian() ) );

        nm_entry = cfg1out.get_syms()->find( "TOPPERS_cfg_sizeof_pointer" );
        sizeof_pointer = static_cast< std::size_t >( cfg1out.get_srec()->get_value( nm_entry.address, 4, cfg1out.is_little_endian() ) );

        for ( std::size_t i = 0; i < sizeof limit_defs / sizeof limit_defs[ 0 ]; ++i )
        {
          element e;
          e.s = limit_defs[ i ].expression;
          nm_entry = cfg1out.get_syms()->find( limit_defs[ i ].name );
          std::tr1::int64_t value = cfg1out.get_srec()->get_value( nm_entry.address, sizeof_signed_t, cfg1out.is_little_endian() );
          if ( sizeof_signed_t < 8 && limit_defs[ i ].is_signed )
          {
            value = cfg1_out::make_signed( static_cast< std::tr1::uint32_t >( value ) );
          }
          mproc.set_var( e.s, var_t( 1, e ) );
        }

        for ( cfg1_out::cfg1_def_table::const_iterator iter( def_table->begin() ), last( def_table->end() );
              iter != last;
              ++iter )
        {
          element e;
          std::tr1::int64_t value;

          nm_entry = cfg1out.get_syms()->find( "TOPPERS_cfg_" + iter->name );
          if ( nm_entry.type >= 0 )
          {
            if ( !iter->expression.empty() && iter->expression[ 0 ] == '@' )  // 式が'@'で始まる場合はアドレス定数式
            {
              value = cfg1out.get_srec()->get_value( nm_entry.address, sizeof_pointer, cfg1out.is_little_endian() );
              if ( sizeof_signed_t < 8 && iter->is_signed )
              {
                value = cfg1_out::make_signed( static_cast< std::tr1::uint32_t >( value ) );
              }

              // 先ほど取り出したアドレスを使って間接参照
              value = cfg1out.get_srec()->get_value( value, 8, cfg1out.is_little_endian() );  // 取り出す値は型に関係なく常に8バイト
              if ( sizeof_signed_t < 8 && iter->is_signed )
              {
                value = cfg1_out::make_signed( static_cast< std::tr1::uint32_t >( value ) );
              }
              e.s = iter->expression.c_str() + 1; // 先頭の'@'を除去
            }
            else  // アドレスではない通常の整数定数式
            {
              value = cfg1out.get_srec()->get_value( nm_entry.address, sizeof_signed_t, cfg1out.is_little_endian() );
              if ( sizeof_signed_t < 8 && iter->is_signed )
              {
                value = cfg1_out::make_signed( static_cast< std::tr1::uint32_t >( value ) );
              }
              e.s = iter->expression;
            }
            e.i = value;
            mproc.set_var( iter->name, var_t( 1, e ) );
          }
        }

        // バイトオーダー
        {
          bool little_endian = cfg1out.is_little_endian();
          element e;
          e.i = little_endian;
          mproc.set_var( "LITTLE_ENDIAN", var_t( 1, e ) );

          e.i = !little_endian;
          mproc.set_var( "BIG_ENDIAN", var_t( 1, e ) );
        }
      }

    }

    //! コンストラクタ
    factory::factory( std::string const& kernel )
      : kernel_( tolower( kernel ) )
    {
    }

    //! デストラクタ
    factory::~factory()
    {
    }

    //! サポートしているオブジェクト情報の取得
  std::vector<std::string> const* factory::get_object_definition_info() const
    {
      // CSVから静的API情報を読み取り、登録するためのローカルクラス
      struct init_t
      {
        init_t()
        {
          boost::any t = global( "api-table" );
          if ( !t.empty() )
          {
            std::vector< std::string > api_tables( boost::any_cast< std::vector< std::string >& >( t ) );
            for ( std::vector< std::string >::const_iterator iter( api_tables.begin() ), last( api_tables.end() );
                  iter != last;
                  ++iter )
            {
              std::string buf;
              std::string api_table_filename = *iter;
              read( api_table_filename.c_str(), buf );
              csv data( buf.begin(), buf.end() );
              for ( csv::const_iterator d_iter( data.begin() ), d_last( data.end() );
                    d_iter != d_last;
                    ++d_iter )
              {
                  unsigned int i;
                  for(i = 0 ; i < d_iter->size() ; i++)
                  {
                      volatile int x = 1;
                      object_definition_table.push_back((*d_iter)[i].c_str());
                  }
              }

            }
          }
        }

        ~init_t()
        {

        
        }

    std::vector<std::string> object_definition_table;
      };
      static init_t init;
    std::vector<std::string> const* result = &init.object_definition_table;
      return result;
    }

    /*!
     * \brief   cfg1_out.c への出力情報テーブルの生成
     * \return  生成した cfg1_out::cfg1_def_table オブジェクトへのポインタ
     * \note    この関数が返すポインタは delete してはならない
     *
     * --cfg1-def-table オプションで指定したファイルから、cfg1_out.c へ出力する情報を読み取り、
     * cfg1_out::cfg1_def_table オブジェクトを生成する。
     *
     * CSV の形式は以下の通り
     *
     *    シンボル名,式[,s|signed]
     *
     * 末尾の s または signed は省略可能。省略時は符号無し整数とみなす。s または signed 指定時は
     * 符号付き整数とみなす。\n
     * 「式」の最初に # があれば前処理式とみなす。
     */
    cfg1_out::cfg1_def_table const* factory::get_cfg1_def_table() const
    {
      struct init_t
      {
        init_t()
        {
          boost::any t = global( "cfg1-def-table" );
          if ( !t.empty() )
          {
            std::vector< std::string > cfg1_def_table = boost::any_cast< std::vector< std::string >& >( t );
            for ( std::vector< std::string >::const_iterator iter( cfg1_def_table.begin() ), last( cfg1_def_table.end() );
                  iter != last;
                  ++iter )
            {
              std::string buf;
              read( iter->c_str(), buf );
              csv data( buf.begin(), buf.end() );
              for ( csv::const_iterator d_iter( data.begin() ), d_last( data.end() );
                    d_iter != d_last;
                    ++d_iter )
              {
                csv::size_type len = d_iter->size();
                if ( len < 2 )
                {
                  toppers::fatal( _( "too little fields in `%1%\'" ), *iter );
                }
                cfg1_out::cfg1_def_t def = { 0 };
                def.name = ( *d_iter )[ 0 ];
                def.expression = ( *d_iter )[ 1 ];
                if ( len >= 3 )
                {
                  std::string is_signed( ( *d_iter )[ 2 ] );
                  def.is_signed = ( is_signed == "s" || is_signed == "signed" );
                }
                if ( len >= 4)
                {
                  def.value1 = ( *d_iter )[ 3 ];
                }
                if ( len >= 5)
                {
                  def.value2 = ( *d_iter )[ 4 ];
                }
                cfg1_def_table_.push_back( def );
              }
            }
          }
        }
        cfg1_out::cfg1_def_table cfg1_def_table_;
      };
      static init_t init;
      cfg1_out::cfg1_def_table const* result = &init.cfg1_def_table_;
      return result;
    }

    //! オブジェクトの交換
    void factory::do_swap( factory& other )
    {
      kernel_.swap( other.kernel_ );
    }

    /*!
     *  \brief  マクロプロセッサの生成
     *  \param[in]  cfg1out cfg1_out オブジェクト
     *  \param[in]  api_map .cfg ファイルに記述された静的API情報
     *  \return     マクロプロセッサへのポインタ
     */
    std::auto_ptr< macro_processor > factory::do_create_macro_processor( cfg1_out const& cfg1out, cfg1_out::cfg_obj_map const& obj_def_map ) const
    {
      typedef macro_processor::element element;
      typedef macro_processor::var_t var_t;
      std::auto_ptr< macro_processor > mproc( new macro_processor );
      element e;

      e.s = " ";    mproc->set_var( "SPC", var_t( 1, e ) );  // $SPC$
      e.s = "\t";   mproc->set_var( "TAB", var_t( 1, e ) );  // $TAB$
      e.s = "\n";   mproc->set_var( "NL",  var_t( 1, e ) );  // $NL$

      // バージョン情報
      e.s = toppers::get_global_string( "version" );
      e.i = toppers::get_global< std::tr1::int64_t >( "timestamp" );
      mproc->set_var( "CFG_VERSION", var_t( 1, e ) );   // $CFG_VERSION$

      // その他の組み込み変数の設定
      set_object_vars( obj_def_map, *mproc );
      set_platform_vars( cfg1out, *mproc );
      e.s = cfg1out.get_includes();
      mproc->set_var( "INCLUDES", var_t( 1, e ) );

      // パス情報
      e.s = boost::lexical_cast< std::string >( toppers::get_global< int >( "pass" ) );
      e.i = toppers::get_global< int >( "pass" );
      mproc->set_var( "CFG_PASS", var_t( 1, e ) );

      return mproc;
    }

    /*!
     *  \brief  マクロプロセッサの生成
     *  \param[in]  cfg1out cfg1_out オブジェクト
     *  \param[in]  api_array .cfg ファイルに記述された静的API情報
     *  \return     マクロプロセッサへのポインタ
     */
    std::auto_ptr< macro_processor > factory::do_create_macro_processor( cfg1_out const& cfg1out, std::vector< object_definition* > const& obj_array ) const
    {
      typedef macro_processor::element element;
      typedef macro_processor::var_t var_t;
      std::auto_ptr< macro_processor > mproc( new macro_processor );
      element e;

      e.s = " ";    mproc->set_var( "SPC", var_t( 1, e ) );  // $SPC$
      e.s = "\t";   mproc->set_var( "TAB", var_t( 1, e ) );  // $TAB$
      e.s = "\n";   mproc->set_var( "NL",  var_t( 1, e ) );  // $NL$

      // バージョン情報
      e.s = toppers::get_global_string( "version" );
      e.i = toppers::get_global< std::tr1::int64_t >( "timestamp" );
      mproc->set_var( "CFG_VERSION", var_t( 1, e ) );   // $CFG_VERSION$

      // その他の組み込み変数の設定
      set_object_vars( obj_array, *mproc );
      set_platform_vars( cfg1out, *mproc );
      e.s = cfg1out.get_includes();
      mproc->set_var( "INCLUDES", var_t( 1, e ) );

      // パス情報
      e.s = boost::lexical_cast< std::string >( toppers::get_global< int >( "pass" ));
      e.i = toppers::get_global< int >( "pass" );
      mproc->set_var( "CFG_PASS", var_t( 1, e ) );

      return mproc;
    }

    std::auto_ptr< cfg1_out > factory::do_create_cfg1_out( std::string const& filename ) const
    {
      return std::auto_ptr< oil::cfg1_out >( new cfg1_out( filename, get_cfg1_def_table() ) );
    }
    std::auto_ptr< checker > factory::do_create_checker() const
    {
      return std::auto_ptr< oil::checker >( new checker );
    }

  }
}
