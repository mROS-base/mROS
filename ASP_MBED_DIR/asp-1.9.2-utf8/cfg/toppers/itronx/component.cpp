/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2010 by TAKAGI Nobuhisa
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
#include <cstring>
#include <cerrno>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <fstream>
#include "toppers/macro_processor.hpp"
#include "toppers/diagnostics.hpp"
#include "toppers/gettext.hpp"
#include "toppers/cpp.hpp"
#include "toppers/global.hpp"
#include "toppers/misc.hpp"
#include "toppers/itronx/static_api.hpp"
#include "toppers/itronx/factory.hpp"
#include "toppers/itronx/component.hpp"
#include <boost/format.hpp>
#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>

namespace toppers
{
  namespace
  {
    typedef macro_processor::element element;
    typedef macro_processor::var_t var_t;
    typedef macro_processor::context context;

    inline std::tr1::int64_t get_i( var_t const& var, context const* p_ctx )
    {
      return macro_processor::to_integer( var, p_ctx );
    }
    inline std::string get_s( var_t const& var, context const* p_ctx )
    {
      return macro_processor::to_string( var, p_ctx );
    }

    inline bool less_element( const element& lhs, const element& rhs )
    {
      return *lhs.i < *rhs.i;
    }

    /*!
     *  \brief  ID番号を割付ける
     *  \param[in]  line      行番号
     *  \param[in]  arg_list  マクロ実引数リスト
     *  \param[in]  p_ctx     マクロコンテキスト
     *  \retval     マクロ返却値
     *  第一引数で指定した種別のオブジェクトに対してID番号を割付け、個々のパラメータを表す変数、
     *  TEXT_LINE、ID_LIST、ORDER_LIST、およびRORDER_LISTを設定する。ただし、ID番号を持たない
     *  オブジェクトの場合、ID_LISTは設定しない。
     */
    var_t bf_assignid( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
    {
      if ( macro_processor::check_arity( line, arg_list.size(), 1, "ASSIGNID" ) )
      {
        itronx::factory* factory = get_global< itronx::factory* >( "factory" );
        std::map< std::string, itronx::static_api::info > const* info_map = factory->get_static_api_info_map();

        std::string type = get_s( arg_list[ 0 ], p_ctx );    // 出力先リストの識別名
        std::string TYPE = toppers::toupper( type );
        typedef std::map< std::string, long > id_map_t;
        typedef std::map< long, std::string > id_rmap_t;
        id_map_t id_map;
        id_rmap_t id_rmap;
        var_t id_list, order_list, rorder_list;
        bool has_id = false;

        std::string id_input_file( get_global< std::string >( "id-input-file" ) );
        if ( !id_input_file.empty() )  // --id-input-file オプションが指定されている場合...
        {
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
              fatal( _( "E_OBJ: `%1%\' is duplicated" ), name );
            }
            else
            {
              id_map[ name ] = value;
              id_rmap[ value ] = name;
            }
          }
        }
        long order = 1;
        for ( int i = 1; ; i++ )
        {
          std::string str_i = boost::lexical_cast< std::string >( i );
          if ( p_ctx->var_map.find( "API.TYPE[" + str_i + "]" ) == p_ctx->var_map.end() )
          {
            break;
          }
          std::string api_name = p_ctx->var_map[ "API.NAME[" + str_i + "]" ].at( 0 ).s;
          std::map< std::string, itronx::static_api::info >::const_iterator it = info_map->find( api_name );
          if ( it == info_map->end() )
          {
            break;
          }
          itronx::static_api::info info = it->second;
          if ( info.type != type )
          {
            continue;
          }

          var_t params = p_ctx->var_map[ "API.PARAMS[" + str_i + "]" ];
          var_t args = p_ctx->var_map[ "API.ARGS[" + str_i + "]" ];
          long id = -1;

          if ( info.id_pos >= 0 )  // ID番号のあるもの...
          {
            if ( !info.slave )
            {
              if ( std::strchr( info.params, '#' ) != 0 ) // オブジェクト識別名を持つ場合...
              {
                for ( id = 1; id_rmap.find( id ) != id_rmap.end(); id++ ) // 未使用のID番号を検索
                  ;
                args.at( info.id_pos ).i = id; // ID番号を設定
              }
              else
              {
                id = static_cast< long >( *args.at( info.id_pos ).i );
              }
              std::string idname = args[ info.id_pos ].s; // IDの字面
              id_map[ idname ] = id;
              id_rmap[ id ] = idname;
              order_list.push_back( args[ info.id_pos ] );
            }
            else
            {
              id_map_t::iterator it = id_map.find( args.at( info.id_pos ).s );
              if ( it == id_map.end() )
              {
                fatal( line, _( "E_NOEXS: `%1%\' is undefined" ), args[ info.id_pos ].s );
              }
              args.at( info.id_pos ).i = it->second;
            }
            has_id = true;
          }
          else  // ID番号の無いもの...
          {
            element e;
            e.i = order;
            order_list.push_back( e );
          }
          std::string str_id = boost::lexical_cast< std::string >( id > 0 ? id : order );

          // 各パラメータの変数を設定
          for ( var_t::size_type i = 0, n = params.size(); i < n; i++ )
          {
            std::string var_name = params[ i ].s + "[" + str_id + "]";
            p_ctx->var_map[ var_name ] = var_t( 1, args.at( i ) );
          }
          p_ctx->var_map[ TYPE + ".TEXT_LINE[" + str_id + "]" ] = p_ctx->var_map[ "API.TEXT_LINE[" + str_i + "]" ];
          ++order;
        }

        id_list = order_list;
        std::sort( id_list.begin(), id_list.end(), less_element );
        rorder_list = order_list;
        std::reverse( rorder_list.begin(), rorder_list.end() );
        p_ctx->var_map[ TYPE + ".ID_LIST" ] = id_list;
        p_ctx->var_map[ TYPE + ".ORDER_LIST" ] = order_list;
        p_ctx->var_map[ TYPE + ".RORDER_LIST" ] = rorder_list;
      }
      return var_t();
    }

    /*!
     *  \brief  指定した連想配列群に静的APIを追加する
     *  \param[in]  line      行番号
     *  \param[in]  arg_list  マクロ実引数リスト
     *  \param[in]  p_ctx     マクロコンテキスト
     *  \retval     マクロ返却値
     *  API.で始まる連想配列群を、必要に応じて適切に変換を行った上で、別のプレフィックスで始まる
     *  連想配列群に追加する。想定する使用方法としては、特定のソフトウェア部品で理解可能な静的API
     *  に関しては、追加を行わないか、他の静的APIに置き換えて追加を行う。理解できない静的APIに関して
     *  はそのまま追加を行う。
     *  この関数の第一引数にはAPI.で始まる連想配列の連番を、第二引数には追加先に連想配列群の
     *  プレフィックスを、第三引数には対象とする静的API名を、第四引数には追加するパラメータシンボルの
     *  並びを、第五引数にはパラメータの並びを指定する。     
     */
    var_t bf_addapi( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
    {
      element e;
      if ( macro_processor::check_arity( line, arg_list.size(), 5, "ADDAPI" ) )
      {
        itronx::factory* factory = get_global< itronx::factory* >( "factory" );
        std::map< std::string, itronx::static_api::info > const* info_map = factory->get_static_api_info_map();

        std::tr1::int64_t order = get_i( arg_list[ 0 ], p_ctx );  // 元の静的APIの連番
        std::string list_name = get_s( arg_list[ 1 ], p_ctx );    // 出力先リストの識別名
        std::string api_name = get_s( arg_list[ 2 ], p_ctx );     // 静的API名
        var_t params = arg_list[ 3 ]; // パラメータシンボルの並び
        var_t args = arg_list[ 4 ];   // パラメータの並び

        std::map< std::string, itronx::static_api::info >::const_iterator it = info_map->find( api_name );
        if ( it != info_map->end() )
        {
          itronx::static_api::info info = it->second;
          std::string str_order = boost::lexical_cast< std::string >( order );
          p_ctx->var_map[ list_name + ".TEXT_LINE[" + str_order + "]" ] = p_ctx->var_map[ "API.TEXT_LINE[" + str_order + "]" ]; 
          e.s = api_name;
          p_ctx->var_map[ list_name + ".NAME[" + str_order + "]" ] = var_t( 1, e );
          e.s = info.type;
          p_ctx->var_map[ list_name + ".TYPE[" + str_order + "]" ] = var_t( 1, e );
          p_ctx->var_map[ list_name + ".PARAMS[" + str_order + "]" ] = params;
          p_ctx->var_map[ list_name + ".ARGS[" + str_order + "]" ] = args;

          e.s.clear();
          if ( !p_ctx->var_map[ list_name + ".ORDER_LIST" ].empty() )
          {
            e.i = *p_ctx->var_map[ list_name + ".ORDER_LIST" ].back().i + 1;
          }
          else
          {
            e.i = 1;
          }
          p_ctx->var_map[ list_name + ".ORDER_LIST" ].push_back( e );

          e.s.clear();
          e.i = 1;
        }
      }
      return var_t( 1, e );
    }

    /*!
     *  \brief  変数群の交換
     *  \param[in]  line      行番号
     *  \param[in]  arg_list  マクロ実引数リスト
     *  \param[in]  p_ctx     マクロコンテキスト
     *  \retval     マクロ返却値
     *  第一引数および第二引数で指定したプレフィックスを持つ変数群を入れ替える。
     *  ‘ADDAPI’関数で、別のプレフィックスを持つ連想配列を組み立てたあとは、この関数を用いることで、
     *  API.をプレフィックスに持つ連想配列と交換することができる。
     */
    var_t bf_swapprefix( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
    {
      if ( macro_processor::check_arity( line, arg_list.size(), 2, "SWAPPREFIX" ) )
      {
        std::string list1_name = get_s( arg_list[ 0 ], p_ctx ) + ".";    // 一方のAPIリストの識別名.
        std::string list2_name = get_s( arg_list[ 1 ], p_ctx ) + ".";    // 他方のAPIリストの識別名.
        std::string::size_type list1_size = list1_name.size();
        std::string::size_type list2_size = list2_name.size();

        std::map< std::string, var_t > temp;
        for ( std::map< std::string, var_t >::const_iterator iter = p_ctx->var_map.begin(), last = p_ctx->var_map.end();
              iter != last;
              ++iter )
        {
          std::pair< std::string, var_t > element = *iter;
          if ( std::strncmp( iter->first.c_str(), list1_name.c_str(), list1_size ) == 0 )
          {
            element.first = list2_name + ( iter->first.c_str() + list1_size );
          }
          else if ( std::strncmp( iter->first.c_str(), list2_name.c_str(), list2_size ) == 0 )
          {
            element.first = list1_name + ( iter->first.c_str() + list2_size );
          }
          temp.insert( element );
        }
        p_ctx->var_map.swap( temp );
      }
      return var_t();
    }

    /*!
     *  \brief  変数群の交換
     *  \param[in]  line      行番号
     *  \param[in]  arg_list  マクロ実引数リスト
     *  \param[in]  p_ctx     マクロコンテキスト
     *  \retval     マクロ返却値
     *  第一引数で指定したプレフィックスで始まる変数群を削除する。’SWAPPREFIX’関数で交換したあと、
     *  不要になった変数群はこの関数で削除しておくことが望ましい。
     */
    var_t bf_cleanvars( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx )
    {
      if ( macro_processor::check_arity( line, arg_list.size(), 1, "CLEANVARS" ) )
      {
        std::string prefix = get_s( arg_list[ 0 ], p_ctx ) + ".";    // 変数の接頭辞
        std::size_t n = prefix.size();
        std::map< std::string, var_t > temp_map;

        for ( std::map< std::string, var_t >::const_iterator iter = p_ctx->var_map.begin(), last =p_ctx->var_map.end();
              iter != last;
              ++iter )
        {
          if ( std::strncmp( iter->first.c_str(), prefix.c_str(), n ) != 0 )
          {
            temp_map.insert( *iter );
          }
        }
        p_ctx->var_map.swap( temp_map );
      }
      return var_t();
    }

    macro_processor::func_t const function_table[] =
    {
      { "ASSIGNID",   &bf_assignid },
      { "ADDAPI",     &bf_addapi },
      { "SWAPPREFIX", &bf_swapprefix },
      { "CLEANVARS",  &bf_cleanvars },
    };

  }

  namespace itronx
  {

    component::component( macro_processor* mproc )
      : mproc_( mproc )
    {
      for ( std::size_t i = 0; i < sizeof( function_table ) / sizeof( function_table[ 0 ] ); i++ )
      {
        mproc_->add_builtin_function( function_table[ i ] );
      }
    }

  }
}
