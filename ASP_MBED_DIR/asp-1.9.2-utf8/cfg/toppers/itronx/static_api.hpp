/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2010 by TAKAGI Nobuhisa
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
 *  \file   toppers/itronx/static_api.hpp
 *  \brief  静的API情報の解析と管理に関する宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class static_api;
 *  \endcode
 */
#ifndef TOPPERS_ITRONX_STATIC_API_HPP_
#define TOPPERS_ITRONX_STATIC_API_HPP_

#include <string>
#include <map>
#include <set>
#include <algorithm>
#include "toppers/workaround.hpp"
#include "toppers/text.hpp"
#include "toppers/codeset.hpp"
#include "toppers/diagnostics.hpp"
#include <boost/any.hpp>
#include <boost/optional.hpp>

namespace toppers
{
  namespace itronx
  {

    /*!
     *  \class  static_api static_api.hpp "toppers/itronx/static_api.hpp"
     *  \brief  コンフィギュレーションファイルに記述された静的APIの情報を管理するためのクラス
     */
    class static_api
    {
    public:
      /*!
       *  \struct info  static_api.hpp "toppers/itronx/static_api.hpp"
       *  \brief  静的APIの仕様に関する情報
       *
       *  static_api::info の params メンバ内の各要素に付けられた接頭辞の意味は次のとおり
       *
       *  - #: オブジェクト識別子
       *  - %: オブジェクト識別子で、かつ自動割付けの対象とならないもの
       *  - .: 符号無し整数定数式パラメータ
       *  - +: 符号付き整数定数式パラメータ
       *  - $: 文字列定数式パラメータ
       *  - &: 一般定数式パラメータ
       *
       *  なお、{ および } も便宜的にパラメータの一種として扱っている（構文解析に必要なため）。
       */
      struct info
      {
        char const* type;     //!< カーネルオブジェクトの種別を表す三文字（"tsk", "sem"等）
        char const* api_name; //!< 静的API名（"CRE_TSK", "CRE_SEM"等）
        char const* params;   //!< パラメータ並び
        int id_pos;           //!< パラメータのうちID番号の位置（-1の場合はID番号無し）
        bool slave;           //!< 他の静的APIに対して従属関係にある場合にtrue（"DEF_TEX"等）
      };
      /*!
       *  \struct parameter static_api.hpp "toppers/itronx/static_api.hpp"
       *  \brief  静的APIのパラメータに関する情報
       */
      struct parameter
      {
        std::string symbol;   //!< 仮引数名
        std::string text;     //!< 実引数の字面
        boost::optional< std::tr1::intmax_t > value;  //!< 実引数の値
        std::string string;
        int order;            //!< リスト形式パラメータの連番
      };
      typedef std::vector< parameter > parameter_container;
      typedef parameter_container::iterator iterator;
      typedef parameter_container::const_iterator const_iterator;
      typedef parameter_container::reference reference;
      typedef parameter_container::const_reference const_reference;
      typedef parameter_container::size_type size_type;

      char const* api_name() const { return pinfo_ != 0 ? pinfo_->api_name : ""; }
      info const* get_info() const { return pinfo_; }

      parameter_container const& params() const { return params_; }
      iterator begin() { return params_.begin(); }
      const_iterator begin() const { return params_.begin(); }
      iterator end() { return params_.end(); }
      const_iterator end() const { return params_.end(); }
      reference at( size_type pos ) { return params_.at( pos ); }
      reference at( std::string const& symbol )
      {
        iterator iter = std::find_if( params_.begin(), params_.end(), match_param_symbol( symbol ) );
        if ( iter == params_.end() )
        {
          throw std::out_of_range( "out of range" );
        }
        return *iter;
      }
      const_reference at( size_type pos ) const { return params_.at( pos ); }
      const_reference at( std::string const& symbol ) const
      {
        const_iterator iter = std::find_if( params_.begin(), params_.end(), match_param_symbol( symbol ) );
        if ( iter == params_.end() )
        {
          throw std::out_of_range( "out of range" );
        }
        return *iter;
      }
      parameter id() const
      {
        if ( pinfo_->id_pos < 0 )
        {
          return parameter();
        }
        return at( pinfo_->id_pos );
      }
      
      text_line const& line() const { return line_; }
      void line( text_line const& value ) { line_ = value; }
      size_type count_integer_params() const;
        
      //! オブジェクトの交換
      void swap( static_api& other )
      {
        std::swap( pinfo_, other.pinfo_ );
        params_.swap( other.params_ );
        line_.swap( other.line_ );
      }

      bool parse( text::const_iterator& next, text::const_iterator last, 
                  std::map< std::string, info > const& info_map,
                  bool ucn = false, codeset_t codeset = ascii );

      bool set_class( std::string const& id )
      {
        return set_block( "*CLASS", id );
      }
      bool set_domain( std::string const& id )
      {
        return set_block( "*DOMAIN", id );
      }

      /*!
       *  \brief  ID番号の割付け
       *  \param[in]  first   ID番号を割り付ける static_api 列の先頭位置
       *  \param[in]  last    ID番号を割り付ける static_api 列の終端 + 1
       */
      template < class ForwardIterator >
        static void assign_id( ForwardIterator first, ForwardIterator last )
      {
        std::map< std::string, long > id_map;
        std::map< std::string, std::set< std::string > > slave_id_set;
        std::vector< std::string > id_res;
        typedef std::vector< std::string >::size_type size_type;

        // 予約済みのID番号を洗い出す
        for ( ForwardIterator iter( first ); iter != last; ++iter )
        {
          static_api::info const* info = iter->get_info();
          if ( info->id_pos >= 0 )
          {
            if ( !info->slave )
            {
              if ( iter->at( info->id_pos ).symbol[0] == '#' )
              {
                boost::optional< std::tr1::int64_t > id_value = *iter->at( info->id_pos ).value;
                if ( id_value )
                {
                  long id = static_cast< long >( *id_value );
                  if ( id > 0 )
                  {
                    long n = static_cast< long >( id_res.size() );
                    if ( n < id + 1 )
                    {
                      n = id + 1;
                    }
                    id_res.resize( n );
                    std::string name( iter->at( info->id_pos ).text );
                    if ( !id_res[ id ].empty() )
                    {
                      fatal( _( "E_OBJ: %1% `%2%\' in %3% is duplicated" ), iter->at( info->id_pos ).symbol.c_str() + 1, id, info->api_name );
                    }
                    id_res[ id ] = name;
                  }
                }
              }
            }
          }
        }

        // 予約されていないID番号を自動割付け
        long id = 1;
        for ( ForwardIterator iter( first ); iter != last; ++iter )
        {
          static_api::info const* info = iter->get_info();
          if ( info->id_pos >= 0 )
          {
            std::string name( iter->at( info->id_pos ).text );
            if ( !info->slave )
            {
              long id_value = -1;
              if ( iter->at( info->id_pos ).symbol[0] == '#' )
              {
                std::vector< std::string >::iterator id_iter( std::find( id_res.begin(), id_res.end(), name ) );
                if ( id_iter != id_res.end() )  // 割り付け済みの場合...
                {
                  id_value = id_iter - id_res.begin();
                }
                else  // まだ割り付けられていない場合...
                {
                  long n = static_cast< long >( id_res.size() );
                  while ( id < n && !id_res[ id ].empty() )
                  {
                    ++id;
                  }
                  if ( n < id + 1 )
                  {
                    n = id + 1;
                  }
                  id_res.resize( n );
                  id_res[ id ] = name;
                  id_value = id;
                  iter->at( info->id_pos ).value = id_value;
                }
                if ( id_map.find( name ) != id_map.end() )
                {
                  fatal( iter->line(), _( "E_OBJ: `%1%\' is duplicated" ), name );
                }
              }
              else if ( iter->at( info->id_pos ).value )
              {
                id_value = static_cast< long >( iter->at( info->id_pos ).value.get() );
              }
              id_map[ name ] = id_value;
            }
            else  // slave
            {
              if ( id_map[ name ] < 1 )
              {
                fatal( iter->line(), _( "E_NOEXS: `%1%\' is undefined" ), iter->at( info->id_pos ).text );
              }
              std::set< std::string >& set = slave_id_set[ info->api_name ];
              if ( set.find( name ) != set.end() )  // DEF_TEX重複定義の判定
              {
                fatal( iter->line(), _( "E_OBJ: `%1%\' is duplicated" ), iter->at( info->id_pos ).text );
              }
              set.insert( name );
            }
          }
        }

        if ( !id_res.empty() && std::find( id_res.begin() + 1, id_res.end(), std::string() ) != id_res.end() )
        {
          error( _( "`%1%\' id numbers do not continue" ), first->get_info()->type );          
        }
      }
    private:
      class match_param_symbol
      {
      public:
        explicit match_param_symbol( std::string const& symbol ) : symbol_( symbol ) {}
        bool operator()( parameter const& param ) const
        {
          return param.symbol == symbol_;
        }
      private:
        std::string symbol_;
      };

      bool set_block( char const* type, std::string const& id );

      info const* pinfo_;
      std::vector< parameter > params_;
      text_line line_;
    };

  }
}

#endif  // ! TOPPERS_ITRONX_STATIC_API_HPP_
