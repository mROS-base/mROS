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
#include <cctype>
#include <sstream>
#include "toppers/diagnostics.hpp"
#include "toppers/itronx/static_api.hpp"
#include "toppers/itronx/static_api_parser.hpp"
#include <boost/spirit/include/classic.hpp>
#include <boost/lexical_cast.hpp>

namespace toppers
{
  namespace itronx
  {
    namespace
    {
      class find_api_predicate
      {
      public:
        explicit find_api_predicate( std::string const& api_name ) : api_name_( api_name ) {}
        bool operator()( static_api::info const& info ) const
        {
          return info.api_name == api_name_;
        }
      private:
        std::string api_name_;
      };
    }

    /*!
     *  \brief  静的APIの構文解析
     *  \param[in,out]  next      テキストの読み込み位置、兼構文解析後の次の読み込み位置の格納先
     *  \param[in]      last      テキストの終端位置
     *  \param[in]      info_map  静的API情報マップ
     *  \param[in]      ucn       国際文字名を有効にする場合は true を指定する。
     *  \param[in]      codeset   文字コード
     *  \retval         true    成功
     *  \retval         false   失敗
     */
    bool static_api::parse( text::const_iterator& next, text::const_iterator last,
                            std::map< std::string, info > const& info_map,
                            bool ucn, codeset_t codeset )
    {
      boost::spirit::classic::parse_info< text::const_iterator > pi;
      std::vector< std::string > tokens;
      c_const_expr_parser cexpr_p( ucn, codeset );
      static_api_parser parser( tokens, cexpr_p );
      static_api temp;

      text::const_iterator next_temp;
      for ( next_temp = next; next_temp != last; ++next_temp )
      {
        if ( !std::isspace( static_cast< unsigned char >( *next_temp ) ) )
        {
          break;
        }
      }
      if ( next_temp == last )
      {
        return false;
      }

      // エラーメッセージ用の行番号
      // この行番号は、静的APIの開始位置のものであるため、エラー発生箇所そのものズバリを指すことはできない。
      temp.line_ = next_temp.line();

      pi = boost::spirit::classic::parse( next_temp, last, parser, boost::spirit::classic::space_p );
      if ( !pi.hit )
      {
        return false;
      }

      // 静的APIが存在するかどうかの判定およびシグニチャの取得
      std::string api_name( tokens.front() );
      std::map< std::string, info >::const_iterator info_iter = info_map.find( api_name );
      info const* pinfo = 0;
      if ( info_iter != info_map.end() )
      {
        pinfo = &info_iter->second;
        temp.pinfo_ = pinfo;
      }
      else
      {
        error( temp.line_, _( "static API `%1%\' is unknown" ), api_name );
        return false;
      }

      // 各パラメータの解析
      std::istringstream iss( pinfo->params );
      int order = 0;            // パラメータリスト内の順序
      bool param_list = false;  // パラメータリスト解析中フラグ
      std::string symbol;
      bool skip = false;

      for ( std::vector< std::string >::const_iterator iter( tokens.begin() + 1 ), last( tokens.end() );
            iter != last;
            ++iter )
      {
        if ( iss.eof() )
        {
          error( temp.line_, _( "too many parameters for static API `%s\'" ), api_name );
          break;
        }
        if ( !param_list )
        {
          iss >> symbol;
          if ( symbol.size() > 3 && symbol.substr( symbol.size() - 3 ) == "..." )
          {
            param_list = true;
            order = 0;
            //symbol.resize( symbol.size() - 3 );
          }
        }
        if ( symbol == "{" || symbol == "}" )
        {
          if ( symbol != *iter )
          {
            error( temp.line_, _( "missing token `%1%\'" ), symbol );
            --iter;
          }
        }
        else if ( *iter == "{" || *iter == "}" )
        {
          if ( param_list && *iter == "}" )
          {
            iss >> symbol;
            if ( symbol != "}" )
            {
              error( temp.line_, _( "illegal token `%1%\'" ), *iter );
            }
            param_list = false;
          }
          else if ( *symbol.rbegin() == '\?' ) // 省略可能パラメータのスキップ
          {
            skip = true;
            --iter;
          }
          else
          {
            error( temp.line_, _( "illegal token `%1%\'" ), *iter );
          }
        }
        else
        {
          parameter value;
          value.symbol = symbol;
          if ( param_list )
          {
            //value.symbol += boost::lexical_cast< std::string >( order++ );
            value.order = order++;
          }
          value.text = *iter;
          value.value = 0;
          temp.params_.push_back( value );
        }
      }
      if ( !iss.eof() )
      {
        iss >> symbol;
        if ( symbol != "}" )
        {
          error( temp.line_, _( "few parameters for static API `%s\'" ), api_name );
        }
        else if ( !skip )
        {
          error( temp.line_, _( "missing token `%1%\'" ), symbol );
        }
        else
        {
          skip = false;
        }
      }
      next = pi.stop;
      swap( temp );
      return true;
    }

    bool static_api::set_block( char const* type, std::string const& id )
    {
      for ( std::vector< parameter >::const_iterator iter( params_.begin() ), last( params_.end() );
            iter != last;
            ++iter )
      {
        if ( iter->symbol == type )
        {
          return false;
        }
      }
      parameter value;
      value.symbol = type;
      value.text = id;
      value.value = 0;
      params_.push_back( value );
      return true;
    }

    static_api::size_type static_api::count_integer_params() const
    {
      size_type result = 0;
      for ( std::vector< parameter >::const_iterator iter( params_.begin() ), last( params_.end() );
            iter != last;
            ++iter )
      {
        if ( !iter->symbol.empty()
          && ( ( iter->symbol[0] == '.' ) || ( iter->symbol[0] == '+' ) ) )
        {
          ++result;
        }
      }
      return result;
    }

  }
}
