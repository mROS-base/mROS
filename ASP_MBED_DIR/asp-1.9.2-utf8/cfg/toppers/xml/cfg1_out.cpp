/* 
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *  
 *  Copyright (C) 2007-2012 by TAKAGI Nobuhisa
 *  Copyright (C) 2010 by Meika Sugimoto
 *  Copyright (C) 2011-2012 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2012 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2012 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2011-2012 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2012 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2012 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2012 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2012 by TOSHIBA CORPOTATION, JAPAN
 *  Copyright (C) 2011-2012 by Witz Corporation, JAPAN
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
 *  $Id: cfg1_out.cpp 153 2013-07-26 06:40:35Z nces-mtakada $
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
#include "toppers/misc.hpp"
#include "toppers/xml/cfg1_out.hpp"
#include "toppers/xml/xml_parser.hpp"
#include <boost/spirit/include/classic.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

/*!
 *  \brief  オブジェクトID番号の割付け
 *  \param[in]  api_map   ソースに記述された静的APIを登録したコンテナ
 */
void child_assign_id(toppers::xml::container::object * pObj, int fixed_id)
{
  std::vector<toppers::xml::container::object*>::const_iterator sub;

  if( pObj->getSubcontainers()->size() != 0 )
  {
    for( sub = pObj->getSubcontainers()->begin() ; sub != pObj->getSubcontainers()->end() ; sub++ )
    {
      child_assign_id( *sub, fixed_id );
    }
  }
  if (pObj->getId() == toppers::xml::container::object::undefined)
  {
    pObj->setId( fixed_id );
  }
}
void assign_id( toppers::xml::cfg1_out::xml_obj_map const& xml_map)
{
  std::map<std::string, std::vector<toppers::xml::container::object*> >::const_iterator p;
  std::vector<toppers::xml::container::object*>::const_iterator q;
  std::vector<toppers::xml::container::object*>::const_iterator sub;
  int serial;

  for(p = xml_map.begin() ; p != xml_map.end() ; p++)
  {
    serial = 1;
    // オブジェクトごとにID割当て
    for(q = (*p).second.begin() ; q != (*p).second.end() ; q++)
    {
      (*q)->setId( serial );
      if((*q)->getSubcontainers()->size() != 0)
      {
        for(sub = (*q)->getSubcontainers()->begin() ; sub != (*q)->getSubcontainers()->end() ; sub++)
        {
          child_assign_id( *sub, serial );
        }
      }
      serial++;
    }
  }

}

//  文字列を置換する
std::string Replace( std::string String1, std::string String2, std::string String3 )
{
    std::string::size_type  Pos( String1.find( String2 ) );

    while( Pos != std::string::npos )
    {
        String1.replace( Pos, String2.length(), String3 );
        Pos = String1.find( String2, Pos + String3.length() );
    }

    return String1;
}

namespace toppers
{
  namespace xml
  {
    //! cfg_out クラスの実装詳細
    struct cfg1_out::implementation
    {
    protected:
      output_file ofile_;
	  std::vector< toppers::xml::container::object* > container_array_; /*  xmlからパースした生データ only xml */
      std::map< std::string, std::string > info_rmap_; /*   xml */

      std::string cfg1_out_list_;
      std::string includes_;

      std::tr1::shared_ptr< s_record > srec_;
      std::tr1::shared_ptr< nm_symbol > syms_;
      bool little_endian_;
      std::size_t base_;
      cfg1_def_table const* def_table_;

      implementation( std::string const& filename, std::ios_base::openmode omode, cfg1_def_table const* def_table = 0 )
        : ofile_( filename, omode ), little_endian_( true ), base_( 1 ), def_table_( def_table )
      {
      }
      virtual ~implementation()
      {
      }
      virtual void validate_type(std::vector< toppers::xml::container::object* > const& objects ,std::map<std::string, toppers::xml::info> const& info_map );
      virtual void validate_multiplicity( toppers::xml::container::object* object ,std::map<std::string, toppers::xml::info> const& info_map );
      virtual string do_out_macro_name(std::vector<toppers::xml::container::parameter*>::const_iterator r, int serial, std::map<std::string, toppers::xml::info> const& info_map);
      virtual string do_search_macro(std::vector< toppers::xml::container::object* > const& objects ,std::map<std::string, toppers::xml::info> const& info_map );
      virtual void do_search_includes(std::vector< toppers::xml::container::object* > const& objects ,std::string include_container, std::list< string > *incstr );
      virtual void do_load_cfg( std::string const& input_file, codeset_t codeset, std::map<std::string, toppers::xml::info> const& info_map );
      virtual void do_generate_includes() const
      {
      }
      virtual void do_generate_cfg1_def() const
      {
        ofile_ << "const uint32 TOPPERS_cfg_magic_number = 0x12345678;\n"
                  "const uint32 TOPPERS_cfg_sizeof_signed_t = sizeof(signed_t);\n"
                  "const uint32 TOPPERS_cfg_sizeof_pointer = sizeof(const volatile void*);\n"
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

    string cfg1_out::search_info_pathname(string shortname) const
    {
      string result;
      std::map< std::string, std::string >::iterator pInfo = pimpl_->info_rmap_.find(shortname);

      if( pInfo != pimpl_->info_rmap_.end() )
      {
        result = pInfo->second;
      }
      return result;
    }

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
	  del_containers();
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
    		std::map<std::string, toppers::xml::info> const& info_map )
    {
      return pimpl_->do_load_cfg( input_file, codeset, info_map );
    }

    // パラメータの型がECUC-NUMERICAL-PARAM-VALUEなのに整数・浮動小数点でない場合はマクロ名として、cfg_out.cへ出力する
    string cfg1_out::implementation::do_out_macro_name( std::vector< toppers::xml::container::parameter* >::const_iterator pPara, int serial,
      std::map<std::string, toppers::xml::info> const& info_map )
    {
      ostringstream stream;
      if( toppers::xml::container::TYPE_INT == (*pPara)->getType() || toppers::xml::container::TYPE_FLOAT == (*pPara)->getType() )
      {
        string contanerDefName( (*pPara)->getParent()->getDefName() );
        string contanerObjName( (*pPara)->getParent()->getObjName() );
        string paramDefName( (*pPara)->getDefName() );
        string paramValue( (*pPara)->getValue() );
        string grandObjName;
        if( (*pPara)->getParent()->getParent() != NULL )
        {
          grandObjName = (*pPara)->getParent()->getParent()->getObjName();
        }

        std::map<std::string, toppers::xml::info>::const_iterator pInfo = info_map.find(contanerDefName);
        if(pInfo != info_map.end())
        {
          contanerDefName = Replace(contanerDefName, pInfo->first, pInfo->second.tfname);
        }
        pInfo = info_map.find(paramDefName);
        if(pInfo != info_map.end())
        {
          paramDefName = Replace(paramDefName, pInfo->first, pInfo->second.tfname);
        }

        boost::replace_all(contanerDefName, ".", "_");
        boost::replace_all(paramDefName, ".", "_");
        if(toppers::xml::container::TYPE_INT == (*pPara)->getType())
        {
          try
          {
            int value = boost::lexical_cast<int>( paramValue );
          }
          catch( const boost::bad_lexical_cast& e)
          {
            stream << "const unsigned_t TOPPERS_cfg_valueof_" << contanerDefName << "_" << paramDefName << "_" << contanerObjName  << "_" << grandObjName << " = ( " << paramValue << " ); \n";
          }
        }
        else
        {
          try
          {
            float value = boost::lexical_cast<float>( paramValue );
          }
          catch( const boost::bad_lexical_cast& e)
          {
            stream << "const unsigned_t TOPPERS_cfg_valueof_" << contanerDefName << "_" << paramDefName << "_" << contanerObjName  << "_" << grandObjName << " = ( " << paramValue << " ); \n";
          }
        }
      }
      return stream.str();
    }

    string cfg1_out::implementation::do_search_macro(std::vector< toppers::xml::container::object* > const& objects,
      std::map<std::string, toppers::xml::info> const& info_map )
    {
      ostringstream stream;

      for ( std::vector< toppers::xml::container::object* >::const_iterator pObj = objects.begin() ;
        pObj != objects.end();
        ++pObj )
      {
        int serial = 0;
        if( (*pObj)->getSubcontainers()->size() != 0)
        {
          stream << do_search_macro( *(*pObj)->getSubcontainers(), info_map );
        }
        for( std::vector< toppers::xml::container::parameter* >::iterator pPara = (*pObj)->getParams()->begin() ;
          pPara != (*pObj)->getParams()->end();
          ++pPara )
        {
          stream << do_out_macro_name( pPara, serial++, info_map );
        }
      }
      return stream.str();
    }

    /*!
     *  \brief  インクルードコンテナ情報を抽出する
     *  \param[in]  objects  XMLでパースしたコンテナの連想配列
     *  \param[in]  include_container 抽出するコンテナのフルパス名
     */
    void cfg1_out::implementation::do_search_includes(std::vector< toppers::xml::container::object* > const& objects,
        std::string include_container, std::list< string > *incstr )
    {
        ostringstream stream;

        for ( std::vector< toppers::xml::container::object* >::const_iterator pObj = objects.begin() ;
            pObj != objects.end();
            ++pObj )
        {
            if( (*pObj)->getSubcontainers()->size() != 0)
            {
                do_search_includes( *(*pObj)->getSubcontainers(), include_container, incstr );
            }
            for( std::vector< toppers::xml::container::parameter* >::iterator pPara = (*pObj)->getParams()->begin() ;
                pPara != (*pObj)->getParams()->end();
                ++pPara )
            {
                if( (*pPara)->getDefName() == include_container )
                    (*incstr).push_back( (*pPara)->getValue() );
            }
        }
        return ;
    }

    /*! 
     *  \brief  多重度情報を検証する 
     *  \param[in]  objects  XMLでパースしたコンテナの連想配列 
     *  \param[in]  info_map ATK2で指定するコンテナ情報の連想配列 
     */ 
    void cfg1_out::implementation::validate_multiplicity( toppers::xml::container::object* object,
      std::map<std::string, toppers::xml::info> const& info_map )
    {
      std::string objPath( object->getDefName() );
      std::map<std::string, toppers::xml::info> search_map;
      std::map<std::string, unsigned int> multi_map;

      // 自コンテナのパラメータとサブコンテナの多重度定義情報をinfo_mapから取得する
      for( std::map<std::string, toppers::xml::info>::const_iterator pInfo = info_map.begin() ;
        pInfo != info_map.end() ;
        ++pInfo )
      {
        // objPathと同じ文字列がある場合（後につづくパスがあってもよい）
        if( pInfo->first.find(objPath) != string::npos )
        {
          // objPathのあとに続く文字列を取得
          std::string str( pInfo->first.substr(objPath.size()) );
          // strのパス情報が1つしかない場合は自コンテナのパラメータかサブコンテナである
          if( 0 == (int)str.rfind("/") )
          {
            search_map.insert(make_pair(pInfo->first, pInfo->second ));
          }
        }
      }

      // 自コンテナのパラメータの多重度を集計する
      for ( std::vector<toppers::xml::container::parameter*>::const_iterator pPara = object->getParams()->begin() ;
        pPara != object->getParams()->end();
        ++pPara )
      {
        std::map<std::string, unsigned int>::iterator pMulti = multi_map.find( (*pPara)->getDefName() );
        if(pMulti == multi_map.end())
        {
          multi_map.insert( make_pair((*pPara)->getDefName(), 1) );
        }
        else
        {
          pMulti->second++;
        }
      }
      // 自コンテナのサブコンテナの多重度を集計する
      for ( std::vector< toppers::xml::container::object* >::iterator pObj = object->getSubcontainers()->begin() ;
        pObj != object->getSubcontainers()->end();
        ++pObj )
      {
        std::map<std::string, unsigned int>::iterator pMulti = multi_map.find( (*pObj)->getDefName() );
        if(pMulti == multi_map.end())
        {
          multi_map.insert( make_pair((*pObj)->getDefName(), 1) );
        }
        else
        {
          pMulti->second++;
        }
      }

      // 多重度情報をチェックする
      for (  std::map<std::string, toppers::xml::info>::const_iterator pSearch = search_map.begin() ;
        pSearch != search_map.end();
        ++pSearch )
      {
        std::map<std::string, unsigned int>::iterator pMulti = multi_map.find( pSearch->first );
        if(pMulti == multi_map.end())
        {
          if( pSearch->second.multimin > 0)
          {
            fatal( _("`%1%\' is not exist.") ,pSearch->first );
          }
        }
        else
        {
          if( pSearch->second.multimin > pMulti->second  || pSearch->second.multimax < pMulti->second )
          {
            fatal( _("`%1%\' multiplicity is out of range. This multiplicity is `%2%'.") ,pSearch->first ,pMulti->second );
          }
        }
      }
      
      // サブコンテナのチェックを行う
      for (  std::vector<toppers::xml::container::object*>::const_iterator pSub = object->getSubcontainers()->begin() ;
        pSub != object->getSubcontainers()->end();
        ++pSub )
      {
        validate_multiplicity(*pSub, info_map);
      }
    }

    /*!
     *  \brief  コンテナのパラメータの型情報を検証する
     *  \param[in]  objects  XMLでパースしたコンテナの連想配列
     *  \param[in]  info_map ATK2で指定するコンテナ情報の連想配列
     */
    void cfg1_out::implementation::validate_type(std::vector< toppers::xml::container::object* > const& objects,
      std::map<std::string, toppers::xml::info> const& info_map )
    {
	  std::list<std::string> moduleNames;
	  boost::split(moduleNames, get_global_string( "XML_ModuleName" ), boost::is_any_of(",") );
	  
      for ( std::vector< toppers::xml::container::object* >::const_iterator pObj = objects.begin() ;
        pObj != objects.end();
        ++pObj )
      {
        for(std::vector< toppers::xml::container::parameter* >::const_iterator pPara = (*pObj)->getParams()->begin() ;
          pPara != (*pObj)->getParams()->end();
          ++pPara )
        {
          std::map<std::string, toppers::xml::info>::const_iterator pInfo = info_map.find( (*pPara)->getDefName() );
          if( pInfo == info_map.end() )
          {
            BOOST_FOREACH(std::string module, moduleNames)
            {
              if ( module.empty() )
              {
                warning( _( "Unknown Parameter(%1%:%2%) : `%3%\'. " ), (*pPara)->getFileName(), (*pPara)->getLine(), (*pPara)->getDefName() );
                (*pPara)->setType(toppers::xml::container::TYPE_UNKNOWN);
              }
              else
              {
                if( (*pPara)->getDefName().find( get_global_string( "XML_ContainerPath" ) + "/" + module ) != string::npos  )
                {
                  warning( _( "Unknown Parameter(%1%:%2%) : `%3%\'. " ), (*pPara)->getFileName(), (*pPara)->getLine(), (*pPara)->getDefName() );
                  (*pPara)->setType(toppers::xml::container::TYPE_UNKNOWN);
                  break;
                }
              }
            }
          }
          else
          {
            // 型の上書き
            if('+' == pInfo->second.type[0])
            {
                (*pPara)->setType( pInfo->second.type_enum ) ;
            }
            // パラメータの型チェック
            else if( (*pPara)->getType() != pInfo->second.type_enum )
            {
              int line = (*pPara)->getLine();
              fatal( _( "Parameter type miss match(%1%:%2%) : `%3%\' is not `%4%\' type. " ), (*pPara)->getFileName(), (*pPara)->getLine(), (*pPara)->getDefName(), pInfo->second.type );
            }
          }
        }
        // サブコンテナがある場合
         for (  std::vector<toppers::xml::container::object*>::iterator pSub = (*pObj)->getSubcontainers()->begin() ;
          pSub != (*pObj)->getSubcontainers()->end();
          ++pSub )
        {
          validate_type( *(*pObj)->getSubcontainers(), info_map );
        }
      }
    }

    /*!
     *  \brief  システムコンフィギュレーションファイルのロード処理の実体
     *  \param[in]  input_file  入力ファイル名
     *  \param[in]  codeset     文字コード
     *  \param[in]  obj_info    オブジェクト情報の連想配列
     */
    void cfg1_out::implementation::do_load_cfg( std::string const& input_file, 
                codeset_t codeset, std::map<std::string, toppers::xml::info> const& info_map )
    {
      // XMLファイルのパース処理
      std::vector< toppers::xml::container::object*> container_array_temp( xml_parser_init(input_file) );

      // インクルードコンテナのフルパス名を取得
      std::string include_container;
      std::string s1;
      for( std::map<std::string, toppers::xml::info>::const_iterator pInfo = info_map.begin() ;
          pInfo != info_map.end() ;
          ++pInfo)
      {
          s1 = pInfo->second.type;
          if(s1.find("INCLUDE", 0 ) != string::npos)
              include_container = pInfo->first;
      }
      if ( include_container.empty() )
        include_container = "/AUTOSAR/EcucDefs/Os/OsInclude/OsIncludeFileName";

      //インクルードコンテナ情報を取得
      std::list< string > incstr;
      do_search_includes( container_array_temp, include_container, &incstr );

      std::ostringstream oss, includes_oss;
      std::list< string >::iterator pIncStr = incstr.begin();
      while( pIncStr != incstr.end() )
      {
        oss << "/* #include \"" << *pIncStr << "\" */\n";
        includes_oss << "#include \"" << *pIncStr << "\"\n";

        ++pIncStr;
      }

      std::string container_path( get_global_string( "XML_ContainerPath" ) );
      std::list<std::string> moduleNames;
      boost::split(moduleNames, get_global_string( "XML_ModuleName" ), boost::is_any_of(",") );
      // XML_ModuleNameで設定されていないモジュールは削除する
      for( std::vector< toppers::xml::container::object* >::iterator pObj = container_array_temp.begin() ;
        pObj != container_array_temp.end();
        ++pObj)
      {
        //std::cout << "defName:" << (*pObj)->getDefName() << std::endl;
        int enaFlg = 0;
        BOOST_FOREACH(std::string module, moduleNames)
        {
          if ( (*pObj)->getDefName().find( container_path + "/" + module ) != string::npos )
          {
            enaFlg++;
          }
        }
        if(enaFlg == 0)
        {
          SAX2Handlers::obj_delete((*pObj));
        }
      }

      // 多重度の検証
      toppers::xml::container::object *obj = new toppers::xml::container::object();
      obj->setSubcontainers(container_array_temp);
      obj->setDefName( container_path );
      validate_multiplicity(obj, info_map);

      // 型情報の検証
      for ( std::vector< toppers::xml::container::object* >::iterator pObj = container_array_temp.begin() ;
        pObj != container_array_temp.end();
        ++pObj )
      {
        validate_type(*(*pObj)->getSubcontainers(), info_map);
      }

      // api-tableの短縮名とURI名の連想配列(info_rmap_)を作成
      for( std::map<std::string, toppers::xml::info>::const_iterator pInfo = info_map.begin() ;
        pInfo != info_map.end() ;
        ++pInfo)
      {
        info_rmap_.insert( pair<std::string, std::string>(pInfo->second.tfname, pInfo->first));
      }

      // XMLパースのパラメータでマクロになっているものを出力する
      oss << do_search_macro(container_array_temp, info_map);
          
      // データメンバへの反映
      std::string cfg1_list_temp( oss.str() );
      std::string includes_temp( includes_oss.str() );

      cfg1_out_list_.swap( cfg1_list_temp );
      container_array_.swap(container_array_temp);
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
                        "#include \"kernel/kernel_int.h\"\n";
      pimpl_->do_generate_includes();
      pimpl_->ofile_ << pimpl_->includes_ << '\n';

      // int128_tは故意に無視
      // int128_tに揃えると処理が重くなりすぎるため
      pimpl_->ofile_ << "\n#ifdef INT64_MAX\n"
                        "  typedef sint64 signed_t;\n"
                        "  typedef uint64 unsigned_t;\n"
                        "#else\n"
                        "  typedef sint32 signed_t;\n"
                        "  typedef uint32 unsigned_t;\n"
                        "#endif\n";

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



    cfg1_out::xml_obj_map cfg1_out::merge(std::map<std::string, toppers::xml::info> const& info_map ) const
    {
      return do_merge(pimpl_->container_array_, info_map);
    }

    /*!
     *  \brief  コンテナごとにXMLコンテナ情報をまとめる
     *  \return コンテナ情報
     *
     *  この関数は、"OsOS"や"OsTask"といった種別をキーとして、その種別に分類されるコンテナ情報の連想配列を生成する。
     */
    void cfg1_out::do_sub_merge(std::vector< toppers::xml::container::object* > objects, xml_obj_map & xml_map,
      std::map<std::string, toppers::xml::info> const& info_map ) const
    {
      for ( std::vector< toppers::xml::container::object* >::iterator pObj = objects.begin() ;
        pObj != objects.end();
        ++pObj )
      {
        if((*pObj)->getSubcontainers()->size() != 0)
        {
          do_sub_merge( *(*pObj)->getSubcontainers(), xml_map, info_map);
        }
        // XMLのフルパス情報を短縮文字列へ置き換える
        string str = replase_xml_pathname(pObj, info_map);
        xml_map[ str ].push_back( *pObj );
      }
    }

    void cfg1_out::do_sub_siblings(std::vector< toppers::xml::container::object* > objects, int siblings) const
    {
        for ( std::vector< toppers::xml::container::object* >::iterator pObj = objects.begin() ;
            pObj != objects.end();
            ++pObj )
        {
            int SubcontinersSiblings = (*pObj)->getSubcontainers()->size();
            if( SubcontinersSiblings != 0)
            {
                do_sub_siblings( *(*pObj)->getSubcontainers(), SubcontinersSiblings );
            }
            (*pObj)->setSiblings(siblings);
        }
    }

    cfg1_out::xml_obj_map cfg1_out::do_merge(std::vector< toppers::xml::container::object* > objects,
      std::map<std::string, toppers::xml::info> const& info_map ) const
    {
      xml_obj_map result; // コンテナの連想配列

      for ( std::vector< toppers::xml::container::object* >::iterator pModule = objects.begin() ;
          pModule != objects.end();
          ++pModule )
      {
        for ( std::vector< toppers::xml::container::object* >::iterator pObj = (*pModule)->getSubcontainers()->begin() ;
            pObj != (*pModule)->getSubcontainers()->end();
            ++pObj )
        {
          {
            string str = replase_xml_pathname(pObj, info_map);
            result[ str ].push_back( *pObj );
          }
        }
        // サブコンテナを連想配列へ登録
        for ( std::vector< toppers::xml::container::object* >::iterator pObj = (*pModule)->getSubcontainers()->begin() ;
              pObj != (*pModule)->getSubcontainers()->end();
              ++pObj )
        {
          if( (*pObj)->getSubcontainers()->size() != 0)
          {
            do_sub_merge( *(*pObj)->getSubcontainers(), result, info_map);
          }
        }
      }

      // ID番号の割付け
      assign_id(result);

      // 兄弟コンテナの数を抽出
      int SiblingsNumber = 0;
      for ( std::map< std::string, std::vector<toppers::xml::container::object*> >::iterator pObj = result.begin() ;
          pObj != result.end();
          ++pObj )
      {
        SiblingsNumber = pObj->second.size();
        for ( std::vector< toppers::xml::container::object* >::iterator p = pObj->second.begin() ;
            p != pObj->second.end();
            ++p )
        {
          // サブコンテナの兄弟コンテナ数を抽出
          int SubcontinersSiblings = (*p)->getSubcontainers()->size();
          if( SubcontinersSiblings != 0)
          {
            do_sub_siblings( *(*p)->getSubcontainers(), SubcontinersSiblings );
          }
          (*p)->setSiblings(SiblingsNumber);
        }
      }

      return result;
    }

    /*
     * コンテナ、パラメータURI名をapi-tableで記述した名称に置き換える
     */
    string cfg1_out::replase_xml_pathname(std::vector< toppers::xml::container::object* >::iterator pObj, 
      std::map<std::string, toppers::xml::info> const& info_map ) const
    {
      string ObjName = (*pObj)->getDefName();
      std::map<std::string, toppers::xml::info>::const_iterator pInfo = info_map.find( ObjName );
      if(pInfo != info_map.end())
      {
        ObjName = Replace(ObjName, pInfo->first, pInfo->second.tfname);
        (*pObj)->setDefName( ObjName );
      }

      for( std::vector< toppers::xml::container::parameter* >::iterator pPara = (*pObj)->getParams()->begin() ;
        pPara != (*pObj)->getParams()->end(); ++pPara)
      {
        string ParamName = (*pPara)->getDefName();
        pInfo = info_map.find( ParamName );
        if(pInfo != info_map.end())
        {
          (*pPara)->setDefName( Replace(ParamName, pInfo->first, pInfo->second.tfname) );
        }
      }
      return ObjName;
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

    void cfg1_out::del_containers()
    {
      for ( std::vector< toppers::xml::container::object* >::iterator pObj = pimpl_->container_array_.begin() ;
        pObj != pimpl_->container_array_.end();
        ++pObj )
      {
        SAX2Handlers::obj_delete((*pObj));
        delete *pObj;
      }
    }
  }
}
