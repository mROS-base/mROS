/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2010 by Meika Sugimoto
 *  Copyright (C) 2012 by TAKAGI Nonbuhisa
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

#include <vector>
#include <map>
#include <algorithm>
#include <functional>

#include "toppers/global.hpp"
#include "toppers/oil/configuration_manager.hpp"

using namespace std;
using namespace toppers::oil;
using namespace toppers::oil::oil_implementation;
using namespace toppers::oil::oil_definition;

namespace toppers
{
  namespace configuration_manager
  {
    // APPMODEを探すファンクタ
    bool find_appmode_object(object_definition *p)
    {
      if(p->get_object_type() == "APPMODE")
      {
        return true;
      }
      return false;
    }

    bool config_manage::read_configuration
      (toppers::text *txt , std::vector<std::string> const& objlist)
    {
        description = txt;

        // ストリングを作る
        description_str = new string();
        typedef text::container::const_iterator const_row_iterator;
        typedef std::string::size_type size_type;
        const_row_iterator first( txt->begin().get_row() ), last( txt->end().get_row() );

        for ( const_row_iterator iter( first ); iter != last; ++iter )
        {
          *description_str += iter->buf;
        }

        // パーサの生成
        try
        {
          parser = new ::oil_parser(description_str , objlist);
        }
        catch( ... )
        {
          toppers::fatal("Memory allocation error.");
        }

      if(parser->do_parse(&oil_impl , &oil_def) == 0)
      {
        std::string kernel_type;
        // APPMODEが定義されていない時はOSDEFAULTAPPMODEを挿入
        toppers::get_global( "kernel", kernel_type );
        if((kernel_type == "atk1")
          && (oil_def->end() == 
            (find_if(oil_def->begin() , oil_def->end() , find_appmode_object))))
        {
          oil_def->push_back(new object_definition("APPMODE OSDEFAULTAPPMODE"));
        }

        return true;
      }
      else
      {
        std::string message;
        int position;
        text::const_iterator iter;

        // エラー位置の取得
        parser->get_error(&position , &message);
        // 該当行の取得
        iter = description->line_at(position);
        text::line_type error_occur = iter.line();
        
        // 文法エラーはすぐに終了
        toppers::fatal(iter.line() , "%1%" , message);
        
        return false;
      }
    }

    bool object_matching(object_definition* obj , oil_object_impl* impl)
    {
      return impl->validate_object_configuration(obj , NULL);
    }

    bool config_manage::validate_and_assign_default_configuration(void)
    {
      bool result = true;
      std::vector<oil_definition::object_definition*>::iterator p;
      std::vector<oil_implementation::oil_object_impl*>::iterator q , found;
      
      // 全てのオブジェクト定義を実装と整合しているかチェックし、
      // デフォルトパラメータの補完を行う
      for(p = oil_def->begin() ; p != oil_def->end() ; p++)
      {
        std::vector<toppers::oil::object_ref> object_refs;
        for(q = oil_impl->begin() ; q != oil_impl->end() ; q++)
        {
          if((*q)->validate_object_configuration(*p , &object_refs) == true)
          {
            break;
          }
        }
        
        if(q >= oil_impl->end())
        {
          // パラメータの整合性チェック
          result = false;
          break;
        }
        
        // リファレンス先が正しいかのチェック
        result = validate_object_reference((*p)->get_name() , &object_refs);
      }

      return result;
    }
    bool config_manage::validate_object_reference
        (std::string obj_name , std::vector<object_ref> *obj_refs)
    {
      std::vector<oil_definition::object_definition*>::iterator q;
      std::vector<object_ref>::iterator p;
      bool result = true;
      
      for(p = obj_refs->begin() ; p != obj_refs->end() ; p++)
      {
        std::vector<oil_definition::object_definition*> name_match;
        // 名前が同じものを探索
        for(q = oil_def->begin() ; q != oil_def->end() ; q++)
        {
          if((*p).obj_name == (*q)->get_name())
          {
            name_match.push_back(*q);
          }
        }
        // 名前が一致しなければその時点でエラー
        if(name_match.empty() == true)
        {
          toppers::error("Object %1%'s parameter %2% reference %3% , but not found." ,
                    obj_name , (*p).obj_type , (*p).obj_name);
          result = false;
          continue;
        }

        // 参照先のタイプが同じか探索
        for(q = name_match.begin() ; q != name_match.end() ; q++)
        {
          if((*p).obj_type == (*q)->get_object_type())
          {
            break;
          }
        }
        // タイプが同じものがなければエラー
        if(q == name_match.end())
        {
          toppers::error("Object %1%'s parameter %2% reference %3% , but referenced object is not %4%_TYPE" , 
                  obj_name , (*p).obj_type , (*p).obj_name , (*p).obj_type);
          result = false;
        }
      }

      return result;
    }

  } /* namespace oil */

} /* namespace toppers */

