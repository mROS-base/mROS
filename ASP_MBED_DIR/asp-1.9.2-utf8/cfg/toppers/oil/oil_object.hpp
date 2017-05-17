/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2010 by Meika Sugimoto
 *  Copyright (C) 2012 by TAKAGI Nobuhisa
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

#ifndef OIL_OBJECT_H
#define OIL_OBJECT_H

#include <string.h>
#include <vector>
#include <map>
#include <algorithm>
#include <limits>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#include "toppers/workaround.hpp"
#include "toppers/diagnostics.hpp"

using namespace std;

namespace toppers
{
  namespace oil
  {
    extern void dump(boost::smatch *r);

    namespace oil_definition
    {
      // クラスの前方宣言
      class object_parameter_def;
      class object_definition;
    }

    // パラメータタイプ
    enum PARAMETER_TYPE
    {
      TYPE_UINT = 1 ,
      TYPE_INT ,
      TYPE_FLOAT ,
      TYPE_STRING ,
      TYPE_BOOLEAN ,
      TYPE_ENUM , 
      TYPE_REF ,
      TYPE_UNKNOWN
    };

    typedef struct
    {
      string obj_type;
      string obj_name;
    } object_ref;

    namespace oil_implementation
    {

      // クラスの前方宣言
      class object_parameter_impl;
      
      class oil_object_impl
      {
        public:
          oil_object_impl(string object_name)
          {
            name = object_name;
          }
          ~oil_object_impl();
          int add_parameter(string param_name , string parameter_descripition);
          bool validate_object_configuration
              (oil_definition::object_definition* obj_def , std::vector<object_ref> *references);
          void display_implementation(void);
        private:
                    // modified by takuya 110823
          //bool oil_object_impl::validate_object_parameter
          bool validate_object_parameter
            (oil_definition::object_parameter_def *obj_param , bool *multiple ,
                std::vector<object_ref> *references);
          string name;
          std::vector<object_parameter_impl*> params;
      };

      // 値制限の種類
      enum VALUE_LIMIT_TYPE
      {
        LIMIT_RANGE = 1 ,
        LIMIT_LIST ,
        LIMIT_NONE
      };
      // デフォルト値の種類
      enum DEFAULT_TYPE
      {
        NO_DEFAULT = 1 ,
        AUTO ,
        HAVE_DEFAULT ,
        HAVE_NOT_DEFINE
      };

      class object_parameter_impl
      {
        public:
          object_parameter_impl(string param_name)
          {
            name = param_name;
          }
          object_parameter_impl(string param_name , string parameter_description)
          {
            name = param_name;

            // AUTO属性の解析
            if(parameter_description.find("WITH_AUTO") != string::npos)
            {
              assign_auto = true;
            }
            else
            {
              assign_auto = false;
            }

            /// ENUMのときに問題あり
            // 複数定義有無の解析
            boost::regex brace("\\[\\s*\\]\\s*(=.+)?;");
            if(boost::regex_search(parameter_description , brace))
            {
              multiple = true;
            }
            else
            {
              multiple = false;
            }
            
            // デフォルト値の解析
            boost::regex default_value("=\\s+(\\S+)\\s*;");
            boost::smatch result;
            string::const_iterator start = parameter_description.begin();
            string::const_iterator end = parameter_description.end();
            if(boost::regex_search(start , end , result , default_value))
            {
              default_description = result.str(1);
              if(default_description == "AUTO")
              {
                default_type = AUTO;
              }
              else if(default_description == "NO_DEFAULT")
              {
                default_type = NO_DEFAULT;
              }
              else
              {
                default_type = HAVE_DEFAULT;
              }
            }
            else
            {
              default_type = HAVE_NOT_DEFINE;
              default_description.erase(0);
            }
          }

          string get_name(void)
          {
            return name;
          }

          DEFAULT_TYPE get_default(string *str)
          {
            if((default_type == HAVE_DEFAULT)
              || (default_type == AUTO))
            {
              *str = default_description;
            }

            return default_type;
          }

          virtual void display_implementation(void)
          {
            cout << "Parameter Name : " << name << endl;
            cout << "Multilple : " << ((multiple == true)? "Yes" : "No") << endl;
            cout << "Auto Assignment : " << ((assign_auto == true)? "Yes" : "No") << endl;
            cout << "Default Value : ";
            if(default_type == NO_DEFAULT)
            {
              cout << "NO_DEFAULT";
            }
            else if(default_type == AUTO)
            {
              cout << "AUTO";
            }
            else
            {
              cout << "HAVE_DEFAULT";
            }
            cout << endl;
          }

          virtual bool validate_parameter_name(string param_name)
          {
            if(name == param_name)
            {
              return true;
            }
            else
            {
              return false;
            }
          }

          bool validate_object_parameter
              (oil_definition::object_parameter_def *obj , bool *auto_param);
          //virtual bool object_parameter_impl::validate_object_parameter
          virtual bool validate_object_parameter
              (oil_definition::object_parameter_def *obj)
          {
            return false;
          }
          virtual void set_parameter_type(oil_definition::object_parameter_def *obj);

          bool get_multiple(void)
          {
            return multiple;
          }
        protected:
          inline const char* return_format(double x , int type)
          {
            return "%lf";
          }

          inline const char* return_format(int64_t x , int type)
          {
            if(type == 0)
            {
                            // modified by takuya 110823
              // return "%I64x";
              return "%llx";
            }
            else
            {
                            // modified by takuya 110823
              // return "%I64d";
              return "%lld";
            }
          }

          inline const char* return_format(uint64_t x , int type)
          {
            if(type == 0)
            {
                            // modified by takuya 110823
              // return "%I64x";
              return "%llx";
            }
            else
            {
                            // modified by takuya 110823
              // return "%I64u";
              return "%llu";
            }
          }

          template <typename T>
          T str2val(const char *str , T dummy)
          {
            T temp;
            T zero = 0 , one = 1;
            if((str[0] == '0') && ((str[1] == 'x') || (str[1] == 'X')))
            {
                            // modified by takuya 110823
              //sscanf_s(&str[2] , return_format(zero , 0) , &temp);
              sscanf(&str[2] , return_format(zero , 0) , &temp);
            }
            else
            {
                            // modified by takuya 110823
              //sscanf_s(&str[0] , return_format(zero , 1) , &temp);
              sscanf(&str[0] , return_format(zero , 1) , &temp);
            }
            return temp;
          }

          template <typename T>
          void split_number_list(const char *str , std::vector<T> *list)
          {
            const char *p = str;
            char val[32];
            int i;
            T zero = 0;
          
            // リストを解析
            do
            {
              i = 0;
              // スペースをとばす
              while(*p == ' ')
              {
                p++;
              }
              // 数値部分の解析
              while(*p != ' ' && (*p != '\0'))
              {
                val[i++] = *p++;
              }
              // 後ろのスペースとカンマもとばす
              while((*p == ' ') || (*p == ','))
              {
                p++;
              }
              // 数値の解析とリストへの挿入
              val[i] = '\0';
              list->push_back((T)str2val(val , zero));
            }while(*p != '\0');
          }

          template <class T , typename Y>
          void dump_limit_value(T* obj)
          {
            if(obj->limit_type == LIMIT_RANGE)
            {
              cout << "Min value : " << obj->min << endl;
              cout << "Max value : " << obj->max << endl;
            }
            else if(obj->limit_type == LIMIT_LIST)
            {
                            // modified by takuya 110823
              //std::vector<Y>::iterator p;
              typename std::vector<Y>::iterator p;

              cout << "Limit value : ";
              for(p = obj->value_list.begin() ; p != obj->value_list.end() ; p++)
              {
                cout << *p << ",";
              }
              cout << endl;
            }
            else
            {
              cout << "Limit value : None" << endl;
            }

            if(obj->default_type == HAVE_DEFAULT)
            {
              cout << "Default value : " << obj->default_value << endl;
            }
            cout << endl;
          }
          
          int id;
          string name;
          bool multiple;
          string default_description;
          bool assign_auto;
          DEFAULT_TYPE default_type;
      };
      
      // INT型（INT32,INT64)
      class parameter_int_impl : public object_parameter_impl
      {
        friend class object_parameter_impl;

        public:
          parameter_int_impl(string param_name , string parameter_description)
            : object_parameter_impl(param_name , parameter_description)
          {
            // 制限値のパース
            boost::regex limit_list("\\[\\s*((\\w)+(\\s*,\\s*\\w+)*)\\s*\\]");
            boost::regex limit_range("\\[\\s*(\\w+)\\s*\\.\\.\\s*(\\w+)\\s*\\]");
            string::const_iterator start = parameter_description.begin();
            string::const_iterator end = parameter_description.end();
            boost::smatch result;
            
            // リスト形式かのチェック
            if(boost::regex_search(start , end , result , limit_list))
            {
              limit_type = LIMIT_LIST;
              string str = result.str(1);
              split_number_list(str.c_str() , &value_list);
            }
            // レンジ形式かのチェック
            else if(boost::regex_search(start , end , result , limit_range))
            {
              limit_type = LIMIT_RANGE;
              string str;

              // 最小値
              str = result.str(1);
              min = str2val(str.c_str() , (int64_t)0);
              // 最大値
              str = result.str(2);
              max = str2val(str.c_str() , (int64_t)0);
            }
            else
            {
              min = 0;
              if(parameter_description.find("INT32") != string::npos)
              {
                max = std::numeric_limits<int32_t>::max();
              }
              else
              {
                max = std::numeric_limits<int64_t>::max();
              }
              limit_type = LIMIT_RANGE;
            }

            // デフォルト値の算出
            if(default_type == HAVE_DEFAULT)
            {
              default_value = str2val(default_description.c_str() , (int64_t)0);
            }
          }
          virtual void display_implementation(void)
          {
            cout << "Parameter types : INT" << endl;
            object_parameter_impl::display_implementation();
            dump_limit_value<parameter_int_impl , int64_t>(this);
          }
          bool validate_object_parameter(oil_definition::object_parameter_def *obj);
          void set_parameter_type(oil_definition::object_parameter_def *obj);
        protected:
          VALUE_LIMIT_TYPE limit_type;
          std::vector<int64_t> value_list;
          int64_t default_value;
          int64_t max;
          int64_t min;
      };

      // UINT型（UINT32,UINT64)
      class parameter_uint_impl : public object_parameter_impl
      {
        friend class object_parameter_impl;

        public:
          parameter_uint_impl(string param_name , string parameter_description)
            : object_parameter_impl(param_name , parameter_description)
          {
            // 制限値のパース
            boost::regex limit_list("\\[\\s*((\\w)+(\\s*,\\s*\\w+)*)\\s*\\]");
            boost::regex limit_range("\\[\\s*(\\w+)\\s*\\.\\.\\s*(\\w+)\\s*\\]");
            string::const_iterator start = parameter_description.begin();
            string::const_iterator end = parameter_description.end();
            boost::smatch result;
            
            // リスト形式かのチェック
            if(boost::regex_search(start , end , result , limit_list))
            {
              limit_type = LIMIT_LIST;
              string str = result.str(1);
              split_number_list(str.c_str() , &value_list);
            }
            // レンジ形式かのチェック
            else if(boost::regex_search(start , end , result , limit_range))
            {
              limit_type = LIMIT_RANGE;
              string str;

              // 最小値
              str = result.str(1);
              min = str2val(str.c_str() , (uint64_t)0);
              // 最大値
              str = result.str(2);
              max = str2val(str.c_str() , (uint64_t)0);
            }
            else
            {
              min = 0;
              if(parameter_description.find("UINT32") != string::npos)
              {
                max = std::numeric_limits<uint32_t>::max();
              }
              else
              {
                max = std::numeric_limits<uint64_t>::max();
              }
              limit_type = LIMIT_RANGE;
            }
            // デフォルト値の算出
            if(default_type == HAVE_DEFAULT)
            {
              default_value = str2val(default_description.c_str() , (uint64_t)0);
            }       
          }

          virtual void display_implementation(void)
          {
            cout << "Parameter types : UINT" << endl;
            object_parameter_impl::display_implementation();
            dump_limit_value<parameter_uint_impl , uint64_t>(this);
          }
          bool validate_object_parameter(oil_definition::object_parameter_def *obj);
          void set_parameter_type(oil_definition::object_parameter_def *obj);
        protected:
          VALUE_LIMIT_TYPE limit_type;
          std::vector<uint64_t> value_list;
          uint64_t max;
          uint64_t min;
          uint64_t default_value;
      };
      
      // FLOAT型（FLOAT)
      class parameter_float_impl : public object_parameter_impl
      {
        friend class object_parameter_impl;

        public:
          parameter_float_impl(string param_name , string parameter_description)
              : object_parameter_impl(param_name , parameter_description)
          {
            // 制限値のパース
            boost::regex limit_list("\\[\\s*((\\w(\\.\\w+)?)+(\\s*,\\s*\\w+(\\.\\w+)?)*)\\s*\\]");
            boost::regex limit_range("\\[\\s*(\\w+(\\.\\w+)?)\\s*\\.\\.\\s*(\\w+(\\.\\w+)?)\\s*\\]");
            string::const_iterator start = parameter_description.begin();
            string::const_iterator end = parameter_description.end();
            boost::smatch result;
            
            // リスト形式かのチェック
            if(boost::regex_search(start , end , result , limit_list))
            {
              limit_type = LIMIT_LIST;
              string str = result.str(1);
              split_number_list(str.c_str() , &value_list);
            }
            // レンジ形式かのチェック
            else if(boost::regex_search(start , end , result , limit_range))
            {
              limit_type = LIMIT_RANGE;
              string str;

              // 最小値
              str = result.str(1);
              min = str2val(str.c_str() , (double)0);
              // 最大値
              str = result.str(3);
              max = str2val(str.c_str() , (double)0);
            }
            else
            {
              limit_type = LIMIT_NONE;
            }
            // デフォルト値の算出
            if(default_type == HAVE_DEFAULT)
            {
              default_value = str2val(default_description.c_str() , (double)0);
            }     
          }
          virtual void display_implementation(void)
          {
            cout << "Parameter types : FLOAT" << endl;
            object_parameter_impl::display_implementation();
            dump_limit_value<parameter_float_impl , double>(this);
          }
          bool validate_object_parameter(oil_definition::object_parameter_def *obj);
          void set_parameter_type(oil_definition::object_parameter_def *obj);
        protected:
          VALUE_LIMIT_TYPE limit_type;
          std::vector<double> value_list;
          double max;
          double min;
          double default_value;
      };
      
      // STRING型
      class parameter_string_impl : public object_parameter_impl
      {
        public:
          parameter_string_impl(string param_name , string parameter_description)
              : object_parameter_impl(param_name , parameter_description)
          {
            // デフォルト値の代入
            if(default_type == HAVE_DEFAULT)
            {
              default_value = default_description;
            }     
          }

          virtual void display_implementation(void)
          {
            cout << "Parameter types : STRING" << endl;
            object_parameter_impl::display_implementation();
            if(default_type == HAVE_DEFAULT)
            {
              cout << "Default value : " << default_value << endl;
            }   
          }
          virtual bool validate_object_parameter
              (oil_definition::object_parameter_def *obj);
          virtual void set_parameter_type(oil_definition::object_parameter_def *obj);
        protected:
          string default_value;
      };

      // BOOLEAN型
      class parameter_boolean_impl : public parameter_string_impl
      {
        public:
          parameter_boolean_impl(string param_name , string parameter_description)
            : parameter_string_impl(param_name , parameter_description)
          {
          }
          virtual bool validate_object_parameter
              (oil_definition::object_parameter_def *obj);
          virtual void set_parameter_type(oil_definition::object_parameter_def *obj);
      };

      // ENUM型
      class parameter_enum_impl : public object_parameter_impl
      {
        public:
          parameter_enum_impl(string param_name , string parameter_description)
              : object_parameter_impl(param_name , parameter_description)
          {
            boost::regex enum_description("(\\w+\\s+[^;]+);(\\s*(.+)\\s)*");
            boost::regex enum_parse("(\\w+\\s+\\[)?\\s*((\\w+)\\s*(\\{([^\\{\\}]*)\\})?)(\\s*,\\s*(.+))*");
            boost::regex subparam_name("([^\\[\\]\\s]+)\\s+((WITH_AUTO)?\\s+(\\[.+\\])?)?([^\\[\\]\\s]+)");
            boost::regex space("\\s*");
            string::const_iterator start = parameter_description.begin();
            string::const_iterator end = parameter_description.end();
            boost::smatch result;
            toppers::oil::oil_implementation::oil_object_impl *sub_impl;
            string name;

            // 1個ずつパラメータを切り分けて処理
            while(boost::regex_search(start , end , result , enum_parse))
            {
              string temp = result.str(5);
              string::const_iterator start2 = temp.begin();
              string::const_iterator end2 = temp.end();
              boost::smatch result2;

              try
              {
                /// try catch
                sub_impl = new oil_object_impl(param_name);
                name = result.str(3);
                // パラメータの切り分け
                while(boost::regex_search(start2 , end2 , result2 , enum_description))
                {
                  string temp2 = result2.str(1);
                  string::const_iterator start3 = temp2.begin();
                  string::const_iterator end3 = temp2.end();
                  boost::smatch result3;
                  // パラメータ名の抽出
                  (void)boost::regex_search(start3 , end3 , result3 , subparam_name);
                  
                  sub_impl->add_parameter(result3.str(5) , result2.str(1) + ";");
                  // パラメータがなくなったらループを抜ける
                  if(boost::regex_match(result2.str(3) , space))
                  {
                    break;
                  }
                  temp.erase(0 , temp.find(result2.str(3)));
                  end2 = temp.end();
                }

                // コンテナへの登録
                sub_params.insert(pair<string , oil_object_impl*>(name , sub_impl));

                // パラメータがなくなったらループを抜ける
                if(boost::regex_match(result.str(7) , space))
                {
                  break;
                }
                parameter_description.erase(0 , parameter_description.find(result.str(7)));
                end = parameter_description.end();
              }
              catch( ... )
              {
                toppers::fatal("Memory allocation error.");
              }
            }
          }

          ~parameter_enum_impl()
          {
            std::map<std::string , oil_object_impl*>::iterator p;

            for(p = sub_params.begin() ; p != sub_params.end() ; p++)
            {
              delete (*p).second;
            }
          }

          virtual void display_implementation(void)
          {
            std::map<std::string , oil_object_impl*>::iterator p;

            cout << "Parameter types : ENUM" << endl;
            object_parameter_impl::display_implementation();

            cout << "-------------- Sub parameters --------------" << endl;
            for(p = sub_params.begin() ; p != sub_params.end() ; p++)
            {
              cout << "Sub param name : " << (*p).first << endl;
              (*p).second->display_implementation();
            }
            cout << "--------------------------------------------" << endl;

          }

          bool validate_object_parameter(oil_definition::object_parameter_def *obj , 
                  std::vector<toppers::oil::object_ref> *object_refs);
          void set_parameter_type
            (oil_definition::object_parameter_def *obj);
        protected:
          std::map<std::string , oil_object_impl*> sub_params;
      };

      // 参照型
      class parameter_object_ref_impl : public object_parameter_impl
      {
        public:
          parameter_object_ref_impl(string param_name , string parameter_description)
              : object_parameter_impl(param_name , parameter_description)
          {
            // 参照するオブジェクト名の抽出
            boost::regex ref_type("(\\w+)_TYPE");
            string::const_iterator start = parameter_description.begin();
            string::const_iterator end = parameter_description.end();
            boost::smatch result;

            if(boost::regex_search(start , end , result , ref_type))
            {
              reftype_name = result.str(1);
            }
            /// デフォルト値の読み込み
          }
          virtual void display_implementation(void)
          {
            cout << "Parameter types : OBJECT REF" << endl;
            object_parameter_impl::display_implementation();
            cout << "Reference Type : " << reftype_name << endl << endl;
          }

          bool validate_object_parameter
              (oil_definition::object_parameter_def *obj , std::vector<toppers::oil::object_ref> *object_refs);
          void set_parameter_type(oil_definition::object_parameter_def *obj);
        protected:
          string reftype_name;
      };
    }

    namespace oil_definition
    {
      // オブジェクト定義のクラス
      class object_definition
      {
        public:
          object_definition(string object_name_type , object_parameter_def* parent = NULL)
          {
            boost::regex object_declaration("(\\w+)\\s+([\\w\\.]+)");
            string::const_iterator start = object_name_type.begin();
            string::const_iterator end = object_name_type.end();
            boost::smatch result;

            // 結び付けられているパラメータへのポインタを格納
            parent_parameter = parent;
            // タイプとネームをパースして格納
            if(boost::regex_match(start , end , result , object_declaration))
            {
              object_type = result.str(1);
              name = result.str(2);
            }
            else
            {
              /* ありえない */
            }
          }

          ~object_definition();

          int add_parameter(string param_name , string param_value);

          string get_object_type(void)
          {
            return object_type;
          }

          std::vector<object_parameter_def*>* get_params(void)
          {
            return &params;
          }
          void display_definition(void);
          void set_id(int obj_id)
          {
            id = obj_id;
          }
          int get_id(void)
          {
            return id;
          }
          std::string get_name(void)
          {
            return name;
          }
          object_parameter_def* get_parent(void)
          {
            return parent_parameter;
          }

        private:
          int id;
          string name;
          string object_type;
          std::vector<object_parameter_def*> params;
          object_parameter_def* parent_parameter;
      };

      // オブジェクト定義パラメータの基本クラス
      class object_parameter_def
      {
        public:
          object_parameter_def(string def_name , string def_value , string parent)
          {
            boost::regex have_subparam("(\\w+)\\s*\\{\\s*(([^\\{\\}])+)\\s*\\}");
            string::const_iterator start = def_value.begin();
            string::const_iterator end = def_value.end();
            boost::smatch result;

            name = def_name;
            parent_name = parent;

            // サブパラメータがある場合はさらに分解
            if(boost::regex_search(start , end , result , have_subparam))
            {
              string type_name;
              object_definition *subparam;

              value = result.str(1);

              type_name = result.str(1) + " SUBPARAM";

              // サブパラメータを格納するオブジェクトを生成
              try
              {
                subparam = new object_definition(name + " " + name , this);
              }
              catch( std::bad_alloc )
              {
                toppers::fatal("Memory allocation error.");
              }

              boost::regex subparam_regex("((\\w+)\\s*=\\s*([\\w\"]+\\s*(\\{\\s*[^\\{\\}]\\s*\\})?));(.+)");
              boost::regex space("\\s*");
              string temp = result.str(2);
              string::const_iterator start2 = temp.begin();
              string::const_iterator end2 = temp.end();
              boost::smatch result2;

              while(boost::regex_search(start2 , end2  , result2 , subparam_regex))
              {
                subparam->add_parameter(result2.str(2) , result2.str(3));

                // 分解し終わったら抜ける
                if(boost::regex_match(result2.str(5) , space))
                {
                  break;
                }
                temp.erase(0 , temp.find(result2.str(5)));
                end2 = temp.end();
              }
              // コンテナに追加
              subparams.push_back(subparam);
            }
            else
            {
              value = def_value;
            }
          }

          ~object_parameter_def()
          {
            std::vector<object_definition*>::iterator p;

            for(p = subparams.begin() ; p != subparams.end() ; p++)
            {
              delete (*p);
            }
          }

          string get_parameter_name(void)
          {
            return name;
          }

          string get_value(void)
          {
            return value;
          }

          string get_parent_name(void)
          {
            return parent_name;
          }

          std::vector<object_definition*>* get_subparams(void)
          {
            return &subparams;
          }

          void display_definition(void)
          {
            std::vector<object_definition*>::iterator p;

            cout <<  "\tName : " << name << " Value : " << value << endl;
            for(p = subparams.begin(); p != subparams.end(); p++)
            {
              cout << "\t";
              (*p)->display_definition();
            }
          }
          void set_type(PARAMETER_TYPE param_type)
          {
            type = param_type;
          }
          PARAMETER_TYPE get_type(void)
          {
            return type;
          }

        protected:
          string name;
          string value;
          string parent_name;
          PARAMETER_TYPE type;
          std::vector<object_definition*> subparams;
      };

      typedef std::map< std::string, std::vector< object_definition* > > cfg_obj_map;
      cfg_obj_map merge(std::vector<object_definition*> *obj_defs ,
        cfg_obj_map& out , string prefix = string("") ,  
        string suffix = string("")  , long fixed_id = UINT_MAX);
      void assign_id(cfg_obj_map obj_defs  , long fixed_id = UINT_MAX);

    } /* oil_definition */


  } /* oil */
}


#endif  /* OIL_OBJECT_H */
