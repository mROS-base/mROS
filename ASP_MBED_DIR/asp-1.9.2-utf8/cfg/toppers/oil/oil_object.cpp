/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
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

#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/cast.hpp>
#include <algorithm>
#include <functional>

#include "toppers/oil/oil_object.hpp"
#include "toppers/diagnostics.hpp"

namespace toppers
{
	namespace oil
	{
		// デバッグ用関数
		void dump(boost::smatch *r)
		{
			unsigned int i;

			for(i = 0 ; i < r->size() ; i++)
			{
				cout << "Match[" << i << "] : " << r->str(i) << endl;
			}
		}
		
		namespace oil_implementation
		{
			int oil_object_impl::add_parameter(string param_name , string parameter_description)
			{
				if((parameter_description.find("UINT32") == 0)
					|| (parameter_description.find("UINT64") == 0))
				{
					this->params.push_back(
						(object_parameter_impl *)(new parameter_uint_impl(param_name , parameter_description)));
				}
				else if((parameter_description.find("INT32") == 0)
						|| (parameter_description.find("INT64") == 0))
				{
					this->params.push_back(
						(object_parameter_impl *)(new parameter_int_impl(param_name , parameter_description)));
				}
				else if(parameter_description.find("ENUM") == 0)
				{
					this->params.push_back(
						(object_parameter_impl *)(new parameter_enum_impl(param_name , parameter_description)));
				}
				else if(parameter_description.find("BOOLEAN") == 0)
				{
					boost::regex bool_with_option("\\[\\s*TRUE");
					string::const_iterator start = parameter_description.begin();
					string::const_iterator end = parameter_description.end();

					// [ TRUE , FALSE ]がある場合はENUMと同じ扱い、ない場合はSTRINGと同じ扱いとする
					if(regex_search(start , end , bool_with_option))
					{
						this->params.push_back(
							(object_parameter_impl *)(new parameter_enum_impl(param_name , parameter_description)));
					}
					else
					{
						this->params.push_back(
							(object_parameter_impl *)(new parameter_boolean_impl(param_name , parameter_description)));
					}
				}
				else if(parameter_description.find("STRING") == 0)
				{
					this->params.push_back(
						(object_parameter_impl *)(new parameter_string_impl(param_name , parameter_description)));
				}
				else if(parameter_description.find("FLOAT") == 0)
				{
					this->params.push_back(
						(object_parameter_impl *)(new parameter_float_impl(param_name , parameter_description)));
				}
				else if(parameter_description.find("_TYPE") != string::npos)
				{
					this->params.push_back(
						(object_parameter_impl *)(new parameter_object_ref_impl(param_name , parameter_description)));
				}
				else
				{
					// ありえない
				}

				return 0;
			}

			oil_object_impl::~oil_object_impl()
			{
				std::vector<object_parameter_impl*>::iterator p;

				for(p = params.begin() ; p != params.end() ; p++)
				{
					delete (*p);
				}
			}

			bool oil_object_impl::validate_object_parameter
				(oil_definition::object_parameter_def *obj_param , bool *multiple ,
						std::vector<object_ref> *references)
			{
				std::vector<object_parameter_impl*>::iterator match;
				bool result = false;

				// 同じ名前を持つパラメータがあるか検索
				// ※find_ifだとダウンキャストに失敗した
				for(match = params.begin() ; match != params.end() ; match++)
				{
					if((*match)->validate_parameter_name(obj_param->get_parameter_name())
						== true)
					{
						break;
					}
				}
				
				// パラメータの属性チェック
				if(match != params.end())
				{
					parameter_object_ref_impl *ref_param;
					parameter_enum_impl *enum_param;

					ref_param = dynamic_cast<parameter_object_ref_impl*>(*match);
					if(ref_param != NULL)
					{
						if(ref_param->validate_object_parameter(obj_param , references) == true)
						{
							*multiple = (*match)->get_multiple();
							result = true;
						}
					}
					else 
					{
						// enumの場合、子要素で参照が出てくる可能性がある
						enum_param = dynamic_cast<parameter_enum_impl*>(*match);
						if(enum_param != NULL)
						{
							if(enum_param->validate_object_parameter(obj_param , references) == true)
							{
								*multiple = (*match)->get_multiple();
								result = true;
							}
							else
							{
								// エラー出力はvalidate_object_parameterで行っているため不要．
								//toppers::error("Object %1%'s parameter %2% can't set %3%." , 
								//	obj_param->get_parent_name() , obj_param->get_parameter_name() , obj_param->get_value());
							}
						}
						else
						{
							if((*match)->validate_object_parameter(obj_param) == true)
							{
								*multiple = (*match)->get_multiple();
								result = true;
							}
							else
							{
								// エラー出力はvalidate_object_parameterで行っているため不要．
								//toppers::error("Object %1%'s parameter %2% can't set %3%." , 
								//	obj_param->get_parent_name() , obj_param->get_parameter_name() , obj_param->get_value());
							}
						}
					}
				}
				else
				{
					// エラー処理
					toppers::error("Object %1% can't define parameter %2%." , 
						obj_param->get_parent_name() , obj_param->get_parameter_name());
				}

				return result;
			}

			// 基本パラメータのチェック
			bool object_parameter_impl::validate_object_parameter
					(oil_definition::object_parameter_def *obj , bool *auto_param)
			{
				// 複数定義はオブジェクト解析側でしか行えない
				// デフォルト定義かどうかもオブジェクトでしか行えない
				set_parameter_type(obj);
								
				// AUTO属性のチェック
				if(obj->get_value() == "AUTO")
				{
					if(assign_auto == false)
					{
						toppers::error("Object %1% : %2% can't use \"AUTO\"" ,
							obj->get_parent_name().c_str() , obj->get_parameter_name().c_str());
						return false;
					}
					else
					{
						*auto_param = true;
					}
				}
				else
				{
					*auto_param = false;
				}

				return true;
			}
			
			template<typename T>
			bool validate_number_paremeter
			(VALUE_LIMIT_TYPE type ,T value , T min , T max , vector<T> *value_list)
			{
				bool result = true;
				
				if(type == LIMIT_RANGE)
				{
					if((value < min) || (value > max))
					{
						result = false;
					}
				}
				else if(type == LIMIT_LIST)
				{
                    // modified by takuya
					//std::vector<T>::iterator p;
					typename std::vector<T>::iterator p;

					// 値のリストに存在するかを検索
					p = find(value_list->begin() , value_list->end() , value);
					if(p >= value_list->end())
					{
						result = false;
					}
				}
				else
				{
					// チェックは必要ない
				}

				return result;
			}

			template<typename T>
			void val2string(std::vector<T> *list , std::string *str)
			{
                // modified by takuya 110823
				//std::vector<T>::iterator p;
				typename std::vector<T>::iterator p;

				for(p = list->begin() ; p != list->end() ; p++)
				{
					try
					{
						*str += boost::lexical_cast<string>(*p) + " , ";
					}
					catch( ... )
					{
						toppers::error("cast error");
					}
				}
			}

			bool parameter_int_impl::validate_object_parameter
					(oil_definition::object_parameter_def *obj)
			{
				bool result = false , assign_auto;
				int64_t value;
				
				// 基本属性のチェック
				if(object_parameter_impl::validate_object_parameter(obj , &assign_auto)
					== false)
				{
					return false;
				}

				// AUTOパラメータならすぐに抜ける
				if(assign_auto == true)
				{
					return true;
				}

				// 値に変換
				if(obj->get_value().find("0x") != string::npos)
				{
                    // modified by takuya 110823
					//(void)sscanf_s(obj->get_value().c_str() , "0x%I64x" , &value);
					(void)sscanf(obj->get_value().c_str() , "0x%llx" , &value);
				}
				else if(obj->get_value().find("0X") != string::npos)
				{
                    // modified by takuya 110823
					//(void)sscanf_s(obj->get_value().c_str() , "0X%I64x" , &value);
					(void)sscanf(obj->get_value().c_str() , "0X%llx" , &value);
				}
				else
				{
					try
					{
						value = boost::lexical_cast<int64_t>(obj->get_value());
					}
					catch(std::exception& exception)
					{
						toppers::error("%1%'s %2% is not INT16/INT32." , 
							obj->get_parent_name() , obj->get_parameter_name());

						return false;
					}
				}

				if(validate_number_paremeter
							(limit_type , value , min , max , &value_list) == false)
				{
					if(limit_type == LIMIT_RANGE)
					{
						toppers::error("Object %1%'s %2% is \"%3%\" not in range [%4% - %5%]" ,
							obj->get_parent_name().c_str() , obj->get_parameter_name().c_str() ,
							obj->get_value() , min , max);
					}
					else // if(limit_type == LIMIT_LIST)
					{
						std::string str;
						val2string(&value_list , &str);
						
						toppers::error("Object %1%'s %2% is \"%3%\" not in range [%4% - %5%]" ,
							obj->get_parent_name().c_str() , obj->get_parameter_name().c_str() ,
							obj->get_value() , min , max);
					}
				}
				else
				{
					result = true;
				}

				return result;
			}

			void parameter_int_impl::set_parameter_type(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_INT);
			}

			bool parameter_uint_impl::validate_object_parameter
				(oil_definition::object_parameter_def *obj)
			{
				bool result = false , assign_auto;
				uint64_t value;
				
				// 基本属性のチェック
				if(object_parameter_impl::validate_object_parameter(obj , &assign_auto)
					== false)
				{
					return false;
				}

				// AUTOパラメータならすぐに抜ける
				if(assign_auto == true)
				{
					return true;
				}

				// 値に変換
				if(obj->get_value().find("0x") != string::npos)
				{
                    // modified by takuya 110823
					//(void)sscanf_s(obj->get_value().c_str() , "0x%I64x" , &value);
					(void)sscanf(obj->get_value().c_str() , "0x%llx" , &value);
				}
				else if(obj->get_value().find("0X") != string::npos)
				{
                    // modified by takuya 110823
					//(void)sscanf_s(obj->get_value().c_str() , "0X%I64x" , &value);
					(void)sscanf(obj->get_value().c_str() , "0X%llx" , &value);
				}
				else
				{
					try
					{
						value = boost::lexical_cast<uint64_t>(obj->get_value());
					}
					catch(std::exception& exception)
					{
						toppers::error("%1% : %2% is not UINT16/UINT32." , 
							obj->get_parent_name() , obj->get_parameter_name());
						return false;
					}
				}

				if(validate_number_paremeter
							(limit_type , value , min , max , &value_list) == false)
				{
					if(limit_type ==LIMIT_RANGE)
					{
						toppers::error("Object %1%'s %2% : %3% is not in range [%4% - %5%]" ,
							obj->get_parent_name().c_str() , obj->get_parameter_name().c_str() ,
							obj->get_value() , min , max);
					}
					else // if(limit_type == LIMIT_LIST)
					{
						std::string str;
						val2string(&value_list , &str);

						toppers::error("Object %1%'s %2% : %3% is not in range value list [%4%]" ,
							obj->get_parent_name().c_str() , obj->get_parameter_name().c_str() ,
							obj->get_value() , str);
					}
				}
				else
				{
					result = true;
				}

				return result;
			}

			void parameter_uint_impl::set_parameter_type(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_UINT);
			}

			bool parameter_float_impl::validate_object_parameter
				(oil_definition::object_parameter_def *obj)
			{
				bool result = false , assign_auto;
				double value;
				
				// 基本属性のチェック
				if(object_parameter_impl::validate_object_parameter(obj , &assign_auto)
					== false)
				{
					return false;
				}

				// AUTOパラメータならすぐに抜ける
				if(assign_auto == true)
				{
					return true;
				}

				// 値に変換
				try
				{
					value = boost::lexical_cast<double>(obj->get_value().c_str());
				}
				catch(std::exception& exception)
				{
					toppers::error("%1% : %2% is not FLOAT." , 
						obj->get_parent_name() , obj->get_parameter_name());

					return false;
				}

				if(validate_number_paremeter
							(limit_type , value , min , max , &value_list) == false)
				{
					if(limit_type ==LIMIT_RANGE)
					{
						toppers::error("Object %1%'s %2%  : %3% is not in range [%4% - %5%]" ,
							obj->get_parent_name().c_str() , obj->get_parameter_name().c_str() ,
							obj->get_value().c_str() , min , max);
					}
					else // if(limit_type == LIMIT_LIST)
					{
						std::string str;
						val2string(&value_list , &str);

						toppers::error("Object %1%' %2% : %3% is not in range value list [%4%]" ,
							obj->get_parent_name().c_str() , obj->get_parameter_name().c_str() ,
							obj->get_value().c_str() , str.c_str());
					}
				}
				else
				{
					result = true;
				}

				return result;
			}

			void parameter_float_impl::set_parameter_type(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_FLOAT);
			}

			bool parameter_enum_impl::validate_object_parameter
					(oil_definition::object_parameter_def *obj , std::vector<toppers::oil::object_ref> *object_refs)
			{
				std::map<std::string , oil_object_impl*>::iterator p;
				bool result = true , assign_auto;

				// 基本属性のチェック
				if(object_parameter_impl::validate_object_parameter(obj , &assign_auto)
					== false)
				{
					return false;
				}

				// AUTOパラメータならすぐに抜ける
				if(assign_auto == true)
				{
					return true;
				}

				// パラメータがENUMのリストにあるか検索(総当たり）
				std::vector<oil_definition::object_definition*>::iterator s;
				std::string str;
				bool hit = false;

				for(p = sub_params.begin() ; p != sub_params.end(); p++)
				{
					// エラー処理用のリスト
					str += (*p).first + " , ";
					// パラメータ名が一致していなければ先の処理は行わない
					if(obj->get_value() == (*p).first)
					{
						hit = true;
					}
				}
				// リストになかった場合の処理
				if(hit == false)
				{
					toppers::error("Object %1%'s %2% : %3% is not in range [%4%]" ,
						obj->get_parent_name().c_str() , obj->get_parameter_name().c_str() , 
						obj->get_value().c_str() , str);

					return false;
				}
				
				// サブパラメータについてもチェック
				for(s = obj->get_subparams()->begin() ; s != obj->get_subparams()->end() ; s++)
				{
					for(p = sub_params.begin() ; p != sub_params.end(); p++)
					{
						/* 同名の設定の定義部を見つけたら再帰で検査 */
						if((*s)->get_parent()->get_value() == (*p).first)
						{
							if((*p).second->validate_object_configuration((*s) , object_refs) == true)
							{
								result = true;
							}
						}
					}
					if(result == true)
					{
						break;
					}
				}
				// 見つからなければエラー
				if((obj->get_subparams()->empty() == false)
					&& (s == obj->get_subparams()->end()))
				{
					result = false;
				}

				return result;
			}

			bool parameter_object_ref_impl::validate_object_parameter
					(oil_definition::object_parameter_def *obj , std::vector<toppers::oil::object_ref> *object_refs)
			{
				// 基本属性のチェック
				// リファレンス先のチェックもここで行うべきだが、
				// リファレンス先のオブジェクト種類を取得できないため後で行う
				if(object_parameter_impl::validate_object_parameter(obj , &assign_auto)
					== false)
				{
					return false;
				}

				// チェックするべきリファレンスの代入
				object_ref ref = { reftype_name , obj->get_value() };
				object_refs->push_back(ref);

				return true;
			}

			void parameter_string_impl::set_parameter_type
					(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_STRING);
			}

			void parameter_boolean_impl::set_parameter_type
					(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_BOOLEAN);
			}
		
			void parameter_enum_impl::set_parameter_type(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_ENUM);
			}

			void parameter_object_ref_impl::set_parameter_type(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_REF);
			}

			bool oil_object_impl::validate_object_configuration
					(oil_definition::object_definition * obj_def , std::vector<object_ref> *references)
			{
				std::vector<oil_definition::object_parameter_def*>::iterator p;
				std::vector<oil_definition::object_parameter_def*> *obj_params;
				std::vector<object_parameter_impl*>::iterator q;
				std::vector<string> defined_parameter;
				std::vector<object_parameter_impl*> undefined_parameter;
				std::vector<string>::iterator found , s;
				bool result = true;
				bool multiple;

				// オブジェクト名が合っているかチェック
				if(name != obj_def->get_object_type())
				{
					return false;
				}

				// パラメータの整合性チェック
				obj_params = obj_def->get_params();
				for(p = obj_params->begin() ; p != obj_params->end() ; p++)
				{
					if(validate_object_parameter(*p , &multiple , references) == false)
					{
						result = false;
					}

					// 複数定義可能かのチェック
					if(((found = find(defined_parameter.begin() , defined_parameter.end() , 
						(*p)->get_parameter_name())) != defined_parameter.end())
							&& (multiple == false))
					{
						toppers::error("Object %1% : %2% can define once." ,
							obj_def->get_name().c_str() , (*p)->get_parameter_name().c_str());

						result = false;
					}
					else
					{
						defined_parameter.push_back((*p)->get_parameter_name());
					}
				}

				if(result == true)
				{
					//
					// デフォルトパラメータの補完
					//

					// オブジェクト定義に記載されていないパラメータの探索
					for(q = params.begin() ; q != params.end() ; q++)
					{
						s = find(defined_parameter.begin() , 
							defined_parameter.end() , (*q)->get_name());

						if(s == defined_parameter.end())
						{
							undefined_parameter.push_back((*q));
						}
					}

					// 探索したパラメータに対してのデフォルトパラメータの補完
					for(q = undefined_parameter.begin() ; q != undefined_parameter.end() ; q++)
					{
						string default_value;
						DEFAULT_TYPE deftype = (*q)->get_default(&default_value);

						if((deftype == HAVE_DEFAULT)
							|| (deftype == AUTO))
						{
							obj_def->add_parameter((*q)->get_name() , default_value);
						}
						else if(deftype == NO_DEFAULT)
						{
							toppers::error("Object %1% : Parmeter %2% is not defined. %2% doesn't have default value." ,
								obj_def->get_name().c_str() , (*q)->get_name().c_str());

							result = false;
						}
						else	/* deftype == HAVE_NOT_DEFINE */
						{
						}

					}
				}

				return result;
			}

			bool parameter_string_impl::validate_object_parameter
					(oil_definition::object_parameter_def *obj)
			{
				// 文字属性は特に制約がないため常にtrue
				return true;
			}
			
			bool parameter_boolean_impl::validate_object_parameter
					(oil_definition::object_parameter_def *obj)
			{
				// TRUEかFALSEでないとならない
				if((obj->get_value() == "TRUE")
					|| (obj->get_value() == "FALSE"))
				{
					return true;
				}

				toppers::error("Object %1%'s %2% is \"%3%\" , but can not set [TRUE , FALSE , ]." ,
					obj->get_parent_name() , obj->get_parameter_name() , obj->get_value());

				return false;
			}

			void oil_object_impl::display_implementation(void)
			{
				std::vector<object_parameter_impl*>::iterator p;
				
				cout << "=========== Object Name : " << name << "===========" << endl << endl;
				for(p = params.begin() ; p != params.end() ; p++)
				{
					(*p)->display_implementation();
				}
				cout << endl << "==================================" << endl;
				
			}

			void object_parameter_impl::set_parameter_type
					(oil_definition::object_parameter_def *obj)
			{
				obj->set_type(TYPE_UNKNOWN);
			}
		}

		namespace oil_definition
		{
			object_definition* find_object(string name)
			{
				return NULL;
			}
			
			int object_definition::add_parameter(string param_name , string param_value)
			{
				try
				{
					object_parameter_def* param_def =
						new object_parameter_def(param_name , param_value , name);
					params.push_back(param_def);
				}
				catch(...)
				{
					// エラー処理
				}

				return 0;
			}

			object_definition::~object_definition()
			{
				std::vector<object_parameter_def*>::iterator p;

				for(p = params.begin() ; p != params.end() ; p++)
				{
					delete (*p);
				}
			}

			void object_definition::display_definition(void)
			{
				std::vector<object_parameter_def*>::iterator p;
				
				for(p = params.begin() ; p != params.end() ; p++)
				{
					(*p)->display_definition();
				}
				cout << endl << endl;
			}

			
		/*!
		 *  \brief  オブジェクトID番号の割付け
		 *  \param[in]  api_map   ソースに記述された静的APIを登録したコンテナ
		 */
		void assign_id( cfg_obj_map obj_def_map , long fixed_id)
		{
		  using namespace toppers;
		  using namespace toppers::oil::oil_definition;

          // modified by takuya 110823
		  //std::map<std::string, std::vector< object_definition* >>::iterator p;
		  std::map<std::string, std::vector< object_definition* > >::iterator p;
		  std::vector<object_definition*>::iterator q;
		  int serial;

		  for(p = obj_def_map.begin() ; p != obj_def_map.end() ; p++)
		  {
			  serial = 0;
			  // オブジェクトごとにID割当て
			  for(q = (*p).second.begin() ; q != (*p).second.end() ; q++)
			  {
				  if(fixed_id == UINT_MAX)
				  {
					(*q)->set_id(serial++);
				  }
				  else
				  {
					(*q)->set_id(fixed_id);
				  }
			  }
		  }

		}

		cfg_obj_map merge(std::vector<object_definition*> *obj_defs , 
				cfg_obj_map& out , string prefix , string suffix , long fixed_id)
		{
		  std::vector<object_definition*>::iterator p , q;
		  std::vector<object_definition*> *sub_objects;
		  std::vector<object_parameter_def*>::iterator r;
		  std::vector<object_parameter_def*> *subparams;   
		  
		  for(p = obj_defs->begin() ; p != obj_defs->end() ; p++)
		  {
			  string s(prefix + (*p)->get_object_type() + suffix);
			  out[ s ].push_back(*p);
		  }

		  // IDを一度割付ける
		  assign_id(out , fixed_id);

		  for(p = obj_defs->begin() ; p != obj_defs->end() ; p++)
		  {
			  long id;
			  subparams = (*p)->get_params();
			  // サブパラメータも登録
			  for(r = subparams->begin() ; r != subparams->end() ; r++)
			  {
				  sub_objects = (*r)->get_subparams();
	   			  id = (*p)->get_id();

				  string s = "." + (*r)->get_value();

				  // サブパラメータの持つ属性は再帰で登録する
				  if(sub_objects->empty() == false)
				  {
					  cfg_obj_map temp;
					  std::string subparam_name = 
						  (*p)->get_object_type() + ".";
					  temp = merge(sub_objects , temp , subparam_name , s , id);

					  // 結果をコピー(ちょうどいいメソッドが無い)
                      // modified by takuya 110823
					  // std::map< std::string, std::vector< object_definition* >>::iterator x;
					  std::map< std::string, std::vector< object_definition* > >::iterator x;
					  for(x = temp.begin() ; x != temp.end() ; x++ )
					  {
						  std::vector< object_definition* >::iterator y;
						  for(y = (*x).second.begin() ; y != (*x).second.end() ; y++)
						  {
							  out[(*x).first].push_back((*y));
						  }
						  //out[(*x).first] = (*x).second;
						  //out[(*x).first].push_back((*x).second);
					  }
				  }
			  }
		  }

		  return out;
		}
		}
	}	/* oil */

}
