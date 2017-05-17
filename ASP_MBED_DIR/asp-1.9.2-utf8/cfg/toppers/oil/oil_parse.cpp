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

#include <string>
#include <iostream>

#include "toppers/oil/oil_parser.hpp"
#include "toppers/diagnostics.hpp"

using namespace std;
using namespace boost::spirit::classic;
using namespace toppers::oil;
using namespace toppers::oil::oil_implementation;
using namespace toppers::oil::oil_definition;


int oil_parser::do_parse(oil_impl **impl , oil_def **def)
{
	oil_grammer grammer(this);
	parse_info<> info;
	
	info = parse(description->c_str() , grammer , space_p);
	if(info.hit)
	{
		*impl = &impl_list;
		*def = &def_list;
		return 0;
	}
	else
	{
		return -1;
	}
}

void eval_tree(tree_match<char const *>::tree_iterator const &iter, int nest) {
	int size; // ノードの子の個数
	
	size = iter->children.size(); // ノードの子の個数を取り出す
	for(int i=0; i<nest; i++)
	{
		cout << "  "; // スペース２個分インデント
	}
	cout << "Node = '" << string(iter->value.begin(), iter->value.end()) << "'";
	if (size > 0)
	{
		cout << " Child Size = " << size << endl; // 子の個数を表示
	} else
	{
		cout << endl;                             // 子が無ければ個数表示しない
	}
	for(int j=0; j<size; j++)
	{
		eval_tree(iter->children.begin()+j, nest+1); // 子の個数分再帰呼び出し
	}
}

int oil_parser::start_objimpl_analysis(std::string object_name)
{
	int result = 0;

	try
	{
		// 構成中のオブジェクト実装部を保存
		current_impl = new oil_object_impl(object_name);
	}
	catch( ... )
	{
		result = -1;
	}
	
	return result;
}

int oil_parser::set_nextparam_name(std::string name)
{
	if(param_addable == true)
	{
		next_param_name = name;
	}

	return 0;
}

int oil_parser::add_parameter_objimpl(std::string parameter)
{
	// パラメータ解析、追加
	if(param_addable)
	{
		current_impl->add_parameter(next_param_name , parameter);
	}

	return 0;
}

int oil_parser::start_objdef_analysis(std::string object_name_type)
{
	int result = 0;

	try
	{
		// 構成中のオブジェクト実装部を保存
		current_def = new object_definition(object_name_type);
	}
	catch( ... )
	{
		result = -1;
	}
	return result;
}

int oil_parser::end_objimpl_analysis(void)
{
	// 構成完了、登録
	impl_list.push_back(current_impl);
	
	return 0;
}

void oil_parser::enable_addparam(bool enabled)
{
	param_addable = enabled;
}

int oil_parser::end_objdef_analysis(void)
{
	static std::vector<std::string> defined_object_name;
	std::string obj_name = current_def->get_name();

	// 既に同じオブジェクトがないかチェック
	if(find(defined_object_name.begin() , defined_object_name.end() , obj_name)
			== defined_object_name.end())
	{
		// 構成完了、登録
		def_list.push_back(current_def);
		// オブジェクト名を追加，登録済みにする
		defined_object_name.push_back(obj_name);
	}
	else
	{
		// 同名のオブジェクト名が存在するため，エラー
		toppers::error("Object %1% redefined." , obj_name);
	}
	
	return 0;
}

int oil_parser::add_attr_value(string str)
{
	attr_description = str;

	if(param_addable == true)
	{
		current_def->add_parameter(next_param_name , attr_description);
	}

	return 0;
}

void oil_parser::set_error(int position , std::string message)
{
	error_position = position;
	error_message = message;
}

void oil_parser::get_error(int *position , std::string *message)
{
	*position = error_position;
	*message = error_message;
}

void oil_parser::dump_implementation(void)
{
	std::vector<oil_object_impl*>::iterator p;

	for(p = impl_list.begin(); p != impl_list.end() ; p++)
	{
		(*p)->display_implementation();
	}
}

void oil_parser::dump_definition(void)
{
	std::vector<object_definition*>::iterator p;
	
	cout << "************** Object Defition **************" << endl;
	for(p = def_list.begin(); p != def_list.end() ; p++)
	{
		(*p)->display_definition();
	}
	cout << "************** Object Defition End **************" << endl;
}
