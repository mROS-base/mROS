/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2009 by TAKAGI Nobuhisa
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
 *  \file   toppers/macro_processor.hpp
 *  \brief  マクロ処理のための宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class macro_processor;
 *  \endcode
 */
#ifndef TOPPERS_MACRO_PROCESSOR_HPP_
#define TOPPERS_MACRO_PROCESSOR_HPP_

#include <string>
#include <vector>
#include <stack>
#include <map>
#include "toppers/workaround.hpp"
#include "toppers/output_file.hpp"
#include "toppers/text.hpp"

namespace toppers
{

  class macro_processor
  {
  public:
    struct context;

    /*!
     *  \struct element macro_processor.hpp "toppers/macro_processor.hpp"
     *  \brief  マクロ中で使用する変数の値
     *
     *  正確には、変数の値は常に順序付きリストとして扱われます。element は順序付きリストの 1 要素を表します。
     */
    struct element
    {
      boost::optional< std::tr1::int64_t > i;
      std::string v;
      std::string s;
    };
    /*!
     *  \brief  マクロ中で使用する変数を表現する型
     */
    typedef std::vector< element > var_t;

    /*!
     *  \struct func_t macro_processor.hpp "toppers/macro_processor.hpp"
     *  \brief  マクロ内で扱う関数を表現する型
     *  \note   元々単純な構造体であったため、互換性のため、集成体と特性を維持させている。
     *  \attention  node の初期化忘れに要注意
     */
    struct func_t
    {
      std::string name;   //!< 関数名
      var_t ( * f )( text_line const& line, std::vector< var_t > const&, context* );  //!< 処理内容
      void const* node;
    };

    struct context
    {
      std::stack< var_t > stack;
      std::map< std::string, var_t > var_map;
      std::map< std::string, func_t > func_map;
      bool in_function;
      output_file target_file;
      text_line line;
      context() : in_function( false ) {}
    };

    struct die_terminate {};

    macro_processor();
    macro_processor( macro_processor const& other );
    explicit macro_processor( text const& in );
    ~macro_processor();
    macro_processor& operator=( macro_processor const& other );
    void swap( macro_processor& other );
    void evaluate( text const& in );
    void set_var( std::string const& name, var_t const& value );
    void set_var( std::string const& name, long name2, var_t const& value );
    var_t const& get_var( std::string const& name ) const;
    var_t const& get_var( std::string const& name, long name2 ) const;
    void add_builtin_function( func_t const& f );

    static void remove_comment( text const& in, text& out );
    static void expand_include( text const& in, text& out );
    static void preprocess( text const& in, text& out );
    static std::tr1::int64_t to_integer( var_t const& var, context const* p_ctx );
    static std::string to_string( var_t const& var, context const* p_ctx );
    static bool check_arity( text_line const& line, std::size_t arity, std::size_t valid, char const* function_name );
    static var_t call_user_function( text_line const& line, std::vector< var_t > const& arg_list, context* p_ctx );
  private:
    context* p_ctx_;
    static func_t const builtin_function_table[];
  };

  inline bool operator == ( macro_processor::element const& lhs, macro_processor::element const& rhs )
  {
    return lhs.i == rhs.i;
  }

  inline bool operator < ( macro_processor::element const& lhs, macro_processor::element const& rhs )
  {
    return lhs.i < rhs.i;
  }

  struct element_eq
  {
    explicit element_eq( std::tr1::int64_t value ) : value_( value ) {}
    bool operator()( toppers::macro_processor::element const& e ) const
    {
      return e.i && ( e.i.get() == value_ );
    }
    std::tr1::int64_t value_;
  };

}

#endif  // ! TOPPERS_MACRO_PROCESSOR_HPP_
