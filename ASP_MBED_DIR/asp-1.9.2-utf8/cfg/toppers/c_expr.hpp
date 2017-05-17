/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2005-2012 by TAKAGI Nobuhisa
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
 *  \file   toppers/c_expr.hpp
 *  \brief  C言語の式の構文解析に関する宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  struct c_expr_parser_base< Derived >;
 *  struct c_expr_parser;
 *  struct c_const_expr_parser;
 *  \endcode
 */
#ifndef TOPPERS_C_EXPR_HPP_
#define TOPPERS_C_EXPR_HPP_

#include "toppers/c_parser.hpp"
#include "toppers/workaround.hpp"

namespace toppers
{

  /*!
   *  \struct c_expr_parser_base c_expr.hpp "toppers/c_expr.hpp"
   *  \brief  C言語の式を構文解析するための基底クラス
   *
   *  実際に使用する際は、 c_expr_parser_base および c_expr_parser_base::definition
   *  クラスを派生する必要があります。 c_expr_parser_base::definition の派生クラスでは、
   *  start メンバ関数を定義して、構文中の必要なルールを取り出すようにしてください。
   *  こうすることで、定数式の文法、一次式の文法といったように、C言語の文法のサブ
   *  セットを容易に作り出すことができます。
   *
   *  \code
   *  // 一次式を取り出す例
   *  struct c_primary_expression : c_expr_parser_base< c_primary_expression >
   *  {
   *    template < class Scanner >
   *    struct definition : c_expr_parser_base< c_primary_expression >::definition
   *    {
   *      rule_t const& start() const { return primary_expression; }
   *    };
   *  };
   *  \endcode
   */
  template < class Derived >
    struct c_expr_parser_base : boost::spirit::classic::grammar< Derived >
  {
  public:
    /*!
     *  \struct definition c_expr.hpp "toppers/c_expr.hpp"
     *  \brief  文法定義
     */
    template < class Scanner >
      struct definition
    {
      typedef boost::spirit::classic::rule< Scanner > rule_t;
      rule_t  primary_expression,   
              expression,
              constant_expression,
              conditional_expression,
              assignment_expression,
              assignment_operator,
              postfix_expression,
              unary_expression,
              unary_operator,
              cast_expression,
              multiplicative_expression,
              additive_expression,
              shift_expression,
              relational_expression,
              equality_expression,
              AND_expression,
              exclusive_OR_expression,
              inclusive_OR_expression,
              logical_AND_expression,
              logical_OR_expression,
              string_literal,
              constant,
              floating_constant,
              decimal_floating_constant,
              hexadecimal_floating_constant,
              integer_constant,
              character_constant,
              declaration_specifiers,
              type_name,
              specifier_qualifier_list,
              storage_class_specifier,
              type_specifier,
              type_qualifier,
              declarator,
              direct_declarator,
              struct_or_union_specifier,
              struct_declaration,
              struct_declarator,
              enum_specifier,
              enumerator,
              abstract_declarator,
              pointer,
              parameter_type_list,
              parameter_list,
              parameter_declaration,
              direct_abstract_declarator;

      c_ident_parser_t identifier;
      c_strlit_parser_t c_strlit_p;
      c_chlit_parser_t c_chlit_p;

      /*!
       *  \brief  コンストラクタ
       *  \param  self  文法クラス（ c_expr_parser_base< Derived > クラスからの継承）への参照
       */
      definition( Derived const& self )
        : identifier( c_ident_parser( self.ucn_, self.c_plus_plus_ ) ),
          c_strlit_p( c_strlit_parser( self.codeset_ ) ),
          c_chlit_p( c_chlit_parser( self.codeset_ ) )
      {
        using namespace boost::spirit::classic;
        static functor_parser< detail::c_integer_constant_parse_functor< boost::uintmax_t > > const c_int_const_p;
        static functor_parser< detail::c_integer_suffix_parse_functor > const c_int_suffix_p;

        primary_expression =  // 複合リテラル未対応
            identifier
          | constant
          | string_literal
          | ( '(' >> expression >> ')' );
        expression =
            assignment_expression % ',';
        constant_expression = 
            conditional_expression;
        conditional_expression =
            logical_OR_expression >> *( '\?' >> expression >> ':' >> logical_OR_expression );
        assignment_expression =
           *( unary_expression >> assignment_operator ) >> conditional_expression;
        assignment_operator =
            ch_p( '=' ) | "*=" | "/=" | "%=" | "+=" | "?=" | "<<=" | ">>=" | "&=" | "^=" | "|=";
        postfix_expression =
            primary_expression >>
          *(
                ( '[' >> expression >> ']' )
              | ( '(' >> list_p( assignment_expression, ',' ) >> ')' )
              | ( '.' >> identifier )
              | ( "->" >> identifier )
              | "++"
              | "--"
            );
        unary_expression =
           *( str_p( "++" ) || "--" ) >>
            (
                ( "sizeof" >> unary_expression )
              | ( str_p( "sizeof" ) >> '(' >> type_name >> ')' )
              | postfix_expression
              | ( unary_operator >> cast_expression )
            );
        unary_operator =
            //chset<>( "&*~!+-" );
		    ch_p( '&' ) | '*' | '~' | '!' | '+' | '-';
        cast_expression =
            *( '(' >> type_name >> ')' ) >> unary_expression
          | +( '(' >> ( type_name | identifier ) >> ')' );  // 構文解析に失敗する不具合対策
        multiplicative_expression =
            cast_expression >>
           *(
                ( '*' >> cast_expression )
              | ( '/' >> cast_expression )
              | ( '%' >> cast_expression )
            );
        additive_expression =
            multiplicative_expression >>
           *(
                ( '+' >> multiplicative_expression )
              | ( '-' >> multiplicative_expression )
            );
        shift_expression =
            additive_expression >>
           *(
                ( "<<" >> additive_expression )
              | ( ">>" >> additive_expression )
            );
        relational_expression =
            shift_expression >>
           *(
                ( '<' >> shift_expression )
              | ( '>' >> shift_expression )
              | ( "<=" >> shift_expression )
              | ( ">=" >> shift_expression )
            );
        equality_expression =
            relational_expression >>
           *(
                ( "==" >> relational_expression )
              | ( "!=" >> relational_expression )
            );
        AND_expression =
            equality_expression >> *( '&' >> equality_expression );
        exclusive_OR_expression =
            AND_expression >> *( '^' >> AND_expression );
        inclusive_OR_expression =
            exclusive_OR_expression >> *( '|' >> exclusive_OR_expression );
        logical_AND_expression =
            inclusive_OR_expression >> *( "&&" >> inclusive_OR_expression );
        logical_OR_expression =
            logical_AND_expression >> *( "||" >> logical_AND_expression );
        string_literal =
            c_strlit_p
          | lexeme_d[ 'L' >> c_strlit_p ];
        constant =
            floating_constant
          | integer_constant
          | identifier  // 列挙定数
          | character_constant;
        floating_constant =
            decimal_floating_constant
          | hexadecimal_floating_constant;
        decimal_floating_constant =
            lexeme_d
            [
              as_lower_d
              [
                ( ( *digit_p >> '.' >> +digit_p ) | ( +digit_p >> '.' ) ) >>
//                'e' >> !chset<>( "+-" ) >> +digit_p >>
				'e' >> !( ch_p( '+' ) | '-' ) >> +digit_p >>
//                !chset<>( "fl" )
			    ( ch_p( 'f' ) | 'l' )
              ]
            ];
        hexadecimal_floating_constant =
            lexeme_d
            [
              as_lower_d
              [
                "0x" >>
                ( ( *xdigit_p >> '.' >> +xdigit_p ) | ( +xdigit_p >> '.' ) ) >>
//                'p' >> !chset<>( "+-" ) >> +digit_p >>
				'p' >> !( ch_p( '+' ) | '-' ) >> +digit_p >>
//                !chset<>( "fl" )
			    ( ch_p( 'f' ) | 'l' )
              ]
            ];
        integer_constant =
            lexeme_d[ c_int_const_p >> !c_int_suffix_p ];
        character_constant =
            c_chlit_p
          | lexeme_d[ 'L' >> c_chlit_p ];
        declaration_specifiers =
           +( storage_class_specifier | type_specifier | type_qualifier );
        type_name =
            specifier_qualifier_list >> !abstract_declarator;
        specifier_qualifier_list =
           +( type_specifier | type_qualifier );
        storage_class_specifier =
            str_p( "auto" )
          | "register"
          | "static"
          | "extern"
          | "typedef";
        type_specifier =
            str_p( "void" ) | "char" | "short" | "int" | "long" | "float" | "double"
          | "signed" | "unsigned"
          | identifier
          | struct_or_union_specifier
          | enum_specifier;
        type_qualifier =
            str_p( "const" ) | "volatile" | "restrict";
        declarator =
            !pointer >> direct_declarator;
        direct_declarator = 
            ( identifier | ( '(' >> declarator >> ')' ) )
            >>
          *(
                ( '[' >> !constant_expression >> ']' )
              | ( '(' >> parameter_type_list >> ')' )
              | ( '(' >> !( identifier % ',' ) >> ')' )
            );
        struct_or_union_specifier =
            lexeme_d[ ( str_p( "struct" ) | "union" ) >> +space_p >> identifier ]
          | ( lexeme_d[ ( str_p( "struct" ) | "union" ) >> +space_p >> !identifier ] >> '{' >> +struct_declaration >> '}' );
        struct_declaration =
            specifier_qualifier_list >> !list_p( struct_declarator, ',' ) >> ';';
            // lisp_p( struct_declarator, ',' )を省略可能としているのは、
            // struct_declarator の identifier を specifier_qualifier_list が
            // typedef 名と間違うことを回避するため
        struct_declarator =
            ( !declarator >> ':' >> constant_expression ) // ビットフィールド
          | declarator;
        enum_specifier =
            ( lexeme_d[ "enum" >> +space_p >> !identifier ] >> '{' >> list_p( enumerator, ',', ',' ) >> '}' )   // C99では末尾のカンマがあってもよい
          | lexeme_d[ "enum" >> +space_p >> identifier ];
        enumerator =
            identifier >> !( '=' >> constant_expression );
        abstract_declarator =
            ( !pointer >> direct_abstract_declarator )
          | pointer;
        pointer =
           +( '*' >> *type_qualifier );
        parameter_type_list =
            parameter_list >> !( ch_p( ',' ) >> "..." );  // 可変個引数
        parameter_list =
            parameter_declaration % ',';
        parameter_declaration = 
            ( declaration_specifiers >> declarator )
          | ( declaration_specifiers >> !abstract_declarator );
        direct_abstract_declarator =
            (
             !( '(' >> abstract_declarator >> ')' ) >>
             +(
                  ( '[' >> !constant_expression >> ']' )
                | ( '(' >> !parameter_type_list >> ')' ) 
              )
            )
          | ( '(' >> abstract_declarator >> ')' );
      }
    };
    bool ucn_;
    codeset_t codeset_;
    bool c_plus_plus_;

    /*!
     *  \brief  コンストラクタ
     *  \param  ucn         国際文字名に対応する場合は true を指定する
     *  \param  codeset     文字コード
     *  \param  c_plus_plus C++ に対応する場合は true を指定する
     */
    explicit c_expr_parser_base( bool ucn = false, codeset_t codeset = ascii, bool c_plus_plus = false )
      : ucn_( ucn ), codeset_( codeset ), c_plus_plus_( c_plus_plus )
    {
    }
  };

  /*!
   *  \class  c_expr_parser c_expr.hpp "toppers/c_expr.hpp" 
   *  \brief  C言語の式の構文解析クラス
   */
  struct c_expr_parser : c_expr_parser_base< c_expr_parser >
  {
    typedef c_expr_parser_base< c_expr_parser > base_t;

    /*!
     *  \struct definition c_expr.hpp "toppers/c_expr.hpp"
     *  \brief  文法定義
     */
    template < class Scanner >
    struct definition : base_t::definition< Scanner >
    {
      typedef typename base_t::definition< Scanner >::rule_t rule_t;

      definition( c_expr_parser const& self ) : base_t::definition< Scanner >( self ) {}
      rule_t const& start() const { return base_t::definition< Scanner >::expression; };
    };

    /*!
     *  \brief  コンストラクタ
     *  \param  ucn     国際文字名に対応する場合は true を指定する
     *  \param  codeset 文字コード
     */
    explicit c_expr_parser( bool ucn = false, codeset_t codeset = ascii )
      : c_expr_parser_base< c_expr_parser >( ucn, codeset )
    {
    }
  };

  /*!
   *  \class  c_const_expr_parser c_expr.hpp "toppers/c_expr.hpp"
   *  \brief  C言語の定数式の構文解析クラス
   */
  struct c_const_expr_parser : c_expr_parser_base< c_const_expr_parser >
  {
    typedef c_expr_parser_base< c_const_expr_parser > base_t;

    /*!
     *  \struct definition c_expr.hpp "toppers/c_expr.hpp"
     *  \brief  文法定義
     */
    template < class Scanner >
    struct definition : base_t::definition< Scanner >
    {
      typedef typename base_t::definition< Scanner >::rule_t rule_t;

      definition( c_const_expr_parser const& self ) : base_t::definition< Scanner >( self ) {}
      rule_t const& start() const { return base_t::definition< Scanner >::constant_expression; };
    };

    /*!
     *  \brief  コンストラクタ
     *  \param  ucn     国際文字名に対応する場合は true を指定する
     *  \param  codeset 文字コード
     */
    explicit c_const_expr_parser( bool ucn = false, codeset_t codeset = ascii )
      : c_expr_parser_base< c_const_expr_parser >( ucn, codeset )
    {
    }
  };

}

#endif  // ! TOPPERS_C_EXPR_HPP_
