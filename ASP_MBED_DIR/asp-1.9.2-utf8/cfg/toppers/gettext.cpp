/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2011 by TAKAGI Nobuhisa
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
#include <stdlib.h>	// Cygwin対策
#include <cstring>
#include <clocale>
#include <string>
#include <map>
#include <fstream>
#include <stdexcept>
#include "toppers/gettext.hpp"
#include "toppers/cpp.hpp"
#include "toppers/global.hpp"
#include <boost/scoped_array.hpp>
#include <boost/any.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

namespace toppers
{
  namespace
  {

    std::map< std::string, std::string > msgcat;

    void register_msgcat( std::string const& msgid, std::string const& msgstr )
    {
      std::string::size_type size = msgstr.size();
      boost::scoped_array< wchar_t > wbuf( new wchar_t[ size + 1 ] );
      boost::scoped_array< char > buf( new char[ size + 1 ] );
      wchar_t* wcs = wbuf.get();
      wchar_t wc = 0;
      for ( std::string::const_iterator iter( msgstr.begin() ), last( msgstr.end() ); iter != last; ++iter )
      {
        int c = static_cast< unsigned char >( *iter );
        if ( ( ( c & 0xc0 ) == 0xc0 ) || ( c < 0x80 ) ) // 先行バイト
        {
          if ( wc != 0 )
          {
            *wcs++ = wc;
            wc = 0;
          }
          if ( ( c & 0x80 ) == 0 )
          {
            wc = static_cast< wchar_t >( c );
          }
          else if ( ( c & 0xe0 ) == 0xc0 )
          {
            wc = static_cast< wchar_t >( c & 0x1f );
          }
          else if ( ( c & 0xf0 ) == 0xe0 )
          {
            wc = static_cast< wchar_t >( c & 0xf );
          }
          else
          {
            // サロゲートは未対応（ここで対応したとしても、文字コード変換時にしくじる可能性大）
          }
        }
        else    // 後続バイト
        {
          wc = static_cast< wchar_t >( ( wc << 6 ) | ( c & 0x3f ) );
        }
      }
      if ( wc != 0 )
      {
        *wcs++ = wc;
      }
      *wcs = L'\0';

      // ↓ この間は決して例外が発生しない
      char const* locale = std::setlocale( LC_CTYPE, "" );
      /* std:: */wcstombs( buf.get(), wbuf.get(), size + 1 );     // Unicode から環境依存の文字コードへ変換
      std::setlocale( LC_CTYPE, locale );
      // ↑ この間は決して例外が発生しない

      msgcat[ msgid ] = std::string( buf.get() );
    }

    bool msgcat_loaded = false;

  }

  /*!
   *  \brief      メッセージカタログのロード
   *  \param[in]  dir     *.po ファイルが存在するディレクトリ
   *  \retval     true    成功
   *  \retval     false   失敗
   *
   *  実装を簡便化するため、.poファイルの記述方法には以下の制約がある。
   *  - msgid, msgstr は必ず行の先頭に記述する。
   *  - msgid, msgstr の直後には、必ず空白類文字一文字とし、その直後に文字列を記述する。
   *  - 文字列のみを記述する行は必ず " で始める。
   *  - .poファイルはの文字コードは必ず UTF-8N とする。
   */
  bool load_msgcat( std::string const& dir )
  {
    namespace fs = boost::filesystem;
    char const* env = std::getenv( "TOPPERS_CFG_LANG" );
//      env = "ja";
    if ( env == 0 )
    {
      return false;
    }
    std::string lang( env );

//    fs::path cfg_dir( dir, fs::native );
    fs::path cfg_dir( dir );  // filesystem3対応
//    fs::path po_file( cfg_dir/fs::path( lang + ".po", fs::native ) );
    fs::path po_file( cfg_dir/fs::path( lang + ".po" ) );  // filesystem3対応

//    std::ifstream ifs( po_file.native_file_string().c_str() );
    std::ifstream ifs( po_file.string().c_str() );  // filesystem3対応
    std::string msgid;
    std::string msgstr;

    while ( ifs )
    {
      std::string str;
      std::getline( ifs, str );

      // 改行文字の違いを吸収
      std::string::size_type pos = str.find_last_not_of( " \t\r\n" ); // ついでに行末の空白類も除去
      if ( pos != std::string::npos && pos < str.size() - 1 )
      {
        char c = str[pos];
        str.erase( pos + 1, std::string::npos );
      }

      if ( str.empty() || str[ 0 ] == '#' || str == "" )
      {
        ;   // 空行またはコメント行
      }
      else
      {
        try
        {
          if ( std::strncmp( str.c_str(), "msgid", sizeof( "msgid" )-1 ) == 0 )
          {
            str.erase( 0, sizeof( "msgid" )-1+1 );
            msgid = expand_quote( str );           
          }
          else if ( std::strncmp( str.c_str(), "msgstr", sizeof( "msgstr" )-1 ) == 0 )
          {
            str.erase( 0, sizeof( "msgstr" )-1+1 );
            msgstr = expand_quote( str );
          }
          else
          {
            msgstr += expand_quote( str );
          }
          if ( !msgid.empty() && !msgstr.empty() )  // 直前の msgid / msgstr を登録
          {
            register_msgcat( msgid, msgstr );
            msgid.clear();
            msgstr.clear();
          }
        }
        catch ( std::invalid_argument& )
        {
          return false;
        }
      }
    }
    msgcat_loaded = true;
    return true;
  }

  /*!
   *  \brief      メッセージの翻訳
   *  \param[in]  message   メッセージID
   *  \return     翻訳後のメッセージ
   */
  std::string const& gettext( std::string const& message )
  {
    static bool f = load_msgcat( get_global< std::string >( "cfg-directory" ) );
    if ( !msgcat_loaded )
    {
      return message;
    }

    std::map< std::string, std::string >::const_iterator iter( msgcat.find( message ) ), last( msgcat.end() );
    if ( iter != last )
    {
      std::string const& result( iter->second );
      return result;
    }
    return message;
  }

}
