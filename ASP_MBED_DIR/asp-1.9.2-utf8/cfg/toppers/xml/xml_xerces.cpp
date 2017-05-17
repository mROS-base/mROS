/* 
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *  
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
 *  $Id: xml_xerces.cpp 158 2013-09-03 10:09:13Z nces-mtakada $
 */

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#if defined(XERCES_NEW_IOSTREAMS)
#include <fstream>
#else
#include <fstream.h>
#endif
#include <xercesc/util/OutOfMemoryException.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "toppers/io.hpp"
#include "toppers/global.hpp"
#include "toppers/diagnostics.hpp"
#include "cfg1_out.hpp"
#include "xml_object.hpp"

#if defined( _MSC_VER ) && _DEBUG
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

using namespace toppers;

namespace toppers
{
  namespace xml
  {
    std::vector< toppers::xml::container::object* > cfg1_out::xml_parser_init( std::string input_file )
    {
      SAX2XMLReader::ValSchemes    valScheme    = SAX2XMLReader::Val_Auto;
      bool                         doNamespaces = true;
      bool                         doSchema = true;
      bool                         schemaFullChecking = false;
      bool                         identityConstraintChecking = true;
      bool                         namespacePrefixes = true;
      bool                         recognizeNEL = false;

      // Initialize the XML4C2 system
      try
      {
        XMLPlatformUtils::Initialize();
      }
      catch (const XMLException& toCatch)
      {
        std::vector<toppers::xml::container::object*> object_array;
        fatal( _("Error during initialization! Message:\n%" ), toNative(toCatch.getMessage()));
        return object_array;
      }

      //
      //  Create a SAX parser object. Then, according to what we were told on
      //  the command line, set it to validate or not.
      //
      SAX2XMLReader* parser = XMLReaderFactory::createXMLReader();
      parser->setFeature(XMLUni::fgSAX2CoreNameSpaces, doNamespaces);
      parser->setFeature(XMLUni::fgXercesSchema, doSchema);
      parser->setFeature(XMLUni::fgXercesHandleMultipleImports, true);
      parser->setFeature(XMLUni::fgXercesSchemaFullChecking, schemaFullChecking);
      parser->setFeature(XMLUni::fgXercesIdentityConstraintChecking, identityConstraintChecking);
      parser->setFeature(XMLUni::fgSAX2CoreNameSpacePrefixes, namespacePrefixes);

      if (valScheme == SAX2XMLReader::Val_Auto)
      {
        parser->setFeature(XMLUni::fgSAX2CoreValidation, true);
        parser->setFeature(XMLUni::fgXercesDynamic, true);
      }
      if (valScheme == SAX2XMLReader::Val_Never)
      {
        parser->setFeature(XMLUni::fgSAX2CoreValidation, false);
      }
      if (valScheme == SAX2XMLReader::Val_Always)
      {
        parser->setFeature(XMLUni::fgSAX2CoreValidation, true);
        parser->setFeature(XMLUni::fgXercesDynamic, false);
      }

      /* External Schema file */
      // XMLファイルのチェック
      namespace fs = boost::filesystem;

      if( !fs::exists( input_file ) )
      {
        fatal( _("'%1%` is not exist."), input_file );
      }
      /* 設定ファイルがある場合はパラメータ名のチェックを行う */
      std::string paraname( get_global_string( "ini-file" ) );
      std::string strAUTOSARVersion;
      std::string strSchema;
      std::string strSchemaLocation;
      std::string strContainerPath;
      std::string strModuleName;
      //std::cout << "AUTOSAR ini-file (ini file name):[" << paraname << "]" << std::endl;
      if( !paraname.empty() )
      {
        strAUTOSARVersion = get_global_string( "XML_AUTOSARVersion" );
        if( strAUTOSARVersion.empty() )
        {
          strAUTOSARVersion = "4";
          warning( _( " \"AUTOSARVersion\" parameter is not found in AUTOSAR ini-file. Use default value." ) );
        }
        strSchema = get_global_string( "XML_Schema" );
        if( strSchema.empty() )
        {
          strSchema = "./AUTOSAR_4-0-3_STRICT.xsd";
          warning( _( " \"Schema\" parameter is not found in AUTOSAR ini-file. Use default value." ) );
        }
        strSchemaLocation = get_global_string( "XML_SchemaLocation" );
        if( strSchemaLocation.empty() )
        {
          strSchemaLocation = "http://autosar.org/schema/r4.0";
          warning( _( " \"SchemaLocation\" parameter is not found in AUTOSAR ini-file. Use default value." ) );
        }
        strContainerPath = get_global_string( "XML_ContainerPath" );
        if( strContainerPath.empty() )
        {
          strContainerPath = "/AUTOSAR/EcucDefs";
          warning( _( " \"ContainerPath\" parameter is not found in AUTOSAR ini-file. Use default value." ) );
        }
      }
      toppers::global( "XML_AUTOSARVersion" ) = strAUTOSARVersion;
      toppers::global( "XML_Schema" )         = strSchema;
      toppers::global( "XML_SchemaLocation" ) = strSchemaLocation;
      toppers::global( "XML_ContainerPath" )  = strContainerPath;

      // XMLファイルの中にxsi:schemaLocation属性があればその要素を取得
      std::string sstr( "xsi:schemaLocation" );
      std::string buf;
      toppers::read( input_file, buf );

      std::list<std::string> results;
      string::size_type index( buf.find( sstr ) );
      if( index != string::npos )
      {
        string::size_type index2( buf.substr( index ).find( "\"" ) );
        string::size_type index3( buf.substr( index+index2+1 ).find( "\"" ) );
        sstr = buf.substr( index+index2+1, index3 );
        split( results, sstr, boost::is_space() );
      }

      // スキーマファイルのチェック
      std::ostringstream ostream;
      if( results.size() == 2 && fs::exists( results.back() ) )
      {
        ostream << sstr;
      }
      else
      {
        std::string schema( get_global_string( "XML_Schema" ) );
        std::string schema_location( get_global_string( "XML_SchemaLocation" ) );

        ostream << schema_location << " " << fs::absolute( get_global_string( "cfg-directory" ).c_str() ).string() << schema;
      }
      XMLCh* str (XMLString::transcode (ostream.str().c_str()));

      parser->setProperty(XMLUni::fgXercesSchemaExternalSchemaLocation, str);

      //
      //  Create our SAX handler object and install it on the parser, as the
      //  document and error handler.
      //
      SAX2Handlers handler;
      parser->setContentHandler(&handler);
      parser->setErrorHandler(&handler);

      //reset error count first
      handler.resetErrors();

      handler.filename = input_file;

      try
      {
        parser->parse(input_file.c_str());
      }
      catch (const OutOfMemoryException&)
      {
        warning("OutOfMemoryException");
      }
      catch (const XMLException& e)
      {
        warning( _("\nError during parsing: '%'\nException message is:  \n%\n"), input_file, toNative(e.getMessage()));
      }
      catch (...)
      {
        warning( _("\nUnexpected exception during parsing: '%'\n"), input_file);
      }

      delete parser;

      XMLPlatformUtils::Terminate();

      return handler.object_array;
    }
  }
}
