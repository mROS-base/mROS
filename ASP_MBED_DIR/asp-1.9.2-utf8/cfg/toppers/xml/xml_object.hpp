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
 *  $Id: xml_object.hpp 137 2012-12-26 07:11:56Z nces-mtakada $
 */

#ifndef TOPPERS_XML_OBJECT_HPP_
#define TOPPERS_XML_OBJECT_HPP_

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#include <xercesc/sax2/Attributes.hpp>
#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/sax/Locator.hpp>

XERCES_CPP_NAMESPACE_USE
using namespace std;

typedef std::basic_string<XMLCh> XercesString;

inline XercesString fromNative(const char* str)
{
  boost::scoped_array<XMLCh> ptr(xercesc::XMLString::transcode(str));
  return XercesString(ptr.get());
}

inline XercesString fromNative(const std::string& str)
{
  return fromNative(str.c_str());
}

inline std::string toNative(const XMLCh* str)
{
  boost::scoped_array<char> ptr(xercesc::XMLString::transcode(str));
  return std::string(ptr.get());
}

inline std::string toNative(const XercesString& str)
{
  return toNative(str.c_str());
}

namespace toppers
{
  namespace xml
  {
    /*
     * \class
     * ARXMLファイルに記述されたXMLの情報を管理するためのクラス
     */
    namespace container
    {
      /* クラスの前方宣言 */
      class object;
      class parameter;
    }

    namespace container
    {
      /* パラメータタイプ */
      enum PARAMETER_TYPE
      {
        TYPE_UNKNOWN = 0,
        TYPE_INT ,
        TYPE_FLOAT ,
        TYPE_STRING ,
        TYPE_BOOLEAN ,
        TYPE_ENUM , 
        TYPE_REF ,
        TYPE_FUNCTION,
        TYPE_INCLUDE,
      };

      /* オブジェクト定義のクラス */
      class object
      {
      public:
        static const int undefined = -1;  /*  オブジェクトの未定義定数  */
        object()
        {
          id = undefined;
          line = undefined;
          parent = NULL;
          siblings = undefined;
        }
        ~object()
        {
        }

        int getId() { return id; }
        int getLine() { return line; }
        std::string getFileName() { return file_name; }
        std::string getDefName()  { return define_name; }
        std::string getObjName() { return object_name; }
        std::vector<parameter*>* getParams() { return &params; }
        std::vector<object*>* getSubcontainers() { return &subcontainers; }
        object* getParent() { return parent; }
        int getSiblings() { return siblings; }

        void setId( int id_ )
        {
          id = id_;
        }
        void setLine( int line_ )
        {
          line = line_;
        }
        void setFileName( std::string str )
        {
          file_name = str;
        }
        void setDefName( std::string str )
        {
          define_name = str;
        }
        void setObjName( std::string str )
        {
          object_name = str;
        }
        void setParams( std::vector<parameter*> p )
        {
          params = p;
        }
        void setSubcontainers( std::vector<object*> p )
        {
          subcontainers = p;
        }
        void setParent( object* p )
        {
          parent = p;
        }
        void setSiblings( int siblings_ )
        {
          siblings = siblings_;
        }
      protected:
        int id;                 /* コンテナオブジェクトのID */
        string define_name;     /* コンテナ名 */
        string object_name;     /* コンテナの実体名 */
        std::vector<parameter*> params;     /* コンテナパラメータへのポインタ */
        std::vector<object*> subcontainers; /* サブコンテナへのポインタ */
        object* parent;         /* 親コンテナへのポインタ */
        string file_name;       /* パースファイル名 */
        int line;               /* パース行番号 */
        int siblings;           /* 兄弟コンテナの数 */
      };

      // コンテナパラメータに関する情報
      class parameter
      {
      public:
        ~parameter() {}

        int getLine() { return line; }
        std::string getFileName() { return file_name; }
        std::string getDefName()  { return define_name; }
        object* getParent() { return parent; }
        string getValue() { return value; }
        PARAMETER_TYPE getType() { return type; }

        void setLine( int line_ )
        {
          line = line_;
        }
        void setFileName( std::string str )
        {
          file_name = str;
        }
        void setDefName( std::string str )
        {
          define_name = str;
        }
        void setParent( object* p )
        {
          parent = p;
        }
        void setType( PARAMETER_TYPE param_type )
        {
          type = param_type;
        }
        void setValue( string val_ )
        {
          value = val_;
        }

      protected:
        string define_name;   /* パラメータ名 */
        PARAMETER_TYPE type;  /* 型 */
        string value;         /* 値 */
        object* parent;       /* コンテナへのポインタ */
        string file_name;     /* パースファイル名 */
        int line;             /* パース行番号 */
      };
    } /* container */

    struct info
    {
      char const* tfname;      /* tfで使用するときに置き換える名前 */
      char const* type;        /* 型情報 */
      container::PARAMETER_TYPE type_enum;  /* 型情報(enum) */
      unsigned int multimin;   /* 多重度最小値 */
      unsigned int multimax;   /* 多重度最大値 */
    };
  } /* xml */
} /* toppers */

class SAX2Handlers : public DefaultHandler
{
public:

  // -----------------------------------------------------------------------
  //  Constructors and Destructor
  // -----------------------------------------------------------------------
  SAX2Handlers();
  ~SAX2Handlers();

  toppers::xml::container::object *obj_temp;
  toppers::xml::container::parameter *para_temp;

  std::vector<toppers::xml::container::object*> object_array; 

  // -----------------------------------------------------------------------
  //  Handlers for the SAX ContentHandler interface
  // -----------------------------------------------------------------------
  void ignorableWhitespace(const XMLCh* const chars, const XMLSize_t length);
  void startElement(const XMLCh* const uri, const XMLCh* const localname, const XMLCh* const qname, const Attributes& attrs);
  void characters(const XMLCh* const chars, const XMLSize_t length);
  void endElement( const XMLCh* const uri, const XMLCh *const localname, const XMLCh *const qname);
  void setDocumentLocator (const Locator *const locator);
  int get_line();
  static void obj_delete(toppers::xml::container::object *pObj);

  // -----------------------------------------------------------------------
  //  Handlers for the SAX ErrorHandler interface
  // -----------------------------------------------------------------------
  void warning(const SAXParseException& exc);
  void error(const SAXParseException& exc);
  void fatalError(const SAXParseException& exc);
  void resetErrors();

  string filename;

private:
  int fEcuModuleConfigurationValues_;
  int fEcucContainerValue_;
  int fSubcontainers_;
  int fSubcontainers_old_;
  int fParameterValues_;
  int fReferenceValues_;
  XercesString currentText_;
  const Locator* locator_; 

  // PaserStrings
  XercesString ecucmodule;
  XercesString ecuccontainer;
  XercesString subcontainer;

  XercesString parameter;
  XercesString ecucnumerical;
  XercesString ecucreference;

  XercesString reference;
  XercesString ecuctextual;
  XercesString ecucboolean;     /* R3.1 only */
  XercesString ecucenum;        /* R3.1 only */
  XercesString ecucfloat;       /* R3.1 only */
  XercesString ecucfunction;    /* R3.1 only */

  XercesString definitionref;
  XercesString defqname;
  XercesString defBool;
  XercesString defEnum;
  XercesString defFloat;
  XercesString defFunction;
  XercesString defInteger;
  XercesString defReference;
  XercesString defString;

  XercesString shortname;
  XercesString value;
  XercesString valueref;

  // xml:space attribute is trim off(default(=TRUE)).
  bool fAttrXmlSpace_;
};

#endif /* TOPPERS_XML_OBJECT_HPP_ */
