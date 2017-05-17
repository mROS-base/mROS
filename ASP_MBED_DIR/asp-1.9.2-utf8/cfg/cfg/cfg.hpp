/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2012 by TAKAGI Nobuhisa
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
#ifndef CFG_HPP_
#define CFG_HPP_

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "toppers/workaround.hpp"
#include "toppers/codeset.hpp"
#include "toppers/global.hpp"
#include "toppers/diagnostics.hpp"
#include "toppers/text.hpp"
#include "toppers/misc.hpp"
#include "toppers/itronx/preprocess.hpp"
#include "toppers/itronx/static_api.hpp"
#include "toppers/itronx/factory.hpp"
#include "toppers/itronx/cfg1_out.hpp"
#include "toppers/oil/preprocess.hpp"
#include "toppers/oil/factory.hpp"
#include "toppers/oil/cfg1_out.hpp"
#ifdef  HAS_CFG_XML
#include "toppers/xml/factory.hpp"
#include "toppers/xml/cfg1_out.hpp"
#endif
#include <boost/utility.hpp>
#include <boost/format.hpp>

#define CFG_VERSION cfg_version

bool cfg0_main();
bool cfg1_main();
bool cfg2_main();
bool cfg3_main();
bool cfg4_main();
void cfg_init();

extern char const cfg_version[];

std::tr1::int64_t cfg_timestamp();

bool read_cfg_file( std::map< std::string, toppers::itronx::static_api::info > const info_map,
                    std::string& cfg1_list, std::string& includes,
                    std::vector< toppers::itronx::static_api >& static_api_array );

#endif  // ! CFG_HPP_
