/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2005-2011 by TAKAGI Nobuhisa
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

/*
 *  toppers/s_record.cpp
 */
#include "toppers/s_record.hpp"
#include <istream>
#include <algorithm>
#include <iterator>
#include <cctype>
#include <cstring>

namespace toppers
{

  /*!
   *  \brief  Sレコードのロード
   *  \param  istr  入力ストリーム
   */
  void s_record::load( std::istream& istr )
  {
    int type = -1;

    while ( istr )
    {
      std::string buf;
      std::getline( istr, buf );
      if ( buf.empty() )
      {
        break;
      }
      record rec = read_record( buf );

      // あまり厳密には処理しない
      if ( '1' <= rec.type && rec.type <= '3' )
      {
        bool done = false;
        typedef std::vector< value_type >::iterator iterator;
        for ( iterator iter( data_.begin() ), last( data_.end() ); iter != last; ++iter )
        {
          if ( rec.address == iter->first + iter->second.size() )
          {
            std::copy( rec.data.begin(), rec.data.end(), std::back_inserter( iter->second ) );
            done = true;
          }
        }
        if ( !done )
        {
          data_.push_back( value_type( rec.address, std::vector< unsigned char >( rec.data.begin(), rec.data.end() ) ) );
        }
      }
      type = rec.type;
    }

    cache_ = data_.begin();
  }

  /*!
   *  \brief  指定アドレスのバイトデータ取得
   *  \param  address   アドレス指定
   *  \return address で指定したアドレスのバイトデータを返す
   */
  int s_record::operator[]( size_type address ) const
  {
    typedef std::vector< value_type >::const_iterator const_iterator;
    if ( cache_ != data_.end() )
    {
      const_iterator iter( cache_ );
      if ( iter->first <= address && address < iter->first + iter->second.size() )
      {
        return iter->second.at( address - iter->first );
      }
    }
    for ( const_iterator iter( cache_ ), last( data_.end() ); iter != last; ++iter )
    {
      if ( iter->first <= address && address < iter->first + iter->second.size() )
      {
        cache_ = iter;
        return iter->second.at( address - iter->first );
      }
    }
    for ( const_iterator iter( data_.begin() ), last( cache_ ); iter != last; ++iter )
    {
      if ( iter->first <= address && address < iter->first + iter->second.size() )
      {
        cache_ = iter;
        return iter->second.at( address - iter->first );
      }
    }
    return -1;
  }

  /*!
   *  \brief  Sレコード中の整数値読み込み
   *  \param  base  読み込み位置の先頭アドレス
   *  \param  size  整数値のバイト数
   *  \param  little_endian リトルエンディアンなら true、ビッグエンディアンなら false を指定
   *  \return 読み込んだ整数値を返す
   */
  boost::uintmax_t s_record::get_value( std::size_t base, std::size_t size, bool little_endian ) const
  {
    boost::uintmax_t value = 0;
    if ( little_endian )
    {
      for ( long j = static_cast< long >( size-1 ); j >= 0; j-- )
      {
        int t = ( *this )[ base + j ];
        if ( t < 0 )
        {
          throw data_error();
        }
        value = ( value << 8 ) | ( t & 0xff );
      }
    }
    else
    {
      for ( std::size_t j = 0; j < size; j++ )
      {
        int t = ( *this )[ base + j ];
        if ( t < 0 )
        {
          throw data_error();
        }
        value = ( value << 8 ) | ( t & 0xff );
      }
    }
    return value;
  }

  unsigned long s_record::lower_bound() const
  {
    return data_.front().first;
  }

  unsigned long s_record::upper_bound() const
  {
    return data_.back().first + data_.back().second.size();
  }

  /*!
   *  \brief  Sレコードの１行読み込み
   *  \param  rec_buf １行バッファ
   *  \return 読み込み結果を返す
   */
  s_record::record const s_record::read_record( std::string const& rec_buf )
  {
    std::string buf( rec_buf );

    // 行末に'\r'または'\n'が残留している場合の対策
    while ( std::isspace( static_cast< unsigned char >( *buf.rbegin() ) ) )
    {
      buf = buf.substr( 0, buf.size()-1 );
    }

    if ( buf.size() < 10 || buf[0] != 'S' )
    {
      throw format_error();
    }
    int ch = static_cast< unsigned char >( buf[1] );
    int address_length = 4;
    std::string::size_type size = buf.size();

    switch ( ch )
    {
    case '1':
    case '9':
      address_length = 4;
      break;
    case '2':
    case '8':
      address_length = 6;
      break;
    case '3':
    case '7':
      address_length = 8;
      break;
    default:
      if ( !std::isdigit( static_cast< unsigned char >( ch ) ) )
      {
        throw format_error();
      }
      break;
    }
    
    record rec;
    rec.type = ch;
    rec.length = xdigit_to_int( buf[2] ) << 4 | xdigit_to_int( buf[3] );
    if ( rec.length * 2u + 4 != buf.size() )
    {
      throw format_error();
    }
    rec.length -= address_length/2 + 1; // アドレスとチェックサムの長さを引いて、データ長に直す

    rec.address = 0;
    int base = 4;
    for ( int i = 0; i < address_length; i++ )
    {
      rec.address = rec.address << 4 | xdigit_to_int( buf[base+i] );
    }

    base += address_length;
    rec.data.reserve( rec.length );
    for ( int i = 0; i < rec.length; i++ )
    {
      rec.data.push_back( static_cast< unsigned char >( xdigit_to_int( buf[base+i*2] ) << 4 | xdigit_to_int( buf[base+i*2+1] ) ) );
    }

    rec.checksum = xdigit_to_int( buf[size-2] ) << 4 | xdigit_to_int( buf[size-1] );

    // チェックサム判定は省略

    return rec;
  }

  /*!
   *  \brief  十六進数字から数値への変換
   *  \param  ch  十六進数字（文字）
   *  \return ch に対応する数値
   */
  int s_record::xdigit_to_int( int ch )
  {
    static char const xdigits[] = "0123456789abcdef";

    ch = std::tolower( static_cast< unsigned char >( ch ) );
    char const* s = std::strchr( xdigits, ch );
    if ( s == 0 )
    {
      return -1;
    }
    return s - xdigits;
  }

}
