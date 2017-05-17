/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2007-2008 by TAKAGI Nobuhisa
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
 *  \file   toppers/csv.hpp
 *  \brief  CSVファイル入出力に関する宣言定義
 */
#ifndef TOPPERS_CSV_HPP_
#define TOPPERS_CSV_HPP_

#include <stdexcept>
#include <ostream>
#include <iterator>
#include <string>
#include <vector>
#include <algorithm>
#include <boost/utility.hpp>
#include "toppers/misc.hpp"

namespace toppers
{

  class csv_error : public std::runtime_error
  {
  public:
    explicit csv_error( long line )
      : std::runtime_error( "CSV parse error" ), line_( line )
    {
    }
    long line() const
    {
      return line_;
    }
  private:
    long line_;
  };

  /*!
   *  \class  basic_csv csv.hpp "toppers/csv.hpp"
   *  \brief  CSV ファイルの解析クラス
   */
  template < typename CharT, class Traits = std::char_traits< CharT > >
    class basic_csv
  {
  public:
    typedef std::basic_string< CharT, Traits > string_type;
    typedef typename std::vector< std::vector< string_type > >::const_iterator const_iterator;
    typedef typename std::vector< std::vector< string_type > >::const_reference const_reference;
    typedef typename std::vector< std::vector< string_type > >::value_type value_type;
    typedef typename std::vector< std::vector< string_type > >::size_type size_type;

    basic_csv();
    template < class InputIterator >
      basic_csv( InputIterator first, InputIterator last )
    {
      read( first, last );
    }

    template < class InputIterator >
      void read( InputIterator first, InputIterator last )
    {
      const CharT dquo = widen< CharT >( '\"' );
      const CharT comma = widen< CharT >( ',' );
      const CharT lf = widen< CharT >( '\n' );
      enum
      {
        none,
        non_escaped,
        escape_opened,
        escape_closed
      } state = none;
      std::vector< std::vector< string_type > > records;
      std::vector< string_type > record;
      string_type field;
      long line = 1;

      record.reserve( 16 );
      field.reserve( 255 );

      while ( first != last )
      {
        CharT ch = *first;
        if ( ch == dquo )
        {
          switch ( state )
          {
          case none:
            state = escape_opened;
            break;
          case escape_opened:
            ++first;
            ch = *first;
            if ( ch == dquo ) // "" だったので、" を追加
            {
              field.push_back( dquo );
            }
            else  // 閉じ "
            {
              state = escape_closed;
            }
            continue;   // ++first をこれ以上実行させない
          default:
            throw csv_error( line );
            break;
          }
        }
        else  // ch != dquo
        {
          if ( state == escape_opened )
          {
            field.push_back( ch );
          }
          else
          {
            if ( ch == comma || ch == lf )
            {
              record.push_back( field );
              field.clear();
              state = none;
              if ( ch == lf ) // 改行（RFC4180 では厳密には CR-LF なければならないが、とりあえず '\n' とする）
              {
                records.push_back( record );
                record.clear();
                ++line;
              }
            }
            else
            {
              field.push_back( ch );
//              CharT const* debug_string = field.c_str();
            }
          }
        }
        ++first;
      }
      records_.swap( records );
    }
    template < class OutputIterator >
      void write( OutputIterator result ) const
    {
      const CharT dquo = widen< CharT >( '\"' );
      const CharT comma = widen< CharT >( ',' );
      const CharT lf = widen< CharT >( '\n' );

      for ( typename std::vector< std::vector< string_type > >::const_iterator r_iter( records_.begin() ), r_last( records_.end() );
            r_iter != r_last;
            ++r_iter )
      {
        for ( typename std::vector< string_type >::const_iterator f_iter( r_iter->begin() ), f_last( r_iter->end() );
              f_iter != f_last;
              ++f_iter )
        {
          std::string field;
          bool need_escape = false;

          for ( typename string_type::const_iterator s_iter( f_iter->begin() ), s_last( f_iter->end() );
                s_iter != s_last;
                ++s_iter )
          {
            CharT ch = *s_iter;
            if ( ch == dquo )
            {
              field.push_back( dquo );
              need_escape = true;
            }
            else if ( ch == comma || ch == lf )
            {
              need_escape = true;
            }
            field.push_back( ch );
          }

          if ( need_escape )
          {
            field = dquo + field + dquo;
          }
          result = std::copy( field.begin(), field.end(), result );
          if ( boost::next( f_iter ) != f_last )
          {
            *result++ = comma;
          }
        }
        *result++ = lf;
      }
    }

    const_iterator begin() const
    {
      return records_.begin();
    }
    const_iterator end() const
    {
      return records_.end();
    }
    const_reference at( size_type pos ) const
    {
      return records_.at( pos );
    }
    const_reference operator[]( size_type pos ) const
    {
      return records_[ pos ];
    }
    bool empty() const
    {
      return records_.empty();
    }
    size_type size() const
    {
      return records_.size();
    }
    void swap( basic_csv& other )
    {
      records_.swap( other.records_ );
    }
  private:
    std::vector< std::vector< string_type > > records_;
  };

  typedef basic_csv< char > csv;
  typedef basic_csv< wchar_t > wcsv;

}

#endif  // TOPPERS_CSV_HPP_
