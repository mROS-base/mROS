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
 *  \file   toppers/text.hpp
 *  \brief  テキストデータ管理に関する宣言定義
 *
 *  このファイルで定義されるクラス
 *  \code
 *  class basic_text< Container, CharT, Traits, Allocator >;
 *  \endcode
 */
#ifndef TOPPERS_TEXT_HPP_
#define TOPPERS_TEXT_HPP_

#include <algorithm>
#include <iterator>
#include <vector>
#include <numeric>
#include "toppers/text_line.hpp"
#include "toppers/misc.hpp"
#include <boost/utility.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/format.hpp>

namespace toppers
{

  /*!
   *  \class  basic_text  text.hpp "toppers/text.hpp"
   *  \brief  テキストデータ管理テンプレートクラス
   *
   *  ファイルから読み込んだテキストデータの管理を行うためのクラスです。
   *  このクラスでは、テキストデータを単なる文字列としてではなく、入力元の
   *  ファイル名と行番号を付加した形で管理します。
   *
   *  \note   このクラスはテキスト編集の用途を想定したものではありません。
   */
  template
  <
    template < typename, class > class Container,
    typename CharT,
    class Traits = std::char_traits< CharT >,
    class Allocator = std::allocator< CharT >
  >
    class basic_text
  {
  public:
    typedef basic_text< Container, CharT, Traits, Allocator > self_t;
    typedef std::basic_string< CharT, Traits, Allocator > string_type;
    typedef ::toppers::text_line line_type;
    typedef basic_line_buf< CharT, Traits, Allocator > line_buf;

    //  以下は一般的なコンテナとの互換のための型定義
    ////////////////////////////////////////////////////////////////////////////////
    typedef CharT value_type;
    typedef Container< line_buf, std::allocator< line_buf > > container;
    typedef value_type& reference;
    typedef value_type const& const_reference;
    typedef value_type* pointer;
    typedef value_type const* const_pointer;
    typedef Allocator allocator_type;
    typedef typename Allocator::difference_type difference_type;
    typedef typename Allocator::size_type size_type;

    /*!
     *  \class  const_iterator text.hpp "toppers/text.hpp"
     *  \brief  toppers::basic_text のイテレータ
     */
    class const_iterator : public boost::iterator_facade< const_iterator, CharT, boost::random_access_traversal_tag, CharT const& >
    {
      friend class boost::iterator_core_access;
    public:
      typedef boost::iterator_facade< const_iterator, CharT, boost::random_access_traversal_tag, CharT const& > base_t;
      typedef typename container::const_iterator row_iterator;
      typedef typename base_t::value_type value_type;
      typedef typename base_t::reference reference;
      typedef typename base_t::pointer pointer;
      typedef typename base_t::difference_type difference_type;

      const_iterator() : row_(), col_( 0 ) {}
      const_iterator( row_iterator row, size_type col = 0 ) : row_( row ), col_( col ) {}
      line_type const& line() const { return row_->line; }
      row_iterator const& get_row() const { return row_; }
      size_type get_col() const { return col_; }
    protected:
      reference dereference() const { return row_->buf[col_]; }
      bool equal( const_iterator const& rhs ) const { return row_ == rhs.row_ && col_ == rhs.col_; }
      void increment() { if ( ++col_ == row_->buf.size() ) { col_ = 0; ++row_; } }
      void decrement() { if ( col_ == 0 ) col_ = ( --row_ )->buf.size(); --col_; }
      void advance( difference_type n )
      {
        typename container::const_iterator row = row_;
        difference_type col = static_cast< difference_type >( col_ );
        if ( n >= 0 )
        {
          while ( static_cast< size_type>( col + n ) > row->buf.size() )
          {
            n -= row->buf.size() - col;
            ++row;
            col = 0;
          }
          col += n;
          if ( col == row->buf.size() )
          {
            col = 0;
            ++row;
          }
        }
        else  // n < 0
        {
          while ( col + n < -1 )
          {
            n += col;
            --row;
            col = row->buf.size() - 1;
          }
          col += n;
          if ( col == -1 )
          {
            --row;
            col = row->buf.size() - 1;
          }
        }
        row_ = row;
        col_ = col;
      }
      difference_type distance_to( const_iterator const& rhs ) const
      {
        difference_type n;
        typename container::const_iterator row = row_;
        if ( rhs.row_ < row_ )
        {
          n = -static_cast< difference_type >( col_ + ( rhs.row_->buf.size() - rhs.col_ ) );
          while ( --row != rhs.row_ )
          {
            n -= row->buf.size();
          }
        }
        else if ( row_ < rhs.row_ )
        {
          n = col_ + rhs.col_;
          while ( ++row != rhs.row_ )
          {
            n += row->buf.size();
          }
        }
        else
        {
          n = static_cast< difference_type >( rhs.col_ ) - static_cast< difference_type >( col_ );
        }
        return n;
      }
    private:
      typename container::const_iterator row_;
      size_type col_;
    };

    /*!
     *  \class  iterator text.hpp "toppers/text.hpp"
     *  \brief  toppers::basic_text のイテレータ
     */
    class iterator : public const_iterator
    {
    public:
      typedef typename container::iterator row_iterator;
      typedef typename const_iterator::value_type value_type;
      typedef typename const_iterator::reference reference;
      typedef typename const_iterator::pointer pointer;
      typedef typename const_iterator::difference_type difference_type;

      iterator() {}
      explicit iterator( row_iterator row, size_type col = 0 ) : const_iterator( row, col ) {}
      value_type& operator*() const { return const_cast< value_type& >( const_iterator::dereference() ); }
      iterator& operator++() { const_iterator::increment(); return *this; }
      iterator operator++( int ) { iterator t( *this ); const_iterator::increment(); return t; }
      iterator& operator--() { const_iterator::decrement(); return *this; }
      iterator operator--( int ) { iterator t( *this ); const_iterator::decrement(); return t; }
    };

    typedef std::reverse_iterator< const_iterator > const_reverse_iterator;
    typedef std::reverse_iterator< iterator > reverse_iterator;

  public:
    /*!
     *  \brief  デフォルトコンストラクタ
     */
    basic_text() {}

    /*!
     *  \brief  コンストラクタ
     *  \param  first   初期化に用いるデータ列の先頭
     *  \param  last    初期化に用いるデータ列の終端+1
     */
    basic_text( const_iterator const& first, const_iterator const& last )
    {
      if ( first != last )
      {
        init( first, last );
      }
    }

    /*!
     *  \brief  コンストラクタ
     *  \param  first   初期化に用いるデータ列の先頭
     *  \param  last    初期化に用いるデータ列の終端+1
     *  \note   ファイル名は"unknown"になります。
     */
    template < class InputIterator >
      basic_text( InputIterator first, InputIterator last )
    {
      if ( first != last )
      {
        init( first, last, append_directive );
      }
    }

    /*!
     *  \brief  コンストラクタ
     *  \param  first   初期化に用いるデータ列の先頭
     *  \param  last    初期化に用いるデータ列の終端+1
     *  \param  directive １行をコンテナに格納するための処理
     *  \note   ファイル名は"unknown"になります。
     *
     *  directive は void directive( container& cont, line_buf& buf )の形式を
     *  とる関数へのポインタまたはファンクタであることを想定しています。
     *  cont は内部でデータの格納に使用するコンテナ、 buf は１行分の文字
     *  列とファイル名・行番号を格納したバッファです。
     */
    template < class InputIterator, class Directive >
      basic_text( InputIterator first, InputIterator last, Directive directive )
    {
      if ( first != last )
      {
        init( first, last, directive );
      }
    }

    /*!
     *  \brief  コンストラクタ
     *  \param  istr    初期化に用いる入力ストリーム
     *  \note   ファイル名は"unknown"になります。
     */
    basic_text( std::basic_istream< CharT, Traits >& istr )
    {
      init( istr, append_directive );
    }

    /*!
     *  \brief  コンストラクタ
     *  \param  istr    初期化に用いる入力ストリーム
     *  \param  directive １行をコンテナに格納するための処理
     *  \note   ファイル名は"unknown"になります。
     *
     *  directive は void directive( container& cont, line_buf& buf )の形式を
     *  とる関数へのポインタまたはファンクタであることを想定しています。
     *  cont は内部でデータの格納に使用するコンテナ、 buf は１行分の文字
     *  列とファイル名・行番号を格納したバッファです。
     */
    template < class Directive >
      basic_text( std::basic_istream< CharT, Traits >& istr, Directive directive )
    {
      init( istr, directive );
    }

    /*!
     *  \brief  先頭位置の取得
     *  \return 先頭文字へのイテレータを返す
     */
    const_iterator begin() const
    {
      return const_iterator( container_.begin() );
    }

    /*!
     *  \brief  先頭位置の取得
     *  \return 先頭文字へのイテレータを返す
     */
    iterator begin()
    {
      return iterator( container_.begin() );
    }

    /*!
     *  \brief  行数の取得
     *  \param  pos  先頭文字からのオフセット
     *  \return posで指定されたオフセットから行数を求め，イテレータを返す
     */
    const_iterator line_at(size_type pos) const
    {
        // modified by takuya
        //typedef text::container::const_iterator const_row_iterator;
        typedef typename container::const_iterator const_row_iterator;
        const_iterator first( container_.begin() ), last( container_.end() );
        int count = 0 , buf_size , i;
        size_type sum = 0;

        while((sum < pos) && (first != last))
        {
            const_row_iterator current( first.get_row() );
            buf_size = current->buf.size();
            sum += buf_size;
            i = 0;
            while(i < buf_size)
            {
                i++;
                first++;
            }
            count += 1;
        }
        
        if(first < last)
        {
            first -= buf_size;
            return const_iterator(first);
        }
        else
        {
            return const_iterator( --last );
        }
    }   

    /*!
     *  \brief  終端位置+1の取得
     *  \return 終端文字の次要素へのイテレータを返す
     */
    const_iterator end() const
    {
      return const_iterator( container_.end() );
    }

    /*!
     *  \brief  終端位置+1の取得
     *  \return 終端文字の次要素へのイテレータを返す
     */
    iterator end()
    {
      return iterator( container_.end() );
    }

    /*!
     *  \brief  逆順先頭位置の取得
     *  \return 逆順先頭文字への逆イテレータを返す
     */
    const_reverse_iterator rbegin() const
    {
      string_type const* pbuf = &container_.back().buf;
      return const_reverse_iterator( container_.rbegin() + pbuf->empty() ? 0 : pbuf->size()-1 );
    }

    /*!
     *  \brief  逆順先頭位置の取得
     *  \return 逆順先頭文字への逆イテレータを返す
     */
    reverse_iterator rbegin()
    {
      string_type* pbuf = &container_.back().buf;
      return reverse_iterator( container_.rbegin() + pbuf->empty() ? 0 : pbuf->size()-1 );
    }

    /*!
     *  \brief  逆順終端位置+1の取得
     *  \return 逆順終端文字の次要素への逆イテレータを返す
     */
    const_reverse_iterator rend() const
    {
      return const_reverse_iterator( container_.rend() );
    }

    /*!
     *  \brief  逆順終端位置+1の取得
     *  \return 逆順終端文字の次要素への逆イテレータを返す
     */
    reverse_iterator rend()
    {
      return reverse_iterator( container_.rend() );
    }

    /*!
     *  \brief  先頭文字の参照
     *  \return 先頭文字への参照を返す
     */
    const_reference front() const
    {
      return container_.front().buf[0];
    }

    /*!
     *  \brief  先頭文字の参照
     *  \return 先頭文字への参照を返す
     */
    reference front()
    {
      return container_.front().buf[0];
    }

    /*!
     *  \brief  終端文字の参照
     *  \return 終端文字への参照を返す
     */
    const_reference back() const
    {
      string_type const* pbuf = &container_.back().buf;
      return pbuf->at( pbuf->size()-1 );
    }

    /*!
     *  \brief  終端文字の参照
     *  \return 終端文字への参照を返す
     */
    reference back()
    {
      string_type* pbuf = &container_.back().buf;
      return pbuf->at( pbuf->size()-1 );
    }

    /*!
     *  \brief  指定位置の文字の参照
     *  \return 指定位置の文字への参照を返す
     */
    const_reference at( size_type pos ) const
    {
      return container_.front().buf[pos];
    }

    /*!
     *  \brief  指定位置の文字の参照
     *  \return 指定位置の文字への参照を返す
     */
    reference at( size_type pos )
    {
      return container_.front().buf[pos];
    }

    /*!
     *  \brief  指定範囲のテキストデータを追加
     *  \param  first 追加に用いるデータ列の先頭
     *  \param  last  追加に用いるデータ列の終端+1
     */
    void append( const_iterator const& first, const_iterator const& last )
    {
      if ( first != last )
      {
        self_t t( *this );
        t.init( first, last );
        swap( t );
      }
    }

    /*!
     *  \brief  指定範囲のテキストデータを追加
     *  \param  first 追加に用いるデータ列の先頭
     *  \param  last  追加に用いるデータ列の終端+1
     *  \note   ファイル名と行番号は追加先の終端のものを継続します。
     *          追加先にデータがない場合は"unknown"になります。
     */
    template < class InputIterator >
      void append( InputIterator first, InputIterator last )
    {
      append( first, last, append_directive );
    }

    /*!
     *  \brief  指定範囲のテキストデータを追加
     *  \param  first 追加に用いるデータ列の先頭
     *  \param  last  追加に用いるデータ列の終端+1
     *  \param  directive １行をコンテナに格納するための処理
     *  \note   ファイル名と行番号は追加先の終端のものを継続します。
     *          追加先にデータがない場合は"unknown"になります。
     *
     *  directive は void directive( container& cont, line_buf& buf )の形式を
     *  とる関数へのポインタまたはファンクタであることを想定しています。
     *  cont は内部でデータの格納に使用するコンテナ、 buf は１行分の文字
     *  列とファイル名・行番号を格納したバッファです。
     */
    template < class InputIterator, class Directive >
      void append( InputIterator first, InputIterator last, Directive directive )
    {
      if ( first != last )
      {
        self_t t( *this );
        t.init( first, last, directive );
        swap( t );
      }
    }

    /*!
     *  \brief  指定範囲のテキストデータを追加
     *  \param  istr      追加に用いる入力ストリーム
     *  \note   ファイル名と行番号は追加先の終端のものを継続します。
     *          追加先にデータがない場合は"unknown"になります。
     */
    void append( std::basic_istream< CharT, Traits >& istr )
    {
      append( istr, append_directive );
    }

    /*!
     *  \brief  指定範囲のテキストデータを追加
     *  \param  istr      追加に用いる入力ストリーム
     *  \param  directive １行をコンテナに格納するための処理
     *  \note   ファイル名と行番号は追加先の終端のものを継続します。
     *          追加先にデータがない場合は"unknown"になります。
     *
     *  directive は void directive( container& cont, line_buf& buf )の形式を
     *  とる関数へのポインタまたはファンクタであることを想定しています。
     *  cont は内部でデータの格納に使用するコンテナ、 buf は１行分の文字
     *  列とファイル名・行番号を格納したバッファです。
     */
    template < class Directive >
      void append( std::basic_istream< CharT, Traits >& istr, Directive directive )
    {
      self_t t( *this );
      t.init( istr, directive );
      swap( t );
    }

    /*!
     *  \brief  指定範囲のテキストデータを代入
     *  \param  first 代入に用いるデータ列の先頭
     *  \param  last  代入に用いるデータ列の終端+1
     */
    void assign( const_iterator const& first, const_iterator const& last )
    {
      self_t t( first, last );
      swap( t );
    }

    /*!
     *  \brief  指定範囲のテキストデータを代入
     *  \param  first 代入に用いるデータ列の先頭
     *  \param  last  代入に用いるデータ列の終端+1
     *  \note   ファイル名は"unknown"になります。
     */
    template < class InputIterator >
      void assign( InputIterator first, InputIterator last )
    {
      self_t t( first, last, append_directive );
      swap( t );
    }

    /*!
     *  \brief  指定範囲のテキストデータを代入
     *  \param  first 代入に用いるデータ列の先頭
     *  \param  last  代入に用いるデータ列の終端+1
     *  \param  directive １行をコンテナに格納するための処理
     *  \note   ファイル名は"unknown"になります。
     *
     *  directive は void directive( container& cont, line_buf& buf )の形式を
     *  とる関数へのポインタまたはファンクタであることを想定しています。
     *  cont は内部でデータの格納に使用するコンテナ、 buf は１行分の文字
     *  列とファイル名・行番号を格納したバッファです。
     */
    template < class InputIterator, class Directive >
      void assign( InputIterator first, InputIterator last, Directive directive )
    {
      self_t t( first, last, directive );
      swap( t );
    }

    /*!
     *  \brief  指定範囲のテキストデータを代入
     *  \param  istr      代入に用いる入力ストリーム
     *  \note   ファイル名と行番号は代入先の終端のものを継続します。
     *          追加先にデータがない場合は"unknown"になります。
     */
    void assign( std::basic_istream< CharT, Traits >& istr )
    {
      self_t t( istr, append_directive );
      swap( t );
    }

    /*!
     *  \brief  指定範囲のテキストデータを代入
     *  \param  istr      代入に用いる入力ストリーム
     *  \param  directive １行をコンテナに格納するための処理
     *  \note   ファイル名と行番号は代入先の終端のものを継続します。
     *          追加先にデータがない場合は"unknown"になります。
     *
     *  directive は void directive( container& cont, line_buf& buf )の形式を
     *  とる関数へのポインタまたはファンクタであることを想定しています。
     *  cont は内部でデータの格納に使用するコンテナ、 buf は１行分の文字
     *  列とファイル名・行番号を格納したバッファです。
     */
    template < class Directive >
      void assign( std::basic_istream< CharT, Traits >& istr, Directive directive )
    {
      self_t t( istr, directive );
      swap( t );
    }

    /*!
     *  \brief  終端に文字を追加
     *  \param  value 追加する文字
     */
    void push_back( value_type value )
    {
      if ( container_.empty() || container_.back().line.file != line_.file )
      {
        line_buf buf( initial_line(), string_type( 1, value ) );
        container_.push_back( buf );
      }
      else
      {
        string_type const* pbuf = &container_.back().buf;
        if ( pbuf->at( pbuf->size()-1 ) == widen( '\n' ) )
        {
          ++line_.line;
          line_buf buf( initial_line(), string_type( 1, value ) );
          container_.push_back( buf );
        }
        else
        {
          container_.back().buf += value;
        }
      }
    }

    /*!
     *  \brief  終端から文字を削除
     */
    void pop_back()
    {
      if ( container_.empty() )
      {
        container_.pop_back();
      }
      else
      {
        string_type const* pbuf = &container_.back().buf;
        pbuf->resize( pbuf->size()-1 );
        if ( container_.back().buf.empty() )
        {
          container_.pop_back();
        }
      }
    }

    /*!
     *  \brief  コンテナが空かどうかの判別
     *  \return 空の場合は true を返す
     */
    bool empty() const
    {
      return container_.empty();
    }

    /*!
     *  \brief  コンテナのサイズ
     *  \return コンテナに含まれている文字数を返す
     */
    size_type size() const
    {
      return std::accumulate( container_.begin(), container_.end(), size_type( 0 ), add_line_size );
    }

    /*!
     *  \brief  コンテナの交換
     *  \param  other   交換対象となるコンテナ
     */
    void swap( self_t& other )
    {
      container_.swap( other.container_ );
      line_.swap( other.line_ );
    }

    /*!
     *  \brief  コンテナのクリア
     */
    void clear()
    {
      container_.clear();
      line_.file.clear();
      line_.line = 0;
    }

    void set_line( std::string const& file, long line )
    {
      line_.file = file;
      line_.line = line;
    }

    line_type get_line() const
    {
      return line_;
    }
  private:
    //! 同種のコンテナを入力元とする初期化処理
    void init( const_iterator const& first, const_iterator const& last )
    {
      line_buf buf( *first.get_row() );
      if ( first.get_col() > 0 )
      {
        buf.buf = buf.buf.substr( first.get_col() );
      }
      container_.push_back( buf );
      std::copy( boost::next( first ).get_row(), last.get_row(), std::back_inserter( container_ ) );
      buf = *last.get_row();
      buf.buf = buf.buf.substr( 0, last.get_col() );
      container_.push_back( buf );
      line_ = buf.line;
    }

    //! 異種のコンテナを入力元とする初期化処理
    template < class InputIterator, class Directive >
      void init( InputIterator first, InputIterator last, Directive directive )
    {
      line_buf buf( initial_line() );
      value_type const nl = widen( '\n' );
      for ( InputIterator iter = first; iter != last; ++iter )
      {
        value_type ch = *iter;
        buf.buf += ch;
        if ( ch == nl )
        {
          directive( container_, buf );
          line_ = buf.line;
        }
      }
      if ( !buf.buf.empty() )
      {
        container_.push_back( buf );
      }
      line_ = buf.line;
    }

    //! 入力ストリームを入力元とする初期化処理
    template < class Directive >
      void init( std::basic_istream< CharT, Traits >& istr, Directive directive )
    {
      line_buf buf( initial_line() );
      value_type const nl = widen( '\n' );
      while ( istr )
      {
        value_type ch;
        istr.get( ch );
        buf.buf += ch;
        if ( ch == nl )
        {
          directive( container_, buf );
          line_ = buf.line;
        }
      }
      if ( !buf.buf.empty() )
      {
        container_.push_back( buf );
      }
      line_ = buf.line;
    }

    //! ファイル名・行番号の初期情報の取得
    line_type const initial_line() const
    {
      if ( line_.line > 0 )
      {
        return line_;
      }
      if ( container_.empty() )
      {
        return line_type( "unknown", 1 );
      }
      return container_.back().line;
    }

    //! シングルバイト文字から別形式の文字型への変換
    static value_type widen( char ch )
    {
      return ::toppers::widen< value_type >( ch );
    }

    //! size 関数内で使用する累算処理
    static size_type add_line_size( size_type value, line_buf const& buf )
    {
      return value + buf.buf.size();
    }

    //! Directive のデフォルト処理
    static void append_directive( container& cont, line_buf& buf )
    {
      cont.push_back( buf );
      ++buf.line.line;
      buf.buf.clear();
    }
  private:
    container container_;
    line_type line_;
  };

  typedef basic_text< std::vector, char > text;

  inline text_line const& get_text_line( text::const_iterator iter )
  {
    return iter.line();
  }

  inline text_line const& get_text_line( text::iterator iter )
  {
    return iter.line();
  }

}

#endif  // ! TOPPERS_TEXT_HPP_
