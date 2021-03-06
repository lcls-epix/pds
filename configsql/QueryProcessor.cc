//--------------------------------------------------------------------------
// File and Version Information:
//  $Id$
//
// Description:
//  Class QueryProcessor...
//
// Author List:
//      Igor Gaponenko
//
//------------------------------------------------------------------------

//-----------------------
// This Class's Header --
//-----------------------

#include "pds/configsql/QueryProcessor.hh"

//-----------------
// C/C++ Headers --
//-----------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

//-------------------------------
// Collaborating Class Headers --
//-------------------------------

//-----------------------------------------------------------------------
// Local Macros, Typedefs, Structures, Unions and Forward Declarations --
//-----------------------------------------------------------------------

//    ----------------------------------------
//    -- Public Function Member Definitions --
//    ----------------------------------------

using namespace Pds_ConfigDb::Sql;

//----------------
// Constructors --
//----------------

QueryProcessor::QueryProcessor (MYSQL* mysql) :
    m_last_sql   (""),
    m_mysql      (mysql),
    m_res        (0),
    m_row        (0),
    m_num_rows   (0),
    m_next_row   (0),
    m_lengths    (0)
{}

//--------------
// Destructor --
//--------------

QueryProcessor::~QueryProcessor () throw ()
{
    this->reset () ;
}

//-----------
// Methods --
//-----------

void
QueryProcessor::execute (const std::string& sql) throw (DatabaseError)
{
    this->reset () ;

    m_last_sql = sql ;

    //    printf("mysql_real_query: %s [%d]\n", sql.c_str(), sql.length());

    if (mysql_real_query (m_mysql, sql.c_str(), sql.length()))
        throw DatabaseError( std::string( "error in mysql_real_query('"+m_last_sql+"'): " ) + mysql_error(m_mysql)) ;

    m_res = mysql_store_result (m_mysql);
    if (!m_res)
        throw DatabaseError( std::string( "error in mysql_store_result(): " ) + mysql_error(m_mysql)) ;

    m_num_rows  = mysql_num_rows     (m_res) ;

    MYSQL_FIELD* fields     = mysql_fetch_fields (m_res) ;
    unsigned int num_fields = mysql_num_fields   (m_res) ;

    //printf("QueryProcessor::execute(%s): get %d rows %d fields\n", sql.c_str(), (int) m_num_rows, num_fields); //!!debug
    for (unsigned int i = 0; i < num_fields; i++)
    {
        //printf("  field %d: %s \n", (int) i, fields[i].name); //!!debug
        m_columns[fields[i].name] = i ;
    }

}

const unsigned long
QueryProcessor::num_rows () const throw (DatabaseError)
{
    if (!this->ready2process())
        throw DatabaseError ("no query made to analyze its result set") ;
    return m_num_rows ;
}

bool
QueryProcessor::empty () const throw (DatabaseError)
{
    if (!this->ready2process())
        throw DatabaseError ("no query made to analyze its result set") ;

    return !m_num_rows;
}

bool
QueryProcessor::next_row () throw (DatabaseError)
{
    if (!this->ready2process())
        throw DatabaseError ("no query made to analyze its result set") ;

    // Check if the result set is empty or there are no more rows
    // in the set to process.
    //
    if (m_next_row >= m_num_rows) return false ;

    m_row = mysql_fetch_row (m_res) ;
    if (!m_row)
        throw DatabaseError( std::string( "error in mysql_fetch_row(): " ) + mysql_error(m_mysql)) ;

    m_lengths = mysql_fetch_lengths (m_res) ;

    m_next_row++ ;

    return true ;
}

void
QueryProcessor::get (unsigned&          val,
                     const std::string& col_name,
                     const bool         nullIsAllowed) throw (WrongParams,
                                                              DatabaseError)
{
    const Cell c = this->cell (col_name) ;
    if (!c.ptr) {
        if (nullIsAllowed) {
            val = 0 ;
            return ;
        } else
            throw DatabaseError ("NULL value in column: "+col_name) ;
    }
    char* end_ptr = 0;
    val = strtoul (c.ptr, &end_ptr, 10) ;
    if (!end_ptr)
        throw DatabaseError ("failed to interpret a value in column: "+col_name) ;
}

void
QueryProcessor::get_time (time_t&            val,
                          const std::string& col_name,
                          const bool         nullIsAllowed) throw (WrongParams,
                                                                   DatabaseError)
{
    const Cell c = this->cell (col_name) ;
    if (!c.ptr) {
        if (nullIsAllowed) {
            val = 0 ;
            return ;
        } else
            throw DatabaseError ("NULL value in column: "+col_name) ;
    }
    struct tm tm;
    memset(&tm,0,sizeof(tm));
    char* end_ptr = strptime(c.ptr,"%Y-%m-%d %T",&tm);
    if (end_ptr != c.ptr+c.len)
      throw DatabaseError (std::string("Failed to interpret time: ")+
                           std::string(c.ptr,c.len));
    val = mktime(&tm);
}

void
QueryProcessor::get (long&              val,
                     const std::string& col_name,
                     const bool         nullIsAllowed) throw (WrongParams,
                                                              DatabaseError)
{
    const Cell c = this->cell (col_name) ;
    if (!c.ptr) {
        if (nullIsAllowed) {
            val = 0 ;
            return ;
        } else
            throw DatabaseError ("NULL value in column: "+col_name) ;
    }
    char* end_ptr = 0;
    val = strtol (c.ptr, &end_ptr, 10) ;
    if (!end_ptr)
        throw DatabaseError ("failed to interpret a value in column: "+col_name) ;
}

void
QueryProcessor::get (uint64_t&          val,
                     const std::string& col_name,
                     const bool         nullIsAllowed) throw (WrongParams,
                                                              DatabaseError)
{
    const Cell c = this->cell (col_name) ;
    if (!c.ptr) {
        if (nullIsAllowed) {
            val = 0 ;
            return ;
        } else
            throw DatabaseError ("NULL value in column: "+col_name) ;
    }
    char* end_ptr = 0;
    val = strtoull (c.ptr, &end_ptr, 10) ;
    if (!end_ptr)
        throw DatabaseError ("failed to interpret a value in column: "+col_name) ;
}

void
QueryProcessor::get (Pds::TypeId&       val,
                     const std::string& col_name,
                     const bool         nullIsAllowed) throw (WrongParams,
                                                              DatabaseError)
{
  get(reinterpret_cast<unsigned&>(val),col_name,nullIsAllowed);
}

void
QueryProcessor::get (double&            val,
                     const std::string& col_name,
                     const bool         nullIsAllowed) throw (WrongParams,
                                                              DatabaseError)
{
    const Cell c = this->cell (col_name) ;
    if (!c.ptr) {
        if (nullIsAllowed) {
            val = 0.0 ;
            return ;
        } else
            throw DatabaseError ("NULL value in column: "+col_name) ;
    }
    char* end_ptr = 0;
    val = strtod (c.ptr, &end_ptr) ;
    if (!end_ptr)
        throw DatabaseError ("failed to interpret a value in column: "+col_name) ;
}

void
QueryProcessor::get (std::string&       str,
                     const std::string& col_name,
                     const bool         nullIsAllowed) throw (WrongParams,
                                                              DatabaseError)
{
    const Cell c = this->cell (col_name) ;
    if (!c.ptr) {
        if (nullIsAllowed) {
            str = "" ;
            return ;
        } else
            throw DatabaseError ("NULL value in column: "+col_name) ;
    }
    str = std::string (c.ptr, c.len) ;
}

void
QueryProcessor::get (void*              blob,
                     int&               blob_len,
                     unsigned           max_len,
                     const std::string& col_name,
                     const bool         nullIsAllowed) throw (WrongParams,
                                                              DatabaseError)
{
    const Cell c = this->cell (col_name) ;
    if (!c.ptr) {
        if (nullIsAllowed) {
            blob_len = 0;
            return ;
        } else
            throw DatabaseError ("NULL value in column: "+col_name) ;
    }
    blob_len = c.len < max_len ? c.len : max_len;
    memcpy(blob, c.ptr, blob_len);
}

QueryProcessor::Cell
QueryProcessor::cell (const std::string& col_name) throw (WrongParams)
{
    if (!this->ready2process())
        throw WrongParams ("no query made to analyze its result set") ;

    if (!m_num_rows)
        throw WrongParams ("the result set is empty") ;

    std::map<std::string, unsigned int >::const_iterator itr = m_columns.find (col_name) ;
    if (itr == m_columns.end()) {
        for(itr = m_columns.begin();itr != m_columns.end(); itr++)
          printf("%s\n",itr->first.c_str());
        throw WrongParams ("no such field as '"+col_name+"' in the result set of query '"+m_last_sql+"'") ;
    }

    const unsigned int col_idx = itr->second ;
    return QueryProcessor::Cell (
        m_row     [col_idx],
        m_lengths [col_idx] ) ;
}

void
QueryProcessor::reset ()
{
    m_last_sql   = "" ;
    if (m_res) {
        mysql_free_result (m_res) ;
        m_res = 0 ;
    }
    m_row = 0 ;

    m_num_rows   = 0 ;
    m_next_row   = 0 ;

    m_columns.clear () ;

    m_lengths    = 0 ;
}

bool
QueryProcessor::ready2process () const
{
    return 0 != m_res ;
}


