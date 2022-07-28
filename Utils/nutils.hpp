#ifndef _nutils_hpp_
#define _nutils_hpp_

#include <deque>
#include <cstdio>
#include <cstdarg>
#include <string>

#include "v5.h"
#include "v5_vcs.h"

using namespace std;
using namespace vex;

class SmartScreen {
  /*
  One V5 brain screen can be divided into multiple "windowed" screens (by rows only; not supported by cols yet)
  User of SmartScreen is required to specify row number for each print
  */
  public:

    SmartScreen(vex::brain::lcd& aV5Screen, int firstRow=1, int maxRows=12): 
      _screen(aV5Screen), _firstRow(firstRow), _maxRows(maxRows) {;}

    // LCD screen row is indexed from 1, not 0
    void setPenColor(color&& c) {this->_screen.setPenColor(c); }

    template <class ... Args>
    void printAt(int row, const char* s, Args ... args);
  protected:
    vex::brain::lcd& _screen;
    int _firstRow;
    int _maxRows;
};  // class SmartScreen


template <class ... Args>
void SmartScreen::printAt(int row, const char* fmt, Args ... args) {
  row = min(row, this->_maxRows);

  int real_row = row + this->_firstRow -1;
  this->_screen.clearLine(real_row);
  this->_screen.setCursor(real_row, 1);
  this->_screen.print(fmt, args...);

}


class RollingScreen {
  /*
  One V5 brain screen can be divided into multiple smart screens (by rows only; not supported by cols yet)
  User of RollingScreen cannot specify which row to print.
  Rather, new row will push all rows up if all visiable rows are used up. 
  This creates rolling (bottom to top) visual effect.
  */
  public:
    RollingScreen(vex::brain::lcd& aV5Screen, int firstRow=1, int maxRows=12): 
      _screen(aV5Screen), _firstRow(firstRow), _maxRows(maxRows), _rows() {;}

    // LCD screen row is indexed from 1, not 0
    void setPenColor(color&& c) {this->_screen.setPenColor(c); }

    template <class ... Args>
    void print(const char* fmt, Args ... args);
  protected:
    vex::brain::lcd& _screen;
    int _firstRow;
    int _maxRows;
    std::deque< std::string > _rows;
};  //class RollingScreen

template<class ... Args>
void RollingScreen::print(const char* fmt, Args ... args) {
  char buf[1024];
  _maxRows = min(_maxRows, 12 - _firstRow +1);
  
  snprintf(buf, 1024, fmt, args...);
  _rows.push_back(std::string(buf));

  while (_rows.size() > _maxRows) {
    _rows.pop_front();
  }

  for (unsigned int i=0; i< _rows.size(); ++i) {
    int realRow = i + _firstRow;
    _screen.clearLine(realRow);
    _screen.setCursor(realRow, 1);
    _screen.print("%d: %s", _rows.size(), _rows[i].c_str() );
  }

}

#endif // _nutils_hpp_