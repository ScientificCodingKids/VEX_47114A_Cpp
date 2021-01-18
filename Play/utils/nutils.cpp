#include "nutils.hpp"
#include "vex.h"

//#include <algorithm>
#include <cstdarg>

void ScrollingScreen::print(const char* fmt...) {
  char buf[1024];

  va_list args;
  va_start(args, fmt);

  vsnprintf(buf, 1024, fmt, args);
  _rows.push_back(std::string(buf));

  int K = 12;

  int n = _rows.size() - K;

  if (n < 0) n = 0;

  int m =  _rows.size();

  if (m > K) m = K;

  Brain.Screen.clearScreen();

  for (unsigned int i=0; i<m; ++i) {
    Brain.Screen.print( _rows[n+i].c_str() );
    Brain.Screen.newLine();
  }

  va_end(args);
}
