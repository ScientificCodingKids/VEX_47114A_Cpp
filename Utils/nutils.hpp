#include <vector>
#include <cstdio>
#include <cstdarg>

template<class T>
class ScrollingScreen {
  public:
    ScrollingScreen(int firstRow=1, int maxRows=12): 
      _firstRow(firstRow), _maxRows(maxRows), _rows() {;}
    // LCD screen row is indexed from 1, not 0

    void print(const char* s...);
  private:
    int _firstRow;
    int _maxRows;
    std::vector<std::string> _rows;
};


template<class T>
void ScrollingScreen<T>::print(const char* fmt...) {
  // todo: current implementation uses unbounded memory
  // need switch to LILO queue only keeping last K elems.
  char buf[1024];

  va_list args;
  va_start(args, fmt);

  vsnprintf(buf, 1024, fmt, args);
  _rows.push_back(std::string(buf));

  int K = this->_maxRows;

  int n = _rows.size() - K;

  if (n < 0) n = 0;

  int m =  _rows.size();

  if (m > K) m = K;

  Brain.Screen.clearScreen();

  for (unsigned int i=0; i<m; ++i) {
    Brain.Screen.setCursor(this->_firstRow + i, 1);
    Brain.Screen.print( _rows[n+i].c_str() );
    Brain.Screen.newLine();
  }

  va_end(args);
}
