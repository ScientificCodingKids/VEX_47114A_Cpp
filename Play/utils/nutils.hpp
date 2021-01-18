#include <vector>
#include <cstdio>

class ScrollingScreen {
  public:
    void print(const char* s...);
  private:
    std::vector<std::string> _rows;
};
