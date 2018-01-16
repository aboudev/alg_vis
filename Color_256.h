#ifndef COLOR_256_H
#define COLOR_256_H

#include <cstddef>

// 256 color cheat sheet
// Source: https://jonasjacek.github.io/colors/

class Color_256
{
public:
  static const unsigned char &r(const std::size_t &i) {
    return m_colors[i][0];
  }

  static const unsigned char &g(const std::size_t &i) {
    return m_colors[i][1];
  }

  static const unsigned char &b(const std::size_t &i) {
    return m_colors[i][2];
  }

private:
  static const unsigned char m_colors[256][3];
};

#endif // COLOR_256_H
