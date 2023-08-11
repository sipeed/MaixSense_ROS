
#include <deque>
#include <iostream>

#include "msa010.hpp"

int main(int argc, char const* argv[]) {
  auto a010 = std::make_unique<msa010>();

  std::deque<std::string> dq;

  std::thread th([&dq] {
    while (true) {
      std::string s;
      std::cin >> s;
      s.push_back('\r');
      dq.push_back(s);
    }
  });

  while (1) {
    a010->keep_connect();

    while (a010->is_connected()) {
      std::string s;
      a010 >> s;
      if (s.size()) {
        std::cerr << s;
        std::cerr << "--------------ONCE--------------" << std::endl;
      }

      if (!dq.empty()) {
        auto s = dq.front();
        a010 << s;
        dq.pop_front();
      }
    }
  }
  return 0;
}
