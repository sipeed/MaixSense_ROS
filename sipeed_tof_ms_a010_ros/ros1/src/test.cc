#include <deque>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "cJSON.h"
#include "frame_struct.h"
#include "serial.hh"

extern frame_t *handle_process(std::string s);

using namespace std;
static int test() {
  Serial ser("/dev/ttyUSB1");
  string s;

  ser << "AT\r";
  ser >> s;
  if (s.compare("OK\r\n")) {
    // not this serial port
    return -1;
  }

  ser << "AT+COEFF?\r";
  ser >> s;
  if (s.compare("+COEFF=1\r\nOK\r\n")) {
    // not this serial port
    return -1;
  }
  ser >> s;
  // cout << s << endl;
  cJSON *cparms = cJSON_ParseWithLength((const char *)s.c_str(), s.length());
  uint32_t tmp;
  tmp = cJSON_GetObjectItem(cparms, "fx")->valueint;
  cout << "fx: " << (tmp >> 18) << "." << (tmp & 0x3ffff) << endl;
  tmp = cJSON_GetObjectItem(cparms, "fy")->valueint;
  cout << "fy: " << (tmp >> 18) << "." << (tmp & 0x3ffff) << endl;
  tmp = cJSON_GetObjectItem(cparms, "u0")->valueint;
  cout << "u0: " << (tmp >> 18) << "." << (tmp & 0x3ffff) << endl;
  tmp = cJSON_GetObjectItem(cparms, "v0")->valueint;
  cout << "v0: " << (tmp >> 18) << "." << (tmp & 0x3ffff) << endl;

  /* do not delete it. It is waiting */
  ser >> s;
  // usleep(2600);

  uint16_t times = 0;

  ser << "AT+DISP=3\r";
  ser >> s;
  if (s.compare("OK\r\n")) {
    // not this serial port
    return -1;
  }
  for (ser >> s; !s.empty(); ser >> s) {
    frame_t *f = handle_process(s);
    if (!f) continue;

    cout << f << " ";
    cv::Mat src(f->frame_head.resolution_rows, f->frame_head.resolution_cols,
                CV_8UC1, f->payload);

    stringstream fmt;
    fmt << "./" << times << ".jpg";
    cv::imwrite(fmt.str(), src);

    free(f);
    times += 1;
    cout << endl;

    if (times == 4) {
      ser << "AT+DISP=1\r";
    }
  }

  cout << "end" << endl;
  return 0;
}

int main(int argc, char const *argv[]) {
  test();
  return 0;
}
