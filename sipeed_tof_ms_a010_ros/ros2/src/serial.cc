// private headers
#include "serial.hh"

#include <iostream>
#include <string>
#include <vector>

#ifndef WIN32
#define MS_A010_DEFAULT_SERIAL_DEV_PATH "/dev/ttyUSB0"
#else
#error "NOT SUPPORTED WIN32 NOW"
#define MS_A010_DEFAULT_SERIAL_DEV_PATH "COMx"
#endif

Serial::Serial() : Serial(MS_A010_DEFAULT_SERIAL_DEV_PATH) {}

Serial::Serial(const std::string &dev_path) : dev_path(dev_path) {
  if (this->dev_path.empty()) {
    std::cerr << "Error: no dev path provided" << std::endl;
    return;
  }
  if (NULL == (this->priv = this->configure_serial())) {
    std::cerr << "Error: failed to configure serial device" << std::endl;
    return;
  }
}

#ifndef WIN32
Serial::~Serial() { delete (int *)this->priv; }
#else
#endif

#ifndef WIN32
// Linux headers
#include <errno.h>      // Error integer and strerror() function
#include <sys/fcntl.h>  // Contains file controls like O_RDWR
#include <unistd.h>     // write(), read(), close()
static inline int serial_setup(int serial_port, unsigned int baudrate);

void *Serial::configure_serial() {
  int fd = -1;

  if ((fd = open(this->dev_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    std::cerr << "path for serial can't open" << std::endl;
    return NULL;
  }
  fcntl(fd, F_SETFL, 0);

  if (serial_setup(fd, 115200u) < 0) {
    std::cerr << "setup serial failed" << std::endl;
    close(fd);
    return NULL;
  }

  return (new int(fd));
}

const std::vector<uint8_t> Serial::readBytes() const {
  uint8_t read_buf[16 * 1024];
  ssize_t bytes_readed = -1;
  if ((bytes_readed = read(*(int *)this->priv, read_buf, sizeof(read_buf))) <
      0) {
    bytes_readed = 0;
  }
  return std::vector<uint8_t>(read_buf, read_buf + bytes_readed);
}

void Serial::writeBytes(const std::vector<uint8_t> &vec) const {
  write(*(int *)this->priv, vec.cbegin().base(), vec.size());
}

// C library headers
#include <string.h>
// Linux headers
#include <sys/ioctl.h>
#include <termios.h>  // Contains POSIX terminal control definitions

/* 115200, 8, N, 1 */
static inline int serial_setup(int serial_port, unsigned int baudrate) {
  struct termios tty;
  // Read in existing settings, and handle any error
  if (tcgetattr(serial_port, &tty) != 0) {
    std::cerr << "Error" << errno << "from tcgetattr: " << strerror(errno)
              << std::endl;
    return -1;
  }
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                           // communication (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all the size bits, then use one of the
                           // statements below
  tty.c_cflag |= CS8;      // 8 bits per byte (most common)
  tty.c_cflag &=
      ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      (CREAD | CLOCAL);  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);  // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                          // (e.g. newline chars)
  tty.c_oflag &=
      ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as
                         // soon as any data is received.

  cfsetspeed(&tty, baudrate);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    std::cerr << "Error" << errno << "from tcsetattr: " << strerror(errno)
              << std::endl;
    return -1;
  }
  return 0;
}
#else
#endif

#if TEST_FUNC_ENABLE
static int test() {
  Serial ser{};
  std::vector<uint8_t> vec;
  std::string s;

  bool cap = false;

  char curr_key;
  while (1) {
    std::cin >> curr_key;
    std::cout << "[handler] curr_key: " << curr_key << std::endl;
    switch (curr_key) {
      case 'a':
        ser << "AT\r";
        break;
      case 's':
        ser << "AT+DISP=3\r";
        cap = true;
        break;
      case 'p':
        ser << "AT+DISP=1\r";
        ser >> s;
        while (!s.empty()) {
          ser >> s;
        }
        cap = false;
        break;
      case 'c':
        ser << "AT+COEFF?\r";
        break;
      case 'w':
        std::cout << std::string(vec.begin(), vec.end()) << std::endl;
        std::vector<uint8_t>().swap(vec);
        break;
    }

    ser >> s;
    if (cap) {
      std::cout << "vec size: " << vec.size() << std::endl;
      vec.insert(vec.end(), s.begin(), s.end());
    } else {
      std::cout << s << std::endl;
    }
  }

  return 0;
}
#endif

// int main(int argc, char const *argv[]) {
//   test();
//   return 0;
// }
