#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace boost;

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace std::literals::chrono_literals;

#define DEFAULT_USRT_PORT "/dev/ttyUSB1"
#define DEFAULT_READ_TIMEDOUT 200

#define LOG_LINE                                                               \
  do {                                                                         \
    std::cout << "["                                                           \
              << duration_cast<milliseconds>(high_resolution_clock::now() -    \
                                             _start)                           \
                         .count() *                                            \
                     0.001                                                     \
              << "][" << __LINE__ << "][" << std::this_thread::get_id() << ']' \
              << std::endl;                                                    \
  } while (0)
static std::chrono::time_point<std::chrono::high_resolution_clock> _start =
    high_resolution_clock::now();

class msa010 {
 private:
  const char* _serial_path;
  std::size_t _read_timedout;
  asio::io_service _ioc;
  asio::serial_port _sp;
  asio::deadline_timer _timeout;
  // asio::executor_work_guard<asio::io_service::executor_type> _worker;
  std::unique_ptr<asio::io_service::work> _worker;
  std::thread _ioc_run_thread;

 public:
  msa010() : msa010(DEFAULT_USRT_PORT) {}
  msa010(const char* __serial_path)
      : _serial_path(__serial_path),
        _read_timedout(DEFAULT_READ_TIMEDOUT),
        _ioc(),
        _sp(_ioc),
        _timeout(_ioc),
        // _worker(asio::make_work_guard(_ioc)),
        _worker(new asio::io_service::work(_ioc)),
        _ioc_run_thread(std::bind([](asio::io_service& _ioc) { _ioc.run(); },
                                  std::ref(_ioc))) {}

  ~msa010() {
    _worker.reset();
    _ioc_run_thread.join();
  };

  inline bool is_connected() { return _sp.is_open(); }

  void keep_connect(const std::function<bool(void)>& f) {
    boost::system::error_code ec;
    if (_sp.is_open()) return;
    do {
      ec = try_connect();
      // std::cout << "[INFO] Try connect to " << _serial_path << "..."
      //           << " Result: " << ec.message() << std::endl;
    } while (f() && ec);
  };

  void keep_connect() {
    keep_connect([] {
      std::this_thread::sleep_for(1s);
      return true;
    });
    // boost::system::error_code ec;
    // if (_sp.is_open()) return;
    // do {
    //   ec = try_connect();
    //   // std::cout << "[INFO] Try connect to " << _serial_path << "..."
    //   //           << " Result: " << ec.message() << std::endl;
    //   std::this_thread::sleep_for(1s);
    // } while (ec);
  };

  void set_read_timedout(std::size_t timeout) {
    this->_read_timedout = timeout;
  }

  template <typename MutableBufferSequence>
  std::size_t read_timedout(const MutableBufferSequence& buffers) {
    return read_timedout(buffers, _read_timedout);
  }

  template <typename MutableBufferSequence>
  std::size_t read_timedout(const MutableBufferSequence& buffers,
                            std::size_t ms) {
    bool is_timeout = false;
    bool had_data = false;
    std::size_t transferred = 0;

    _timeout.expires_from_now(boost::posix_time::milliseconds(ms));
    _timeout.async_wait([this, &had_data,
                         &is_timeout](const boost::system::error_code& error) {
      if (error) {
        switch (error.value()) {
          case boost::asio::error::operation_aborted: /* Operation canceled */ {
            // Data was read and this timeout was canceled
            // std::cout << "[WARN][Timeout] Cancelled, Has Data..." <<
            // std::endl;
            had_data = true;
            return;
          } break;
          default:
            break;
        }
      }

      // timeout
      if (this->_sp.is_open()) {
        // std::cout << "[WARN][Timeout] Was Fired, No Data..." << std::endl;
        this->_sp.cancel();  // will cause read_callback to fire with an error
      }
      is_timeout = true;
    });

    _sp.async_read_some(buffers, [this, &transferred](
                                     const boost::system::error_code& error,
                                     std::size_t bytes_transferred) {
      this->_timeout
          .cancel();  // will cause wait_callback to fire with an error
      if (error) {
        // No data was read!
        switch (error.value()) {
          case boost::asio::error::eof: /* End of file */ {
            /* disconnect */
            // std::cout << "[WARN][Serial] End of file..." << std::endl;
            this->_sp.close();
          } break;
          case boost::asio::error::bad_descriptor: /* Bad file descriptor */ {
            // std::cout << "[WARN][Serial] Bad file descriptor..." <<
            // std::endl;
            this->_sp.close();
          } break;
        }
        return;
      }

      transferred = bytes_transferred;
    });

    while (!(is_timeout || had_data)) {
      std::this_thread::sleep_for(20ms);
    }

    return transferred;
  }

  std::size_t read_some(std::string& s) {
#define MAX_SIZE ((25 * 25 + 22) * 4 * 4)
    std::size_t len;
    boost::system::error_code ec;

    s.reserve(MAX_SIZE);
    len = this->_sp.read_some(asio::buffer(s, MAX_SIZE), ec);
    if (ec) this->_sp.close();

    return len;
  }

  inline msa010& operator<<(const std::string& s) {
    boost::system::error_code ec;
    this->_sp.write_some(asio::buffer(s), ec);
    if (ec) this->_sp.close();
    return *this;
  }

  inline msa010& operator>>(std::string& s) {
#define MAX_SIZE ((25 * 25 + 22) * 4 * 4)
    static char b[MAX_SIZE];
    std::size_t len;

    s.clear();
    len = read_timedout(asio::buffer(b));
    if (len) {
      s.append(b, len);
    }

    return *this;
  }

  friend inline std::unique_ptr<msa010>& operator<<(
      std::unique_ptr<msa010>& a010, const std::string& s) {
    *a010 << s;
    return a010;
  }

  friend inline std::unique_ptr<msa010>& operator>>(
      std::unique_ptr<msa010>& a010, std::string& s) {
    *a010 >> s;
    return a010;
  }

 private:
  boost::system::error_code try_connect() {
    using sp = boost::asio::serial_port;

    boost::system::error_code ec;
    _sp.open(_serial_path, ec);
    if (ec) return ec;
    try {
      // 波特率
      _sp.set_option(sp::baud_rate(115200));
      // 字符大小
      _sp.set_option(sp::character_size(8));
      // 奇偶校验，可以为serial_port::parity::none / odd / even。
      _sp.set_option(sp::parity(sp::parity::none));
      // 停止位，可以为serial_port::stop_bits::one / onepointfive /two
      _sp.set_option(sp::stop_bits(sp::stop_bits::one));
      // 流量控制，可以为serial_port::flow_control::type，enum类型，可以是none
      _sp.set_option(sp::flow_control(sp::flow_control::none));
    } catch (boost::system::system_error const& ex) {
      ec = ex.code();
      _sp.close();
    }
    return ec;
  }
};