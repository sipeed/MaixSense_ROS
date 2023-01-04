#include <string>
#include <vector>

class Serial {
 private:
  const std::string &dev_path;
  void *priv = NULL;

  void *configure_serial();
  const std::vector<uint8_t> readBytes() const;
  void writeBytes(const std::vector<uint8_t> &vec) const;

 public:
  Serial();
  Serial(const std::string &dev_path);
  ~Serial();

  inline Serial &operator>>(std::string &s) {
    std::vector<uint8_t> v = this->readBytes();
    std::string(v.cbegin(), v.cend()).swap(s);
    return *this;
  }

  inline Serial &operator<<(const std::string &s) {
    this->writeBytes(std::vector<uint8_t>(s.cbegin(), s.cend()));
    return *this;
  }
};