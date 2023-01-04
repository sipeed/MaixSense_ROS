#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "cJSON.h"
#include "frame_struct.h"
#include "serial.hh"

extern frame_t *handle_process(std::string s);

using namespace std::chrono_literals;

class SipeedTOF_MSA010_Publisher : public rclcpp::Node {
#define ser (*pser)
 private:
  Serial *pser;
  float uvf_parms[4];

 public:
  SipeedTOF_MSA010_Publisher() : Node("sipeed_tof_ms_a010") {
    std::string s;
    this->declare_parameter("device", "/dev/ttyUSB0");
    rclcpp::Parameter device_param = this->get_parameter("device");
    s = device_param.as_string();
    std::cout << "use device: " << s << std::endl;
    pser = new Serial(s);

    ser << "AT\r";
    ser >> s;
    if (s.compare("OK\r\n")) {
      // not this serial port
      return;
    }

    ser << "AT+COEFF?\r";
    ser >> s;
    if (s.compare("+COEFF=1\r\nOK\r\n")) {
      // not this serial port
      return;
    }

    s = s.substr(14, s.length() - 14);
    if (s.length() == 0) {
      ser >> s;
    }
    // cout << s << endl;
    cJSON *cparms = cJSON_ParseWithLength((const char *)s.c_str(), s.length());
    uint32_t tmp;
    uvf_parms[0] =
        ((float)((cJSON_GetObjectItem(cparms, "fx")->valueint) / 262144.0f));
    uvf_parms[1] =
        ((float)((cJSON_GetObjectItem(cparms, "fy")->valueint) / 262144.0f));
    uvf_parms[2] =
        ((float)((cJSON_GetObjectItem(cparms, "u0")->valueint) / 262144.0f));
    uvf_parms[3] =
        ((float)((cJSON_GetObjectItem(cparms, "v0")->valueint) / 262144.0f));
    std::cout << "fx: " << uvf_parms[0] << std::endl;
    std::cout << "fy: " << uvf_parms[1] << std::endl;
    std::cout << "u0: " << uvf_parms[2] << std::endl;
    std::cout << "v0: " << uvf_parms[3] << std::endl;

    /* do not delete it. It is waiting */
    ser >> s;

    ser << "AT+DISP=3\r";
    ser >> s;
    if (s.compare("OK\r\n")) {
      // not this serial port
      return;
    }

    publisher_depth =
        this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
    publisher_pointcloud =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    timer_ = this->create_wall_timer(
        30ms, std::bind(&SipeedTOF_MSA010_Publisher::timer_callback, this));
  }

  ~SipeedTOF_MSA010_Publisher() {}

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      publisher_pointcloud;

  void timer_callback() {
    std::string s;
    std::stringstream sstream;
    frame_t *f;
  _more:
    ser >> s;
    if (s.empty()) {
      return;
    }
    f = handle_process(s);
    if (!f) {
      goto _more;
    }
    // cout << f << endl;
    uint8_t rows, cols, *depth;
    rows = f->frame_head.resolution_rows;
    cols = f->frame_head.resolution_cols;
    depth = f->payload;
    cv::Mat md(rows, cols, CV_8UC1, depth);

    sstream << md.size();

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "tof";

    sensor_msgs::msg::Image msg_depth =
        *cv_bridge::CvImage(header, "mono8", md).toImageMsg().get();
    RCLCPP_INFO(this->get_logger(), "Publishing: depth:%s",
                sstream.str().c_str());
    publisher_depth->publish(msg_depth);

    sensor_msgs::msg::PointCloud2 pcmsg;
    pcmsg.header = header;
    pcmsg.height = rows;
    pcmsg.width = cols;
    pcmsg.is_bigendian = false;
    pcmsg.point_step = 16;
    pcmsg.row_step = pcmsg.point_step * rows;
    pcmsg.is_dense = false;
    pcmsg.fields.resize(pcmsg.point_step / 4);
    pcmsg.fields[0].name = "x";
    pcmsg.fields[0].offset = 0;
    pcmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[0].count = 1;
    pcmsg.fields[1].name = "y";
    pcmsg.fields[1].offset = 4;
    pcmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[1].count = 1;
    pcmsg.fields[2].name = "z";
    pcmsg.fields[2].offset = 8;
    pcmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[2].count = 1;
    pcmsg.fields[3].name = "rgb";
    pcmsg.fields[3].offset = 12;
    pcmsg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    pcmsg.fields[3].count = 1;

    float fox = uvf_parms[0];
    float foy = uvf_parms[1];
    float u0 = uvf_parms[2];
    float v0 = uvf_parms[3];

    pcmsg.data.resize((pcmsg.height) * (pcmsg.width) * (pcmsg.point_step),
                      0x00);
    uint8_t *ptr = pcmsg.data.data();
    for (int j = 0; j < pcmsg.height; j++)
      for (int i = 0; i < pcmsg.width; i++) {
        float cx = (((float)i) - u0) / fox;
        float cy = (((float)j) - v0) / foy;
        float dst = ((float)depth[j * (pcmsg.width) + i]) / 1000;
        float x = dst * cx;
        float y = dst * cy;
        float z = dst;

        *((float *)(ptr + 0)) = x;
        *((float *)(ptr + 4)) = y;
        *((float *)(ptr + 8)) = z;
        const uint8_t *color = color_lut_jet[depth[j * (pcmsg.width) + i]];
        uint32_t color_r = color[0];
        uint32_t color_g = color[1];
        uint32_t color_b = color[2];
        *((uint32_t *)(ptr + 12)) =
            (color_r << 16) | (color_g << 8) | (color_b << 0);
        ptr += pcmsg.point_step;
      }
    publisher_pointcloud->publish(pcmsg);

    free(f);
  }

  const uint8_t color_lut_jet[256][3] = {
      {128, 0, 0},     {132, 0, 0},     {136, 0, 0},     {140, 0, 0},
      {144, 0, 0},     {148, 0, 0},     {152, 0, 0},     {156, 0, 0},
      {160, 0, 0},     {164, 0, 0},     {168, 0, 0},     {172, 0, 0},
      {176, 0, 0},     {180, 0, 0},     {184, 0, 0},     {188, 0, 0},
      {192, 0, 0},     {196, 0, 0},     {200, 0, 0},     {204, 0, 0},
      {208, 0, 0},     {212, 0, 0},     {216, 0, 0},     {220, 0, 0},
      {224, 0, 0},     {228, 0, 0},     {232, 0, 0},     {236, 0, 0},
      {240, 0, 0},     {244, 0, 0},     {248, 0, 0},     {252, 0, 0},
      {255, 0, 0},     {255, 4, 0},     {255, 8, 0},     {255, 12, 0},
      {255, 16, 0},    {255, 20, 0},    {255, 24, 0},    {255, 28, 0},
      {255, 32, 0},    {255, 36, 0},    {255, 40, 0},    {255, 44, 0},
      {255, 48, 0},    {255, 52, 0},    {255, 56, 0},    {255, 60, 0},
      {255, 64, 0},    {255, 68, 0},    {255, 72, 0},    {255, 76, 0},
      {255, 80, 0},    {255, 84, 0},    {255, 88, 0},    {255, 92, 0},
      {255, 96, 0},    {255, 100, 0},   {255, 104, 0},   {255, 108, 0},
      {255, 112, 0},   {255, 116, 0},   {255, 120, 0},   {255, 124, 0},
      {255, 128, 0},   {255, 132, 0},   {255, 136, 0},   {255, 140, 0},
      {255, 144, 0},   {255, 148, 0},   {255, 152, 0},   {255, 156, 0},
      {255, 160, 0},   {255, 164, 0},   {255, 168, 0},   {255, 172, 0},
      {255, 176, 0},   {255, 180, 0},   {255, 184, 0},   {255, 188, 0},
      {255, 192, 0},   {255, 196, 0},   {255, 200, 0},   {255, 204, 0},
      {255, 208, 0},   {255, 212, 0},   {255, 216, 0},   {255, 220, 0},
      {255, 224, 0},   {255, 228, 0},   {255, 232, 0},   {255, 236, 0},
      {255, 240, 0},   {255, 244, 0},   {255, 248, 0},   {255, 252, 0},
      {254, 255, 1},   {250, 255, 6},   {246, 255, 10},  {242, 255, 14},
      {238, 255, 18},  {234, 255, 22},  {230, 255, 26},  {226, 255, 30},
      {222, 255, 34},  {218, 255, 38},  {214, 255, 42},  {210, 255, 46},
      {206, 255, 50},  {202, 255, 54},  {198, 255, 58},  {194, 255, 62},
      {190, 255, 66},  {186, 255, 70},  {182, 255, 74},  {178, 255, 78},
      {174, 255, 82},  {170, 255, 86},  {166, 255, 90},  {162, 255, 94},
      {158, 255, 98},  {154, 255, 102}, {150, 255, 106}, {146, 255, 110},
      {142, 255, 114}, {138, 255, 118}, {134, 255, 122}, {130, 255, 126},
      {126, 255, 130}, {122, 255, 134}, {118, 255, 138}, {114, 255, 142},
      {110, 255, 146}, {106, 255, 150}, {102, 255, 154}, {98, 255, 158},
      {94, 255, 162},  {90, 255, 166},  {86, 255, 170},  {82, 255, 174},
      {78, 255, 178},  {74, 255, 182},  {70, 255, 186},  {66, 255, 190},
      {62, 255, 194},  {58, 255, 198},  {54, 255, 202},  {50, 255, 206},
      {46, 255, 210},  {42, 255, 214},  {38, 255, 218},  {34, 255, 222},
      {30, 255, 226},  {26, 255, 230},  {22, 255, 234},  {18, 255, 238},
      {14, 255, 242},  {10, 255, 246},  {6, 255, 250},   {2, 255, 254},
      {0, 252, 255},   {0, 248, 255},   {0, 244, 255},   {0, 240, 255},
      {0, 236, 255},   {0, 232, 255},   {0, 228, 255},   {0, 224, 255},
      {0, 220, 255},   {0, 216, 255},   {0, 212, 255},   {0, 208, 255},
      {0, 204, 255},   {0, 200, 255},   {0, 196, 255},   {0, 192, 255},
      {0, 188, 255},   {0, 184, 255},   {0, 180, 255},   {0, 176, 255},
      {0, 172, 255},   {0, 168, 255},   {0, 164, 255},   {0, 160, 255},
      {0, 156, 255},   {0, 152, 255},   {0, 148, 255},   {0, 144, 255},
      {0, 140, 255},   {0, 136, 255},   {0, 132, 255},   {0, 128, 255},
      {0, 124, 255},   {0, 120, 255},   {0, 116, 255},   {0, 112, 255},
      {0, 108, 255},   {0, 104, 255},   {0, 100, 255},   {0, 96, 255},
      {0, 92, 255},    {0, 88, 255},    {0, 84, 255},    {0, 80, 255},
      {0, 76, 255},    {0, 72, 255},    {0, 68, 255},    {0, 64, 255},
      {0, 60, 255},    {0, 56, 255},    {0, 52, 255},    {0, 48, 255},
      {0, 44, 255},    {0, 40, 255},    {0, 36, 255},    {0, 32, 255},
      {0, 28, 255},    {0, 24, 255},    {0, 20, 255},    {0, 16, 255},
      {0, 12, 255},    {0, 8, 255},     {0, 4, 255},     {0, 0, 255},
      {0, 0, 252},     {0, 0, 248},     {0, 0, 244},     {0, 0, 240},
      {0, 0, 236},     {0, 0, 232},     {0, 0, 228},     {0, 0, 224},
      {0, 0, 220},     {0, 0, 216},     {0, 0, 212},     {0, 0, 208},
      {0, 0, 204},     {0, 0, 200},     {0, 0, 196},     {0, 0, 192},
      {0, 0, 188},     {0, 0, 184},     {0, 0, 180},     {0, 0, 176},
      {0, 0, 172},     {0, 0, 168},     {0, 0, 164},     {0, 0, 160},
      {0, 0, 156},     {0, 0, 152},     {0, 0, 148},     {0, 0, 144},
      {0, 0, 140},     {0, 0, 136},     {0, 0, 132},     {0, 0, 128}};
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SipeedTOF_MSA010_Publisher>());
  rclcpp::shutdown();

  return 0;
}
