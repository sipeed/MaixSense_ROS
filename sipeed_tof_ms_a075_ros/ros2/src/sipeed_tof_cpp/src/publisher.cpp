#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <semaphore.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <evhttp.h>
#include <event2/event.h>
#include <event2/http.h>
#include <event2/bufferevent.h>

#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>

#include "cJSON.h"

#include "process.hpp"

using namespace std::chrono_literals;


const std::string HOST="192.168.233.1";
int PORT=80;
int IMAGECOUNT=0;

int inttriggermode=1;
int deepmode=0;
int deepshift=5;
int irmode=0;
int statusmode=0;
int statusmask=7;
int rgbmode=0;
int rgbres=0;
int expose_time=0;


#define DEBUG_PRINT 1

struct event_base *base;
struct evhttp_connection *conn;
struct evhttp_request *req;

uint8_t requestdata[10*1024*1024];
int retuestdata_size=0;
sem_t sem_request;

void http_request_done(struct evhttp_request *req, void *arg){
    retuestdata_size=-1;
    int s = evbuffer_get_length(req->input_buffer);
    if(DEBUG_PRINT)printf("request got size:%d\n",s);
    if(s<sizeof(requestdata)){
        if(DEBUG_PRINT)printf("copy out\n");
        retuestdata_size = evbuffer_remove(req->input_buffer, requestdata, sizeof(requestdata));
        sem_post(&sem_request);
    }
    // terminate event_base_dispatch()
    event_base_loopbreak((struct event_base *)arg);
}

void init_libevent(){
    base = event_base_new();
    conn = evhttp_connection_base_new(base, NULL, HOST.c_str(), PORT);
    sem_init(&sem_request, 0, -1);
}

int send_request(enum evhttp_cmd_type type,const char* url,void* data,int datasize){
    req = evhttp_request_new(http_request_done, base);
    evhttp_add_header(req->output_headers, "Host", HOST.c_str());
    //evhttp_add_header(req->output_headers, "Connection", "close");

    if(datasize)
    {
        // auto buf=evhttp_request_get_output_buffer(req);
        // char txtbuf[10];
        // sprintf(txtbuf,"%d",datasize);
        // evhttp_add_header(req->output_headers, "Content-Length", txtbuf);
        // printf("buf:%p,add size:%d\n",buf,datasize);
        evbuffer_add(req->output_buffer,data,datasize);
    }
    evhttp_make_request(conn, req, type, url);
    evhttp_connection_set_timeout(req->evcon, 60);

    int ret=event_base_dispatch(base);
    sem_wait(&sem_request);
    return ret;
}

class PointCloudPublisher : public rclcpp::Node
{
  public:
    PointCloudPublisher()
    : Node("sipeed_tof"), count_(0)
    {
        CaliInst = new Tof_Filter(320,240,7);
        CaliInst->TemporalFilter_cfg(0.5);
        CaliInst->set_kernel_size(7);


        init_libevent();
        int ret=send_request(EVHTTP_REQ_GET,"/getinfo",0,0);
        //printf("get ret:%d size:%d\n",ret,retuestdata_size);
        info_t info_all;
        memcpy(&info_all,requestdata,sizeof(info_t));
        uvf_parms = CaliInst->parse_info(&info_all);

        static uint8_t lut_got[65536];
        ret=send_request(EVHTTP_REQ_GET,"/get_lut",0,0);
        printf("get ret:%d size:%d\n",ret,retuestdata_size);
        memcpy(&lut_got,requestdata,sizeof(lut_got));
        CaliInst->set_lut(lut_got);

        ret=send_request(EVHTTP_REQ_GET,"/CameraParms.json",0,0);
        printf("get ret:%d size:%d\n",ret,retuestdata_size);
        cJSON *cparms = cJSON_ParseWithLength((const char*)requestdata,retuestdata_size); 
        //cparms.R_Matrix_data,cparms.T_Vec_data,cparms.Camera_Matrix_data,cparms.Distortion_Parm_data
        float R_data[9];
        float T_data[3];
        float RGB_CM_data[9];
        float D_VEC_data[5];
        cJSON * tempobj = cJSON_GetObjectItem(cparms,"R_Matrix_data");
        for(int i=0;i<9;i++)
            R_data[i]=cJSON_GetArrayItem(tempobj,i)->valuedouble;
        
        tempobj = cJSON_GetObjectItem(cparms,"T_Vec_data");
        for(int i=0;i<3;i++)
            T_data[i]=cJSON_GetArrayItem(tempobj,i)->valuedouble;
        
        tempobj = cJSON_GetObjectItem(cparms,"Camera_Matrix_data");
        for(int i=0;i<9;i++)
            RGB_CM_data[i]=cJSON_GetArrayItem(tempobj,i)->valuedouble;

        tempobj = cJSON_GetObjectItem(cparms,"Distortion_Parm_data");
        for(int i=0;i<5;i++)
            D_VEC_data[i]=cJSON_GetArrayItem(tempobj,i)->valuedouble;
        
        CaliInst->setCameraParm(R_data,T_data,RGB_CM_data,D_VEC_data);

        all_config_t config;
        config.triggermode=1;//0:STOP 1:AUTO 2:SINGLE
        config.deepmode=1;//0:16bit 1:8bit
        config.deepshift=255;//for 8bit mode
        config.irmode=1;//0:16bit 1:8bit
        config.statusmode=1;//0:16bit 1:2bit 2:8bit 3:1bit
        config.statusmask=7;//for 1bit mode 1:1 2:2 4:3
        config.rgbmode=1;//0:YUV 1:JPG
        config.rgbres=0;//0:800*600 1:1600*1200
        config.expose_time=0;
        ret=send_request(EVHTTP_REQ_POST,"/set_cfg",&config,sizeof(all_config_t));
        printf("get ret:%d size:%d\n",ret,retuestdata_size);

        publisher_pc = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
        publisher_rgb = this->create_publisher<sensor_msgs::msg::Image>("rgb", 10);
        publisher_d = this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
        publisher_i = this->create_publisher<sensor_msgs::msg::Image>("intensity", 10);
        publisher_s = this->create_publisher<sensor_msgs::msg::Image>("status", 10);
        timer_ = this->create_wall_timer(30ms, std::bind(&PointCloudPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        // printf("%s:%d\n",__FILE__,__LINE__);
        int ret=send_request(EVHTTP_REQ_GET,"/getdeep",0,0);
        // printf("get ret:%d size:%d\n",ret,retuestdata_size);
        stackframe_old_t oldframe = CaliInst->DecodePkg(requestdata);
        // printf("%s:%d\n",__FILE__,__LINE__);
		auto data_out1 = CaliInst->TemporalFilter((uint16_t*)oldframe.depth);
        // printf("%s:%d\n",__FILE__,__LINE__);
		auto data_out2 = CaliInst->SpatialFilter(data_out1,0);
        // printf("%s:%d\n",__FILE__,__LINE__);
		auto data_out3 = CaliInst->FlyingPointFilter(data_out2,0.03);
        // printf("%s:%d\n",__FILE__,__LINE__);
        auto tempcali=CaliInst->TOF_cali(data_out3,std::vector<uint16_t>((uint16_t*)oldframe.status,(uint16_t*)((uint8_t*)oldframe.status)+sizeof(Image_t)));
		// printf("%s:%d\n",__FILE__,__LINE__);
        auto colormap_temp=CaliInst->MapRGB2TOF(tempcali,std::vector<uint8_t>((uint8_t*)oldframe.rgb,(uint8_t*)((uint8_t*)oldframe.rgb)+sizeof(oldframe.rgb)));
        // printf("%s:%d\n",__FILE__,__LINE__);
        auto tempcali_ir=CaliInst->TOF_cali(std::vector<uint16_t>((uint16_t*)oldframe.ir,(uint16_t*)((uint8_t*)oldframe.ir)+sizeof(Image_t)),std::vector<uint16_t>((uint16_t*)oldframe.status,(uint16_t*)((uint8_t*)oldframe.status)+sizeof(Image_t)));

        std_msgs::msg::Header header;
        if (false)
            header.stamp = this->get_clock()->now();
        else
            header.stamp = rclcpp::Time(oldframe.framestamp*1000);

        if (false)
            header.frame_id = "None";
        else
            header.frame_id = "tof";

        // auto rgbmsg = sensor_msgs::msg::Image();
        Mat mRGB(600, 800, CV_8UC4,(void*)oldframe.rgb);
        Mat md(240, 320, CV_16UC1,(void*)oldframe.depth);
        Mat mi(240, 320, CV_16UC1,(void*)oldframe.ir);
        Mat ms(240, 320, CV_16UC1,(void*)oldframe.status);
        Mat mRGB_bgr;
        cvtColor(mRGB,mRGB_bgr,CV_RGBA2BGR);
        sensor_msgs::msg::Image rgbmsg =
                *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mRGB_bgr)
                .toImageMsg().get();
        sensor_msgs::msg::Image dmsg =
                *cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", md)
                .toImageMsg().get();
        sensor_msgs::msg::Image imsg =
                *cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", mi)
                .toImageMsg().get();
        sensor_msgs::msg::Image smsg =
                *cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", ms)
                .toImageMsg().get();
        rgbmsg.header=header;
        dmsg.header=header;
        imsg.header=header;
        smsg.header=header;
        RCLCPP_INFO(this->get_logger(), "Publishing");
        publisher_rgb->publish(rgbmsg);
        publisher_d->publish(dmsg);
        publisher_i->publish(imsg);
        publisher_s->publish(smsg);

        sensor_msgs::msg::PointCloud2 pcmsg;
        pcmsg.header=header;
        pcmsg.height=240;
        pcmsg.width=320;
        pcmsg.is_bigendian=false;
        pcmsg.point_step=20;
        pcmsg.row_step=pcmsg.point_step*320;
        pcmsg.is_dense=false;
        pcmsg.fields.resize(5);
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
        pcmsg.fields[4].name = "intensity";
        pcmsg.fields[4].offset = 16;
        pcmsg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pcmsg.fields[4].count = 1;

        float fox=uvf_parms[0];
		float foy=uvf_parms[1];
		float u0=uvf_parms[2];
		float v0=uvf_parms[3];
        // printf("%f,%f,%f,%f\n",fox,foy,u0,v0);
        pcmsg.data.resize(320*240 *20, 0x00);
        uint8_t *ptr = pcmsg.data.data();
        // for (int j=0;j<240;j++){
        //     for (int i=0;i<320;i++)
        //         std::cout<<tempcali[j*320+i]<<",";
        //         std::cout<<std::endl;
        // }
        
        for (int j=0;j<240;j++)
        for (int i=0;i<320;i++)
        {
            float cx=(((float)i)-u0)/fox;
            float cy=(((float)j)-v0)/foy;
            float dst=((float)tempcali[j*320+i])/4.0/1000;
            float x = dst*cx;
            float y = dst*cy;
            float z = dst;

            *((float*)(ptr +  0)) = x;
            *((float*)(ptr +  4)) = y;
            *((float*)(ptr +  8)) = z;
            uint32_t color=colormap_temp[j*320+i];
            uint32_t color_r=color&0xff;
            uint32_t color_g=(color&0xff00)>>8;
            uint32_t color_b=(color&0xff0000)>>16;
            *((uint32_t*)(ptr + 12)) = (color_r<<16)|(color_g<<8)|(color_b<<0);
            *((float*)(ptr + 16)) = tempcali_ir[j*320+i];
            ptr += 20;
        }
        publisher_pc->publish(pcmsg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_d;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_i;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_s;
    size_t count_;
    Tof_Filter *CaliInst;
    std::vector<float> uvf_parms;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}


