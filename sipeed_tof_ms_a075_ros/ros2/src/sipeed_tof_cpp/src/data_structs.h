#include <stdint.h>
struct __attribute__ ((packed)) all_config_t{
    uint8_t triggermode;//0:STOP 1:AUTO 2:SINGLE
    uint8_t deepmode;//0:16bit 1:8bit
    uint8_t deepshift;//for 8bit mode
    uint8_t irmode;//0:16bit 1:8bit
    uint8_t statusmode;//0:16bit 1:2bit 2:8bit 3:1bit
    uint8_t statusmask;//for 1bit mode 1:1 2:2 4:3
    uint8_t rgbmode;//0:YUV 1:JPG 2：None
    uint8_t rgbres;//0:800*600 1:1600*1200
    int32_t expose_time;
    void check_and_fix(){
        if(triggermode>2)triggermode=0;
        if(deepmode>1)deepmode=0;
        if(deepshift>11&&deepshift!=255)deepmode=0;
        if(irmode>1)irmode=0;
        if(statusmode>3)statusmask=0;
        if(statusmask>7)statusmask=0;
        if(rgbmode>3)rgbmode=0;
        if(rgbres>3)rgbres=0;
    }
    int get_depth_size(){
        switch (deepmode) {
            case 0:return 320*240*2;
            case 1:return 320*240;
        }
    }
    int get_ir_size(){
        switch (irmode) {
            case 0:return 320*240*2;
            case 1:return 320*240;
        }
    }
    int get_status_size(){
        switch (statusmode) {
            case 0:return 320*240*2;
            case 1:return 320*240/4;
            case 2:return 320*240;
            case 3:return 320*240/8;
        }
    };

};


#define PIXEL_ROWS (240)
#define PIXEL_COLS (320)
typedef uint16_t Image_t[PIXEL_ROWS][PIXEL_COLS];
struct stackframe_old_t{
    uint64_t frameid;
    uint64_t framestamp;
    Image_t depth;
    Image_t ir;
    Image_t status;
    uint8_t rgb[800*600*4];
};


struct __attribute__ ((packed)) stackframe_t{
    uint64_t frameid;
    uint64_t framestamp;
    all_config_t config;
    int deepsize;
    int rgbsize;
    uint8_t * depth;
    uint8_t * ir;
    uint8_t * status;
    uint8_t * rgb;
};

typedef struct {
    uint8_t cali_mode;
    float fx;
    float fy;
    float u0;
    float v0;
    float k1;
    float k2;
    float k3;
    float k4_p1;
    float k5_p2;
    float skew;
} __attribute__((packed)) LensCoeff_t;

typedef struct {
    char sensor_pn[9]; // Sensor part number
    char module_vendor[2];
    char module_type[2];
    char module_sn[16]; // module serial number
    char vcsel_id[4];
    char bin_version[3];
    uint8_t cali_algo_ver[2]; // Algorithm version for  this coefficient. X.Y.
                              // e.g 9.62
    uint8_t firmware_ver[2];  // ISP firmware version
} __attribute__((packed)) ModuleInfor_t;

struct __attribute__((packed)) info_t
{
    ModuleInfor_t module_info;
    LensCoeff_t coeff;
};
