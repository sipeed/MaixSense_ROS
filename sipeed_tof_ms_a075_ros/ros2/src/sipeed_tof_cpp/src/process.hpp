
#include "opencv2/opencv.hpp"
#include <jpeglib.h>

#include "data_structs.h"
using namespace cv;

class Tof_Filter {
public:
        Tof_Filter(int width,int height,int kernelsize){
        bufferdata= (uint16_t*)malloc(width*height*30*2);
        bufferspat= (uint16_t*)malloc(width*height*2*3);
        memset(bufferdata,0,width*height*2);
        memset(bufferspat,0,width*height*2*3);
        imgw=width;
        imgh=height;
        kernel_size=kernelsize;
    }
    void filterTof(Mat &deep,Mat &inst,Mat &out,Mat &kernel){
        out.create(deep.rows,deep.cols,CV_32FC1);
        int bs=kernel.size[0]/2;
        int ks=kernel.size[0];
//        printf("rows:%d,cols:%d\n",deep.rows,deep.cols);
        Mat buf1;
        Mat buf2;
        copyMakeBorder(deep,	buf1, bs, bs, bs, bs, BORDER_REPLICATE);
        copyMakeBorder(inst,	buf2, bs, bs, bs, bs, BORDER_REPLICATE);
        for(int xp=0;xp<deep.cols;xp++)
            for(int yp=0;yp<deep.rows;yp++){
                Mat depi,iri;
                buf1(Rect(xp,yp,ks,ks)).copyTo(depi);
                buf2(Rect(xp,yp,ks,ks)).copyTo(iri);
//                std::cout<<kernel<<std::endl;
//                std::cout<<depi<<std::endl;
//                std::cout<<iri<<std::endl;
//                printf("mul1\n");
                Mat a=kernel.mul(depi);
//                printf("mul2\n");
                Mat b=kernel.mul(iri);
//                printf("mul3\n");
                Mat c=a.mul(b);
//                printf("mul4\n");
                Mat d=c.mul(buf1(Rect(xp,yp,ks,ks)));
//                printf("mul5\n");
                float num_out=sum(d)[0]/sum(c)[0];
//                printf("mul6\n");
                out.at<float>(yp,xp)=num_out;
//                printf("mul7\n");
            }
    }

    std::vector<uint16_t> SpatialFilter(const uint16_t* data_data,int filtertype) {
//        printf("vecsize:%lu\n", data.size());
        memcpy(bufferspat,data_data,imgw*imgh*2*2);
        Mat deep_img=Mat(240,320,CV_16UC1,bufferspat);
        Mat ir_img=Mat(240,320,CV_16UC1,bufferspat+320*240);
        deep_img.convertTo(deep_img,CV_32FC1);
        ir_img.convertTo(ir_img,CV_32FC1);
        int kernelsize=kernel_size;
        Mat kernel=cv::getGaussianKernel(kernelsize,-1, CV_32F);
        Mat filter_out;
        if(filtertype==1){
            Mat kernel2D=Mat(kernelsize,kernelsize,CV_32FC1);
            for(int i=0;i<kernelsize;i++)
                for(int j=0;j<kernelsize;j++){
                    kernel2D.at<float>(i,j)=kernel.at<float>(i)*kernel.at<float>(j);
                }
            filterTof(deep_img,ir_img,filter_out,kernel2D);
        }
        else{
            filter2D(deep_img,filter_out,CV_32FC1,kernel,Point(-1,-1),0,BORDER_REPLICATE);
        }
        filter_out.convertTo(filter_out,CV_16UC1);
        std::vector<uint16_t>data_out(filter_out.begin<uint16_t>(), filter_out.end<uint16_t>());
        return data_out;
    }
    uint16_t* TemporalFilter(uint16_t*datap) {
//        printf("vecsize:%lu\n", data.size());
        if(temp_time_changed){
            temp_time_changed=0;
            memcpy(bufferdata,datap,imgw*imgh*2);
        }
        Mat deep_img=Mat(240,320,CV_16UC1);
        for(int xp=0;xp<deep_img.cols;xp++)
            for(int yp=0;yp<deep_img.rows;yp++){
                uint32_t sum=0;
                sum+=float(datap[320*yp+xp])*temple_alpha+float(bufferdata[320*yp+xp])*(1.0-temple_alpha);
                deep_img.at<uint16_t>(yp,xp)=sum;
            }
        memcpy(bufferdata,deep_img.datastart,imgw*imgh*2);
        // std::vector<uint16_t>data_out(deep_img.begin<uint16_t>(), deep_img.end<uint16_t>());
        return bufferdata;
    }
    std::vector<uint16_t> FlyingPointFilter(std::vector<uint16_t> &data,float threshold) {
        Mat deep_img=Mat(240,320,CV_16UC1,(uint16_t*)data.data());
        Mat l_img,r_img,u_img,d_img;
        deep_img(Rect(0,0,319,240)).copyTo(l_img);
        deep_img(Rect(1,0,319,240)).copyTo(r_img);
        deep_img(Rect(0,0,320,239)).copyTo(u_img);
        deep_img(Rect(0,1,320,239)).copyTo(d_img);
        Mat abs_diff(240,320,CV_16UC1,Scalar_<uint16_t>(0));
        Mat tmph,tmpv;
        Mat tmp1(240,320,CV_16UC1,Scalar_<uint16_t>(0));
        Mat tmp2(240,320,CV_16UC1,Scalar_<uint16_t>(0));
        Mat tmp3(240,320,CV_16UC1,Scalar_<uint16_t>(0));
        Mat tmp4(240,320,CV_16UC1,Scalar_<uint16_t>(0));
        cv::absdiff(l_img,r_img,tmph);
        cv::absdiff(u_img,d_img,tmpv);
        tmph.copyTo(tmp1(Rect(0,0,319,240)));
        tmph.copyTo(tmp2(Rect(1,0,319,240)));
        tmpv.copyTo(tmp3(Rect(0,0,320,239)));
        tmpv.copyTo(tmp4(Rect(0,1,320,239)));
        cv::max(tmp1,abs_diff,abs_diff);
        cv::max(tmp2,abs_diff,abs_diff);
        cv::max(tmp3,abs_diff,abs_diff);
        cv::max(tmp4,abs_diff,abs_diff);
        Mat absf,deepf;
        deep_img.convertTo(deepf,CV_32FC1);
        abs_diff.convertTo(absf,CV_32FC1);
        absf/=deepf;
        Mat mask=absf<threshold;
        Mat imgout;
        deep_img.copyTo(imgout,mask);
//        std::cout<<mask<<std::endl;
        std::vector<uint16_t>data_out(imgout.begin<uint16_t>(), imgout.end<uint16_t>());
        return data_out;
    }
    std::vector<uint8_t> NV12_RGB(std::vector<uint8_t> data,int width,int height){
        Mat mYUV(height + height/2, width, CV_8UC1, (void*)data.data());
        Mat mRGB;//(height, width, CV_8UC4);
        cvtColor(mYUV, mRGB, COLOR_YUV2RGBA_NV21);
        //flip(mRGB,mRGB,0);
        std::vector<uint8_t>data_out;
        data_out.assign((uint8_t*)mRGB.datastart, (uint8_t*)mRGB.dataend);
//        printf("size:%d\n",data_out.size());
        auto ret=data_out;
//        auto ret= emscripten::val(emscripten::typed_memory_view(5, "1111"));
        return ret;
    }
    std::vector<uint8_t> RGB_RGBA(std::vector<uint8_t> data,int width,int height){
        Mat mYUV(height, width, CV_8UC3, (void*)data.data());
        Mat mRGB;//(height, width, CV_8UC4);
        cvtColor(mYUV, mRGB, COLOR_RGB2RGBA);
        //flip(mRGB,mRGB,0);
        std::vector<uint8_t>data_out;
        data_out.assign((uint8_t*)mRGB.datastart, (uint8_t*)mRGB.dataend);
//        printf("size:%d\n",data_out.size());
        auto ret=data_out;
//        auto ret= emscripten::val(emscripten::typed_memory_view(5, "1111"));
        return ret;
    }
    std::vector<uint16_t> TOF_cali(std::vector<uint16_t> deep,std::vector<uint16_t> status){
        if(!parm_inited)
            return std::vector<uint16_t>({0});
        Mat mDEEP(240,320, CV_16U,(void*)deep.data());
        Mat mSTATUS(240, 320, CV_16U,(void*)status.data());
        Mat mDEEP_out(240*2,320, CV_16U);
        mDEEP_out*=0;
        mDEEP_out(Rect(0,240,320,240))+=5;
        for (int i = 0; i < 320; i++)
            for (int j = 0; j < 240; j++) {
                if(!vail_map.at<uint8_t>(j,i))
                    continue;
                float x_pos=deepmap_x.at<int16_t >(j,i);
                float y_pos=deepmap_y.at<int16_t >(j,i);
                mDEEP_out.at<uint16_t>(j,i)=mDEEP.at<uint16_t>(y_pos,x_pos);
                mDEEP_out.at<uint16_t>(j+240,i)=mSTATUS.at<uint16_t>(y_pos,x_pos);
            }
        std::vector<uint16_t>data_out(mDEEP_out.begin<uint16_t>(), mDEEP_out.end<uint16_t>());
        return data_out;
    }
    std::vector<uint32_t> MapRGB2TOF(std::vector<uint16_t> deep,std::vector<uint8_t> rgb){
        if(!parm_inited)
            return std::vector<uint32_t>();
        Mat mDEEP(240,320, CV_16U,(void*)deep.data());
        Mat mRGB(600, 800, CV_8UC4,(void*)rgb.data());
        Mat mDEEPf;
        mDEEP.convertTo(mDEEPf,CV_32FC1,0.25);
        Mat RGB_out(240,320, CV_8UC4);
        RGB_out*=0;
        float u0=ToFcameraMatrix.at<float>(2);
        float v0=ToFcameraMatrix.at<float>(5);
        float fx=ToFcameraMatrix.at<float>(0);
        float fy=ToFcameraMatrix.at<float>(4);
        Mat pos(240*320,3,CV_32F);
        for (int i = 0; i < 320; i++)
        for (int j = 0; j < 240; j++) {
            if(!vail_map.at<uint8_t>(j,i))
                continue;
            float cx=(i-u0)/fx;
            float cy=(j-v0)/fy;
            float dst=mDEEPf.at<float>(j,i);
            float x = dst*cx;
            float y = dst*cy;
            float z = dst;
            pos.at<float>(j*320+i,0)=x;
            pos.at<float>(j*320+i,1)=y;
            pos.at<float>(j*320+i,2)=z;
        }
//        float R_data[]={0.999009948014846,-0.035988877304778,-0.026151949788032,0.038600978837311,0.993464765304480,0.107413800469572,0.022115338572483,-0.108316946083506,0.993870389432630};
        Mat R_MAT(3,3,CV_32F,R_data);
//        float T_data[]={-13.764129748178371,12.774782478789565,-3.663691499294785};
        Mat T_MAT(1,3,CV_32F,T_data);
//        std::cout<<T_MAT<<std::endl;
//        float RGB_CM_data[]={8.731012535254073e+02,-1.766830173252281,4.032462441863750e+02,0,8.738528440154463e+02,2.770907184013972e+02,0,0,1};
        Mat RGB_CM(3,3,CV_32F,RGB_CM_data);
//        float D_VEC_data[]={0.039207535514683,0.108443361234459,0.005477497943480,-0.003425915814048,-0.782406374511559};
        Mat D_VEC(1,5,CV_32F,D_VEC_data);
        Mat outpoints;
        Mat outpointsi;
        Mat R_MAT2;
//        T_MAT*=-1;
        Rodrigues(R_MAT.t(),R_MAT2);


//            std::cout<<R_MAT<<std::endl;
//            std::cout<<T_MAT<<std::endl;
//            std::cout<<RGB_CM<<std::endl;
//            std::cout<<D_VEC<<std::endl;
        projectPoints(pos,
                      R_MAT2, T_MAT,
                      RGB_CM,
                      D_VEC,
                      outpoints
        );
        outpoints.convertTo(outpointsi,CV_16S);
        for (int i = 0; i < 320; i++)
            for (int j = 0; j < 240; j++) {
                auto pos=outpointsi.at<Point_<int16_t>>(j*320+i);
                if(pos.x<0||pos.x>799||pos.y<0||pos.y>599)
                    continue;
                RGB_out.at<Vec4b>(j,i)=mRGB.at<Vec4b>(pos);
            }
//        static int onceprint;
//            if((onceprint++)==0)
//        std::cout<<outpoints<<std::endl;
//        std::cout<<map2<<std::endl;
//        mRGB(Rect(0,0,320,240)).copyTo(RGB_out);
        std::vector<uint32_t> data_out;
        data_out.assign((uint32_t*)RGB_out.datastart, (uint32_t*)RGB_out.dataend);
//        printf("size:%d\n",data_out.size());
        auto ret = data_out;
        return ret;
    }
    std::vector<float> parse_info(const info_t *datap){


        info=datap->module_info;
        lens=datap->coeff;
        printf("PN:");
        for (int c = 0; c < 9; c++) {
            printf("%c", info.sensor_pn[c]);
        }
        printf("\n");
        printf("module_vendor:");
        for (int c = 0; c < 2; c++) {
            printf("%c", info.module_vendor[c]);
        }
        printf("\n");
        printf("module_type:");
        for (int c = 0; c < 2; c++) {
            printf("%c", info.module_type[c]);
        }
        printf("\n");
        printf("SN:");
        for (int c = 0; c < 16; c++) {
            printf("%c", info.module_sn[c]);
        }
        printf("\n");
        printf("vcsel_id:");
        for (int c = 0; c < 4; c++) {
            printf("%c", info.vcsel_id[c]);
        }
        printf("\n");
        printf("bin_version:");
        for (int c = 0; c < 3; c++) {
            printf("%c", info.bin_version[c]);
        }
        printf("\n");
        printf("cali_algo_ver:%d.%d\n", info.cali_algo_ver[0],
               info.cali_algo_ver[1]);
        printf("firmware_ver:%x.%x\n", info.firmware_ver[0], info.firmware_ver[1]);
        printf("Lenscoef:");
        printf("cali_mode:           %d\n", lens.cali_mode);
        printf("fx:           %e\n", lens.fx);
        printf("fy:           %e\n", lens.fy);
        printf("u0:           %e\n", lens.u0);
        printf("v0:           %e\n", lens.v0);
        printf("k1:           %e\n", lens.k1);
        printf("k2:           %e\n", lens.k2);
        printf("k3:           %e\n", lens.k3);
        printf("k4_p1:        %e\n", lens.k4_p1);
        printf("k5_p2:        %e\n", lens.k5_p2);
        printf("skew:         %e\n", lens.skew);

        float cam_matrix[9] = { lens.fx,0,lens.u0,0,lens.fy,lens.v0,0,0,1};
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, cam_matrix);
        cameraMatrix.copyTo(ToFcameraMatrix);
        float R_matrix[9] = {1,0,0,0,1,0,0,0,1};
        cv::Mat RMatrix = cv::Mat(3, 3, CV_32F, R_matrix);
        float cam_dist_data[5] = {lens.k1,lens.k2,lens.k4_p1,lens.k5_p2,lens.k3};
        cv::Mat cam_dist = cv::Mat(1, 5, CV_32F, cam_dist_data);
        cv::Mat map1,map2;
        initUndistortRectifyMap	(cameraMatrix,
                                    cam_dist,
                                    RMatrix,
                                    cameraMatrix,
                                    Size2i(320,240),
                                    CV_16SC2,
                                    map1,
                                    map2
        );
        vail_map.create(240,320,CV_8UC1);
        vail_map*=0;
        deepmap_x.create(240,320,CV_16SC1);
        deepmap_y.create(240,320,CV_16SC1);
        std::cout<<map1.cols<<","<<map1.rows<<std::endl;
        std::cout<<map1.dims<<std::endl;
//        std::cout<<map1.at<Point_<uint16_t>>(0,0)<<std::endl;
//        std::cout<<map1<< std::endl;
        for(int x=0;x<320;x++)
            for(int y=0;y<240;y++){
                auto point=map1.at<Point_<int16_t>>(y,x);
                if(point.x<0||point.x>=320||point.y<0||point.y>=240)
                    continue;
                vail_map.at<uint8_t>(y,x)=1;
                deepmap_x.at<int16_t >(y,x)=point.x;
                deepmap_y.at<int16_t >(y,x)=point.y;
            }
//        std::cout<<deepmap_x<< std::endl;
//        std::cout<<deepmap_y<< std::endl;
//        std::cout<<vail_map<< std::endl;
        parm_inited=1;
        parms_to_fontend=std::vector<float>({lens.fx,lens.fy,lens.u0,lens.v0});
        auto ret = parms_to_fontend;
        return  ret;
    }
    void setCameraParm(const float* Rdata,const float* Tdata,const float* CMdata,const float* Ddata){
        for(int i=0;i<9;i++)
            R_data[i]=Rdata[i];
        for(int i=0;i<3;i++)
            T_data[i]=Tdata[i]*-1;
        for(int i=0;i<9;i++)
            RGB_CM_data[i]=CMdata[i];
        for(int i=0;i<5;i++)
            D_VEC_data[i]=Ddata[i];
        Mat R_MAT(3,3,CV_32F,R_data);
        Mat T_MAT(1,3,CV_32F,T_data);
        Mat RGB_CM(3,3,CV_32F,RGB_CM_data);
        Mat D_VEC(1,5,CV_32F,D_VEC_data);
        std::cout<<"R_MAT:"<<R_MAT<<std::endl;
        std::cout<<"T_MAT:"<<T_MAT<<std::endl;
        std::cout<<"RGB_CM:"<<RGB_CM<<std::endl;
        std::cout<<"D_VEC:"<<D_VEC<<std::endl;
    }
    void TemporalFilter_cfg(float times) {
        temple_alpha=times>1?1:times;
        temple_alpha=times<0?0:times;
        printf("times:%f\n",times);
        temp_time_changed=1;
    }
    void free_mem(){
        free(bufferdata);
        free(bufferspat);
    }
    void set_kernel_size(int k){
        kernel_size=k;
    }
    stackframe_old_t DecodePkg(uint8_t* deep_package ) {
        stackframe_t frame;
        memcpy(&frame,deep_package,sizeof(stackframe_t)-4*sizeof(uint8_t*));
        frame.depth=(deep_package)+sizeof(stackframe_t)-4*sizeof(uint8_t*);
        frame.ir=frame.depth+frame.config.get_depth_size();
        frame.status=frame.ir+frame.config.get_ir_size();
        frame.rgb=frame.status+frame.config.get_status_size();
        static stackframe_old_t frame_to_return;
        frame_to_return.frameid=frame.frameid;
        frame_to_return.framestamp=frame.framestamp;
        switch(frame.config.deepmode){
            case 1:
                if(frame.config.deepshift==255)
                    for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.depth)[i]=depth_LUT[(((uint8_t*)frame.depth)[i])];
                else
                    for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.depth)[i]=((uint16_t)(((uint8_t*)frame.depth)[i]))<<frame.config.deepshift;
                break;
            case 0:
                for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.depth)[i]=((uint16_t*)frame.depth)[i];
                break;
            default:break;
        }
        switch(frame.config.irmode){
            case 1:
                for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.ir)[i]=((uint16_t)(((uint8_t*)frame.ir)[i]))*16;
                break;
            case 0:
                for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.ir)[i]=((uint16_t*)frame.ir)[i];
                break;
            default:break;
        }
        switch(frame.config.statusmode){
            case 2:
                for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.status)[i]=((uint16_t)(((uint8_t*)frame.status)[i]));
                break;
            case 0:
                for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.status)[i]=((uint16_t*)frame.status)[i];
                break;
            case 1:
                for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.status)[i]=((((uint8_t*)frame.status)[i/4])>>(2*(i%4)))&3;
                break;
            case 3:
                for(int i=0;i<320*240;i++)((uint16_t*)frame_to_return.status)[i]=((((uint8_t*)frame.status)[i/8])&1)?3:0;
                break;
            default:break;
        }
        switch (frame.config.rgbmode) {
            case 0:
                memcpy(frame_to_return.rgb,frame.rgb,frame.rgbsize);
                break;
            case 1:
            
                struct jpeg_decompress_struct info; //for our jpeg info
                struct jpeg_error_mgr err;          //the error handler
                info.err = jpeg_std_error(& err);
                err.error_exit=[](j_common_ptr cinfo){};
                jpeg_create_decompress(& info);   //fills info structure
                jpeg_mem_src(&info,frame.rgb,frame.rgbsize);
                if(JPEG_HEADER_OK==jpeg_read_header(&info, TRUE)){
                    jpeg_start_decompress(&info);
                    unsigned char *jdata = (unsigned char *)malloc(info.output_width*info.output_height*3);
                    while (info.output_scanline < info.output_height) // loop
                    {
                        unsigned char * pt=jdata + 3* info.output_width * info.output_scanline;
                        jpeg_read_scanlines(&info, &pt, 1);
                    }
                    jpeg_finish_decompress(&info);   //finish decompressing
                    Mat mRGB(480, 640, CV_8UC3, (void*)jdata);
                    Mat mRGBA;//(height, width, CV_8UC4);
                    cvtColor(mRGB, mRGBA, COLOR_RGB2RGBA);
                    resize(mRGBA,mRGBA,Size(800,600));
                    //flip(mRGB,mRGB,0);
                    memcpy(frame_to_return.rgb,mRGBA.datastart,mRGBA.dataend-mRGBA.datastart);
                    free(jdata);
                }
                break;
            case 2:
                memset(frame_to_return.rgb,0, sizeof(frame_to_return.rgb));
                break;

        }
        return frame_to_return;
    }
    void set_lut(const uint8_t *datap){

        float tempdata[256]={0};
        float tempcounts[256]={0};
        for(int i=0;i<65536;i++){
            tempdata[datap[i]]+=i;
            tempcounts[datap[i]]+=1;
        }
        for(int i=0;i<256;i++){
            if(tempcounts[i]==0)
                depth_LUT[i]=0;
            else
                depth_LUT[i]=tempdata[i]/tempcounts[i];
        }
    }

    uint16_t depth_LUT[256];
    uint16_t *bufferdata=NULL;
    uint16_t *bufferspat=NULL;
    float temple_alpha=0.5;
    int kernel_size=0;
    int imgw;
    int imgh;
    int correntid=0;
    int parm_inited=0;
    int temp_time_changed=1;
    LensCoeff_t lens;
    ModuleInfor_t info;
    Mat vail_map;
    Mat deepmap_x;
    Mat deepmap_y;
    Mat ToFcameraMatrix;
    //When Calibration, RGB Camera is Camera1, ToF as Camera2
    //in MATLAB stereoParams.RotationOfCamera2' (here remember to transpose matrix)
    float R_data[9]={1,0,0,0,1,0,0,0,1};
    //in MATLAB stereoParams.TranslationOfCamera2
    float T_data[3]={0,0,0};
    //in MATLAB stereoParams.CameraParameters1.IntrinsicMatrix' (here remember to transpose matrix)
    float RGB_CM_data[9]={800,0,400,0,800,300,0,0,1};
    //k1 k2 p1 p2 k3
    float D_VEC_data[5]={0,0,0,0,0};
    std::vector<float> parms_to_fontend;
};