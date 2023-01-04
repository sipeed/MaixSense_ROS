from email.header import Header
import re
from cv2 import COLOR_RGB2RGBA
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Image,PointField
from std_msgs.msg import Header
import requests
import cv2
import struct
import json
from cv_bridge import CvBridge

import time
from multiprocessing import Process, Pipe

HOST='192.168.233.1'
PORT=8081
IMAGECOUNT=0

triggermode=1
deepmode=0
deepshift=5
irmode=0
statusmode=0
statusmask=7
rgbmode=0
rgbres=0
expose_time=0

def decodeLensCoeff(datain:bytes):
    return struct.unpack('<9s2s2s16s4s3s2s2sBffffffffff', datain)
class PointCloudPublisher(Node):
    def __init__(self):
        r=requests.get('http://{}:{}/getinfo'.format(HOST,PORT))
        print('infosize:',len(r.content))
        sensor_pn,module_vendor, module_type,module_sn,vcsel_id,bin_version,cali_algo_ver,\
        firmware_ver,cali_mode,fx,fy,u0,v0,k1,k2, k3,k4_p1,k5_p2,skew=decodeLensCoeff(r.content)
        print(f'{sensor_pn=}')
        print(f'{module_vendor=}')
        print(f'{module_type=}')
        print(f'{module_sn=}')
        print(f'{vcsel_id=}')
        print(f'{bin_version=}')
        print(f'{cali_algo_ver=}')
        print(f'{firmware_ver=}')
        print(f'{cali_mode=}')
        print(f'{fx=}')
        print(f'{fy=}')
        print(f'{u0=}')
        print(f'{v0=}')
        print(f'{k1=}')
        print(f'{k2=}')
        print(f'{k3=}')
        print(f'{k4_p1=}')
        print(f'{k5_p2=}')
        print(f'{skew=}')
        cameraMatrix = np.array([fx,0,u0,0,fy,v0,0,0,1]).reshape((3,3))
        ToFcameraMatrix=cameraMatrix
        RMatrix = np.identity(3)
        cam_dist = np.array([k1,k2,k4_p1,k5_p2,k3])
        map1=np.zeros((240,320,2))
        map2=np.zeros((240,320,2))
        cv2.initUndistortRectifyMap	(cameraMatrix,
                                    cam_dist,
                                    RMatrix,
                                    cameraMatrix,
                                    (320,240),
                                    cv2.CV_16SC2,
                                    map1,
                                    map2
        );
        self.vail_map=np.zeros((240,320))
        self.vail_mapd=np.zeros((240,320))
        self.deepmap_x=np.zeros((240,320))
        self.deepmap_y=np.zeros((240,320))
        for x in range(320):
            for y in range(240):
                point=map1[y,x];
                if point[0]<0 or point[0]>=320 or point[1]<0 or point[1]>=240 :
                    continue;
                self.vail_map[y,x]=1;
                self.vail_mapd[int(point[1]),int(point[0])]=1;
                self.deepmap_x[y,x]=point[0];
                self.deepmap_y[y,x]=point[1];
        self.lens_fx=fx
        self.lens_fy=fy
        self.lens_u0=u0
        self.lens_v0=v0

        r=requests.get('http://{}:{}/CameraParms.json'.format(HOST,PORT))
        print('camera parms size:',len(r.content))
        cameraparms=json.loads(r.content)
        self.R_MAT=np.array(cameraparms["R_Matrix_data"]).reshape((3,3))
        self.T_MAT=np.array(cameraparms["T_Vec_data"]).reshape((1,3))
        self.RGB_CM=np.array(cameraparms['Camera_Matrix_data']).reshape((3,3))
        self.D_VEC=np.array(cameraparms['Distortion_Parm_data']).reshape((1,5))
					
        r=requests.post('http://{}:{}/set_cfg'.format(HOST,PORT),data=struct.pack('BBBBBBBBi',triggermode,deepmode,deepshift,irmode,statusmode,statusmask,rgbmode,rgbres,expose_time))
        if(r.text=='OK'):
            print('set Done!')
        else:
            return
        super().__init__('sipeed_tof')
        self.publisher_pc = self.create_publisher(PointCloud2, 'cloud', 10)
        self.publisher_rgb = self.create_publisher(Image, 'rgb', 10)
        self.publisher_d = self.create_publisher(Image, 'depth', 10)
        self.publisher_i = self.create_publisher(Image, 'intensity', 10)
        self.publisher_s = self.create_publisher(Image, 'status', 10)
        self.bridge=CvBridge()
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.deepth_pipe = Pipe()
        self.p = Process(target=self.deepth_get_callback)
        self.p.start()

    def deepth_get_callback(self):
        while True:
            print("[cb] start get %fms"%(time.time()))
            r=requests.get('http://{}:{}/getdeep'.format(HOST,PORT))
            self.deepth_pipe[0].send_bytes(r.content)
            print("[cb] end get %fms"%(time.time()))

    def timer_callback(self):


        print("start get %fms"%(time.time()))
        # r=requests.get('http://{}:{}/getdeep'.format(HOST,PORT))
        # print('realsize:',len(r.content))
        # frame=r.content
        frame = self.deepth_pipe[1].recv_bytes()
        print("end get %fms"%(time.time()))
        frameid,timestamp,triggermode,deepmode,deepshift,irmode,statusmode,statusmask,rgbmode,rgbres,expose_time,depthsize,rgbsize=struct.unpack('QQBBBBBBBBiii', frame[:36])
        # print(frameid,timestamp,triggermode,deepmode,deepshift,irmode,statusmode,statusmask,rgbmode,rgbres,expose_time,depthsize,rgbsize)
        depthdata=frame[36:36+depthsize]
        rgbdata=frame[36+depthsize:36+depthsize+rgbsize]
        # print(len(rgbdata))
        if deepmode==1:
            # print('res:',2**deepshift*0.25,'mm,range:',255*2**deepshift*0.25,'mm')
            depthimg = np.frombuffer(depthdata[0:320*240], dtype="uint8").reshape((240,320)).astype(float)*(2**deepshift)
            depthdata=depthdata[320*240:]
        if deepmode==0:
            depthimg = np.frombuffer(depthdata[0:320*240*2], dtype="uint16").reshape((240,320))
            depthdata=depthdata[320*240*2:]
        if irmode==1:
            irimg = np.frombuffer(depthdata[0:320*240], dtype="uint8").reshape((240,320))
            depthdata=depthdata[320*240:]
        if irmode==0:
            irimg = np.frombuffer(depthdata[0:320*240*2], dtype="uint16").reshape((240,320))
            depthdata=depthdata[320*240*2:]
        if(statusmode==0):
            statusimg = np.frombuffer(depthdata[0:320*240*2], dtype="uint16").reshape((240,320))
        if(statusmode==2):
            statusimg = np.frombuffer(depthdata[0:320*240], dtype="uint8").reshape((240,320))
        if(statusmode==1):
            statusimg = np.zeros((240,320),dtype='uint8')
            for y in range(240):
                for x in range(320):
                    statusimg[y,x]=((depthdata[int((y*320+x)/4)]&(0x3<<((x%4)*2)))>>((x%4)*2))*64
        if(statusmode==3):
            statusimg = np.zeros((240,320),dtype='uint8')
            for y in range(240):
                for x in range(320):
                    statusimg[y,x]=((depthdata[int((y*320+x)/8)]&(0x1<<(x%8)))>>(x%8))*255
        if rgbmode==1:
            rgbimage = np.frombuffer(rgbdata, dtype="uint8")
            rgbimage = cv2.imdecode(rgbimage, cv2.IMREAD_COLOR)
        if rgbmode==0:
            rgbimage = np.frombuffer(rgbdata, dtype="uint8").reshape(900,800)
            rgbimage = cv2.cvtColor(rgbimage, cv2.COLOR_YUV2BGR_I420)
        if rgbimage is None:
            return
        rgbimage=cv2.resize(rgbimage,(800,600))
        rgbimage=cv2.cvtColor(rgbimage,COLOR_RGB2RGBA)

        depthimg=depthimg.astype('float32')/4.0
        irimg=irimg.astype('float32')
        
        points=np.zeros((240*320,3))
        cx = np.array([t for t in range(0,320)]).reshape(1, 320).repeat(240, 0)
        cy = np.array([t for t in range(0,240)]).reshape(240, 1).repeat(320, 1)

        cx = (cx-self.lens_u0)/self.lens_fx
        cy = (cy-self.lens_v0)/self.lens_fy

        points_x = (depthimg * cx).flatten()
        points_y = (depthimg * cy).flatten()
        points_z = (depthimg).flatten()

        points[:,0] = points_x
        points[:,1] = points_y
        points[:,2] = points_z

        # for i in range(320):
        #     for j in range(240):
        #         cx=(i-self.lens_u0)/self.lens_fx
        #         cy=(j-self.lens_v0)/self.lens_fy
        #         dst=depthimg[j,i]
        #         x = dst*cx
        #         y = dst*cy
        #         z = dst
        #         points[j*320+i,0]=x
        #         points[j*320+i,1]=y
        #         points[j*320+i,2]=z

        R_MAT2,_=cv2.Rodrigues(self.R_MAT.transpose())
        print("start proj %fms"%(time.time()))
        outpoints,_=cv2.projectPoints(points,
                      R_MAT2, self.T_MAT,
                      self.RGB_CM,
                      self.D_VEC,
        );
        print("end proj %fms"%(time.time()))

        outpointsi=outpoints.astype('int').squeeze()
        # print(outpointsi.shape)
        C=np.zeros((240*320,4),dtype='uint8')
        outpointsi[np.where((outpointsi[:,0]<0) +
                            (outpointsi[:,0]>799) +
                            (outpointsi[:,1]<0) +
                            (outpointsi[:,1]>599))]=0
        C = rgbimage[outpointsi[:,1], outpointsi[:,0]]
        # for i in range(320):
        #     for j in range(240):
        #         pos=outpointsi[j*320+i];
        #         # print(pos)
        #         if pos[0]<0 or pos[0]>799 or pos[1]<0 or pos[1]>599:
        #             continue;
        #         C[j*320+i]=rgbimage[pos[1],pos[0]];
            
        msg = PointCloud2()

        # cr=np.random.normal((240*320))
        # cg=np.random.normal((240*320))
        # cb=np.random.normal((240*320))
        
        # C[:,0]=np.resize(cr,(320*240))
        # C[:,0]=np.resize(cg,(320*240))
        # C[:,0]=np.resize(cb,(320*240))

        C=C.view('uint32')
        # print(C)
        pointsC=np.zeros((points.shape[0],1),dtype={'names':('x','y','z','rgba','intensity'),'formats':('f4','f4','f4','u4','f4')})
        points = points.astype('float32')
        points=points/1000
        pointsC['x']=points[:,0].reshape((-1,1))
        pointsC['y']=points[:,1].reshape((-1,1))
        pointsC['z']=points[:,2].reshape((-1,1))
        pointsC['rgba']=C
        pointsC['intensity']=irimg.reshape((-1,1))

        header = Header()
        if False:
            header.stamp = self.get_clock().now()
        else:
            header.stamp = Time(nanoseconds=timestamp*1000).to_msg()

        if False:
            header.frame_id = 'None'
        else:
            header.frame_id = 'tof'
        msg.header=header
        msg.height=240
        msg.width = 320
        msg.fields=[
            PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
            PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
            PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
            PointField(name='rgb',offset=12,datatype=PointField.UINT32,count=1),
            PointField(name='intensity',offset=16,datatype=PointField.FLOAT32,count=1),
        ]
        msg.is_bigendian=False
        msg.point_step=20
        msg.row_step=msg.point_step*320
        msg.is_dense=False
        # cost 210ms need to be optimized
        print("start tobytes %fms"%(time.time()))
        # msg.data=pointsC.tobytes()
        msg.data = struct.pack("%us"%(pointsC.size), pointsC)
        print("end tobytes %fms"%(time.time()))
        self.publisher_pc.publish(msg)
        depthmsg=self.bridge.cv2_to_imgmsg(depthimg)
        intensitymsg=self.bridge.cv2_to_imgmsg(irimg)
        statusmsg=self.bridge.cv2_to_imgmsg(statusimg)
        rgbmsg=self.bridge.cv2_to_imgmsg(rgbimage)
        depthmsg.header=header
        intensitymsg.header=header
        statusmsg.header=header
        rgbmsg.header=header
        self.publisher_d.publish(depthmsg)
        self.publisher_i.publish(intensitymsg)
        self.publisher_s.publish(statusmsg)
        self.publisher_rgb.publish(rgbmsg)
        self.get_logger().info('Publishing: {}'.format(msg.header.stamp))
        print("end all %fms\n\n"%(time.time()))


def main(args=None):
    rclpy.init(args=args)

    pc_pub = PointCloudPublisher()

    rclpy.spin(pc_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pc_pub.destroy_node()
    rclpy.shutdown()
