# 收集训练代码：stm32指定一组运动序列（FNN，FFN，NNN等8种）。进行循环：PC端成功获取一帧图像数据，发送信号给stm32，stm32根据控制序列运动一个步长，稳定（2秒）后，发送信号给PC端。直到stm32完成整个控制序列。
# -*- coding: utf-8 -*-：
import sys
from turtle import color
from openni import openni2
import numpy as np
import cv2
import time
import serial

f_x = 575.868
f_y = 575.868
c_x = 322.993
c_y = 244.211
max_depth = 4000
min_depth = 20
RESOULTION_X = 640
RESOULTION_Y = 480
DATA_PATH = './data/'

# hsv range 用于提取末端轮廓

# 黄色海绵
# lower = np.array([18, 10, 184]) 
# upper = np.array([43, 95, 249])

# # 蓝色海绵
# lower = np.array([84, 127, 88]) 
# upper = np.array([123, 228, 167])

# # 蓝色胶带
# lower = np.array([106, 89, 43]) 
# upper = np.array([128, 211, 199])

lower = np.array([71, 0, 105]) 
upper = np.array([139, 163, 255])

port = serial.Serial(
    port = "/dev/ttyUSB0",
    baudrate = 115200,
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
)

# Wait a second to let the port initialize
time.sleep(1)


# 输入二维的mxn矩阵，输出水平翻转后的矩阵
def horizontalFlip(arr):
    h,w = arr.shape
    arr_180 = arr.reshape(1,int(arr.size))
    arr_180 = arr_180[0][::-1].reshape(h,w)
    return arr_180[::-1]

# 根据rgb图象目标像素位置 w,h 以及点云图 得到对应的目标空间位置
def get_position(position,img_size):
    w = img_size[0]
    h = img_size[1]
    pixel = 640*h+w
    x = position[pixel,0]
    y = position[pixel,1]
    z = position[pixel,2]
    return [x,y,z]

# 提取单个像素点(h,w)对应的相机坐标，输出第一个变量表示该坐标是否有效
def convertDepthToPointCloud_single_point(depth_img,w,h):
    width = 640
    height = 480

    fdx = f_x * ((float)(width) / RESOULTION_X)
    fdy = f_y * ((float)(height) / RESOULTION_Y)
    u0 =  c_x * ((float)(width)/ RESOULTION_X)
    v0 = c_y * ((float)(height) / RESOULTION_Y)

    # 对像素点操作
    depth = depth_img[h,w]
    if (depth <= 0 and depth < min_depth and depth > max_depth): # 无效的深度值
        return False,0,0,0

    # 转换点云数据
    tx = (w - u0) / fdx
    ty = (h - v0) / fdy
    world_x = depth * tx
    world_y = depth * ty
    world_z = depth
    
    if world_x*world_y*world_z == 0.0 or world_x*world_y*world_z==-0.0:
        return False,0,0,0
    return True,-world_x, world_y, world_z

# 转换成点云图并保存
def convertDepthToPointCloud(depth_img,frame_cnt):
    width = 640
    height = 480
    valid_count = 307200
    if depth_img is None:
        print("depth frame is NULL!")
        return 0

    fdx = f_x * ((float)(width) / RESOULTION_X)
    fdy = f_y * ((float)(height) / RESOULTION_Y)
    u0 =  c_x * ((float)(width)/ RESOULTION_X)
    v0 = c_y * ((float)(height) / RESOULTION_Y)

    # 初始化点云文件
    PLY_FILE = DATA_PATH + 'pointcloud_' + str(frame_cnt)+'.ply'
    with open(PLY_FILE, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex %d\n" %valid_count)
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")

        # 对每个像素点操作
        for v in range(height):
            for u in range(width):
                # depth = depth_img[v * width + u]
                depth = depth_img[v,u]
                # 统计有效深度点
                if (depth <= 0 and depth < min_depth and depth > max_depth):
                    continue
                valid_count += 1

                # 转换点云数据
                tx = (u - u0) / fdx
                ty = (v - v0) / fdy
                world_x = depth * tx
                world_y = depth * ty
                world_z = depth
                
                # 存储点云数据
                f.write("%f %f %f 255 255 255\n" %(world_x, world_y, world_z))

    return 1

# 获取一帧深度图
def read_depth_image(depth_stream):
    depth_frame = depth_stream.read_frame()    # 获取一帧深度图
    raw_buf = depth_frame.get_buffer_as_uint16()
    buf_array = np.array([raw_buf[i] for i in range(640*480)])
    depth_image = buf_array.reshape(480,640)
    # depth_image = np.frombuffer(raw_buf, dtype=np.uint16)
    # depth_image.shape = (1, 480, 640)
    # depth_image = np.concatenate((depth_image, depth_image, depth_image), axis=0)
    # depth_image = np.swapaxes(depth_image, 0, 2)
    # depth_image = np.swapaxes(depth_image, 0, 1)
    depth_image = horizontalFlip(depth_image)
    return depth_image

# 获取一帧彩色图
def read_color_image(color_stream):
    color_frame = color_stream.read_frame()    # 获取一帧彩色图
    raw_buf = color_frame.get_buffer_as_triplet()
    b_array = np.array([raw_buf[i][0] for i in range(640*480)])
    g_array = np.array([raw_buf[i][1] for i in range(640*480)])
    r_array = np.array([raw_buf[i][2] for i in range(640*480)])

    # convert to uint8 image
    color_image = np.zeros([480,640,3])
    color_image[:, :, 0] = r_array.reshape(480, 640)
    color_image[:, :, 1] = g_array.reshape(480, 640)
    color_image[:, :, 2] = b_array.reshape(480, 640)
    # if FLIP:
    #     color_image = np.flipud(color_image.astype(np.uint8))
    # else:
    color_image = np.fliplr(color_image.astype(np.uint8)).astype(np.uint8)
    return color_image

# 保存RGB图像文件
def save_color_image(color_image,frame_cnt): 
    RGB_FILE = DATA_PATH + 'img_' + str(frame_cnt)+'.png'
    cv2.imwrite(RGB_FILE, color_image)  

# 获取并保存一帧深度和彩色图
def get_a_frame(dStream,cStream,frame_cnt):
    depth_img = read_depth_image(dStream)
    color_img = read_color_image(cStream)

    # 循环直到成功获取一帧深度和彩色图
    while(depth_img is None or color_img is None ):
        depth_img = read_depth_image(dStream)
        color_img = read_color_image(cStream)
    
    # 展示深度图和RGB图
    # cv2.imshow('depth', depth_img)
    # cv2.imshow('color', color_img)
    # key = cv2.waitKey(1)
    # if int(key) == ord('q'):
    #     break
    
    # 转换并保存点云文件
    # convertDepthToPointCloud(depth_img,frame_cnt)

    # 保存彩色图
    # save_color_image(color_img,frame_cnt)
    
    return depth_img,color_img

# 从一帧数据识别并获取目标点的相机坐标 
def analyze_frame(color_img,depth_img,frame_cnt):
    marked_imgPath = DATA_PATH + 'img_marked_' + str(frame_cnt) + '.png'
    gs_frame = cv2.GaussianBlur(color_img, (5, 5), 0)  # using GaussianBlur
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # From BGR to HSV
    erode_hsv = cv2.erode(hsv, None, iterations=2)  # erode to reduce noise
    in_range_hsv = cv2.inRange(erode_hsv, lower, upper)  # color_dist[ball_color]['Lower'],  二值化：将在两个阈值内的像素值设置为白色（255），而不在阈值区间内的像素值设置为黑色（0）
    cnt_s = cv2.findContours(in_range_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 轮廓提取

    max_cont_area = 0
    max_cont =  0
    mark = 0
    center_size = 3
    for i in cnt_s:
        if mark == 0:
            max_cont_area = cv2.contourArea(i)
            max_cont = i
            mark = 1
        else:
            curr_cont_area = cv2.contourArea(i)
            if curr_cont_area > 0 and curr_cont_area > max_cont_area: # 设置连接域面积
                max_cont_area = curr_cont_area
                max_cont = i  

    # 画出最大连接域及其中心
    M = cv2.moments(max_cont)
    if(M["m00"]==0):
        return 0,0,0,0
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # draw the contour and center of the shape on the image
    cv2.drawContours(color_img, [max_cont], -1, (0, 255, 0), 2)
    cv2.circle(color_img, (cX, cY), center_size, (255, 255, 255), -1)
    # 保存包含标记点的图像
    cv2.imwrite(marked_imgPath, color_img)  

    return convertDepthToPointCloud_single_point(depth_img,cX,cY) # return valid,x,y,z


# 根据STM32传入的字符串，保存传感器数据或获取状态
def analyze_mcu_msg(msg):
    # state,p1,p2,p3,l,r,p,y = msg.split( )
    state,p1,p2,p3 = msg.split( )
    # return int(state),float(p1),float(p2),float(p3),float(l),float(r),float(p),float(y)
    return int(state),float(p1),float(p2),float(p3)

# coordinate transformation from camera to manipulator coordinate 
def coor_trans(cx,cy,cz):
    c_pose = np.array([cx,cy,cz])
    d = np.array([-2,-540,490]) # original point deviation
    # angleZ = np.pi/2
    # angleY = -2*np.pi/3
    # Rz = np.array([[np.cos(angleZ),-np.sin(angleZ),0],[np.sin(angleZ),np.cos(angleZ),0],[0,0,1]])
    # Ry = np.array([[np.cos(angleY),0,np.sin(angleY)],[0,1,0],[-np.sin(angleY),0,np.cos(angleY)]])
    # R = Ry@Rz
    angleX = 2*np.pi/3
    R = np.array([[1,0,0],[0,np.cos(angleX),-np.sin(angleX)],[0,np.sin(angleX),np.cos(angleX)]])
    m_pose = R.T@c_pose + d
    return m_pose[0],m_pose[1],m_pose[2]


if __name__ == "__main__": 

    openni2.initialize()                # openni初始化
    dev = openni2.Device.open_any()     # 打开相机设备
    print("Device detected.\r\n")
    print(dev.get_device_info())

    # 创建并设置深度图数据流
    depth_stream = dev.create_depth_stream()

    # 图像模式注册,彩色图与深度图对齐
    dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)

    # depth_stream.configure_mode(640,480,30,openni2.PIXEL_FORMAT_DEPTH_1_MM)  # width, height, fps, pixel_format 
    dMode = depth_stream.get_video_mode()
    dMode.resolutionX = 640
    dMode.resolutionY = 480
    dMode.fps = 30
    dMode.pixelFormat = openni2.PIXEL_FORMAT_DEPTH_1_MM
    depth_stream.set_video_mode(dMode)    
    depth_stream.start() 

    # 创建彩色图数据流
    color_stream = dev.create_color_stream()   # 创建彩色图数据流
    # color_stream.configure_mode(640,480,30,openni2.PIXEL_FORMAT_RGB888)  # width, height, fps, pixel_format 
    cMode = color_stream.get_video_mode()
    cMode.resolutionX = 640
    cMode.resolutionY = 480
    cMode.fps = 30
    cMode.pixelFormat = openni2.PIXEL_FORMAT_RGB888
    color_stream.set_video_mode(cMode)
    color_stream.camera.set_auto_white_balance(False)
    color_stream.camera.set_auto_exposure(True)

    # 彩色图和深度图同步
    dev.set_depth_color_sync_enabled(True)

    color_stream.start()
    SENSOR_TIP_FILE = DATA_PATH + 'sensor_and_tip_data.csv' # 存储传感器和末端位置数据

    with open(SENSOR_TIP_FILE, 'w') as f:
        # 循环两步骤：1.PC端成功获取一帧图像数据，发送信号给stm32   2.等待stm32完成一个步长的运动并返回信号。当stm32完成整个控制序列后（state = 0) 跳出循环
        state = 1 # 状态变量：控制序列是否完成，完成则跳出循环
        frame_cnt = 1
        
        time.sleep(10)
        port.write("ok\r\n".encode('utf-8'))  # 发送信号给stm32
        # msg = port.readline().decode()  # 阻塞式读取串口信息
        # state,p1,p2,p3,center_len,roll,pitch,yaw = analyze_mcu_msg(msg)
        # f.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,center_len,roll,pitch,yaw,0,0,0))
        # print("%f,%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,center_len,roll,pitch,yaw))


        while state:
            ########## step 1: PC端等待32运动完后一步的数据，然后获取该帧的图像数据，发送信号给stm32
            msg = port.readline().decode()  # 阻塞式读取串口信息
            # state,p1,p2,p3,center_len,roll,pitch,yaw = analyze_mcu_msg(msg)
            state,p1,p2,p3 = analyze_mcu_msg(msg)
            print("step1: successfully get sensor data.")

            ########## step 2: 获取该帧的图像数据，发送信号给stm32
            valid = False
            while(valid == False):
                depth_img,color_img = get_a_frame(depth_stream,color_stream,frame_cnt)
                valid,camera_x,camera_y,camera_z = analyze_frame(color_img,depth_img,frame_cnt)  # 获得相机坐标系下的末端位置
            frame_cnt += 1
            sensor_mani_x,sensor_mani_y,sensor_mani_z = coor_trans(camera_x,camera_y,camera_z)   # 通过坐标变换得到机械臂坐标系下的末端位置
            port.write("ok\r\n".encode('utf-8'))  # 发送信号给stm32

            # 判断当stm32完成整个控制序列后（state = 0) 跳出循环
            if state==0: # 完成预设的运动序列，跳出循环
                port.write("ok\r\n".encode('utf-8'))
                print("Movement finished.")
                # f.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,center_len,roll,pitch,yaw,sensor_mani_x,sensor_mani_y,sensor_mani_z))
                # print("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,center_len,roll,pitch,yaw,sensor_mani_x,sensor_mani_y,sensor_mani_z))
                f.write("%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,sensor_mani_x,sensor_mani_y,sensor_mani_z))
                print("%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,sensor_mani_x,sensor_mani_y,sensor_mani_z))

                break

            # 存储传感器和末端位置数据
            # f.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,center_len,roll,pitch,yaw,sensor_mani_x,sensor_mani_y,sensor_mani_z))
            # print("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,center_len,roll,pitch,yaw,sensor_mani_x,sensor_mani_y,sensor_mani_z))
            f.write("%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,sensor_mani_x,sensor_mani_y,sensor_mani_z))
            print("%f,%f,%f,%f,%f,%f\n" %(p1,p2,p3,sensor_mani_x,sensor_mani_y,sensor_mani_z))

    # 结束，关闭
    depth_stream.stop()
    color_stream.stop()
    openni2.unload()
    dev.close()

