# 手动获取一帧原始相机彩色和深度图像（不带图像识别）

# -*- coding: utf-8 -*-：
import sys
from openni import openni2
import numpy as np
import cv2

f_x = 575.868
f_y = 575.868
c_x = 322.993
c_y = 244.211
max_depth = 4000
min_depth = 20
RESOULTION_X = 640
RESOULTION_Y = 480
DATA_PATH = './data/'

# 输入一个m*n的二维numpy矩阵，输出水平翻转后的矩阵
def horizontalFlip(arr):
    h,w = arr.shape
    arr_180 = arr.reshape(1, int(arr.size))
    arr_180 = arr_180[0][::-1].reshape(h, w)
    return arr_180[::-1]

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
        # color_image = np.flipud(color_image.astype(np.uint8))

    color_image = np.fliplr(color_image.astype(np.uint8))
    return color_image

def save_color_image(color_image,frame_cnt): # 保存RGB图像文件
    RGB_FILE = DATA_PATH + 'img_' + str(frame_cnt)+'.png'
    # color_image = cv2.flip(color_image,1,dst=None) # 水平镜像
    cv2.imwrite(RGB_FILE, color_image)  

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
    convertDepthToPointCloud(depth_img,frame_cnt)

    # 保存彩色图
    save_color_image(color_img,frame_cnt)
    
    print("Successfully get a frame.\r\n")
    return 1

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
    # cMode.set_white_balance(4000)
    # cMode.set_exposure(50)
    color_stream.set_video_mode(cMode)
    color_stream.camera.set_auto_white_balance(True)
    color_stream.camera.set_auto_exposure(True)
    # color_stream.camera.set_exposure(10000)

    # 彩色图和深度图同步
    dev.set_depth_color_sync_enabled(True)

    color_stream.start()

    # 获取数据流
    frame_cnt = 1
    while True:
        command = input("Wait for command(c or e): ")
        if command == 'c':
             success = get_a_frame(depth_stream,color_stream,frame_cnt)
             frame_cnt += 1
        else:
            print("Finished. Ending Device.")
            break


    # 结束，关闭
    depth_stream.stop()
    color_stream.stop()
    openni2.unload()
    dev.close()

