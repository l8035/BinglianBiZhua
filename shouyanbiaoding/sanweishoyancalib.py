#!/usr/bin/env python
 
import numpy as np
import time
import cv2
# from real.realsenseD415 import Camera
# from real.delta_chuanKo import Moving
import serial
import time
# delta的运动范围
# workspace_limits = np.asarray([[0, 100], [-100, 100], [-400, -300]])
workspace_limits = np.asarray([[-0.05, 0.05], [-0.05, 0.05], [-0.35, -0.25]])
calib_grid_step = 0.05
 
# 棋盘格中心点相对于末端执行器的距离
# checkerboard_offset_from_tool = [0,-0.13,0.02]  # change me!
checkerboard_offset_from_tool = [0.13, -0.065]
 
# Construct 3D calibration grid across workspace
# 这一部分就是生成机械臂移动的坐标
print(1 + (workspace_limits[0][1] - workspace_limits[0][0]) / calib_grid_step)
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1],
                          int(1 + (workspace_limits[0][1] - workspace_limits[0][0]) / calib_grid_step))
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1],
                          int(1 + (workspace_limits[1][1] - workspace_limits[1][0]) / calib_grid_step))
# gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1],
#                           int(1 + (workspace_limits[2][1] - workspace_limits[2][0]) / calib_grid_step))
calib_grid_x, calib_grid_y = np.meshgrid(gridspace_x, gridspace_y)
num_calib_grid_pts = calib_grid_x.shape[0] * calib_grid_x.shape[1]  # 得到的是有多少组坐标
 
calib_grid_x.shape = (num_calib_grid_pts, 1)
calib_grid_y.shape = (num_calib_grid_pts, 1)
calib_grid_pts = np.concatenate((calib_grid_x*1000, calib_grid_y*1000,), axis=1)
print(calib_grid_pts) #以上代码用于生成并联机械臂的运动坐标
 
measured_pts = []#世界坐标系
observed_pts = []#相机坐标系
observed_pix = []#像素坐标系
 
#购买相机的内参:相机的内参
cam_intrinsics = np.array([643.07762734,0,321.2784282 ,0,643.86631838, 261.73479799, 0, 0, 1]).reshape(3, 3)
#print(cam_intrinsics)

#移动机械臂
#(1)先将机械臂复位
#ser.write('G28'.encode())
print('Collecting data...')
#（2）将机械臂移动到工作空间中的
Index=0
ser = serial.Serial('COM5',115200, timeout = 1)
time.sleep(10)
cap = cv2.VideoCapture(1)
flag = cap.isOpened()
for calib_pt_idx in range(num_calib_grid_pts):  
    tool_position = calib_grid_pts[calib_pt_idx, :]
    tool_config = [tool_position[0], tool_position[1]]
    tool_config1 = [tool_position[0], tool_position[1]]
    print(f"tool position and orientation:{tool_config1}")
    #传给delta机械臂坐标
    gcode=f"GO1 X{tool_config[0]} Y{tool_position[1]} Z-350"
    print(gcode)
    gcodeLine = gcode + '\n'
    ser.write(gcodeLine.encode())
    time.sleep(2) 
    response = ser.readline()
    print(response) 
    checkerboard_size = (5,5)
    time.sleep(4) 
    while(flag):
        ret, frame = cap.read()
        cv2.imshow("Capture_Paizhao",frame)
        k = cv2.waitKey(1)& 0xFF
        if (response.find('Ok'.encode()) > -1)or(response.find('Init Success!'.encode()) > -1): #按下S键以后才进行棋盘格
        # 查找棋盘格中心点
            refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            # 获取机器人的相机数据，将颜色图像和深度图像分别赋值给 camera_color_img 和 camera_depth_img
            # 这里其实就是获取d435i的相机的颜色图和深度图像，只不过这里进行了集成
            #bgr_color_data = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # 将相机颜色图像从 RGB 格式转换为 BGR 格式
            #cv2.imwrite(str(1) + ".jpg", frame)
            #由于frame本身就是BGR因此上面这一步不用。
            gray_data = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # BGR 格式转换为灰度图像
            # 寻找角点，找到角点，返回 checkerboard_found 为 true; corners为像素空间的角点
            # 第一个参数Image，传入拍摄的棋盘图Mat图像，必须是8位的灰度或者彩色图像；
            # 第二个参数patternSize，每个棋盘图上内角点的行列数，一般情况下，行列数不要相同，便于后续标定程序识别标定板的方向；
            # 第三个参数corners，用于存储检测到的内角点图像坐标位置，一般是数组形式；
            # 第四个参数flage：用于定义棋盘图上内角点查找的不同处理方式，有默认值。
            checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None)
            print(checkerboard_found)
            if checkerboard_found:
                # 角点进行亚像素级别的精确化优化，以提高角点的准确性
                corners_refined = cv2.cornerSubPix(gray_data, corners, (5, 5), (-1, -1), refine_criteria)
        
                # Get observed checkerboard center 3D point in camera space
                # 因为棋盘格是5*5，所以中间的格子是第12个索引，说白了就是为了获得棋盘格的中心点像素坐标
                checkerboard_pix = np.round(corners_refined[12, 0, :]).astype(int)
                checkerboard_z = 10
                # 下面就是用的像素坐标转换为相机坐标公式求出的XY，robot.cam_intrinsics也就是d435i的内参
                checkerboard_x = np.multiply(checkerboard_pix[0] - cam_intrinsics[0][2], checkerboard_z / cam_intrinsics[0][0])
                checkerboard_y = np.multiply(checkerboard_pix[1] - cam_intrinsics[1][2], checkerboard_z / cam_intrinsics[1][1])
                # if checkerboard_z == 0:
                #     continue
                # Save calibration point and observed checkerboard center
                # 像素中心点的三维坐标
                observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])
                # tool_position[2] += checkerboard_offset_from_tool
                # 得到机械臂的坐标
                tool_position = tool_position + checkerboard_offset_from_tool
                measured_pts.append(tool_position)
                observed_pix.append(checkerboard_pix)
                print(observed_pix)
                # Draw and display the corners
                # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
                # 第一个参数image，8位灰度或者彩色图像；
                # 第二个参数patternSize，每张标定棋盘上内角点的行列数；
                # 第三个参数corners，初始的角点坐标向量，同时作为亚像素坐标位置的输出，所以需要是浮点型数据；
                # 第四个参数patternWasFound，标志位，用来指示定义的棋盘内角点是否被完整的探测到，true表示别完整的探测到，
                # 函数会用直线依次连接所有的内角点，作为一个整体，false表示有未被探测到的内角点，这时候函数会以（红色）圆圈标记处检测到的内角点；
                vis = cv2.drawChessboardCorners(frame, (1, 1), corners_refined[12, :, :], checkerboard_found)
                cv2.imwrite('%06d.png' % len(measured_pts), vis)
                # image_RGB = np.hstack((vis, camera_color_img))
                # # cv2.imshow('Calibration',vis)
                # cv2.imshow('Calibration', image_RGB)
                # cv2.waitKey(1000)
                break
            else:
                print('------ 没找到角点 -------')
                break
        else :
            break
ser.close()
np.savetxt('measured_pts.txt', measured_pts, delimiter=' ')
np.savetxt('observed_pts.txt', observed_pts, delimiter=' ')
 