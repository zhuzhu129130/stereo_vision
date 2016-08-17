#ifndef STEREORECTIFY_H
#define STEREORECTIFY_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "../basefunc/basefunc.h"

using namespace std;

/***
 *	双目校正的输入参数
 */
typedef struct _StereoParams_r
{
    cv::Size		imageSize;				// 图像分辨率
    cv::Mat       calib_M_L;
    cv::Mat       calib_D_L;
    cv::Mat       calib_M_R;
    cv::Mat       calib_D_R;

    cv::Mat			R;		// 旋转矩阵
    cv::Mat			T;	// 平移向量
    cv::Mat			E;		// 本质矩阵
    cv::Mat			F;	// 基础矩阵
}StereoParams_r;

/***
 *	双目校正的输出参数
 */
typedef struct _RemapMatrixs_r
{
    cv::Size		imageSize;				// 图像分辨率

    cv::Mat		Calib_mX_L;	// 左视图 X 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mY_L;	// 左视图 Y 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mX_R;	// 右视图 X 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mY_R;	// 右视图 Y 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_Q;		    // 用于计算三维点云的 Q 矩阵
    cv::Rect	Calib_Roi_L;	    // 左视图校正后的有效区域的矩形
    cv::Rect	Calib_Roi_R;	    // 右视图校正后的有效区域的矩形
    cv::Mat     Calib_Mask_Roi;		// 左视图校正后的有效区域
}RemapMatrixs_r;

class stereoRectify
{
public:
    stereoRectify();

    StereoParams_r stereoParams;
    RemapMatrixs_r remapMat;

    cv::Size imageSize;

    bool loadCalibDatas(string xmlFilePath);//功能 :导入双目相机标定参数

    int rectify(ParamsReader::RunParams& runParams);
    int rectifySingleCamera();//功能 : 生成单个摄像头的校正矩阵
    int rectifyStereoCamera(string method);//功能 : 执行双目摄像机校正，生成双目校正数据
    int remapImage(cv::Mat& img1, cv::Mat& img2, string method);//功能 : 对图像进行校正

    int saveStereoDatas(string filename, string method);//功能 :保存双目校正参数

};

#endif // STEREORECTIFY_H
