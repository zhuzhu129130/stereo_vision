#ifndef CALIB_DATATYPES_H
#define CALIB_DATATYPES_H
# pragma once  //只要在头文件的最开始加入这条杂注，就能够保证头文件只被编译一次

#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
using namespace std;

/***
 *	棋盘角点数据 结构体
 */
typedef struct _CornerDatas
{
    int			nPoints;			// 棋盘角点总数
    int			nImages;			// 棋盘图像数
    int			nPointsPerImage;	// 每幅棋盘的角点数
    cv::Size	imageSize;			// 图像分辨率
    cv::Size	boardSize;			// 棋盘尺寸
    vector<vector<cv::Point3f> >	objectPoints;	// 棋盘角点世界坐标序列
    vector<vector<cv::Point2f> >	imagePoints1;	// 左视图的棋盘角点像素坐标序列
    vector<vector<cv::Point2f> >	imagePoints2;	// 右视图的棋盘角点像素坐标序列
}CornerDatas;

/***
 *	单目标定的输出参数
 */
typedef struct _CameraParams
{
    cv::Size		imageSize;				// 图像分辨率
    cv::Mat			M;			// 摄像机矩阵
    cv::Mat			D;	// 摄像机畸变参数
    vector<cv::Mat> rvecs;				// 棋盘图片的旋转矩阵
    vector<cv::Mat>  tvecs;			// 棋盘图片的平移向量
    int				flags;					// 单目标定所用的标志位
}CameraParams;

/***
 *	双目标定的输出参数
 */
typedef struct _StereoParams
{
    cv::Size		imageSize;		// 图像分辨率
    CameraParams	cameraParams1;	// 左摄像机标定参数
    CameraParams	cameraParams2;	// 右摄像机标定参数
    cv::Mat			R;		// 旋转矩阵
    cv::Mat			T;	// 平移向量
    cv::Mat			E;		// 本质矩阵
    cv::Mat			F;	// 基础矩阵
    int				  flags;			// 双目标定所用的标志位
    double          alpha;          // 双目校正效果的缩放系数，取值 0~1 或 -1
}StereoParams;

/***
 *	双目校正的输出参数
 */
/*struct RemapMatrixs
{
    cv::Mat		mX1;	// 左视图 X 方向像素映射矩阵
    cv::Mat		mY1;	// 左视图 Y 方向像素映射矩阵
    cv::Mat		mX2;	// 右视图 X 方向像素映射矩阵
    cv::Mat		mY2;	// 右视图 Y 方向像素映射矩阵
    cv::Mat		Q;		    // 用于计算三维点云的 Q 矩阵
    cv::Rect	    roi1;	    // 左视图有效区域的矩形
    cv::Rect	    roi2;	    // 右视图有效区域的矩形
};*/

/***
 *	双目校正的输出参数
 */
typedef struct _RemapMatrixs
{
    cv::Mat       calib_M_L;
    cv::Mat       calib_D_L;
    cv::Mat       calib_M_R;
    cv::Mat       calib_D_R;

    cv::Mat		Calib_mX_L;	// 左视图 X 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mY_L;	// 左视图 Y 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mX_R;	// 右视图 X 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mY_R;	// 右视图 Y 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_Q;		    // 用于计算三维点云的 Q 矩阵
    cv::Rect	    Calib_Roi_L;	    // 左视图校正后的有效区域的矩形
    cv::Rect	    Calib_Roi_R;	    // 右视图校正后的有效区域的矩形
    cv::Mat        Calib_Mask_Roi;		// 左视图校正后的有效区域
}RemapMatrixs;

/***
 *	双目校正方法
 */
enum RECTIFYMETHOD { RECTIFY_BOUGUET, RECTIFY_HARTLEY };

//cv::Size imgSize;


#endif // CALIB_DATATYPES_H
