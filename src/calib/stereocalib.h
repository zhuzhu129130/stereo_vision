#ifndef STEREOCALIB_H
#define STEREOCALIB_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "../basefunc/basefunc.h"

using namespace std;

#define RESULT_OK   0
#define RESULT_FAIL 1

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


class stereoCalib
{
public:
    stereoCalib();
    ~stereoCalib();

    cv::Mat img_left;
    cv::Mat img_right;
    cv::Size imageSize;
    cv::Mat viewGray;

    CornerDatas cornerDatas;
    StereoParams stereoParams;

public:    
    void calib_process(ParamsReader::RunParams& runParams);//棋盘格进行立体校正
    void calibrationInit(int cornersX,int cornersY,float squareSize,CornerDatas& cornerDatas);//初始化角点参数，长宽，大小等
    int calibrationAddSample(cv::Mat& imageLeft,cv::Mat& imageRight,CornerDatas& cornerDatas); //功能 : 检测棋盘角点

    bool calibration(CornerDatas& cornerDatas,StereoParams& stereoParams); //进行立体标定
    bool calibSingle(CornerDatas& cornerDatas,StereoParams& stereoParams);
    double getSingleCalibrateError( const vector<vector<cv::Point3f> >& objectPoints,
                                             const vector<vector<cv::Point2f> >& imagePoints,
                                             const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
                                             const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                             vector<float>& perViewErrors);
    bool calibStereo(CornerDatas& cornerDatas,StereoParams& stereoParams);
    double getStereoCalibrateError(CornerDatas& cornerDatas,StereoParams& stereoParams);

    int loadCornerData(const char* filename, CornerDatas& cornerDatas);
    int saveCornerData(const char* filename, const CornerDatas& cornerDatas);
    int loadCameraParams(const char* filename, CameraParams& cameraParams);
    int saveCameraParams(const CameraParams& cameraParams, const char* filename = "cameraParams.yml");
    int loadCalibDatas(string xmlFilePath,StereoParams& stereoParams);
    int saveCalibDatas(const char* filename, CornerDatas& cornerDatas, StereoParams& stereoParams);


};

#endif // STEREOCALIB_H
