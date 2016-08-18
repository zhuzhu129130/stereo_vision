#ifndef STEREORECONSTRUCTION_H
#define STEREORECONSTRUCTION_H

#include <stdio.h>
#include <fstream>
#include <map>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../basefunc/basefunc.h"
#include "./libelas/elas.h"
#include "./sgm/stereo_sgm.h"

using namespace std;

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

typedef struct _SgbmParams
{
    int sad_window_size;                  // 3 - 11
    int sad_window_size_max;
    int P1;                              // 70
    int P1_max;
    int P2;                             // 800
    int P2_max;
    int frame_num;
    int frame_num_max;
    int pre_filter_cap;                  // 70
    int pre_filter_cap_max;
    int min_disparity;
    int min_disparity_max;
    //int number_of_disparities;          // or 256 max
    int number_of_disparities;          // or 256 max
    int number_of_disparities_max;
    int uniqueness_ratio;                 // 0 - 15
    int uniqueness_ratio_max;
    int speckle_window_size;            // 100
    int speckle_window_size_max;
    int speckle_range;
    int speckle_range_max;
    int disp12_max_diff;                 // 1
    int disp12_max_diff_max;
    int full_dp;
    int full_dp_max;

}SgbmParams;


class stereoReconstruction
{
public:
    stereoReconstruction();

    RemapMatrixs remapMat;
    double _cx, _cy, f, _tx_inv, _cx_cx_tx_inv;

    cv::Size imageSize;
    cv::Mat img1gray, img2gray; //灰度图
    cv::Mat img1remap, img2remap; //校正后的图像
    cv::Mat img1border, img2border; //边界延拓的图像
    cv::Mat dispborder; //边界延拓后的视差图
    cv::Mat disp; //与原始图像一致的视差图
    cv::Mat disparity; //16位的视差图
    cv::Mat disp8u; //8位的视差图
    cv::Mat img1p,img2p; //与视差图一致的校正图

    cv::Mat pointClouds;

    cv::StereoBM		bm;				// 立体匹配 BM 方法
    cv::StereoSGBM		sgbm;	// 立体匹配 SGBM 方法
    cv::StereoVar		var;			// 立体匹配 VAR 方法
    double				FL;				// 左摄像机校正后的焦距值
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };

    SgbmParams sgbmParams;

    int ViewWidth;                   // 视场宽度
    int ViewHeight;                  // 视场高度
    int ViewDepth;                   // 视场深度

    int numberOfDisparies;			// 视差变化范围
    int SADWindowSize; /**< Size of the block window. Must be odd */
    int  numberOfDisparities; /**< Range of disparity */

    double minVal; double maxVal; //视差图的极值

public:
    int loadRectifyDatas(string xmlFilePath);

    void stereo_Reconstruction(string xmlFilePath,RemapMatrixs& remapMat,ParamsReader::RunParams& runParams);

    void Disp_compute(cv::Mat& imgleft,cv::Mat& imgright,ParamsReader::RunParams& runParams);

    void saveDisp(const char* filename, const cv::Mat& mat);

    int getPointClouds(cv::Mat& disparity, cv::Mat& pointClouds,RemapMatrixs& remapMat);

    void savePointClouds(cv::Mat& pointClouds, const char* filename);

    int getDisparityImage(cv::Mat& disparity, bool isColor = true);

    void getTopDownView(cv::Mat& pointClouds, cv::Mat& image /*= cv::Mat()*/);

    void getSideView(cv::Mat& pointClouds,cv::Mat& image /*= cv::Mat()*/);

    int bmMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight);

    int sgbmMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight,ParamsReader::RunParams& runParams);

    int varMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight);

    int elasMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight);

    //int sgmMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight);

    int remapImage(cv::Mat& imgleft, cv::Mat& imgright, RemapMatrixs& remapMat,string method = "RECTIFY_BOUGUET");

    void reproject(cv::Mat& disp, cv::Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr);

    double getDepth(int startx, int starty, int endx, int endy);

};

#endif // STEREORECONSTRUCTION_H
