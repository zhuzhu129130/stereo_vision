/*************************************************************************
    > File Name: ar_cv.cpp
    > Author: zhu
    > Mail: zhqr010230@126.com
    > Created Time: 2016年04月08日 星期五 10时30分42秒
 ************************************************************************/
//# pragma once  //只要在头文件的最开始加入这条杂注，就能够保证头文件只被编译一次
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include "./basefunc/basefunc.h"
#include "./calib/stereocalib.h"
#include "./rectify/stereorectify.h"
#include "./reconstruct/stereoreconstruction.h"
using namespace std;

ParamsReader pd;
ParamsReader::RunParams runParams;

int startx, starty, endx, endy;
bool drawing = false;
static void onMouse(int event, int x, int y, int flags, void*);
cv::Mat disp8u;
stereoReconstruction *ss;

void createBarSgbm(SgbmParams& sgbmParams);

void updateSgbmParameters(cv::StereoSGBM& sgbm,SgbmParams& sgbmParams);


boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
    viewer->addCoordinateSystem ( 1.0 );
    viewer->initCameraParameters ();
    return viewer;
}

void init_runParams(ParamsReader::RunParams& runParams)
{
    //程序运行参数calib，rectify，reconstruct，process
    runParams.processType = pd.getData("processType");
    cout << runParams.processType <<endl;

    //===================立体标定参数=====================================
    //图像序列列表#calib_image/imagelist.xml
    runParams.calib_imagepath = pd.getData("imagepath");

    //立体标定参数保存文件目录
    runParams.stereocalib_path = pd.getData("stereocalib_path");

    //棋盘格长宽大小参数
    runParams.cornerX = atoi(pd.getData("cornerX").c_str());
    runParams.cornerY = atoi(pd.getData("cornerY").c_str());
    runParams.squareSize = atof(pd.getData("squareSize").c_str());
    //====================================================================

    //=============立体校正参数=============================================
    //校正参数计算后测试图像
    runParams.image1_test = pd.getData("image1_test");
    runParams.image2_test = pd.getData("image2_test");

    //立体校正参数 RECTIFY_HARTLEY，RECTIFY_BOUGUET
    runParams.rectifymethod = pd.getData("rectifymethod");

    runParams.rectifyParams_path = pd.getData("rectifyParams_path");
   //=============立体校正参数============================================

   //================立体重建参数=========================================
   //起始与终止索引
   runParams.start_index  =   atoi( pd.getData( "start_index" ).c_str() );
   runParams.end_index =   atoi( pd.getData( "end_index"   ).c_str() );

   //数据所在目录，及图像名称前缀和后缀
   runParams.img_left=pd.getData( "img_left" );
   runParams.left_extension=pd.getData( "left_extension" );
   runParams.img_right=pd.getData( "img_right" );
   runParams.right_extension=pd.getData( "right_extension" );

   //视差计算算法  BM , SGBM, VAR, ELAS
   runParams.DisparityType = pd.getData( "DisparityType" );
   //================立体重建参数=========================================

   //================数据源==============================================
   //三种数据源：image，camera，video
    runParams.file_type = pd.getData( "file_type" );
    cout << runParams.file_type <<endl;

    runParams.image1_test = pd.getData("image1_dir");
    runParams.image2_test = pd.getData("image2_dir");

    runParams.camera1 = atoi(pd.getData("camera1").c_str());
    runParams.camera2 = atoi(pd.getData("camera2").c_str());

    runParams.video1_dir = pd.getData("video1_dir");
    runParams.video2_dir = pd.getData("video2_dir");

    runParams.detector = pd.getData( "detector" );
    runParams.descriptor = pd.getData( "descriptor" );
    runParams.good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );

}

int main()
{
    stereoRectify  *sr;

    RemapMatrixs remapMat;

    init_runParams(runParams);

    if(runParams.processType == "calib")
    {
        stereoCalib * sc = new stereoCalib();
        sc->calib_process(runParams);
        delete sc;
    }

    if(runParams.processType == "rectify")
    {
        sr = new stereoRectify();
        sr->rectify(runParams);
        cv::Mat imgleft = cv::imread(runParams.image1_test,1);
        cv::Mat imgright = cv::imread(runParams.image2_test,1);
        sr->remapImage(imgleft, imgright, runParams.rectifymethod);
        delete sr;
    }

    if(runParams.processType == "reconstruct")
    {
        ss = new stereoReconstruction();

        int64 t = cv::getTickCount();

        cv::Mat img_left,img_right;

        cv::namedWindow("disparity", CV_WINDOW_AUTOSIZE);
        cv::setMouseCallback("disparity", onMouse);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = createVisualizer( point_cloud_ptr );

        if(runParams.file_type == "camera")
        {
            cv::VideoCapture cap;
            cv::Mat img_src;
            cap.open(runParams.camera1);
            //cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
            //cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
            if (!cap.isOpened())
            {
                cout << "capture device failed to open!" << endl;
                return 1;
             }
             int width;
             int height;
             while(cap.read(img_src))
             {
                 //------------------分割-----------------------------
                 width  = img_src.size().width/2;
                 height = img_src.size().height;

                 //---------扩展ROI区域-----------------------------
                 cv::Rect rect_L = cv::Rect(0,0,width,height);
                 img_left = img_src(rect_L);
                 cv::Rect rect_R = cv::Rect(width,0,width,height);
                 img_right = img_src(rect_R);

                 cv::imshow("left",img_left);
                 cv::imshow("right",img_right);
                 cv::waitKey(1);
             }
        }

        else if(runParams.file_type == "video")
        {
            cv::VideoCapture cap1,cap2;
            cap1.open(runParams.video1_dir);
            cap2.open(runParams.video2_dir);
            while(1)
            {
                cap1.read(img_left);
                cap2.read(img_right);
                cv::imshow("left",img_left);
                cv::imshow("right",img_right);
                cv::waitKey(1);
            }
        }

        else if(runParams.file_type == "image")
        {
            updateSgbmParameters(ss->sgbm,ss->sgbmParams);
            //createBarSgbm(ss->sgbmParams);
            while(1)
            {
                img_left = cv::imread(runParams.image1_test);
                img_right = cv::imread(runParams.image2_test);
                //cv::imshow("left",img_left);
                //cv::imshow("right",img_right);
                //cv::waitKey(1);

                ss->loadRectifyDatas(runParams.rectifyParams_path);
                //sr.remapImage(img_left,img_right,remapMat,"RECTIFY_BOUGUET");
                ss->Disp_compute(img_left,img_right,runParams);
                ss->reproject(ss->disp8u, img_left, point_cloud_ptr);
                cout << "PointCloud size: "<< point_cloud_ptr->size()<<"\n";

                disp8u = ss->disp8u;
                imshow("disparity", ss->disp8u);

                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
                viewer->updatePointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "reconstruction");
                viewer->spinOnce();

                cv::waitKey(1);
                t = cv::getTickCount() - t;
                cout<<"Time elapsed: "<< t*1000/cv::getTickFrequency() <<endl;
            }

        }

        delete ss;
    }

    if(runParams.processType == "process")
    {
        basefunc *bf = new basefunc();
        ss = new stereoReconstruction();

        cv::Mat  img_left,img_right;
        ss->loadRectifyDatas(runParams.rectifyParams_path);
        for ( int currIndex=runParams.start_index; currIndex<runParams.end_index+1; currIndex++ )
        {
                //cout<<"Reading files "<<currIndex<<endl;
                bf->readFrame( currIndex,img_left,img_right,runParams ); // 读取currFrame
                //sr.remapImage(img_left,img_right,remapMat,runParams.rectifymethod);
                ss->Disp_compute(img_left,img_right,runParams);
                ss->getPointClouds(ss->disparity,ss->pointClouds,remapMat);
                cv::imshow("pointClouds",ss->disparity);
                cv::waitKey(0);
                ss->saveDisp("../params/disp.txt",ss->disparity);
               // ss.getDisparityImage(ss.disparity,false);
               ss->getSideView(ss->pointClouds,img_left);
               ss->savePointClouds(ss->pointClouds,"../params/pointClouds.txt");

        }
        delete bf;
        delete ss;
    }
}

static void onMouse(int event, int x, int y, int flags, void*) {
    if (event==CV_EVENT_LBUTTONDOWN) {
        drawing = true;
        startx = x; starty = y;
    }
    else if (event == CV_EVENT_LBUTTONUP) {
        drawing = false;
        endx = x; endy = y;
        cv::Mat cropped = disp8u(cv::Rect(startx, starty, abs(startx-endx), abs(starty-endy)));
        cout << "mean: " << mean(cropped)[0] <<"\n";
        cv::rectangle(disp8u, cv::Point(startx, starty), cv::Point(x, y), cv::Scalar(0, 0, 255), 1);
        cv::imshow("disparity", disp8u);
        ss->getDepth(startx, starty, endx, endy);
    }
}

void updateSgbmParameters(cv::StereoSGBM& sgbm,SgbmParams& sgbmParams)
{
    if(sgbmParams.P2 <= sgbmParams.P1) {
      sgbmParams.P2 = sgbmParams.P1 + 1;
      cout << "Warning: P2 too small! Using P2 = " << sgbmParams.P2 << "\n";
    }
    int residue = sgbmParams.number_of_disparities % 16;
    if(residue != 0) {
      sgbmParams.number_of_disparities += (16 - residue);
      std::cout << "Warning: number_of_disparities \% 16 != 0! Using number_of_disparities = " << sgbmParams.number_of_disparities << "\n";
    }
    sgbm.preFilterCap = sgbmParams.pre_filter_cap;
    sgbm.SADWindowSize = sgbmParams.sad_window_size;
    sgbm.P1 = sgbmParams.P1;
    sgbm.P2 = sgbmParams.P2;
    sgbm.minDisparity = sgbmParams.min_disparity;
    sgbm.numberOfDisparities = sgbmParams.number_of_disparities;
    sgbm.uniquenessRatio = sgbmParams.uniqueness_ratio;
    sgbm.speckleWindowSize = sgbmParams.speckle_window_size;
    sgbm.speckleRange = sgbmParams.speckle_range;
    sgbm.disp12MaxDiff = sgbmParams.disp12_max_diff;
    sgbm.fullDP = sgbmParams.full_dp;
}

void on_trackbar(int, void*)
{
    if(runParams.DisparityType == "SGBM")
        updateSgbmParameters(ss->sgbm,ss->sgbmParams);
    //else if()
}

void createBarSgbm(SgbmParams& sgbmParams)
{
    cv::namedWindow("Parameters");
    cv::createTrackbar("preFilterCap", "Parameters", &sgbmParams.pre_filter_cap, sgbmParams.pre_filter_cap_max, on_trackbar);
    cv::createTrackbar("SADWindowSize", "Parameters", &sgbmParams.sad_window_size, sgbmParams.sad_window_size_max, on_trackbar);
    cv::createTrackbar("P1", "Parameters", &sgbmParams.P1, sgbmParams.P1_max, on_trackbar);
    cv::createTrackbar("P2", "Parameters", &sgbmParams.P2, sgbmParams.P2_max, on_trackbar);
    cv::createTrackbar("minDisparity", "Parameters", &sgbmParams.min_disparity, sgbmParams.min_disparity_max, on_trackbar);
    cv::createTrackbar("numberOfDisparities", "Parameters", &sgbmParams.number_of_disparities, sgbmParams.number_of_disparities_max, on_trackbar);
    cv::createTrackbar("uniquenessRatio", "Parameters", &sgbmParams.uniqueness_ratio, sgbmParams.uniqueness_ratio_max, on_trackbar);
    cv::createTrackbar("speckleWindowSize", "Parameters", &sgbmParams.speckle_window_size, sgbmParams.speckle_window_size_max, on_trackbar);
    cv::createTrackbar("speckleRange", "Parameters", &sgbmParams.speckle_range, sgbmParams.speckle_range_max, on_trackbar);
    cv::createTrackbar("disp12MaxDiff", "Parameters", &sgbmParams.disp12_max_diff, sgbmParams.disp12_max_diff_max, on_trackbar);
    cv::createTrackbar("fullDP", "Parameters", &sgbmParams.full_dp, sgbmParams.full_dp_max, on_trackbar);
    cv::imshow("Parameters", cv::Mat::zeros(1, 1400, CV_8U));
}





