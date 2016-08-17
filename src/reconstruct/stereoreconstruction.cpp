#include "stereoreconstruction.h"

stereoReconstruction::stereoReconstruction()
{
    ViewWidth = 400;
    ViewHeight = 400;
    ViewDepth = 400;
    numberOfDisparies = 0;
}

/*----------------------------
 * 功能 : 载入双目定标结果数据
 *----------------------------
 * 函数 : stereoReconstruction::loadRectifyDatas
 * 访问 : public
 * 返回 : 1		成功
 *		 0		读入校正参数失败
 *		 -1		定标参数的图像尺寸与当前配置的图像尺寸不一致
 *		 -2		校正方法不是 BOUGUET 方法
 *		 -99	未知错误
 *
 * 参数 : xmlFilePath	[in]	双目定标结果数据文件
 */
int stereoReconstruction::loadRectifyDatas(string xmlFilePath)
{
    // 读入摄像头定标参数 Q roi1 roi2 mapx1 mapy1 mapx2 mapy2
    try
    {
        cv::FileStorage fs(xmlFilePath, cv::FileStorage::READ);
        if ( !fs.isOpened() )
        {
            return (0);
        }

        cv::FileNodeIterator it = fs["imageSize"].begin();
        it >> imageSize.width >> imageSize.height;

        fs["leftCameraMatrix"] >> remapMat.calib_M_L;
        fs["leftDistortCoefficients"] >> remapMat.calib_D_L;
        fs["rightCameraMatrix"] >> remapMat.calib_M_R;
        fs["rightDistortCoefficients"] >> remapMat.calib_D_R;

        vector<int> roiVal1;
        vector<int> roiVal2;

        fs["leftValidArea"] >> roiVal1;
        remapMat.Calib_Roi_L.x = roiVal1[0];
        remapMat.Calib_Roi_L.y = roiVal1[1];
        remapMat.Calib_Roi_L.width = roiVal1[2];
        remapMat.Calib_Roi_L.height = roiVal1[3];

        fs["rightValidArea"] >> roiVal2;
        remapMat.Calib_Roi_R.x = roiVal2[0];
        remapMat.Calib_Roi_R.y = roiVal2[1];
        remapMat.Calib_Roi_R.width = roiVal2[2];
        remapMat.Calib_Roi_R.height = roiVal2[3];

        fs["QMatrix"] >> remapMat.Calib_Q;
        _cx = remapMat.Calib_Q.at<double>(0, 3);
        _cy = remapMat.Calib_Q.at<double>(1, 3);
        f = remapMat.Calib_Q.at<double>(2, 3);
        _tx_inv = remapMat.Calib_Q.at<double>(3, 2);
        _cx_cx_tx_inv = remapMat.Calib_Q.at<double>(3, 3);

        fs["remapX1"] >> remapMat.Calib_mX_L;
        fs["remapY1"] >> remapMat.Calib_mY_L;
        fs["remapX2"] >> remapMat.Calib_mX_R;
        fs["remapY2"] >> remapMat.Calib_mY_R;

        remapMat.Calib_Mask_Roi = cv::Mat::zeros(imageSize, CV_8UC1);
        cv::rectangle(remapMat.Calib_Mask_Roi, remapMat.Calib_Roi_L, cv::Scalar(255), -1);

        bm.state->roi1 = remapMat.Calib_Roi_L;
        bm.state->roi2 = remapMat.Calib_Roi_R;

        string method;
        fs["rectifyMethod"] >> method;
        if (method != "BOUGUET")
        {
            return (-2);
        }

    }
    catch (std::exception& e)
    {
        return (-99);
    }

    return 1;
}

void stereoReconstruction::Disp_compute(cv::Mat& imgleft,cv::Mat& imgright,ParamsReader::RunParams& runParams)
{
    SADWindowSize = 21; /**< Size of the block window. Must be odd */
    numberOfDisparities =16*2; /**< Range of disparity */
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((imageSize.width/8) + 15) & -16;

    float scale = 1.f;
    if( scale != 1.f )
    {
        cv::Mat temp1, temp2;
        int method = scale < 1 ? CV_INTER_AREA : CV_INTER_CUBIC;
        cv::resize(imgleft, temp1, cv::Size(), scale, scale, method);
        imgleft = temp1;
        cv::resize(imgright, temp2, cv::Size(), scale, scale, method);
        imgright = temp2;
    }

    if(runParams.DisparityType == "BM")
    {
        bmMatch(imgleft,imgright,remapMat,img1p,img2p);
    }
    if(runParams.DisparityType == "SGBM")
    {
        sgbmMatch(imgleft,imgright,remapMat,img1p,img2p,runParams);
    }
    if(runParams.DisparityType == "VAR")
    {
        varMatch(imgleft,imgright,remapMat,img1p,img2p);
    }
    if(runParams.DisparityType == "ELAS")
    {
        elasMatch(imgleft,imgright,remapMat,img1p,img2p);
    }

}

void stereoReconstruction::saveDisp(const char* filename, const cv::Mat& mat)
{
    FILE* fp = fopen(filename, "wt");
    fprintf(fp, "%02d\n", mat.rows);
    fprintf(fp, "%02d\n", mat.cols);
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            int disp = (int)mat.at<float>(y, x);  // 这里视差矩阵是CV_16S 格式的，故用 short 类型读取
            fprintf(fp, "%d\n", disp);          // 若视差矩阵是 CV_32F 格式，则用 float 类型读取
        }
        //fprintf(fp, "\n");
    }
    fclose(fp);
}

void stereoReconstruction::reproject(cv::Mat& disp8u, cv::Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    double px, py, pz, pw;
    unsigned char pb, pg, pr;
    for (int i=0;i<img.rows;i++) {
        uchar* rgb = img.ptr<uchar>(i);
        for (int j=0;j<img.cols;j++) {
            int d = static_cast<unsigned>(disp8u(cv::Rect(j, i, 1, 1)).at<uchar>(0));
            if (d==0) continue;
            pw = -1.0*(double) (d)*_tx_inv + _cx_cx_tx_inv;
            px = static_cast<double> (j) + _cx;
            py = static_cast<double> (i) + _cy;
            pz = f;

            px/=pw;
            py/=pw;
            pz/=pw; pz*=-1; //pz inverted

            if (img.channels() == 3)
            {
                pb = rgb[3*j];
                pg = rgb[3*j+1];
                pr = rgb[3*j+2];
            }
            else if(img.channels() == 1)
            {
                pb = rgb[j];
                pg = rgb[j];
                pr = rgb[j];
            }

            pcl::PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;

            uint32_t _rgb = ((uint32_t) pr << 16 |
              (uint32_t) pg << 8 | (uint32_t)pb);
            point.rgb = *reinterpret_cast<float*>(&_rgb);
            //ptr->clear();
            point_cloud_ptr->push_back(point);

        }
    }
    ptr = point_cloud_ptr;
    ptr->width = (int) ptr->points.size();
    ptr->height = 1;
    //pcl::io::savePCDFile("pcl.pcd", *ptr);
}

double stereoReconstruction::getDepth(int startx, int starty, int endx, int endy)
{
    double sum = 0; int count = 0;
    double sum_d = 0;
    for (int i = startx; i<=endx; i++) {
        for (int j = starty; j<=endy; j++) {
            int d = static_cast<unsigned>(disp8u(cv::Rect(i, j, 1, 1)).at<uchar>(0));
            if (d==0) continue;
            sum_d+=d;
            double pw = -1.0*(double)(d)*_tx_inv + _cx_cx_tx_inv;
            sum+=f/pw;
            count++;
        }
    }
    cout << std::fixed;
    cout << "Average distance: " << sum/count<<"\n";
    cout << "Average disparity: " << sum_d/count << "\n\n";
    return sum/count;
}

/*----------------------------
 * 功能 : 计算三维点云
 *----------------------------
 * 函数 : stereoReconstruction::getPointClouds
 * 访问 : public
 * 返回 : 0 - 失败，1 - 成功
 *
 * 参数 : disparity		[in]	视差数据
 * 参数 : pointClouds	[out]	三维点云
 */
int stereoReconstruction::getPointClouds(cv::Mat& disparity, cv::Mat& pointClouds,RemapMatrixs& remapMat)
{
    if (disparity.empty())
    {
        return 0;
    }

    //计算生成三维点云
    cv::reprojectImageTo3D(disparity, pointClouds, remapMat.Calib_Q, true);
    pointClouds *= 1.6;

    // 校正 Y 方向数据，正负反转
    // 原理参见：http://blog.csdn.net/chenyusiyuan/article/details/5970799
    for (int y = 0; y < pointClouds.rows; ++y)
    {
        for (int x = 0; x < pointClouds.cols; ++x)
        {
            cv::Point3f point = pointClouds.at<cv::Point3f>(y,x);
            point.y = -point.y;
            pointClouds.at<cv::Point3f>(y,x) = point;
        }
    }

    return 1;
}

/*----------------------------
 * 功能 : 获取伪彩色视差图
 *----------------------------
 * 函数 : stereoReconstruction::getDisparityImage
 * 访问 : public
 * 返回 : 0 - 失败，1 - 成功
 *
 * 参数 : disparity			[in]	原始视差数据
 * 参数 : disparityImage		[out]	伪彩色视差图
 * 参数 : isColor			[in]	是否采用伪彩色，默认为 true，设为 false 时返回灰度视差图
 */
int stereoReconstruction::getDisparityImage(cv::Mat& disparity, bool isColor)
{
    // 将原始视差数据的位深转换为 8 位
    //cv::Mat disp8u;
    cv::Mat disparityImage;
    if (disparity.depth() != CV_8U)
    {
        if (disparity.depth() == CV_8S)
        {
            disparity.convertTo(disp8u, CV_8U);
        }
        else
        {
            disparity.convertTo(disp8u, CV_8U, 255/(numberOfDisparies*16.));
        }
    }
    else
    {
        disp8u = disparity;
    }

    // 转换为伪彩色图像 或 灰度图像
    if (isColor)
    {
        if (disparityImage.empty() || disparityImage.type() != CV_8UC3 || disparityImage.size() != disparity.size())
        {
            disparityImage = cv::Mat::zeros(disparity.rows, disparity.cols, CV_8UC3);
        }

        for (int y=0;y<disparity.rows;y++)
        {
            for (int x=0;x<disparity.cols;x++)
            {
                uchar val = disp8u.at<uchar>(y,x);
                uchar r,g,b;

                if (val==0)
                    r = g = b = 0;
                else
                {
                    r = 255-val;
                    g = val < 128 ? val*2 : (uchar)((255 - val)*2);
                    b = val;
                }

                disparityImage.at<cv::Vec3b>(y,x) = cv::Vec3b(r,g,b);
            }
        }
    }
    else
    {
        disp8u.copyTo(disparityImage);
    }

#if DEBUG_SHOW
    cv::imshow("color",disparityImage);
    cv::waitKey(1);
#endif

    return 1;
}


/*----------------------------
 * 功能 : 获取环境俯视图
 *----------------------------
 * 函数 : stereoReconstruction::savePointClouds
 * 访问 : public
 * 返回 : void
 *
 * 参数 : pointClouds	[in]	三维点云数据
 * 参数 : topDownView	[out]	环境俯视图
 * 参数 : image       	[in]	环境图像
 */
void stereoReconstruction::getTopDownView(cv::Mat& pointClouds, cv::Mat& image /*= cv::Mat()*/)
{
    cv::Mat topDownView;
    int VIEW_WIDTH = ViewWidth, VIEW_DEPTH = ViewDepth;
    cv::Size mapSize = cv::Size(VIEW_DEPTH, VIEW_WIDTH);

    if (topDownView.empty() || topDownView.size() != mapSize || topDownView.type() != CV_8UC3)
        topDownView = cv::Mat(mapSize, CV_8UC3);

    topDownView = cv::Scalar::all(50);

    if (pointClouds.empty())
        return;

    if (image.empty() || image.size() != pointClouds.size())
        image = 255 * cv::Mat::ones(pointClouds.size(), CV_8UC3);

    for(int y = 0; y < pointClouds.rows; y++)
    {
        for(int x = 0; x < pointClouds.cols; x++)
        {
            cv::Point3f point = pointClouds.at<cv::Point3f>(y, x);
            int pos_Z = point.z;

            if ((0 <= pos_Z) && (pos_Z < VIEW_DEPTH))
            {
                int pos_X = point.x + VIEW_WIDTH/2;
                if ((0 <= pos_X) && (pos_X < VIEW_WIDTH))
                {
                    topDownView.at<cv::Vec3b>(pos_X,pos_Z) = image.at<cv::Vec3b>(y,x);
                }
            }
        }
    }
#if 1//DEBUG_SHOW
    cv::imshow("TopDownView",topDownView);
    cv::waitKey(1);
#endif
}

/*----------------------------
 * 功能 : 获取环境视图
 *----------------------------
 * 函数 : stereoReconstruction::getSideView
 * 访问 : public
 * 返回 : void
 *
 * 参数 : pointClouds	[in]	三维点云数据
 * 参数 : sideView    	[out]	环境侧视图
 * 参数 : image       	[in]	环境图像
 */
void stereoReconstruction::getSideView(cv::Mat& pointClouds,  cv::Mat& image /*= cv::Mat()*/)
{
    cv::Mat sideView;
    int VIEW_HEIGTH = ViewHeight, VIEW_DEPTH = ViewDepth;
    cv::Size mapSize = cv::Size(VIEW_DEPTH, VIEW_HEIGTH);

    if (sideView.empty() || sideView.size() != mapSize || sideView.type() != CV_8UC3)
        sideView = cv::Mat(mapSize, CV_8UC3);

    sideView = cv::Scalar::all(50);

    if (pointClouds.empty())
        return;

    if (image.empty() || image.size() != pointClouds.size())
        image = 255 * cv::Mat::ones(pointClouds.size(), CV_8UC3);

    for(int y = 0; y < pointClouds.rows; y++)
    {
        for(int x = 0; x < pointClouds.cols; x++)
        {
            cv::Point3f point = pointClouds.at<cv::Point3f>(y, x);
            int pos_Y = -point.y + VIEW_HEIGTH/2;
            int pos_Z = point.z;

            if ((0 <= pos_Z) && (pos_Z < VIEW_DEPTH))
            {
                if ((0 <= pos_Y) && (pos_Y < VIEW_HEIGTH))
                {
                    sideView.at<cv::Vec3b>(pos_Y,pos_Z) = image.at<cv::Vec3b>(y,x);
                }
            }
        }
    }
#if 1//DEBUG_SHOW
    cv::imshow("sideView",sideView);
    cv::waitKey(1);
#endif
}


/*----------------------------
 * 功能 : 保存三维点云到本地 txt 文件
 *----------------------------
 * 函数 : StereoReconstruction::savePointClouds
 * 访问 : public
 * 返回 : void
 *
 * 参数 : pointClouds	[in]	三维点云数据
 * 参数 : filename		[in]	文件路径
 */
void stereoReconstruction::savePointClouds(cv::Mat& pointClouds, const char* filename)
{
    const double max_z = 1.0e4;
    try
    {
        FILE* fp = fopen(filename, "wt");
        for(int y = 0; y < pointClouds.rows; y++)
        {
            for(int x = 0; x < pointClouds.cols; x++)
            {
                cv::Vec3f point = pointClouds.at<cv::Vec3f>(y, x);
                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
                    fprintf(fp, "%d %d %d\n", 0, 0, 0);
                else
                    fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
            }
        }
        fclose(fp);
    }
    catch (std::exception* e)
    {
        printf("Failed to save point clouds. Error: %s \n\n", e->what());
    }
}

/*----------------------------
 * 功能 : 基于 BM 算法计算视差
 *----------------------------
 * 函数 : stereoReconstruction::bmMatch
 * 访问 : public
 * 返回 : 0 - 失败，1 - 成功
 *
 * 参数 : imgeft		[in]	左摄像机帧图
 * 参数 : imgright		[in]	右摄像机帧图
 * 参数 : disparity		[out]	视差图
 * 参数 : imageLeft		[out]	处理后的左视图，用于显示
 * 参数 : imageRight		[out]	处理后的右视图，用于显示
 */
int stereoReconstruction::bmMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight)
{
    // 输入检查
    if (imgleft.empty() || imgright.empty())
    {
        disparity = cv::Scalar(0);
        return 0;
    }

    SADWindowSize = 19;
    //bm.state->roi1 = remapMat.Calib_Roi_L;//左右视图的有效像素区域，一般由双目校正阶段的 cvStereoRectify 函数传递，也可以自行设定。
    //bm.state->roi2 = remapMat.Calib_Roi_R;//一旦在状态参数中设定了 roi1 和 roi2，OpenCV 会通过cvGetValidDisparityROI 函数计算出视差图的有效区域，在有效区域外的视差值将被清零。
    //bm.State->preFilterSize=41;//预处理滤波器窗口大小,5-21,odd
    bm.state->preFilterCap = 31; //63,1-31//预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值,
    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9; //SAD窗口大小5-21
    bm.state->minDisparity = 0; //64 最小视差，默认值为 0
    bm.state->numberOfDisparities = numberOfDisparities; //128视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍
    bm.state->textureThreshold = 10;//低纹理区域的判断阈值。如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，则该窗口对应的像素点的视差值为 0
    bm.state->uniquenessRatio = 25;//5-15 视差唯一性百分比， 视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0
    bm.state->speckleWindowSize = 100;//检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查
    bm.state->speckleRange = 32;//视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
    bm.state->disp12MaxDiff = -1;//左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。超过该阈值的视差值将被清零。该参数默认为 -1，即不执行左右视差检查。
                                                      //注意在程序调试阶段最好保持该值为 -1，以便查看不同视差窗口生成的视差效果。

    // 转换为灰度图
    cv::cvtColor(imgleft,img1gray,CV_BGR2GRAY);//numberOfDisparies
    cv::cvtColor(imgright,img2gray,CV_BGR2GRAY);

    // 校正图像，使左右视图行对齐
    cv::remap(img1gray, img1remap, remapMat.Calib_mX_L, remapMat.Calib_mY_L, cv::INTER_LINEAR);		// 对用于视差计算的画面进行校正
    cv::remap(img2gray, img2remap, remapMat.Calib_mX_R, remapMat.Calib_mY_R, cv::INTER_LINEAR);

    // 对左右视图的左边进行边界延拓，以获取与原始视图相同大小的有效视差区域
    if (numberOfDisparies != bm.state->numberOfDisparities)
        numberOfDisparies = bm.state->numberOfDisparities;
    cv::copyMakeBorder(img1remap, img1border, 0, 0, bm.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    cv::copyMakeBorder(img2remap, img2border, 0, 0, bm.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    // 计算视差
    bm(img1border, img2border, dispborder);

    // 截取与原始画面对应的视差区域（舍去加宽的部分）
    disp = dispborder.colRange(bm.state->numberOfDisparities, img1border.cols);
    disp.copyTo(disparity, remapMat.Calib_Mask_Roi);

    // 输出处理后的图像
    imageLeft = img1remap.clone();
    imageRight = img2remap.clone();
    cv::rectangle(imageLeft, remapMat.Calib_Roi_L, CV_RGB(0,0,255), 3);
    cv::rectangle(imageRight, remapMat.Calib_Roi_R, CV_RGB(0,0,255), 3);

    //-- Check its extreme values
    cv::minMaxLoc( disp, &minVal, &maxVal );
#if DEBUF_INFO_SHOW
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)
#endif

    //-- 4. Display it as a CV_8UC1 image
    disparity.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

#if DEBUG_SHOW
    cv::imshow("left",imgleft);
    cv::imshow("right",imgright);
    cv::imshow("Disp",disp8u);
    cv::waitKey(1);
#endif

    return 1;
}

/*----------------------------
 * 功能 : 基于 SGBM 算法计算视差
 *----------------------------
 * 函数 : stereoReconstruction::sgbmMatch
 * 访问 : public
 * 返回 : 0 - 失败，1 - 成功
 *
 * 参数 : frameLeft		[in]	左摄像机帧图
 * 参数 : frameRight		[in]	右摄像机帧图
 * 参数 : disparity		[out]	视差图
 * 参数 : imageLeft		[out]	处理后的左视图，用于显示
 * 参数 : imageRight		[out]	处理后的右视图，用于显示
 */
int stereoReconstruction::sgbmMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight,ParamsReader::RunParams& runParams)
{
   /* // 输入检查
    if (imgleft.empty() || imgright.empty())
    {
        disparity = cv::Scalar(0);
        return 0;
    }

    // 复制图像
    cv::Mat img1proc, img2proc;
    imgleft.copyTo(img1proc);
    imgright.copyTo(img2proc);*/

    SADWindowSize = 7;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3; //3-11

    int cn = imgleft.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;//P1、P2的值越大，视差越平滑。P2>P1，可取（50，800）或者（40，2500）
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities; //128,256
    sgbm.uniquenessRatio = 25;    //10,0
    sgbm.speckleWindowSize = 100; //200
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = -1;
    sgbm.fullDP = (runParams.DisparityType == "STEREO_HH");

    // 校正图像，使左右视图行对齐
    cv::remap(imgleft, img1remap, remapMat.Calib_mX_L, remapMat.Calib_mY_L, cv::INTER_LINEAR);		// 对用于视差计算的画面进行校正
    cv::remap(imgright, img2remap, remapMat.Calib_mX_R, remapMat.Calib_mY_R, cv::INTER_LINEAR);

    // 对左右视图的左边进行边界延拓，以获取与原始视图相同大小的有效视差区域
    if (numberOfDisparies != sgbm.numberOfDisparities)
        numberOfDisparies = sgbm.numberOfDisparities;
    cv::copyMakeBorder(img1remap, img1border, 0, 0, sgbm.numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    cv::copyMakeBorder(img2remap, img2border, 0, 0, sgbm.numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    // 计算视差
    sgbm(img1border, img2border, dispborder);

    // 截取与原始画面对应的视差区域（舍去加宽的部分）
    disp = dispborder.colRange(sgbm.numberOfDisparities, img1border.cols);
    disp.copyTo(disparity, remapMat.Calib_Mask_Roi);

    // 输出处理后的图像
    imageLeft = img1remap.clone();
    imageRight = img2remap.clone();
    cv::rectangle(imageLeft, remapMat.Calib_Roi_L, CV_RGB(0,255,0), 3);
    cv::rectangle(imageRight, remapMat.Calib_Roi_R, CV_RGB(0,255,0), 3);

    //-- Check its extreme values
    cv::minMaxLoc( disparity, &minVal, &maxVal );
#if DEBUF_INFO_SHOW
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)
#endif

    //-- 4. Display it as a CV_8UC1 image
    disparity.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

#if DEBUG_SHOW
    cv::imshow("left",imgleft);
    cv::imshow("right",imgright);
    cv::imshow("Disp",disp8u);
    cv::waitKey(1);
#endif

    return 1;
}

/*----------------------------
 * 功能 : 基于 VAR 算法计算视差
 *----------------------------
 * 函数 : stereoReconstruction::varMatch
 * 访问 : public
 * 返回 : 0 - 失败，1 - 成功
 *
 * 参数 : frameLeft		[in]	左摄像机帧图
 * 参数 : frameRight		[in]	右摄像机帧图
 * 参数 : disparity		[out]	视差图
 * 参数 : imageLeft		[out]	处理后的左视图，用于显示
 * 参数 : imageRight		[out]	处理后的右视图，用于显示
 */
int stereoReconstruction::varMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight)
{
   /* // 输入检查
    if (imgleft.empty() || imgright.empty())
    {
        disparity = cv::Scalar(0);
        return 0;
    }

    // 复制图像
    cv::Mat img1proc, img2proc;
    imgleft.copyTo(img1proc);
    imgright.copyTo(img2proc);*/

    var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
    var.nIt = 25;
    var.minDisp = -numberOfDisparities;
    var.maxDisp = 0; //var.minDisp+numberOfDisparities
    var.poly_n = 3;
    var.poly_sigma = 0.0;
    var.fi = 15.0f;
    var.lambda = 0.03f;
    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;


    // 校正图像，使左右视图行对齐
    cv::remap(imgleft, img1remap, remapMat.Calib_mX_L, remapMat.Calib_mY_L, cv::INTER_LINEAR);		// 对用于视差计算的画面进行校正
    cv::remap(imgright, img2remap, remapMat.Calib_mX_R, remapMat.Calib_mY_R, cv::INTER_LINEAR);

    // 对左右视图的左边进行边界延拓，以获取与原始视图相同大小的有效视差区域
    if (numberOfDisparies != var.maxDisp)
        numberOfDisparies = var.maxDisp;
    cv::copyMakeBorder(img1remap, img1border, 0, 0, var.maxDisp, 0, IPL_BORDER_REPLICATE);
    cv::copyMakeBorder(img2remap, img2border, 0, 0, var.maxDisp, 0, IPL_BORDER_REPLICATE);

    // 计算视差
    var(img1border, img2border, dispborder);

    // 截取与原始画面对应的视差区域（舍去加宽的部分）
    disp = dispborder.colRange(var.maxDisp, img1border.cols);
    disp.copyTo(disparity, remapMat.Calib_Mask_Roi);

    // 输出处理后的图像
    imageLeft = img1remap.clone();
    imageRight = img2remap.clone();
    cv::rectangle(imgright, remapMat.Calib_Roi_L, CV_RGB(0,0,255), 3);
    cv::rectangle(imgright, remapMat.Calib_Roi_L, CV_RGB(0,0,255), 3);

    //-- Check its extreme values
    cv::minMaxLoc( disp, &minVal, &maxVal );
#if DEBUF_INFO_SHOW
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)
#endif

    //-- 4. Display it as a CV_8UC1 image
    disparity.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

#if DEBUG_SHOW
    cv::imshow("left",imgleft);
    cv::imshow("right",imgright);
    cv::imshow("Disp",disp8u);
    cv::waitKey(1);
#endif

    return 1;
}

/*----------------------------
 * 功能 : 基于 Elas 算法计算视差
 *----------------------------
 * 函数 : stereoReconstruction::elasMatch
 * 访问 : public
 * 返回 : 0 - 失败，1 - 成功
 *
 * 参数 : imgLeft		[in]	左摄像机帧图
 * 参数 : imgRight		[in]	右摄像机帧图
 * 参数 : disparity		[out]	视差图
 * 参数 : imageLeft		[out]	处理后的左视图，用于显示
 * 参数 : imageRight		[out]	处理后的右视图，用于显示
 */
int stereoReconstruction::elasMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight)
{
    // 转换为灰度图
    cv::cvtColor(imgleft,img1gray,CV_BGR2GRAY);//numberOfDisparies
    cv::cvtColor(imgright,img2gray,CV_BGR2GRAY);

    // 校正图像，使左右视图行对齐
    cv::remap(img1gray, img1remap, remapMat.Calib_mX_L, remapMat.Calib_mY_L, cv::INTER_LINEAR);		// 对用于视差计算的画面进行校正
    cv::remap(img2gray, img2remap, remapMat.Calib_mX_R, remapMat.Calib_mY_R, cv::INTER_LINEAR);

    // 计算视差
    // generate disparity image using LIBELAS
    int bd = 0;
    const int32_t dims[3] = {imageSize.width,imageSize.height,imageSize.width};
    cv::Mat leftdpf = cv::Mat::zeros(imageSize, CV_32F);
    cv::Mat rightdpf = cv::Mat::zeros(imageSize, CV_32F);
    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);
    elas.process(img1remap.data,img2remap.data,leftdpf.ptr<float>(0),rightdpf.ptr<float>(0),dims);
    cv::Mat(leftdpf(cv::Rect(bd,0,imgleft.cols,imgleft.rows))).copyTo(disp);
    disp.convertTo(disparity,CV_16S,16);

    // 输出处理后的图像
    imageLeft = img1remap.clone();
    imageRight = img2remap.clone();
    cv::rectangle(imageLeft, remapMat.Calib_Roi_L, CV_RGB(0,0,255), 3);
    cv::rectangle(imageRight, remapMat.Calib_Roi_R, CV_RGB(0,0,255), 3);

    //-- Check its extreme values
    cv::minMaxLoc( disparity, &minVal, &maxVal );
#if DEBUF_INFO_SHOW
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)
#endif

    //-- 4. Display it as a CV_8UC1 image
    disparity.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.));
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

#if DEBUG_SHOW
    cv::imshow("left",imgleft);
    cv::imshow("right",imgright);
    cv::imshow("Disp",disp8u);
    cv::waitKey(1);
#endif

    return 1;
}

/*----------------------------
 * 功能 : 基于 sgm 算法计算视差
 *----------------------------
 * 函数 : stereoReconstruction::sgmMatch
 * 访问 : public
 * 返回 : 0 - 失败，1 - 成功
 *
 * 参数 : imgLeft		[in]	左摄像机帧图
 * 参数 : imgRight		[in]	右摄像机帧图
 * 参数 : disparity		[out]	视差图
 * 参数 : imageLeft		[out]	处理后的左视图，用于显示
 * 参数 : imageRight		[out]	处理后的右视图，用于显示
 */
/*int stereoReconstruction::sgmMatch(cv::Mat& imgleft, cv::Mat& imgright,RemapMatrixs& remapMat, cv::Mat& imageLeft, cv::Mat& imageRight)
{
    // 转换为灰度图
    cv::cvtColor(imgleft,img1gray,CV_BGR2GRAY);//numberOfDisparies
    cv::cvtColor(imgright,img2gray,CV_BGR2GRAY);

    // 校正图像，使左右视图行对齐
    cv::remap(img1gray, img1remap, remapMat.Calib_mX_L, remapMat.Calib_mY_L, cv::INTER_LINEAR);		// 对用于视差计算的画面进行校正
    cv::remap(img2gray, img2remap, remapMat.Calib_mX_R, remapMat.Calib_mY_R, cv::INTER_LINEAR);

    double sigma = 0.7;
    cv::GaussianBlur(img1remap, img1remap, cv::Size(3,3), sigma);
    cv::GaussianBlur(img2remap, img2remap, cv::Size(3,3), sigma);

    recon::StereoSGMParams sgm_params;
    sgm_params.disp_range = 128; //256
    //sgm_params.window_sz = 1;
    sgm_params.window_sz = 5;  //3
    //sgm_params.penalty1 = 15;
    //sgm_params.penalty1 = 3;         //best - 3 Daimler - 10
    //sgm_params.penalty2 = 100;
    sgm_params.penalty2 = 60;        // best - 60 Daimler - 50
    // Axel tractor
    sgm_params.disp_range = 270;
    sgm_params.penalty1 = 7;

    recon::StereoSGM sgm(sgm_params);
    sgm.compute(img1remap, img2remap, disp);
    disp.convertTo(disparity,CV_16S,16);

    // 输出处理后的图像
    imageLeft = img1remap.clone();
    imageRight = img2remap.clone();
    cv::rectangle(imageLeft, remapMat.Calib_Roi_L, CV_RGB(0,0,255), 3);
    cv::rectangle(imageRight, remapMat.Calib_Roi_R, CV_RGB(0,0,255), 3);

    //-- Check its extreme values
    cv::minMaxLoc( disparity, &minVal, &maxVal );
#if DEBUF_INFO_SHOW
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)
#endif

    //-- 4. Display it as a CV_8UC1 image
    disparity.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.));
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image
    //cv::equalizeHist(disp8u, disp8u);

#if DEBUG_SHOW
    cv::imshow("left",imgleft);
    cv::imshow("right",imgright);
    cv::imshow("Disp",disp8u);
    cv::waitKey(1);
#endif

    return 1;

}*/

//功能 : 对图像进行校正
int stereoReconstruction::remapImage(cv::Mat& imgleft, cv::Mat& imgright, RemapMatrixs& remapMat,string method)
{
   cv::Mat imgleft_c = imgleft.clone();
   cv::Mat imgright_c = imgright.clone();

#if DEBUG_SHOW
   cv::Mat canvas;
   double sf;
   int w, h;
   sf = 600./MAX(imageSize.width, imageSize.height);
   w = cvRound(imageSize.width*sf);
   h = cvRound(imageSize.height*sf);
   canvas.create(h, w*2, CV_8UC3);
#endif

   if ( !remapMat.Calib_mX_L.empty() && !remapMat.Calib_mY_L.empty() )
   {
       cv::remap( imgleft_c, imgleft, remapMat.Calib_mX_L, remapMat.Calib_mY_L, cv::INTER_LINEAR );

#if DEBUG_SHOW
       cv::Mat canvasPart = canvas(cv::Rect(0, 0, w, h));
       cv::resize(imgleft, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
       if(method == "RECTIFY_BOUGUET")
       {
           cv::Rect vroi(cvRound(remapMat.Calib_Roi_L.x*sf), cvRound(remapMat.Calib_Roi_L.y*sf),
                                         cvRound(remapMat.Calib_Roi_L.width*sf), cvRound(remapMat.Calib_Roi_L.height*sf));
            cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
       }
#endif
   }
   if ( !remapMat.Calib_mX_R.empty() && !remapMat.Calib_mY_R.empty() )
   {
       cv::remap( imgright_c, imgright, remapMat.Calib_mX_R, remapMat.Calib_mY_R, cv::INTER_LINEAR );

#if DEBUG_SHOW
       cv::Mat canvasPart = canvas(cv::Rect(w, 0, w, h));
       cv::resize(imgright, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
       if(method == "RECTIFY_BOUGUET")
       {
           cv::Rect vroi(cvRound(remapMat.Calib_Roi_R.x*sf), cvRound(remapMat.Calib_Roi_R.y*sf),
                                         cvRound(remapMat.Calib_Roi_R.width*sf), cvRound(remapMat.Calib_Roi_R.height*sf));
            cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
       }
#endif
   }

#if DEBUG_SHOW
   for(int j = 0; j < canvas.rows; j += 16 )
        cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);

   cv::imshow("rectify_left",imgleft);
   cv::imshow("rectify_right",imgright);
   cv::imshow("rectified_image", canvas);
   cv::waitKey(0);
#endif

   /*
   cv::Mat img_l_remap,img_r_remap;
   remap(imgleft, img_l_remap, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
   remap(imgright, img_r_remap, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
   int rows = imgleft.rows, cols = imgleft.cols;
   for (int i = 40; i<rows; i+=40) {
       line(imgleft, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
       line(imgright, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
       line(img_l_remap, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
       line(img_r_remap, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
   }
   cv::imshow("imgsrc_L",imgleft);
   cv::imshow("imgsrc_r",imgright);
   cv::imshow("img_l_remap",img_l_remap);
   cv::imshow("img_r_remap",img_r_remap);
   cv::waitKey(1);
   */
   return 1;
}
