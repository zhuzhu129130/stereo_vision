#include "stereorectify.h"

stereoRectify::stereoRectify()
{
}


//功能 :导入双目相机标定参数
bool stereoRectify::loadCalibDatas(string xmlFilePath)
{
    cv::FileStorage fs(xmlFilePath, cv::FileStorage::READ);
    if (fs.isOpened())
    {
        cv::FileNodeIterator it = fs["imageSize"].begin();
        it >> imageSize.width >> imageSize.height;

        fs["leftCameraMatrix"]				>> stereoParams.calib_M_L;
        fs["leftDistortCoefficients"]	    >> stereoParams.calib_D_L;
        fs["rightCameraMatrix"]		    >> stereoParams.calib_M_R;
        fs["rightDistortCoefficients"]	>> stereoParams.calib_D_R;
        cv::Mat_<float> rotMat;
        fs["rotationMatrix"] >> rotMat;
        cv::Rodrigues(rotMat,stereoParams.R);//由向量转换成3x3矩阵。
        fs["translationVector"] >>stereoParams.T;
    }
    fs.release();

    return 1;
}

int stereoRectify::rectify(ParamsReader::RunParams& runParams)
{
    cout << "Rectify start..." << endl;

    loadCalibDatas(runParams.stereocalib_path);

    rectifyStereoCamera(runParams.rectifymethod);

    //rectifySingleCamera(stereoParams,remapMat);

    saveStereoDatas(runParams.rectifyParams_path,runParams.rectifymethod);

    cout << "Rectify END!" << endl;
    return 1;
}

//功能 : 生成单个摄像头的校正矩阵
int stereoRectify::rectifySingleCamera()
{
   cv::initUndistortRectifyMap(
       stereoParams.calib_M_L,
       stereoParams.calib_D_L,
       cv::Mat(),
       cv::getOptimalNewCameraMatrix(
           stereoParams.calib_M_L,
           stereoParams.calib_D_L,
           imageSize, 1, imageSize, 0),
           imageSize,
           CV_16SC2,
           remapMat.Calib_mX_L,
           remapMat.Calib_mY_L);

   return true;
}

//功能 : 执行双目摄像机校正，生成双目校正数据
int stereoRectify::rectifyStereoCamera(string method = "RECTIFY_BOUGUET")
{
   //初始化
   remapMat.Calib_mX_L = cv::Mat(imageSize, CV_32FC1);
   remapMat.Calib_mY_L = cv::Mat(imageSize, CV_32FC1);
   remapMat.Calib_mX_R = cv::Mat(imageSize, CV_32FC1);
   remapMat.Calib_mY_R = cv::Mat(imageSize, CV_32FC1);

   cv::Mat R1, R2, P1, P2, Q;
   cv::Rect roi1, roi2;
   //执行双目校正
   cv::stereoRectify(
       stereoParams.calib_M_L, stereoParams.calib_D_L,
       stereoParams.calib_M_R, stereoParams.calib_D_R,
       imageSize,
       stereoParams.R,stereoParams.T,
       R1,R2, P1, P2, Q,
       cv::CALIB_ZERO_DISPARITY, 1,
       imageSize,
       &roi1, &roi2);

   //使用HARTLEY方法的额外处理
   if (method == "RECTIFY_HARTLEY")
   {
     /*  vector<cv::Point2f> allimgpt[2];
       for(int  i = 0; i < cornerDatas.nImages; i++ )
       {
           copy(cornerDatas.imagePoints1[i].begin(), cornerDatas.imagePoints1[i].end(), back_inserter(allimgpt[0]));
           copy(cornerDatas.imagePoints2[i].begin(), cornerDatas.imagePoints2[i].end(), back_inserter(allimgpt[1]));
       }

       cv::Mat F, H1, H2;
       F = cv::findFundamentalMat(
                   cv::Mat(allimgpt[0]),
                   cv::Mat(allimgpt[1]),
                   cv::FM_8POINT, 0, 0);
       cv::stereoRectifyUncalibrated(
                   cv::Mat(allimgpt[0]),
                   cv::Mat(allimgpt[1]),
           F, cornerDatas.imageSize, H1, H2, 3);

       R1 = stereoParams.cameraParams1.M.inv() * H1 * stereoParams.cameraParams1.M;
       R2 = stereoParams.cameraParams2.M.inv() * H2 * stereoParams.cameraParams2.M;
       P1 = stereoParams.cameraParams1.M;
       P2 = stereoParams.cameraParams2.M;*/
   }

   //生成图像校正所需的像素映射矩阵
   cv::initUndistortRectifyMap(
       stereoParams.calib_M_L, stereoParams.calib_D_L,
       R1, P1,
       imageSize,
       CV_16SC2,
       remapMat.Calib_mX_L,remapMat.Calib_mY_L);

   cv::initUndistortRectifyMap(
       stereoParams.calib_M_R,stereoParams.calib_D_R,
       R2, P2,
       imageSize,
       CV_16SC2,
       remapMat.Calib_mX_R, remapMat.Calib_mY_R);

   //输出数据
   Q.copyTo(remapMat.Calib_Q);
   remapMat.Calib_Roi_L = roi1;
   remapMat.Calib_Roi_R = roi2;

   return 1;
}

/*----------------------------
 * 功能 : 保存双目校正参数
 *----------------------------
 * 函数 : stereoRectify::saveStereoDatas
 * 访问 : public
 * 返回 : 0 - 操作失败，1 - 操作成功
 *
 * 参数 : filename		[in]	保存路径/文件名
 * 参数 : method			[in]	双目校正方法
 * 参数 : stereoParams	[in]	双目标定结果
 * 参数 : remapMatrixs	[in]	图像校正像素映射矩阵
 */
int stereoRectify::saveStereoDatas(string filename, string method)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    if (fs.isOpened())
    {
        time_t rawtime;
        time(&rawtime);
        fs << "calibrationDate" << asctime(localtime(&rawtime));

        fs << "imageSize" << "[" << imageSize.width << imageSize.height << "]";

        fs << "leftCameraMatrix"			<< stereoParams.calib_M_L;
        fs << "leftDistortCoefficients"		<< stereoParams.calib_D_L;
        fs << "rightCameraMatrix"			<< stereoParams.calib_M_R;
        fs << "rightDistortCoefficients"	<< stereoParams.calib_D_R;
        fs << "rotationMatrix"				<< stereoParams.R;
        fs << "translationVector"			<< stereoParams.T;
        fs << "foundationalMatrix"			<< stereoParams.F;

        if (method == "RECTIFY_BOUGUET")
        {
            fs << "rectifyMethod" << "BOUGUET";
            fs << "leftValidArea" << "[:"
                << remapMat.Calib_Roi_L.x << remapMat.Calib_Roi_L.y
                << remapMat.Calib_Roi_L.width << remapMat.Calib_Roi_L.height << "]";
            fs << "rightValidArea" << "[:"
                << remapMat.Calib_Roi_R.x << remapMat.Calib_Roi_R.y
                << remapMat.Calib_Roi_R.width << remapMat.Calib_Roi_R.height << "]";
            fs << "QMatrix" << remapMat.Calib_Q;
        }
        else
            fs << "rectifyMethod" << "HARTLEY";

        fs << "remapX1" << remapMat.Calib_mX_L;
        fs << "remapY1" << remapMat.Calib_mY_L;
        fs << "remapX2" << remapMat.Calib_mX_R;
        fs << "remapY2" << remapMat.Calib_mY_R;

        fs.release();
        return 1;
    }
    else
    {
        return 0;
    }
}

//功能 : 对图像进行校正
int stereoRectify::remapImage(cv::Mat& imgleft, cv::Mat& imgright, string method = "RECTIFY_BOUGUET")
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

   return 1;
}
