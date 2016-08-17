#include "stereocalib.h"

stereoCalib::stereoCalib()
{
 }
 stereoCalib::~stereoCalib()
 {
 }

 //棋盘格进行立体校正
 void stereoCalib::calib_process(ParamsReader::RunParams& runParams)
 {
     cout<<"stereoCalib start ... "<<endl;

     basefunc *bf = new basefunc();
     vector<string> list;

     calibrationInit(runParams.cornerX,runParams.cornerY,runParams.squareSize, cornerDatas);

     bf->loadImageList(runParams.calib_imagepath, list);
     for(uint i=0;i<list.size();i=i+2){     
                 img_left = cv::imread(list[i], 1);
                 img_right = cv::imread(list[i+1], 1);
#if DEBUG_SHOW
         cv::imshow("calib_left",img_left);
         cv::imshow("calib_right",img_right);
         cv::waitKey(1);
#endif
         if(!img_left.empty() & !img_right.empty()){
             calibrationAddSample(img_left,img_right,cornerDatas);
         }else{
             break;
         }
    }
    imageSize = cv::Size(img_left.cols,img_left.rows);
    calibration(cornerDatas,stereoParams);

    saveCameraParams(stereoParams.cameraParams1, "../params/cameraParams_left.yml");
    saveCameraParams(stereoParams.cameraParams2, "../params/cameraParams_right.yml");
    saveCalibDatas("../params/stereoCalib.yml",cornerDatas,stereoParams);

    cout<<"stereoCalib END! "<<endl;

    cv::destroyAllWindows();
    delete bf;
 }

 //初始化角点参数，长宽，大小等
 void stereoCalib::calibrationInit(int cornersX,int cornersY,float squareSize,CornerDatas& cornerDatas){
     cornerDatas.boardSize.width = cornersX;
     cornerDatas.boardSize.height = cornersY;
     cornerDatas.nPointsPerImage = cornersX * cornersY;
     cornerDatas.nImages = 0;
     cornerDatas.objectPoints.resize(20, vector<cv::Point3f>(cornerDatas.nPointsPerImage, cv::Point3f(0,0,0)));
     for(int k = 0;k<20;k++)
     {
         int n = 0;
         for( int i = 0; i < cornerDatas.boardSize.height; i++ )
                 for( int j = 0; j < cornerDatas.boardSize.width; j++ )
                     cornerDatas.objectPoints[k][n++] = (cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));
     }

 }

 //功能 : 检测棋盘角点
 int stereoCalib::calibrationAddSample(cv::Mat& imageLeft,cv::Mat& imageRight,CornerDatas& cornerDatas)
 {
     cv::Mat imageleft = imageLeft.clone();
     cv::Mat imageright = imageRight.clone();
     cornerDatas.imageSize =  cv::Size(imageleft.cols,imageleft.rows);
     int result1 = 0;
     int result2 = 0;
     vector<cv::Point2f> corners1;
     vector<cv::Point2f> corners2;

     //FIND CHESSBOARDS AND CORNERS THEREIN:
     result1 = cv::findChessboardCorners(
         imageleft, cornerDatas.boardSize,
         corners1,
         cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK
     );
     if(result1){
         //Calibration will suffer without subpixel interpolation
         cv::cvtColor(imageleft, viewGray, CV_BGR2GRAY);
         cv::cornerSubPix(
             viewGray, corners1,
             cv::Size(11, 11),cv::Size(-1,-1),
             cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER, 30, 0.01)
         );
#if DEBUG_SHOW
         cv::drawChessboardCorners(imageleft, cornerDatas.boardSize, cv::Mat(corners1),result1);
         cv::imshow("chess1",imageleft);
         cv::waitKey(1);
#endif
     }

     result2 = cv::findChessboardCorners(
         imageright, cornerDatas.boardSize,
         corners2,
         cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK
       );
     if(result2){
         //Calibration will suffer without subpixel interpolation
         cv::cvtColor(imageright, viewGray, CV_BGR2GRAY);
         cv::cornerSubPix(
             viewGray, corners2,
             cv::Size(11, 11),cv::Size(-1,-1),
             cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER, 30, 0.01)
         );
#if DEBUG_SHOW
         cv::drawChessboardCorners(imageright, cornerDatas.boardSize, cv::Mat(corners2),result2);
         cv::imshow("chess2",imageright);
         cv::waitKey(1);
#endif
     }

     if(result1 & result2){
         cornerDatas.imagePoints1.push_back(corners1);
         cornerDatas.imagePoints2.push_back(corners2);

         cornerDatas.nImages++;
         return 1;
     }else{
         return 0;
     }
 }

 //进行立体标定
 bool stereoCalib::calibration(CornerDatas& cornerDatas,StereoParams& stereoParams)
 {
     stereoParams.cameraParams1.flags = CV_CALIB_FIX_PRINCIPAL_POINT| CV_CALIB_USE_INTRINSIC_GUESS |
                                                                       CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_ZERO_TANGENT_DIST;
     stereoParams.cameraParams2.flags = CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_USE_INTRINSIC_GUESS |
                                                                       CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_SAME_FOCAL_LENGTH| CV_CALIB_FIX_INTRINSIC;
     stereoParams.flags = 1;
     cornerDatas.objectPoints.resize(cornerDatas.nImages);

     bool ok = calibStereo(cornerDatas,stereoParams);

     return ok;
 }

 //功能 : 执行2目摄像机标定
 bool stereoCalib::calibStereo(CornerDatas& cornerDatas,StereoParams& stereoParams)
 {
     //单目标定
     double rms1 = cv::calibrateCamera(
                 cornerDatas.objectPoints,
                 cornerDatas.imagePoints1,
                 imageSize,
                 stereoParams.cameraParams1.M, stereoParams.cameraParams1.D,
                 stereoParams.cameraParams1.rvecs,stereoParams.cameraParams1.tvecs,
                 CV_CALIB_ZERO_TANGENT_DIST|CV_CALIB_FIX_PRINCIPAL_POINT |CV_CALIB_FIX_ASPECT_RATIO|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5
                 );
     double rms2 = cv::calibrateCamera(
                  cornerDatas.objectPoints,
                  cornerDatas.imagePoints2,
                  imageSize,
                  stereoParams.cameraParams2.M, stereoParams.cameraParams2.D,
                  stereoParams.cameraParams2.rvecs,stereoParams.cameraParams2.tvecs,
                  CV_CALIB_ZERO_TANGENT_DIST|CV_CALIB_FIX_PRINCIPAL_POINT |CV_CALIB_FIX_ASPECT_RATIO|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5
                  );

     bool ok = cv::checkRange(stereoParams.cameraParams2.M) && cv::checkRange(stereoParams.cameraParams2.D) &&
                      cv::checkRange(stereoParams.cameraParams2.M) && cv::checkRange(stereoParams.cameraParams2.D);//检查内参数矩阵数据是否正确

     vector<float> reprojErrs1,reprojErrs2;
     double totalAvgErr1 = getSingleCalibrateError(cornerDatas.objectPoints, cornerDatas.imagePoints1,
                                                  stereoParams.cameraParams1.rvecs, stereoParams.cameraParams1.tvecs,
                                                  stereoParams.cameraParams1.M, stereoParams.cameraParams1.D, reprojErrs1);
     double totalAvgErr2 = getSingleCalibrateError(cornerDatas.objectPoints, cornerDatas.imagePoints2,
                                                   stereoParams.cameraParams2.rvecs, stereoParams.cameraParams2.tvecs,
                                                   stereoParams.cameraParams2.M, stereoParams.cameraParams2.D, reprojErrs2);

#if DEBUF_INFO_SHOW
     cout << "Re-projection error reported by calibrateCamera: "<< rms1 << endl;
     cout << "Re-projection error reported by calibrateCamera: "<< rms2 << endl;
     cout << "M1"<<stereoParams.cameraParams1.M<<endl;
     cout << "M2"<<stereoParams.cameraParams2.M<<endl;
     cout << (ok ? "SingleCalibration succeeded!  " : "Calibration failed!  ")<<". avg re projection error = "  << totalAvgErr1<<"  and  "<<totalAvgErr2<<endl ;
#endif

     //CALIBRATE THE STEREO CAMERAS
     double rms = cv::stereoCalibrate(
         cornerDatas.objectPoints,
         cornerDatas.imagePoints1,
         cornerDatas.imagePoints2,
         stereoParams.cameraParams1.M, stereoParams.cameraParams1.D,
         stereoParams.cameraParams2.M, stereoParams.cameraParams2.D,
         imageSize, stereoParams.R, stereoParams.T, stereoParams.E,stereoParams.F,
         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,100, 1e-5),
         cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_RATIONAL_MODEL +
        // cv::CALIB_FIX_PRINCIPAL_POINT+cv::CALIB_USE_INTRINSIC_GUESS+
         cv::CALIB_FIX_INTRINSIC+ cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_SAME_FOCAL_LENGTH + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_RATIONAL_MODEL +
         +cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5
     );
     double err = getStereoCalibrateError(cornerDatas,stereoParams);

#if DEBUF_INFO_SHOW
     cout << "calibrateCamera: "<< rms << endl;
     cout << "M1"<<stereoParams.cameraParams1.M<<endl;
     cout << "M2"<<stereoParams.cameraParams2.M<<endl;
     cout << "StereoCalibration succeeded! " << ". avg re projection error = "  << err << endl ;   // : "StereoCalibration failed"
#endif

     return 1;
 }


 // * 功能 : 执行双目摄像机标定,若每个摄像机尚未标定，则首先进行单目标定，再进行双目标定
 bool stereoCalib::calibSingle(CornerDatas& cornerDatas,StereoParams& stereoParams)
 {
     //单目标定
     double rms1 = cv::calibrateCamera(
                 cornerDatas.objectPoints,
                 cornerDatas.imagePoints1,
                 cornerDatas.imageSize,
                 stereoParams.cameraParams1.M, stereoParams.cameraParams1.D,
                 stereoParams.cameraParams1.rvecs,stereoParams.cameraParams1.tvecs,
                 CV_CALIB_ZERO_TANGENT_DIST|CV_CALIB_FIX_PRINCIPAL_POINT |CV_CALIB_FIX_ASPECT_RATIO|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5
                 );

     cout << "Re-projection error reported by calibrateCamera: "<< rms1 << endl;
     bool ok = cv::checkRange(stereoParams.cameraParams2.M) && cv::checkRange(stereoParams.cameraParams2.D);
     vector<float> reprojErrs1;
     double totalAvgErr1 = getSingleCalibrateError(cornerDatas.objectPoints, cornerDatas.imagePoints1,
                                                   stereoParams.cameraParams1.rvecs, stereoParams.cameraParams1.tvecs,
                                                   stereoParams.cameraParams1.M, stereoParams.cameraParams1.D, reprojErrs1);
     cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = "  << totalAvgErr1<<endl ;

     return ok;
 }

 //功能 : 计算单目标定误差
 double stereoCalib::getSingleCalibrateError( const vector<vector<cv::Point3f> >& objectPoints,
                                          const vector<vector<cv::Point2f> >& imagePoints,
                                          const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
                                          const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                          vector<float>& perViewErrors)
 {
     vector<cv::Point2f> imagePoints_1;
     int i, totalPoints = 0;
     double totalErr = 0, err;
     perViewErrors.resize(objectPoints.size());

     for( i = 0; i < (int)objectPoints.size(); ++i )
     {
         cv::projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                        distCoeffs, imagePoints_1);
         err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints_1),cv::NORM_L2);

         int n = (int)objectPoints[i].size();
         perViewErrors[i] = (float) sqrt(err*err/n);
         totalErr        += err*err;
         totalPoints     += n;
     }

     return sqrt(totalErr/totalPoints);
 }

 //功能 : 计算双目标定误差
 double stereoCalib::getStereoCalibrateError(CornerDatas& cornerDatas,StereoParams& stereoParams)
 {
     // 利用对极线约束检查立体校正效果
     // because the output fundamental matrix implicitly includes all the output information, we can check the quality of calibration using the epipolar geometry constraint: m2^t*F*m1=0

     vector<cv::Vec3f> epilines[2];
     vector<vector<cv::Point2f> > imagePoints[2];
     cv::Mat cameraMatrix[2], distCoeffs[2];
     int npoints = 0;
     int i,j,k;
     double err;

     imagePoints[0] = cornerDatas.imagePoints1;
     imagePoints[1] = cornerDatas.imagePoints2;
     cameraMatrix[0] =stereoParams.cameraParams1.M;
     cameraMatrix[1] = stereoParams.cameraParams2.M;
     distCoeffs[0] = stereoParams.cameraParams1.D;
     distCoeffs[1] = stereoParams.cameraParams2.D;

     for( i = 0; i < cornerDatas.nImages; i++ )
     {
         int npt = (int)imagePoints[0][i].size();
         cv::Mat imgpt[2];

         for( k = 0; k < 2; k++ )
         {
             imgpt[k] = cv::Mat(imagePoints[k][i]);
             // 计算校正后的棋盘角点坐标
             cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
             // 计算对极线
             cv::computeCorrespondEpilines(imgpt[k], k+1, stereoParams.F, epilines[k]);
         }

         // 计算对极线误差
         for( j = 0; j < npt; j++ )
         {
             double errij =
                 fabs(imagePoints[0][i][j].x * epilines[1][j][0] +
                      imagePoints[0][i][j].y * epilines[1][j][1] + epilines[1][j][2]) +
                 fabs(imagePoints[1][i][j].x * epilines[0][j][0] +
                      imagePoints[1][i][j].y * epilines[0][j][1] + epilines[0][j][2]);
             err += errij;
         }
         npoints += npt;
     }
     err /= npoints;

     return err;
 }

 /*----------------------------
  * 功能 : 载入棋盘角点数据信息
  *----------------------------
  * 函数 : stereoCalib::loadCornerData
  * 访问 : public
  * 返回 : 0 - 操作失败，1 - 操作成功
  *
  * 参数 : filename		[in]	本地文件路径/文件名
  * 参数 : cornerDatas	[out]	导入的棋盘角点数据
  */
 int stereoCalib::loadCornerData(const char* filename, CornerDatas& cornerDatas)
 {
     cv::FileStorage fs(filename, cv::FileStorage::READ);
     if (fs.isOpened())
     {
         fs["nPoints"]			>> cornerDatas.nPoints;
         fs["nImages"]			>> cornerDatas.nImages;
         fs["nPointsPerImage"]	>> cornerDatas.nPointsPerImage;

         cv::FileNodeIterator it = fs["imageSize"].begin();
         it >> imageSize.width >> imageSize.height;

         cv::FileNodeIterator bt = fs["boardSize"].begin();
         bt >> cornerDatas.boardSize.width >> cornerDatas.boardSize.height;

         for (int i=0; i<cornerDatas.nImages;i++)
         {
             std::stringstream imagename;
             imagename << "image" << i;

             cv::FileNode img = fs[imagename.str()];
             vector<cv::Point3f> ov;
             vector<cv::Point2f> iv1, iv2;
             for (int j=0; j<cornerDatas.nPointsPerImage; j++)
             {
                 std::stringstream nodename;
                 nodename << "node" << j;

                 cv::FileNode pnt = img[nodename.str()];
                 cv::Point3f op;
                 cv::Point2f ip1, ip2;
                 cv::FileNodeIterator ot = pnt["objectPoints"].begin();
                 ot >> op.x >> op.y >> op.z;
                 cv::FileNodeIterator it1 = pnt["imagePoints1"].begin();
                 it1 >> ip1.x >> ip1.y;
                 cv::FileNodeIterator it2 = pnt["imagePoints2"].begin();
                 it2 >> ip2.x >> ip2.y;

                 iv1.push_back(ip1);
                 iv2.push_back(ip2);
                 ov.push_back(op);
             }
             cornerDatas.imagePoints1.push_back(iv1);
             cornerDatas.imagePoints2.push_back(iv2);
             cornerDatas.objectPoints.push_back(ov);
         }

         fs.release();
         return 1;
     }
     else
     {
         return 0;
     }
 }


 /*----------------------------
  * 功能 : 保存棋盘角点数据信息
  *----------------------------
  * 函数 : stereoCalib::saveCornerData
  * 访问 : public
  * 返回 : 0 - 操作失败，1 - 操作成功
  *
  * 参数 : filename		[in]	本地文件路径/文件名
  * 参数 : cornerDatas	[in]	待导出的棋盘角点数据
  */
 int stereoCalib::saveCornerData(const char* filename, const CornerDatas& cornerDatas)
 {
     cv::FileStorage fs(filename, cv::FileStorage::WRITE);
     if (fs.isOpened())
     {
         time_t rawtime;
         time(&rawtime);
         fs << "calibrationDate" << asctime(localtime(&rawtime));

         //fs << "nPoints"			<< cornerDatas.nPoints;
         fs << "nImages"			<< cornerDatas.nImages;
         fs << "nPointsPerImage" << cornerDatas.nPointsPerImage;

         fs << "imageSize" << "[" << imageSize.width << imageSize.height << "]";

         fs << "boardSize" << "[" << cornerDatas.boardSize.width << cornerDatas.boardSize.height << "]";

         for (int i=0; i<cornerDatas.nImages;i++)
         {
             std::stringstream imagename;
             imagename << "image" << i;

             fs << imagename.str() << "{";

             for (int j=0; j<cornerDatas.nPointsPerImage; j++)
             {
                 std::stringstream nodename;
                 nodename << "node" << j;

                 fs << nodename.str() << "{";

                 cv::Point3f op = cornerDatas.objectPoints[i][j];
                 cv::Point2f ip1 = cornerDatas.imagePoints1[i][j];
                 cv::Point2f ip2 = cornerDatas.imagePoints2[i][j];

                 fs << "objectPoints" << "[:";
                 fs << op.x << op.y << op.z << "]";

                 fs << "imagePoints1" << "[:";
                 fs << ip1.x << ip1.y << "]";

                 fs << "imagePoints2" << "[:";
                 fs << ip2.x << ip2.y << "]";

                 fs << "}";
             }

             fs << "}";
         }

         fs.release();
         return 1;
     }
     else
     {
         return 0;
     }
 }

 /*----------------------------
  * 功能 : 载入已标定好的摄像机内部参数
  *----------------------------
  * 函数 : stereoCalib::loadCameraParams
  * 访问 : public
  * 返回 : 0 - 操作失败，1 - 操作成功
  *
  * 参数 : filename		[in]	参数文件路径/文件名
  * 参数 : cameraParams	[out]	读入的摄像机参数
  */
 int stereoCalib::loadCameraParams(const char* filename, CameraParams& cameraParams)
 {
     cv::FileStorage fs(filename, cv::FileStorage::READ);
     if (fs.isOpened())
     {
         cv::FileNodeIterator it = fs["imageSize"].begin();
         it >> imageSize.width >> imageSize.height;

         fs["cameraMatrix"]				>> cameraParams.M;
         fs["distortionCoefficients"]	>> cameraParams.D;
         fs["flags"]						>> cameraParams.flags;

         int nImages = 0;
         fs["nImages"] >> nImages;

         for (int i = 0; i < nImages; i++)
         {
             char matName[50];
             sprintf(matName, "rotaionMatrix_%d", i);

             cv::Mat rotMat;
             fs[matName] >> rotMat;
             cameraParams.rvecs.push_back(rotMat);
         }

         for (int i = 0; i < nImages; i++)
         {
             char matName[50];
             sprintf(matName, "translationMatrix_%d", i);

             cv::Mat tranMat;
             fs[matName] >> tranMat;
             cameraParams.tvecs.push_back(tranMat);
         }

         fs.release();
         return 1;
     }
     else
     {
         return 0;
     }
 }


 /*----------------------------
  * 功能 : 保存已标定好的摄像机内部参数
  *----------------------------
  * 函数 : stereoCalib::saveCameraParams
  * 访问 : public
  * 返回 : 0 - 操作失败，1 - 操作成功
  *
  * 参数 : cameraParams	[in]	已标定的摄像机参数
  * 参数 : filename		[in]	参数文件路径/文件名
  */
 int stereoCalib::saveCameraParams(const CameraParams& cameraParams, const char* filename /* = "cameraParams.yml" */)
 {
     std::string filename_ = filename;

     //按当前时间生成文件名
     if (filename_ == "")
     {
         int strLen = 20;
         char *pCurrTime = (char*)malloc(sizeof(char)*strLen);
         memset(pCurrTime, 0, sizeof(char)*strLen);
         time_t now;
         time(&now);
         strftime(pCurrTime, strLen , "%Y_%m_%d_%H_%M_%S_", localtime(&now));

         filename_ =  pCurrTime;
         filename_ += "cameraParams.yml";
     }

     //写入数据
     cv::FileStorage fs(filename_.c_str(), cv::FileStorage::WRITE);
     if (fs.isOpened())
     {
         time_t rawtime;
         time(&rawtime);
         fs << "calibrationDate" << asctime(localtime(&rawtime));

         char flagText[1024];
         sprintf( flagText, "flags: %s%s%s%s%s",
             cameraParams.flags & cv::CALIB_FIX_K3 ? "fix_k3" : "",
             cameraParams.flags & cv::CALIB_USE_INTRINSIC_GUESS ? " + use_intrinsic_guess" : "",
             cameraParams.flags & cv::CALIB_FIX_ASPECT_RATIO ? " + fix_aspect_ratio" : "",
             cameraParams.flags & cv::CALIB_FIX_PRINCIPAL_POINT ? " + fix_principal_point" : "",
             cameraParams.flags & cv::CALIB_ZERO_TANGENT_DIST ? " + zero_tangent_dist" : "" );
         cvWriteComment(*fs, flagText, 0);

         fs << "flags"					<< cameraParams.flags;

         fs << "imageSize" << "[" << cameraParams.imageSize.width << cameraParams.imageSize.height << "]";

         fs << "cameraMatrix"			<< cameraParams.M;
         fs << "distortionCoefficients"	<< cameraParams.D;

         int nImages = cameraParams.rvecs.size();
         fs << "nImages"	<< nImages;
         for (int i = 0; i < nImages; i++)
         {
             char matName[50];
             sprintf(matName, "rotaionMatrix_%d", i);

             fs << matName << cameraParams.rvecs[i];
         }
         for (int i = 0; i < nImages; i++)
         {
             char matName[50];
             sprintf(matName, "translationMatrix_%d", i);

             fs << matName << cameraParams.tvecs[i];
         }

         fs.release();
         return 1;
     }
     else
     {
         return 0;
     }
 }

 /*----------------------------
  * 功能 : 保存双目校正参数
  *----------------------------
  * 函数 : stereoCalib::saveCalibDatas
  * 访问 : public
  * 返回 : 0 - 操作失败，1 - 操作成功
  *
  * 参数 : filename		[in]	保存路径/文件名
  * 参数 : method			[in]	双目校正方法
  * 参数 : cornerDatas	[in]	棋盘角点数据
  * 参数 : stereoParams	[in]	双目标定结果
  */
 int stereoCalib::saveCalibDatas(const char* filename, CornerDatas& cornerDatas, StereoParams& stereoParams)
 {
     cv::FileStorage fs(filename, cv::FileStorage::WRITE);

     if (fs.isOpened())
     {
         time_t rawtime;
         time(&rawtime);
         fs << "calibrationDate" << asctime(localtime(&rawtime));

         fs << "num_boards"	<< cornerDatas.nImages;
         fs << "imageSize" << "[" << imageSize.width << imageSize.height << "]";

         char flagText[1024];
         sprintf( flagText, "flags: %s%s%s%s%s",
             stereoParams.flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+ use_intrinsic_guess" : "",
             stereoParams.flags & cv::CALIB_FIX_ASPECT_RATIO ? " + fix_aspect_ratio" : "",
             stereoParams.flags & cv::CALIB_FIX_PRINCIPAL_POINT ? " + fix_principal_point" : "",
             stereoParams.flags & cv::CALIB_FIX_INTRINSIC ? " + fix_intrinsic" : "",
             stereoParams.flags & cv::CALIB_SAME_FOCAL_LENGTH ? " + same_focal_length" : "" );

         cvWriteComment(*fs, flagText, 0);

         fs << "stereoCalibrateFlags"		<< stereoParams.flags;
         fs << "leftCameraMatrix"			<< stereoParams.cameraParams1.M;
         fs << "leftDistortCoefficients"		<< stereoParams.cameraParams1.D;
         fs << "rightCameraMatrix"			<< stereoParams.cameraParams2.M;
         fs << "rightDistortCoefficients"	<< stereoParams.cameraParams2.D;
         fs << "rotationMatrix"				<< stereoParams.R;
         fs << "translationVector"			<< stereoParams.T;
         fs << "foundationalMatrix"			<< stereoParams.F;

         fs.release();
         return 1;
     }
     else
     {
         return 0;
     }
 }

 /*----------------------------
  * 功能 : 载入双目定标结果数据
  *----------------------------
  * 函数 : stereoCalib::loadCalibDatas
  * 访问 : public
  * 返回 : 1		成功
  *		 0		读入校正参数失败
  *		 -1		定标参数的图像尺寸与当前配置的图像尺寸不一致
  *		 -99	未知错误
  *
  * 参数 : xmlFilePath	[in]	双目定标结果数据文件
  */
 int stereoCalib::loadCalibDatas(string xmlFilePath,StereoParams& stereoParams)
 {
     // 读入摄像头定标参数 Q roi1 roi2 mapx1 mapy1 mapx2 mapy2
     try
     {
         cv::FileStorage fs(xmlFilePath, cv::FileStorage::READ);
         if ( !fs.isOpened() )
         {
             return (0);
         }

         cv::Size imgSize;
         cv::FileNodeIterator it = fs["imageSize"].begin();
         it >> imgSize.width >> imgSize.height;
         if (imgSize.width != imageSize.width || imgSize.height != imageSize.height){
             return (-1);
         }

         fs["leftCameraMatrix"] >> stereoParams.cameraParams1.M;
         fs["leftDistortCoefficients"] >> stereoParams.cameraParams1.D;
         fs["rightCameraMatrix"] >> stereoParams.cameraParams2.M;
         fs["rightDistortCoefficients"] >> stereoParams.cameraParams1.D;
         fs["rotationMatrix"]				>> stereoParams.R;
         fs ["translationVector"]		>> stereoParams.T;
         fs ["foundationalMatrix"]  	>> stereoParams.F;

     }
     catch (std::exception& e)
     {
         return (-99);
     }

     return 1;
 }


