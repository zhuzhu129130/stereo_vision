#include "basefunc.h"


basefunc::basefunc()
{
}

bool basefunc::loadImageList(string file, vector<string> &list)
{
    bool loadSuccess = false;
    cv::FileNode fNode;
    cv::FileStorage fStorage(file, cv::FileStorage::READ);

    if (fStorage.isOpened())
    {
        fNode = fStorage.getFirstTopLevelNode();

        if (fNode.type() == cv::FileNode::SEQ)
        {
            for(cv::FileNodeIterator iterator = fNode.begin(); iterator != fNode.end(); ++iterator)
            {
                list.push_back((string) *iterator);
            }

            loadSuccess = true;
        }
    }

    return loadSuccess;
}

bool basefunc::loadImages(vector<string> fileList, vector<cv::Mat> &images)
{
    bool emptyImage = false;
    for (uint i = 0; i < fileList.size() && !emptyImage; ++i)
    {
        cv::Mat curImg = cv::imread(fileList[i]);
        if (!curImg.empty())
        {
            images.push_back(curImg);
        }
        else
        {
            emptyImage = true;
        }
    }

    return (!emptyImage && images.size() > 0);
}

void basefunc::readFrame( int index, cv::Mat& img_left,cv::Mat& img_right,ParamsReader::RunParams& runParams)
{
    stringstream ss;
    ss<<runParams.img_left<<index<<runParams.left_extension;
    string filename;
    ss>>filename;
    img_left = cv::imread(filename);

    ss.clear();
    filename.clear();
    ss<<runParams.img_right<<index<<runParams.right_extension;
    ss>>filename;

    img_right= cv::imread( filename );

#if DEBUG_SHOW
    cv::imshow("left",img_left);
    cv::imshow("right",img_right);
    cv::waitKey(1);
 #endif
}


void basefunc::ConvertRaw12toRaw8(const cv::Mat& raw12, cv::Mat& raw8)
{
  raw8.create(raw12.rows, raw12.cols, CV_8U);
  for (int i = 0; i < raw12.rows; i++) {
    for (int j = 0; j < raw12.cols; j++) {
      raw8.at<uint8_t>(i,j) = uint8_t(255.0 * ((double)raw12.at<uint16_t>(i,j) / 4095.0));
      //std::cout << raw12.at<uint16_t>(i,j) << "\n";
      //printf("%d\n", raw12.at<uint16_t>(i,j));
    }
  }
}

void basefunc::ConvertRGB12toBGR8(const cv::Mat& rgb12, cv::Mat& bgr8)
{
  bgr8.create(rgb12.rows, rgb12.cols, CV_8UC3);
  for (int i = 0; i < rgb12.rows; i++) {
    for (int j = 0; j < rgb12.cols; j++) {
      bgr8.at<cv::Vec3b>(i,j)[0] = uint8_t(255.0 * ((double)rgb12.at<cv::Vec3s>(i,j)[0] / 4095.0));
      bgr8.at<cv::Vec3b>(i,j)[1] = uint8_t(255.0 * ((double)rgb12.at<cv::Vec3s>(i,j)[1] / 4095.0));
      bgr8.at<cv::Vec3b>(i,j)[2] = uint8_t(255.0 * ((double)rgb12.at<cv::Vec3s>(i,j)[2] / 4095.0));
      //std::cout << raw12.at<uint16_t>(i,j) << "\n";
    }
  }
}


//=====================计算图像的参考视差图=======================
void basefunc::FixGroundtruth(cv::Mat& img_gt) {
  for (int i = 0; i < img_gt.rows; i++) {
    for (int j = 0; j < img_gt.cols; j++) {
      uint8_t& b = img_gt.at<cv::Vec3b>(i,j)[0];
      uint8_t& g = img_gt.at<cv::Vec3b>(i,j)[1];
      uint8_t& r = img_gt.at<cv::Vec3b>(i,j)[2];
      if (r == 255 && g == 0 && b == 0) {
        r = 0;
        g = 0;
        b = 255;
      }
      else {
        r = 0;
        g = 255;
        b = 0;
      }
    }
  }
}


