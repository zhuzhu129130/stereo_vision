#ifndef BASEFUNC_H
#define BASEFUNC_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace std;

//调试信息输出宏,预编译宏,
#ifndef DEBUG_SHOW
#define	DEBUG_SHOW  (0)//显示调试图像信息
#endif

#ifndef DEBUF_INFO_SHOW
#define	DEBUF_INFO_SHOW  (0)//显示调试数据
#endif


// 参数读取类
class ParamsReader
{
public:
    struct RunParams
    {

        //程序运行参数calib，rectify，reconstruct，process
        string processType;
        //===================立体标定参数=====================================
        //图像序列列表#calib_image/imagelist.xml
        string calib_imagepath;

        //立体标定参数保存文件目录
        string stereocalib_path;

        //棋盘格长宽大小参数
        int cornerX;
        int cornerY;
        float squareSize;
        //====================================================================

        //=============立体校正参数============================================
        //校正参数计算后测试图像
        string image1_test;
        string image2_test;

        //立体校正参数 RECTIFY_HARTLEY，RECTIFY_BOUGUET
        string rectifymethod;

        string rectifyParams_path;
       //=============立体校正参数============================================

       //================立体重建参数=========================================
       //起始与终止索引
        int start_index;
        int end_index;

        //数据所在目录，及图像名称前缀和后缀
        string img_left;
        string left_extension;
        string img_right;
        string right_extension;

        //视差计算算法  BM , SGBM, VAR, ELAS
        string DisparityType;
        //================立体重建参数=========================================

        //================数据源==============================================
        //三种数据源：image，camera，video
        string file_type;

        int camera1;
        int camera2;

        string video1_dir;
        string video2_dir;

        string detector;
        string descriptor;
        double good_match_threshold;

    };
public:
    ParamsReader( string filename="../parameters.txt" )
    {
        ifstream fin( filename.c_str() ); //打开这个文件
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof()) //判断是不是文件结尾
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }

public:
    map<string, string> data;
};

class basefunc
{
public:
    basefunc();
    bool loadImages(vector<string> fileList, vector<cv::Mat> &images);
    bool loadImageList(string file, vector<string> &list);

    void readFrame( int index, cv::Mat& img_left,cv::Mat& img_right,ParamsReader::RunParams& runParams);

    void ConvertRaw12toRaw8(const cv::Mat& raw12, cv::Mat& raw8);

    void ConvertRGB12toBGR8(const cv::Mat& rgb12, cv::Mat& bgr8);

    void FixGroundtruth(cv::Mat& img_gt);
};




#endif // BASEFUNC_H
