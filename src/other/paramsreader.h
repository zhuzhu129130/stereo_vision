#ifndef PARAMSREADER_H
#define PARAMSREADER_H

# pragma once  //只要在头文件的最开始加入这条杂注，就能够保证头文件只被编译一次
// 各种头文件
// C++标准库
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
// 参数读取类
class ParamsReader
{
public:
    struct RunParams
    {

        //程序运行参数calib，match，process
        string processType;
        //===================立体标定参数=====================================
        //图像序列列表#calib_image/imagelist.xml
        string calib_imagepath;

        //立体标定参数保存文件目录
        string stereocalib_path;

        //棋盘格长宽大小参数
        int cornerX;
        int cornerY;
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

        //视差计算算法  BM , SGBM, VAR
        string DisparityType;
        //================立体重建参数=========================================

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

#endif // PARAMSREADER_H
