#ifndef VIDEO_SAVER_H
#define VIDEO_SAVER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <time.h>
using namespace std;
using namespace cv;

string IntToString(int value)
{
    ostringstream convert;
    convert << value;
    return convert.str();
}

string GetCurrentTime()
{
    time_t Time = time(NULL);
    tm* LocalTime = localtime(&Time);
    string Result;

    // add the year
    Result += IntToString(LocalTime->tm_year + 1900) + "-";
    // add the month
    Result += IntToString(LocalTime->tm_mon + 1) + "-";
    // add the day
    Result += IntToString(LocalTime->tm_mday) + "_";
    // add the hour
    Result += IntToString(LocalTime->tm_hour) + "-";
    // add the minutes
    Result += IntToString(LocalTime->tm_min) + "-";
    // add the seconds
    Result += IntToString(LocalTime->tm_sec);

    return Result;
}

string GetCurrentTime2()
{
    time_t Time = time(NULL);
    tm* LocalTime = localtime(&Time);
    string Result;

    // add the year
    Result += IntToString(LocalTime->tm_year + 1900) + "-";
    // add the month
    Result += IntToString(LocalTime->tm_mon + 1) + "-";
    // add the day
    Result += IntToString(LocalTime->tm_mday) + " ";
    // add the hour
    Result += IntToString(LocalTime->tm_hour) + ":";
    // add the minutes
    Result += IntToString(LocalTime->tm_min) + ":";
    // add the seconds
    Result += IntToString(LocalTime->tm_sec);

    return Result;
}

class VideoSaver{
    public:
        VideoSaver(int num = 1500);
        void write(Mat &frame,string &rcd_path);
    private:
        int frameCount;
        int maxFrame;
        VideoWriter writer;
};

VideoSaver::VideoSaver(int num)
{
    frameCount = 0;
    maxFrame = num;
}

void VideoSaver::write(Mat &frame,string &rcd_path)
{
    if( frameCount % maxFrame == 0 )
    {
        string fileName = rcd_path + "Video_" + GetCurrentTime() + ".avi";
        writer.open(fileName, writer.fourcc('M', 'J', 'P', 'G'), 25.0, frame.size());
    }
    frameCount++;
    writer.write(frame);
}

#endif // VIDEO_SAVER_H
