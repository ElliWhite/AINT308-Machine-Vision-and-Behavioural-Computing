#include <iostream>
#include "sobel.h"


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    
    if (false){
        
        VideoCapture cap(0); // open the default camera
        if(!cap.isOpened())  // check if we succeeded
            return -1;
        while(true) {
            Mat frame;
            cap >> frame; // get a new frame from camera
            cv::Mat grad= cv::Mat(frame.size(),CV_64F);
            cv::Mat phase= cv::Mat(frame.size(),CV_64F);
            int ret=DOsobel (frame, grad,phase);
            if (ret==-1) break;
        }
    }
    else {
        cv::Mat img = imread("/Users/culverhouse/Teaching/AINT308/OWL-lectures/images/NingNing-s.jpg");
        cv::Mat Igrad= cv::Mat(img.size(),CV_64F);
        cv::Mat Iphase= cv::Mat(img.size(),CV_64F);
        int ret=DOsobel (img, Igrad,Iphase);
        cv::imwrite("/Users/culverhouse/Teaching/AINT308/OWL-lectures/images/NingNing-s-grad.jpg",Igrad);
        cv::imwrite("/Users/culverhouse/Teaching/AINT308/OWL-lectures/images/NingNing-s-phase.jpg",Iphase);
        
    }
    return 0;
}
