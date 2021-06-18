#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture inputVideo(1);
    //inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    //inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    if (!inputVideo.isOpened())
    {
        cout << "Could not open the input video " << endl;
        return -1;
    }
    Mat frame;
    string imgname;
    int f = 1;
    while (1) //Show the image captured in the window and repeat
    {
        inputVideo >> frame;              // read
        if (frame.empty()) break;         // check if at end
        imshow("Camera", frame);
        char key = waitKey(1);
        if (key == 27)break;
        if (key == 'q' || key == 'Q')
        {
            imgname = "/home/chrisliu/NewDisk/ROSws/img_ws/src/img_extract/catch_imgs/" + to_string(f++) + ".jpg";
            imwrite(imgname, frame);
        }
    }
    cout << "Finished writing" << endl;
    return 0;
}