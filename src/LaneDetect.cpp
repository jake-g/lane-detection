/*------------------------------------------------------------------------------------------*\
 Lane Detection
 Tracks lanes and their convergence. 

 by Jake Garrison and Brian Magnuson
 Compile:

 g++ $(pkg-config --cflags --libs opencv) LaneDetect.cpp

 General idea and some code modified from:
 chapter 7 of Computer Vision Programming using the OpenCV Library.
 by Robert Laganiere, Packt Publishing, 2011.


 Notes:
 Add up number on lines that are found within a threshold of a given rho,theta and
 use that to determine a score.  Only lines with a good enough score are kept.

 Calculation for the distance of the car from the center.  This should also determine
 if the road in turning. Might not want to be in the center of the road for a turn.

 Several other parameters can be played with: min vote on houghp, line distance and gap.  Some
 type of feed back loop might be good to self tune these parameters.

 Added filter on theta angle to reduce horizontal and vertical lines.
 Added image ROI to reduce false lines from things like trees/powerlines
 \*------------------------------------------------------------------------------------------*/

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include "linefinder.h"

#define PI 3.1415926

using namespace cv;
using namespace std;

int main() {

    // Set-Up
    int houghVote = 200;
    string arg = "";

    // Set up windows
    bool showOriginal = 1;
    bool showCanny = 1;
    bool showHough = 1;
    bool showHoughP = 1;

    // Capture Input
    string window_name = "Processed Video";
    namedWindow(window_name, CV_WINDOW_KEEPRATIO); //resizable window;
    VideoCapture capture(arg);

    if (!capture.isOpened())  // Caputure Camera
    {capture.open(atoi(arg.c_str()));}

    capture.set(CV_CAP_PROP_POS_MSEC, 100000); //start the video at 100 seconds in

    double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    std::cout << "Frame Size = " << dWidth << "x" << dHeight << std::endl;
    Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));

    // Encode resulting video
    // VideoWriter oVideoWriter ("LaneDetection.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true);

    // Process Frame
    Mat image;
    double frameItr = 0;
    image = imread(arg);
    int crestCount = 0, frameSkip = 0;
    while (1)
    {
        // capture on intervals to make vid smoother
        capture >> image;
        frameItr += 100;
        capture.set(CV_CAP_PROP_POS_MSEC, frameItr);

        if (image.empty())
            break;

        Mat gray;
        cvtColor(image,gray,CV_RGB2GRAY);
        vector<string> codes;
        Mat corners;
        findDataMatrix(gray, codes, corners);
        drawDataMatrixCodes(image, codes, corners);

        // ROI
        // optimized? -=> yes
        int top = 0;
        int left = 0;
        int width = 800;
        int height = 600;

        Rect roi(left,top,width,height);
        Mat imgROI = image(roi);
        Scalar val = Scalar(0, 0, 0);
        copyMakeBorder(imgROI, imgROI, 2, 2, 2, 2, BORDER_CONSTANT, val);

        // Display the image
        if(showOriginal) {
            namedWindow("Original Image");
            imshow("Original Image",imgROI);
            imwrite("original.bmp", imgROI);
        }

        // Canny algorithm
        Mat contours;
        Canny(imgROI,contours,100,200);
        Mat contoursInv;
        threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

        // Display Canny image
        if(showCanny) {
            namedWindow("Contours");
            imshow("Contours1",contours); // use contoursInv for white
            imwrite("contours.bmp", contours);
        }

        /*
         Hough tranform for line detection with feedback
         Increase by 25 for the next frame if we found some lines.
         This is so we don't miss other lines that may crop up in the next frame
         but at the same time we don't want to start the feed back loop from scratch.
         */
        std::vector<Vec2f> lines;
        if (houghVote < 1 or lines.size() > 2) { // we lost all lines. reset
            houghVote = 300;
        }
        else{ houghVote += 25;}
        while(lines.size() < 4 && houghVote > 0){
            HoughLines(contours,lines,1,PI/180, houghVote);
            houghVote -= 5;
        }
        std::cout << houghVote << "\n";
        Mat result(imgROI.size(),CV_8U,Scalar(255));
        imgROI.copyTo(result);

        // Draw the lines
        std::vector<Vec2f>::const_iterator it= lines.begin();
        Mat hough(imgROI.size(),CV_8U,Scalar(0));
        while (it!=lines.end()) {

            float rho= (*it)[0];   // first element is distance rho
            float theta= (*it)[1]; // second element is angle theta

            if ( (theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66) ) { // filter to remove vertical and horizontal lines

                // point of intersection of the line with first row
                Point pt1(rho/cos(theta),0);
                // point of intersection of the line with last row
                Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
                // draw a line: Color = Scalar(R, G, B), thickness
                line( result, pt1, pt2, Scalar(255,255,255), 1);
                line( hough, pt1, pt2, Scalar(255,255,255), 1);
            }

            //std::cout << "line: (" << rho << "," << theta << ")\n";
            ++it;
        }

        // Display the detected line image
        if(showHough){
            namedWindow("Detected Lines with Hough");
            imshow("Detected Lines with Hough",result);
            imwrite("hough.bmp", result);
        }

        // Create LineFinder instance
        LineFinder ld;

        // Set probabilistic Hough parameters
        ld.setLineLengthAndGap(10,60); // min accepted length and gap
        ld.setMinVote(15); // sit > 3 to get rid of "spiderweb"

        // Detect lines
        std::vector<Vec4i> li= ld.findLines(contours);
        Mat houghP(imgROI.size(),CV_8U,Scalar(0));
        ld.setShift(0,0);
        ld.drawDetectedLines(houghP);
        std::cout << "First Hough" << "\n";

        if(showHoughP){
            namedWindow("Detected Lines with HoughP");
            imshow("Detected Lines with HoughP", houghP);
            imwrite("houghP.bmp", houghP);
        }

        // bitwise AND of the two hough images
        bitwise_and(houghP,hough,houghP);
        Mat houghPinv(imgROI.size(),CV_8U,Scalar(0));
        Mat dst(imgROI.size(),CV_8U,Scalar(0));
        threshold(houghP,houghPinv,150,255,THRESH_BINARY_INV); // threshold and invert to black lines

        if(showHoughP){
            namedWindow("Detected Lines with Bitwise");
            imshow("Detected Lines with Bitwise", houghP);
        }


        Canny(houghPinv,contours,100,350);
        li = ld.findLines(contours);
        // Display Canny image
//        if(showCanny){
//            namedWindow("Contours");
//            imshow("Contours2",contours);
//            imwrite("contours.bmp", contoursInv);
//        }

        // Test to draw point
        //ld.drawPoint(image, Point(320,130));

        // Set probabilistic Hough parameters
        // more strict than above HoughP
        ld.setLineLengthAndGap(5,2);
        ld.setMinVote(1);
        ld.setShift(top, left);

        // draw point on image where line intersection occurs
        int yShift = 25;
        int allowableFrameSkip = 5;
        ld.drawDetectedLines(image);
        cv::Point iPnt = ld.drawIntersectionPunto(image, 2);

        // track hill crest
        int gap = 20;
        cv::Point lptl(0, image.rows / 2 + yShift);
        cv::Point lptr(gap, image.rows / 2 + yShift);
        line(image, lptl, lptr, Scalar(255, 255, 255), 1);// left mid line

        cv::Point rptl(image.cols - gap, image.rows / 2 + yShift);
        cv::Point rptr(image.cols, image.rows / 2 + yShift);
        line(image, rptl, rptr, Scalar(255, 255, 255), 1);// right mid line

        cv::Point ulpt(0, image.rows / 2 - 50 + yShift);
        cv::Point urpt(image.cols, image.rows / 2 - 50 + yShift);
   //     line(image, ulpt, urpt, Scalar(255, 255, 255), 1);// upper line

        bool hillCrestFound = (iPnt.y < (image.rows / 2 + yShift)) && (iPnt.y > (image.rows / 2 - 50 + yShift));
        if(hillCrestFound) {
            crestCount++;
            frameSkip = 0;
        } else if(crestCount != 0 && frameSkip < allowableFrameSkip)
            frameSkip++;
        else {
            crestCount = 0;
            frameSkip = 0;
        }

        cv::Point txtPt(image.cols / 2 - 31, image.rows / 2 - 140);
        if(crestCount > 3)
            putText(image, "tracking", txtPt, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2, 8);

        std::stringstream stream;
        stream << "Lines Segments: " << lines.size();

        putText(image, stream.str(), Point(10,image.rows-10), 1, 0.8, Scalar(0,255,0),0);
        imshow(window_name, image);
        imwrite("processed.bmp", image);

        //	oVideoWriter.write(image); //writer the frame into the file

        char key = (char) waitKey(10);
        lines.clear();
    }
}
