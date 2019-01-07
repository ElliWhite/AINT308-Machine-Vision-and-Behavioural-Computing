// owl.cpp : Defines the entry point for the console application.
/* Phil Culverhouse Oct 2016 (c) Plymouth UNiversity
 *
 * Uses IP sockets to communicate to the owl robot (see owl-comms.h)
 * Uses OpenCV to perform normalised cross correlation to find a match to a template
 * (see owl-cv.h).
 * PWM definitions for the owl servos are held in owl-pwm.h
 * includes bounds check definitions
 * requires setting for specific robot
 *
 * This demosntration programs does the following:
 * a) loop 1 - take picture, check arrow keys
 *             move servos +5 pwm units for each loop
 *             draw 64x64 pixel square overlaid on Right image
 *             if 'c' is pressed copy patch into a template for matching with left
 *              exit loop 1;
 * b) loop 2 - perform Normalised Cross Correlation between template and left image
 *             move Left eye to centre on best match with template
 *             (treats Right eye are dominate in this example).
 *             loop
 *             on exit by ESC key
 *                  go back to loop 1
 *
 * First start communcations on Pi by running 'python PFCpacket.py'
 * Then run this program. The Pi server prints out [Rx Ry Lx Ly] pwm values and loops
 *
 * NOTE: this program is just a demonstrator, the right eye does not track, just the left.
 */

#include <iostream>
#include <fstream>

#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#include <sys/time.h>               // for set time intervals


#include <iostream>                 // for standard I/O
#include <string>                   // for strings


using namespace std;
using namespace cv;

#define PI 3.14159265
struct timeval tv;                  // create structure of current time in different forms (seconds, milliseconds etc)

const Mat OWLresult;                // correlation result passed back from matchtemplate
cv::Mat Frame;
Mat Left, Right;                    // images
bool inLOOP=true;
int key;

// function to read in a frame from the cameras and display it to the screen
void frameCapture(VideoCapture cap2, string source2)
{
    if (!cap2.read(Frame))                              // try to read in frame
    {
        cout  << "Could not open the input video: " << source2 << endl;
        //         break;
    }
    Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1);         // Note that Left/Right are reversed now
    //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);

    // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
    Left= FrameFlpd( Rect(0, 0, 640, 480));             // using a rectangle
    Right=FrameFlpd( Rect(640, 0, 640, 480));           // using a rectangle
    Mat RightCopy;                                      // copy images
    Mat LeftCopy;
    Left.copyTo(LeftCopy);
    Right.copyTo(RightCopy);
    rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 );  // draw white rect
    rectangle( LeftCopy, target, Scalar::all(255), 2, 8, 0 );   // draw white rect
    imshow("Left",LeftCopy);imshow("Right", RightCopy);         // show frames
    key = waitKey(30);                                          // this is a pause long enough to allow a stable photo to be taken.
    printf("%d",key);                                           // print the pressed key (if any)
}


int main(int argc, char *argv[])
{
    char receivedStr[1024];
    ostringstream CMDstream; // string packet
    string CMD;
    int N;

    Rx = RxLm; Lx = LxLm;
    Ry = RyC; Ly = LyC;
    Neck = NeckC;

    string source ="http://10.0.0.10:8080/stream/video.mjpeg";
    string PiADDR = "10.0.0.10";

    //SETUP TCP COMMS
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);



    /***********************
 * LOOP continuously for testing
 */
    // RyC=RyC-40; LyC=LyC+40; // offset for cross on card
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck = NeckC;



    while (inLOOP){
        // move servos to centre of field
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        VideoCapture cap (source);              // Create input
        if (!cap.isOpened())                    // Try to open video stream
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }

        frameCapture(cap,source);               // Capture a frame

        // Task 1: Neck Control. Sinusoidal motion
        printf("**** Task 1 ****\n");
        // Left to Right

        Neck = NeckC;                           // Start with neck in centre

        double angle = 0;                       // variables for sinusoidal motion
        double sinOutput = 0;

        while (inLOOP){

            CMDstream.str("");                  // Update servo positions
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
            waitKey(1);

            //full right to left, left to right motion
            for (angle=0; angle<=360; angle++){

                sinOutput = sin(angle*PI/180);      //sin function needs radians, hence PI/180 conversion

                //output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
                Neck = NeckR + ((NeckL - NeckR) / (-1 - 1)) * (sinOutput - 1);

                CMDstream.str("");              // Update servo positions
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());

                frameCapture(cap,source);       // Capture a frame

                // Exit task 1. Press space to quit
                if (key == 32) {
                    inLOOP = false;             // To break from task 1
                    break;
                }
            }
        }

        inLOOP = true; //reset
        // End task 1

        // Task 2: Stereo Eye Control
        printf("**** Task 2 ****\n");
        //Rect region_of_interest = Rect(x, y, w, h);
        while (inLOOP){

            double val;                 // Variables for stereo control
            val = 180.0 / PI;
            double xPosL;
            double xPosR;
            const double yDist = 200;	// Distance of virtual object in mm
            double LAngle;
            double RAngle;
            float pwmStep = 0.113;
            int LpwmStepCalc;
            int RpwmStepCalc;

            if(inLOOP){
                // tmpPos = X position of target in relation to left eye whilst looking at the OWL. Moving right to left
                // if object is to the outside of the corresponding eye (toe out) then the position is negative
                // if object is to the inside of the corresponding eye (toe in) then the position is positive
                for(signed int tmpPos = -50; tmpPos < 155; tmpPos=tmpPos+4){
                    xPosL = tmpPos;
                    xPosR = 65 - tmpPos;                // xPosR is 65 - tmpPos as the eyes are 65mm apart
                    LAngle = atan(xPosL/yDist) * val;   // calculate angle required for cameras by using trig
                    RAngle = atan(xPosR/yDist) * val;   // (* val) is needed to convert radians to degrees
                    LpwmStepCalc = LAngle/pwmStep;      // calculate how many steps are needed as we know 1 PWM step is 0.113 degrees
                    RpwmStepCalc = RAngle/pwmStep;
                    Lx = LxC + LpwmStepCalc;            // assign new positions to servos
                    Rx = RxC - RpwmStepCalc;

                    if ((Lx > LxLm) && (Lx < LxRm) && (Rx > RxLm) && (Rx < RxRm)){      // check to make sure the positions are in the ranges of the eyes. If so, then update positions
                        CMDstream.str("");
                        CMDstream.clear();
                        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                        CMD = CMDstream.str();
                        RxPacket = OwlSendPacket (u_sock, CMD.c_str());
                    }

                    frameCapture(cap,source);           // Capture a frame

                    // Exit task 2. Press space to quit
                    if (key == 32) {
                        inLOOP = false;                 // To break from task 2
                        break;
                    }

                }
            }
            // check if space has been previously pressed
            if(inLOOP){
                // tmpPos = X position of target in relation to left eye whilst looking at the OWL. Moving left to right
                // if object is to the outside of the corresponding eye (toe out) then the position is negative
                // if object is to the inside of the corresponding eye (toe in) then the position is positive
                for(signed int tmpPos = 155; tmpPos > -50; tmpPos=tmpPos-4){
                    xPosL = tmpPos;
                    xPosR = 65 - tmpPos;
                    LAngle = atan(xPosL/yDist) * val;
                    RAngle = atan(xPosR/yDist) * val;
                    LpwmStepCalc = LAngle/pwmStep;
                    RpwmStepCalc = RAngle/pwmStep;
                    Lx = LxC + LpwmStepCalc;
                    Rx = RxC - RpwmStepCalc;
                    if ((Lx > LxLm) && (Lx < LxRm) && (Rx > RxLm) && (Rx < RxRm)){
                        CMDstream.str("");
                        CMDstream.clear();
                        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                        CMD = CMDstream.str();
                        RxPacket = OwlSendPacket (u_sock, CMD.c_str());
                     }

                    frameCapture(cap,source);
                    // Exit task 2. Press space to quit
                    if (key == 32) {
                        inLOOP = false; // To break from task 2
                        break;
                    }

                }
            }

        } 

        inLOOP = true;
        // End task 2




        // Start task 3: Chameleon Eye Vision
        printf("**** Task 3 ****\n");
        gettimeofday(&tv,NULL);

		int timeNow = tv.tv_sec;
        int targetTime = timeNow + 10;

        while (timeNow < targetTime){
            
            int LXrand = rand() % 670 + 1180;   //rand X eye pos
            int LYrand = rand() % 820 + 1180;   //rand Y eye pos
            int RXrand = rand() % 690 + 1200;   //rand X eye pos
            int RYrand = rand() % 880 + 1120;   //rand Y eye pos
            int Trand = rand() % 3;       //rand Time interval (time between each eye movement)

            // Continue to update feed whilst waiting for Trand
            int waitTime = timeNow + Trand;
            while (timeNow < waitTime){
                // Check to see if time has elapsed (random time interval)
                gettimeofday(&tv,NULL);
                timeNow = tv.tv_sec;

                frameCapture(cap,source);
				
            }

            //Update Left Eye
            Lx = LXrand;
            Ly = LYrand;
            // Update postion (send pos)
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
			
			Trand = rand() % 3;       //rand Time interval (time between each eye movement)
            // Continue to update feed whilst waiting for Trand
            waitTime = timeNow + Trand;
            while (timeNow < waitTime){
                // Check to see if time has elapsed (randon time interval)
                gettimeofday(&tv,NULL);
                timeNow = tv.tv_sec;

                frameCapture(cap,source);
                
            }

            //Update Right Eye
            Rx = RXrand;
            Ry = RYrand;
            // Update postion (send pos)
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
			
			// Update current time to see if target time has elapsed (10 seconds)
            gettimeofday(&tv,NULL);
            timeNow = tv.tv_sec;
        }
        printf("Time Elapsed\n");

        inLOOP = true;
        // End task 3

        // Task 4A: Emotive behaviour
        // Rolling eyes
        printf("**** Task 4A: Roll eyes ****\n");


        //Set eyes to center, Neck to Center
        Rx = RxC; //1525
        Ry = RyC; //1380
        Lx = LxC; // 1450
        Ly = LyC; // 1600
        Neck = NeckC; // 1465
        // Update postion (send pos)
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());
        while (inLOOP){

            Rx = RxC; //1525
            Ry = RyC; //1380
            Lx = LxC; // 1450
            Ly = LyC; // 1600
            Neck = NeckC; // 1465

            // Eyes up
            while (Ly > 1300){
                Ly = Ly - 40;
                Ry = Ry + 40;

                // Update postion (send pos)
                CMDstream.str("");
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                frameCapture(cap,source);
                // Exit task. Press space to quit
                if (key == 32) {
                    inLOOP = false; // To break from task 2
                    break;
                }
            }

            if(inLOOP){
                // Eyes Right
                while (Lx < 1850){
                    Lx = Lx + 20;
                    Rx = Rx + 20;

                    // Update postion (send pos)
                    CMDstream.str("");
                    CMDstream.clear();
                    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                    CMD = CMDstream.str();
                    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                    frameCapture(cap,source);
                    // Exit task. Press space to quit
                    if (key == 32) {
                        inLOOP = false; // To break from task 2
                        break;
                    }
                }
            }

            if(inLOOP){
                // Eyes down
                while (Ly < LyC){
                    Ly = Ly + 40;
                    Ry = Ry - 40;
                    Neck = Neck - 10;

                    // Update postion (send pos)
                    CMDstream.str("");
                    CMDstream.clear();
                    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                    CMD = CMDstream.str();
                    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                    frameCapture(cap,source);
                    // Exit task. Press space to quit
                    if (key == 32) {
                        inLOOP = false; // To break from task 2
                        break;
                    }
                }
            }
            /*
            if(inLOOP){
                // Eyes Right
                while (Lx > LxC){
                   Lx = Lx - 20;
                   Rx = Rx - 20;

                   // Update postion (send pos)
                   CMDstream.str("");
                   CMDstream.clear();
                   CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                   CMD = CMDstream.str();
                   RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                   frameCapture(cap,source);
                   // Exit task. Press space to quit
                   if (key == 32) {
                       inLOOP = false; // To break from task 2
                       break;
                   }
                }
            }*/
            //now wait for 3 seconds
            gettimeofday(&tv,NULL);
            timeNow = tv.tv_sec;
            targetTime = timeNow + 3;
            if(inLOOP){
                while (timeNow < targetTime){

                    gettimeofday(&tv,NULL);
                    timeNow = tv.tv_sec;

                    frameCapture(cap,source);
                    // Exit task. Press space to quit
                    if (key == 32) {
                        inLOOP = false; // To break from task 2
                        break;
                    }

                }
            }
        }

        inLOOP = true;


        // Task 4B: Emotive behaviour
        // Shame
        printf("**** Task 4B: Shame ****");

        int maxDown = 0;

        // find maximum difference in movement in y direction
        if( (RyC - RyBm) > (LyBm - LyC) ) {
            maxDown = LyBm - LyC;
        } else {
            maxDown = RyC - RyBm;
        }


        //Need to slowly move down then fast away
        while(inLOOP){

            //Reset servos
            Rx = RxC;
            Lx = LxC;
            Ry = RyC;
            Ly = LyC;
            Neck = NeckC;

            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());

            int z = 0;

            //moving down for left is positive
            //moving down for right is negative
            if(inLOOP){
                while(z <= maxDown) {
                    Ry = Ry - 20;
                    Ly = Ly + 20;
                    CMDstream.str("");
                    CMDstream.clear();
                    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                    CMD = CMDstream.str();
                    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                    z = z + 20;
                    frameCapture(cap,source);
                    // Exit task. Press space to quit
                    if (key == 32) {
                        inLOOP = false; // To break from task 2
                        break;
                    }
                }
            }

            //now wait for a 0.5s
            gettimeofday(&tv,NULL);
            timeNow = tv.tv_usec;
            targetTime = timeNow + 500000;		// timeNow is in uSeconds, hence the large 500000
            while (timeNow < targetTime){

                gettimeofday(&tv,NULL);
                timeNow = tv.tv_sec;

                frameCapture(cap,source);
                // Exit task. Press space to quit
                if (key == 32) {
                    inLOOP = false; // To break from task 2
                    break;
                }

            }

            //now move eyes to right
            //right for both eyes is positive
            int maxRight = 0;
            // find max movement in x direction
            if( (RxRm - RxC) > (LxRm - LxC) ) {
                maxRight = LxRm - LxC;
            } else {
                maxRight = RxRm - RxC;
            }

            z = 0;

            if(inLOOP){
                while(z <= maxRight) {
                    Rx = Rx + 20;
                    Lx = Lx + 20;
                    //start moving neck halfway through eye movement. Right is negative
                    if(z > (maxRight/2)){
                        Neck = Neck - 5;
                    }

                    CMDstream.str("");
                    CMDstream.clear();
                    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                    CMD = CMDstream.str();
                    RxPacket= OwlSendPacket (u_sock, CMD.c_str());

                    z = z + 20;

                    frameCapture(cap,source);
                    // Exit task. Press space to quit
                    if (key == 32) {
                        inLOOP = false; // To break from task 2
                        break;
                    }
                }
            }
            //now wait for 3 seconds
            gettimeofday(&tv,NULL);
            timeNow = tv.tv_sec;
            targetTime = timeNow + 3;
            if(inLOOP){
                while (timeNow < targetTime){

                    gettimeofday(&tv,NULL);
                    timeNow = tv.tv_sec;

                    frameCapture(cap,source);
                    // Exit task. Press space to quit
                    if (key == 32) {
                        inLOOP = false; // To break from task 2
                        break;
                    }

                }
            }



        }

        inLOOP = true;

        //End Task 4B



        //Rect region_of_interest = Rect(x, y, w, h);
        while (inLOOP){
            frameCapture(cap, source);
            printf("%d",key);//mrs added 01/02/2017 to diagnose arrow keys returned code ***************************************************
            switch (key){
            case 56://2490368: Numpad up arrow
                Ry=Ry+5;Ly=Ly-5; // was Ly=+5 Changed BILL
                break;
            case 50://2621440: Numpad down arrow
                Ry=Ry-5;Ly=Ly+5; // was Ly=-5 BILL
                break;
            case 52://2424832:Numpad left arrow
                Rx=Rx-5;Lx=Lx-5;
                break;
            case 54://2555904: Numpad right arrow
                Rx=Rx+5;Lx=Lx+5;
                break;
            case 97:    //'a' key
                Neck = Neck + 5;
                break;
            case 100:    //'d' key
                Neck = Neck - 5;
                break;
            case 99: // lowercase 'c'
                OWLtempl= Right(target);
                imshow("templ",OWLtempl);
                waitKey(1);
                inLOOP=false; // quit loop and start tracking target
                break; // left
            default:
                key=key;
                //nothing at present
            }

                CMDstream.str("");
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());

                if (0) {
                    for (int i=0;i<10;i++){
                        Rx=Rx-50; Lx=Lx-50;
                        CMDstream.str("");
                        CMDstream.clear();
                        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                        CMD = CMDstream.str();
                        RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                        //waitKey(100); // cut the pause for a smooth persuit camera motion
                    }
                }
        } // END cursor control loop
        // close windows down
        destroyAllWindows();
        // just a ZMCC
        // right is the template, just captured manually
        inLOOP=true; // run through the loop until decided to exit
        while (inLOOP) {
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                break;
            }
            Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
            //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left= FrameFlpd( Rect(0, 0, 640, 480)); // using a rectangle
            Right=FrameFlpd( Rect(640, 0, 640, 480)); // using a rectangle

            //Rect target= Rect(320-32, 240-32, 64, 64); //defined in owl-cv.h
            //Mat OWLtempl(Right, target);
            OwlCorrel OWL;
            OWL = Owl_matchTemplate( Right,  Left, OWLtempl, target);
            /// Show me what you got
            Mat RightCopy;
            Right.copyTo(RightCopy);
            rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 );
            rectangle( Left, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle( OWLresult, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );

            imshow("Owl-L", Left);
            imshow("Owl-R", RightCopy);
            imshow("Correl",OWL.Result );
            if (waitKey(10)== 27) inLOOP=false;
// P control
            double KPx=0.1; // track rate X
            double KPy=0.1; // track rate Y
            double LxScaleV = LxRangeV/(double)640; //PWM range /pixel range
            double Xoff= 320-(OWL.Match.x + OWLtempl.cols)/LxScaleV ; // compare to centre of image
            int LxOld=Lx;

            Lx=LxOld-Xoff*KPx; // roughly 300 servo offset = 320 [pixel offset


            double LyScaleV = LyRangeV/(double)480; //PWM range /pixel range
            double Yoff= (250+(OWL.Match.y + OWLtempl.rows)/LyScaleV)*KPy ; // compare to centre of image
            int LyOld=Ly;
            Ly=LyOld-Yoff; // roughly 300 servo offset = 320 [pixel offset

            cout << Lx << " " << Xoff << " " << LxOld << endl;
            cout << Ly << " " << Yoff << " " << LyOld << endl;
            //Atrous

            //Maxima

            // Align cameras

            // ZMCC disparity map

            // ACTION

            // move to get minimise distance from centre of both images, ie verge in to targe
            // move servos to position
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());


            } // end if ZMCC
        } // end while outer loop
#ifdef __WIN32__
        closesocket(u_sock);
#else
        close(clientSock);
#endif
        exit(0); // exit here for servo testing only
    }
