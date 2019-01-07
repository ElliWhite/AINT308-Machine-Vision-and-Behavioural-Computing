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
#include <sstream>
#include <list>

#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#include <sys/time.h>


#include <iostream> // for standard I/O
#include <string>   // for strings


using namespace std;
using namespace cv;

#define PI 3.14159265
struct timeval tv;

const Mat OWLresult;// correlation result passed back from matchtemplate
cv::Mat Frame;
Mat Left, Right; // images
bool inLOOP=true;
int key;

//Salience feature scales
float K0 = 0.0;
float K1 = 0.0;
float K2 = 0.0;
float K3 = 0.0;
float K4 = 0.0;
float K5 = 0.0;

//Salience feature sliders
int K0_slider;
int K1_slider;
int K2_slider;
int K3_slider;
int K4_slider;
int K5_slider;
int thresh_slider = 255;

//Global threshold. Starts at max so nothing is detected
int thresh = 255;

const int slider_max = 100;
const int thresh_slider_max = 255;

//variable to hold the current salience point we are trying to saccade to
char location = 0;

//OwlCorrels for template matching
OwlCorrel OWL_L;
OwlCorrel OWL_R;
OwlCorrel OWL_Static;

//string to hold distance estimate text
std::string text;

//rectangle to hold to target
Rect Target;

//matrix to hold the template to be matched
Mat OWLtempl;

//images
Mat Right_Image;
Mat Left_Image;

//function to capture a frame from the cameras
void frameCapture(VideoCapture cap2, string source2, Mat *LeftCopy, Mat *RightCopy, Mat *blankLeftCopy, Mat *blankRightCopy)
{
    //try to read a frame
    if (!cap2.read(Frame))
    {
        cout  << "Could not open the input video: " << source2 << endl;
        //         break;
    }
	//flip the images
    Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
    // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
    Left= FrameFlpd( Rect(0, 0, 640, 480)); // using a rectangle
    Right=FrameFlpd( Rect(640, 0, 640, 480)); // using a rectangle
	//two frames to draw a rectangle on the centre
    Mat RightCopy;
    Mat LeftCopy;
	//two frames that don't have anything drawn on
	Mat blankRightCopy;
    Mat blankLeftCopy;
	//create copies
    Left.copyTo(*LeftCopy);
    Right.copyTo(*RightCopy);
	Left.copyTo(*blankLeftCopy);
    Right.copyTo(*blankRightCopy);
	//draw rectangles on the original frames in the centre
    rectangle( *RightCopy, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
    rectangle( *LeftCopy, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
	//show frames
    imshow("Left",*LeftCopy);imshow("Right", *RightCopy);
    key = waitKey(30); //display frames and take note of any key press

}


//structure maxValLocation which stores the location of the maximum bright spot in a quadrant
struct maxValLocation {
    int val;
    cv::Point location;
};

//structure to sort maxValLocations by the val parameter
struct by_val {
    bool operator()(maxValLocation const &a, maxValLocation const &b) {
        return a.val > b.val;
    }
};


//callback for for the trackbar positions change
static void on_trackbar( int, void* )
{
   K0 = (double) K0_slider / slider_max;
   K1 = (double) K1_slider / slider_max;
   K2 = (double) K2_slider / slider_max;
   K3 = (double) K3_slider / slider_max;
   K4 = (double) K4_slider / slider_max;
   K5 = (double) K5_slider / slider_max;
   thresh = (double) thresh_slider;
}


/*************************
***********MAIN***********
*************************/
int main(int argc, char *argv[])
{
    char receivedStr[1024];
    ostringstream CMDstream; // string packet
    string CMD;
    int N;

    string source ="http://10.0.0.10:8080/stream/video.mjpeg";
    string PiADDR = "10.0.0.10";

    //SETUP TCP COMMS
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);
    
	//reset servo values to centre positions
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck = NeckC;

	//loop forever
    while (inLOOP){
		
        // move servos to centre of field
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

		//open video stream
        VideoCapture cap (source);              
		
		//check if could open the source
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }

        //loop to move the owl onto a target and save the target
        while (inLOOP){
			
			//capture a frame
            frameCapture(cap, source, &Left_Image, &Right_Image, &blankLeft, &blankRight);
         
			//do different things depending on the key pressed
            switch (key){
            case 56://2490368: Numpad up arrow
                Ry=Ry+5;Ly=Ly-5; //move eyes up
                break;
            case 50://2621440: Numpad down arrow
                Ry=Ry-5;Ly=Ly+5; //move eyes down
                break;
            case 52://2424832:Numpad left arrow
                Rx=Rx-5;Lx=Lx-5; //move eyes left
                break;
            case 54://2555904: Numpad right arrow
                Rx=Rx+5;Lx=Lx+5; //move eyes right
                break;
			//move neck left or right
            case 97:    //'a' key
                Neck = Neck + 5; //move neck left
                break;
            case 100:    //'d' key
                Neck = Neck - 5; //move neck right
                break;
            case 99: // lowercase 'c'
				//save the part of the right image target frame in the rectangle as OWLtemp1
                OWLtempl= Right(target);
				//show the target for debugging purposes
                imshow("templ",OWLtempl);
                waitKey(10);
                inLOOP=false; // quit loop and start tracking target
                break; // left
            default:
                key=key;
                //nothing at present
            }
			
			//update servo positions with new locations
			CMDstream.str("");
			CMDstream.clear();
			CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
			CMD = CMDstream.str();
			RxPacket= OwlSendPacket (u_sock, CMD.c_str());

                
        } // END cursor control loop
        // close windows down
        destroyAllWindows();

        inLOOP=true; //set to true to enter next loop


        // Assignment 2: Task 1: Cross-correlation & Servo control
        while (inLOOP) {
			
			//capture a frame. This time we use the "blank" versions as they don't have the original rectangles drawn on them
            frameCapture(cap, source, &Left_Image, &Right_Image, &blankLeft, &blankRight);
			
			//create two new OwlCorrel structures
            OwlCorrel OWL_L;
            OwlCorrel OWL_R;
			
			//match templates onto left and right images where the template is the target from the previous loop
            OWL_L = Owl_matchTemplate_L( blankRight, blankLeft, OWLtempl, target);
            OWL_R = Owl_matchTemplate_R( blankRight, blankLeft, OWLtempl, target);

			//draw rectangles onto the images at the point where the template has been matches
            rectangle( blankRight, OWL_R.Match, Point( OWL_R.Match.x + OWLtempl.cols , OWL_R.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle( blankLeft, OWL_L.Match, Point( OWL_L.Match.x + OWLtempl.cols , OWL_L.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
            rectangle( OWL_L.Result, OWL_L.Match, Point( OWL_L.Match.x + OWLtempl.cols , OWL_L.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
			
			//show the images
            imshow("Owl-L", blankLeft);
            imshow("Owl-R", blankRight);
            imshow("Template", OWLtempl);	//show the target
            imshow("Correl",OWL_L.Result ); //show the correlation result
			
			//check to see if the 'ESC' key was pressed. If so then next time time we try to go round the loop, leave
			key = waitKey(10);
            if (key== 27) {
				inLOOP=false;
			}

            double KPx=0.15; // track rate X
            double KPy=0.15; // track rate Y
			
			//compare the target in the left eye to the centre of its frame
            double LxScaleV = LxRangeV/(double)640; //PWM range /pixel range
			//find out how many pixels off the target is from the centre in X direction
            double Xoff= 390-(OWL_L.Match.x + OWLtempl.cols)/LxScaleV ; 
            int LxOld=Lx;

            Lx=LxOld-Xoff*KPx; // roughly 300 servo offset = 320 [pixel offset]

            double LyScaleV = LyRangeV/(double)480; //PWM range /pixel range
			//find out how many pixels off the target is from the centre in Y direction
            double Yoff= (250+(OWL_L.Match.y + OWLtempl.rows)/LyScaleV)*KPy ; // compare to centre of image
            int LyOld=Ly;
            Ly=LyOld-Yoff; // roughly 300 servo offset = 320 [pixel offset


			//compare the target in the right eye to the centre of its frame
            double RxScaleV = RxRangeV/(double)640; //PWM range /pixel range
			//find out how many pixels off the target is from the centre in X direction
            double RXoff= 370-(OWL_R.Match.x + OWLtempl.cols)/RxScaleV ; // compare to centre of image
            int RxOld=Rx;

            Rx=RxOld-RXoff*KPx; // roughly 300 servo offset = 320 [pixel offset]

            double RyScaleV = RyRangeV/(double)415; //PWM range /pixel range
			//find out how many pixels off the target is from the centre in Y direction
            double RYoff= (250-(OWL_R.Match.y + OWLtempl.rows)/RyScaleV)*KPy ; // compare to centre of image
            int RyOld=Ry;
            Ry=RyOld+RYoff; // roughly 300 servo offset = 320 [pixel offset]


            // Calculating eye servo angles
            float pwmStep = 0.113;

            int LxDelta = Lx - LxC;         // change in pos in pwmsteps from center
            double L_A = pwmStep * LxDelta;    // Left angle

            int RxDelta = RxC - Rx;         // flipped so that when both eyes are toe-in the angles are positive, making the math easier
            double R_A = pwmStep * RxDelta;    // Right angle

			//print initially determined angles
            printf("L_A = %f\n", L_A);
            printf("R_A = %f\n", R_A);


            // Calculating hypotenuse

            //Make sure both L_A and R_A are positive
            if(L_A < 0){
                L_A = L_A * -1;
            }
            if(R_A < 0){
                R_A = R_A * -1;
            }

            L_A = 90 - L_A;     //Needed as we have angles from the centre but need angles from perpendicular to the owl face
            R_A = 90 - R_A;

            printf("L_A = %f\n", L_A);
            printf("R_A = %f\n", R_A);
			
			//calculate third angle in the triangle
            double theta3 = 180 - (L_A + R_A);
            double rad_to_deg = 180.0 / PI; // radians to degrees
            double deg_to_rad = PI / 180.0; // degrees to radians

            printf("theta3 = %f\n", theta3);
			
			//convert degrees to radians
            double L_A_rad = L_A * deg_to_rad; // sin function takes radians as an input
            double R_A_rad = R_A * deg_to_rad ;
			
			//calculate sine of the servo angle and convert back to degrees
            double sin_L_A = sin(L_A_rad)* rad_to_deg; // The trig math requires degrees
            double sin_R_A = sin(R_A_rad)* rad_to_deg;

			//calculate hypotenuses
            double R_Hypot = (65*sin_L_A) / (sin(theta3*deg_to_rad)*rad_to_deg);
            printf("R_Hypot = %f\n", R_Hypot);

            double L_Hypot = (65*sin_R_A) / (sin(theta3*deg_to_rad)*rad_to_deg);
            printf("L_Hypot = %f\n", L_Hypot);
			
			//calculate distance estimates
            double distanceEst_R = (R_Hypot * sin_R_A) / (sin(90*deg_to_rad)*rad_to_deg);
            printf("Distance Estimate R = %f\n", distanceEst_R);

            double distanceEst_L = (L_Hypot * sin_L_A) / (sin(90*deg_to_rad)*rad_to_deg);
            printf("Distance Estimate L = %f\n", distanceEst_L);
			
			//calculate new distance estimate by transforming distances to a new scale
			//this was determined by comparing real distance to previously determined distance estimates
            double newDistanceEst = (distanceEst_R - 42) / (53/100);

			//output new distance estimate
            printf("New Distance Estimate = %f\n", newDistanceEst);

			//output how far off the targets are from the centre of the frames
            cout << Lx << " " << Xoff << " " << LxOld << endl;
            cout << Ly << " " << Yoff << " " << LyOld << endl;
            cout << Rx << " " << RXoff << " " << RxOld << endl;
            cout << Ry << " " << RYoff << " " << RyOld << endl;


            // move to get minimise distance from centre of both images, ie verge in to target
            // move servos to position
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());


		}

       
        destroyAllWindows();
        inLOOP = true;
		
        int i;

		
		// Assignment 2: Task 3: Saccadic eye control
        printf("Saccadic eye control\n");
		
        //take 60 frames to stabilise the camera and have the exposure adjusts correctly
        for(int i = 0; i < 60; i++){
            key=waitKey(20);
			//capture a frame
            frameCapture(cap, source, &Left_Image, &Right_Image, &blankLeft, &blankRight);
        }
		
		//create an image that holds one of the first frames
		//this will be used to show where the eyes have saccaded to overall (or at least try to)
        Mat staticLeft;
        Left_Image.copyTo(staticLeft);


        while (inLOOP) {
			
			//capture a frame
            frameCapture(cap, source, &Left_Image, &Right_Image, &blankLeft, &blankRight);
			
			//create black rectangles around the left image
            cv::rectangle(
                Left_Image,
                cv::Point(0, 0),
                cv::Point(640, 32),
                cv::Scalar(0, 0, 0), -1
            );
            cv::rectangle(
                Left_Image,
                cv::Point(0, 0),
                cv::Point(32, 480),
                cv::Scalar(0, 0, 0), -1
            );

            cv::rectangle(
                Left_Image,
                cv::Point(0, 480-32),
                cv::Point(640, 480),
                cv::Scalar(0, 0, 0), -1
            );

            cv::rectangle(
                Left_Image,
                cv::Point(640-32, 0),
                cv::Point(640, 480),
                cv::Scalar(0, 0, 0), -1
            );

            // Exit task. Press space to quit
            if (key == 27) {
                inLOOP = false; // To break from task
                printf("Quit\n");
                break;
            }

            // Declare source image as left or right eye. This is the base image that all processing is done on
            Mat lSource = blankLeft;

            int radius = 41;
            
            Mat lGray;
			//convert image to gray
			
            cvtColor(lSource, lGray, cv::COLOR_RGB2GRAY);

            Mat lGauss;
			
            //add Gaussian blur to average out pixels
            GaussianBlur(lGray, lGauss, Size (radius, radius), 0, 0);

            //Create difference-of-gaussian filter
            Mat lDoG = lSource;
            Mat g1, g2;
            int k=21; int g = 31;
            GaussianBlur(lDoG, g1, Size(g,g), 0);
            GaussianBlur(lDoG, g2, Size(g*k,g*k), 0);
            Mat lDoGresult = (g1-g2)*2;
			
			//convert to black and white
            cvtColor(lDoGresult, lDoGresult, cv::COLOR_RGB2GRAY);

            //Create a HSV colour-space version of the original image
            Mat lHSV;
            cvtColor(lSource, lHSV, CV_RGB2HSV);

			//Create vector of matrices to hold the three channels of the HSV image
            vector<Mat> lHSV_split, rHSV_split;
			
            //split images into the 3 channels
            split(lHSV, lHSV_split);
			
			//Assign the split image to their individual matrices
            Mat lHue = lHSV_split[0];
            Mat lSaturation = lHSV_split[1];
            Mat lVal = lHSV_split[2];


            // Harris corner detection
            Mat lHarris

            // Detector parameters
            int blockSize = 8;
            int apertureSize = 15;
            double Hk = 0.04;

			//Perform Harris corner detection
            cornerHarris(lGray, lHarris, blockSize, apertureSize, Hk, BORDER_DEFAULT);

            //result is dilated for marking the corners
            dilate(lHarris, lHarris, Mat(), Point(-1, -1), 2, 1, 1);

            // Normalizing
            Mat lHarris_norm, rHarris_norm;
            Mat lHarris_norm_scaled, rHarris_norm_scaled;
            normalize(lHarris, lHarris_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
            convertScaleAbs(lHarris_norm, lHarris_norm_scaled);
 
			//Create named window that holds the salience map of the left image. 
            namedWindow("Left Salience in Colour", WND_PROP_AUTOSIZE ); // Create Window
			//Add trackbars
            createTrackbar("Saturation", "Left Salience in Colour", &K0_slider, slider_max, on_trackbar);
            createTrackbar("Hue", "Left Salience in Colour", &K4_slider, slider_max, on_trackbar);
            createTrackbar("Value", "Left Salience in Colour", &K5_slider, slider_max, on_trackbar);
            createTrackbar("Bright Spots (Gauss)", "Left Salience in Colour", &K1_slider, slider_max, on_trackbar);
            createTrackbar("Contrast Edges (DoG)", "Left Salience in Colour", &K2_slider, slider_max, on_trackbar);
            createTrackbar("Corners (Harris)", "Left Salience in Colour", &K3_slider, slider_max, on_trackbar);
            createTrackbar("Threshold", "Left Salience in Colour", &thresh_slider, thresh_slider_max, on_trackbar);

            //Create salience feature map
            Mat lSalience = (K0 * lSaturation) + (K1 * lGauss) + (K2 * lDoGresult) + (K3 * lHarris_norm_scaled) + (K4 * lHue) + (K5 * lVal);
            
			//Add some median blur to the salience map to smooth it out a bit
            Mat lBlurSalience;
            medianBlur(lSalience, lBlurSalience, 11);
			
			//Transform the salience map into a coloured map by adding a colour map to it
            Mat lSalienceColour;
            applyColorMap(lSalience, lSalienceColour, COLORMAP_JET);

            //convert gray back to colour. Will still be b&w but means can draw on coloured circles
            Mat lResultSalience;
            cvtColor(lBlurSalience, lResultSalience, cv::COLOR_GRAY2RGB);
			
			//Show the coloured salience map
            imshow("Left Salience in Colour", lSalienceColour);



			//create vector of structure maxValLocation
            std::vector<maxValLocation> lMaxValLocations;


            //Split the salience map into a grid and perform a grid search
            int gridSize = 4;
            int x = 0; int y = 0; // top left position
            int width = 640/gridSize; int height = 480/gridSize; // Window size

            int quadNo = 1;
            int z = 1;
			//loop over all of the parts of the grid, finding the brightest spot in each quad
            for(int i = 0; i<gridSize; i++){
                for(int j=0; j<gridSize; j++){

                    //Draw the first rectangle onto the resultant salience map
                    x = (640/gridSize) * j;
                    y = (480/gridSize) * i;
                    Rect lRect(x, y, width, height);
                    rectangle(lResultSalience, lRect, cv::Scalar(0, 0, 0));
     
					//Take the first rectangle out of the resultant salience map and save it as Quadrant
                    Mat lQuadrant(lResultSalience, lRect);
					
					//convert the quadrant to grayscale
                    Mat lGrayQuad;
                    cvtColor(lQuadrant, lGrayQuad, cv::COLOR_RGB2GRAY);

                    // Max brightness for the quadrant
                    // minMaxLoc doesn't work
                    int lMaxVal = 0;
                    int lPixVal = 0;
                    int lMaxLocationX = 0, lMaxLocationY = 0;
					//iterate over whole quadrant looking for the brightest pixel (closest to white)
                    for( int row = 0; row < lGrayQuad.rows ; row++ ) {
                        for( int col = 0; col < lGrayQuad.cols; col++ ) {
							//read pixel value
                            lPixVal = lGrayQuad.at<uchar>(row,col);
							//if brighter than the previously stored brightest and above threshold, save it
                            if(  (lPixVal > lMaxVal) && (lPixVal > thresh) ) {
                               lMaxVal = lPixVal;
                               lMaxLocationX = col;
                               lMaxLocationY = row;

                              }
                         }
                    }
                   
					//translate the max location found in the grid back onto the original salience map
                    int lPosX = ((640/gridSize) * j) + lMaxLocationX;
                    int lPosY = ((480/gridSize) * i) + lMaxLocationY;
                    
					//add the brightest value and its location onto the list of maxValLocations
                    if( (lMaxLocationX > 0) || (lMaxLocationY > 0)){ 
                        maxValLocation newLoc = {lMaxVal, Point(lPosX,lPosY)};
                        lMaxValLocations.push_back(newLoc);
                    }
                    //next quadrant
                    quadNo += 1;

                }    
            }	//finished going over all quads

			//sort maxValLocations so largest value is in position 1
            std::sort(lMaxValLocations.begin(), lMaxValLocations.end(), by_val());


            //Draw circles onto the salience map with the brightness of their colour based on the position in the list
            if(lMaxValLocations.size()>0){
                for (int x=0; x < lMaxValLocations.size(); x++){
                    //scale the colour of the circle based on its position in the list of max val points
                    //brighter the green, the higher up the list
                    int tmpColour = (255/lMaxValLocations.size()) * (lMaxValLocations.size()-(x));
                    circle(lResultSalience, Point(lMaxValLocations[x].location.x, lMaxValLocations[x].location.y), 5, Scalar{0, tmpColour, 0}, 2, 8);
                }
            }
            
            //show the resultant salience map with the coloured circles drawn on
            imshow("Left Resultant Salience", lResultSalience);
 
            Point oldPoint(0,0);
            Point newPoint(0,0);
            key = waitKey(1);
			
			//perform template matching, using the current maxValLocation as the target
            if(lMaxValLocations.size()>location){
				//define target as the current maxValLocation
                Target = Rect(lMaxValLocations[location].location.x-32, lMaxValLocations[location].location.y-32, 64, 64);
                Mat OWLtempl(blankLeft, Target);
				
				//store the old matched template point from the static image
                oldPoint = Point( OWL_Static.Match.x + OWLtempl.cols/2 , OWL_Static.Match.y + OWLtempl.rows/2);
				//match templates
                OWL_L = Owl_matchTemplate_L( blankRight, blankLeft, OWLtempl, Target);
                OWL_R = Owl_matchTemplate_R( blankRight, blankLeft, OWLtempl, Target);
                OWL_Static = Owl_matchTemplate_L(blankRight, staticLeft, OWLtempl, Target);
				//store new point of matched template point
                newPoint = Point( OWL_Static.Match.x + OWLtempl.cols/2 , OWL_Static.Match.y + OWLtempl.rows/2);
				//draw circle onto the static image of the matched point
                circle(staticLeft, Point( OWL_Static.Match.x + OWLtempl.cols/2 , OWL_Static.Match.y + OWLtempl.rows/2), 5, Scalar::all(255), 1);
				//draw rectangles onto the original images around the matched template points
                rectangle( blankRight, OWL_R.Match, Point( OWL_R.Match.x + OWLtempl.cols , OWL_R.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
                rectangle( blankLeft, OWL_L.Match, Point( OWL_L.Match.x + OWLtempl.cols , OWL_L.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
                rectangle( OWL_L.Result, OWL_L.Match, Point( OWL_L.Match.x + OWLtempl.cols , OWL_L.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );

                //show images
                imshow("left", blankLeft);
                imshow("Static Left", staticLeft);
                imshow("right", blankRight);

            }


            double KPx=0.15; // track rate X
            double KPy=0.1; // track rate Y
            signed int pixelProx = 10;  //number of pixels that the target has to be within to the centre of the frame to count as a 'hit'
            //NEED TO MOVE ONLY IF MAXVALLOCATIONS CONTAINS ANY VALUES
            if(lMaxValLocations.size()>location){

                //Move to points in order of highest salience to lowest, [0],[1],[2]
				//calculate how many pixels off the target is from the centre of the images
                double LxScaleV = LxRangeV/(double)640; //PWM range /pixel range
                double Xoff= 380-(lMaxValLocations[location].location.x)/LxScaleV ; // compare to centre of image
                double LyScaleV = LyRangeV/(double)480; //PWM range /pixel range
                double Yoff= (250+(lMaxValLocations[location].location.y)/LyScaleV)*KPy ; // compare to centre of image
                cout << "Left off " << Xoff << " " << Yoff << endl;

                double RxScaleV = RxRangeV/(double)640; //PWM range /pixel range
                double RXoff= 330-(OWL_R.Match.x + OWLtempl.cols/2)/RxScaleV ; // compare to centre of image
                double RyScaleV = RyRangeV/(double)415; //PWM range /pixel range
                double RYoff= (250-(OWL_R.Match.y + OWLtempl.rows)/RyScaleV)*KPy ; // compare to centre of image
                cout << "Right off " << RXoff << " " << RYoff << endl;

                printf("location = %d\n", location);

				//Check that the centre  of left eye is more than 10 pixels away. If less than 10 pixels then we've hit the target
                if(Xoff > pixelProx || Xoff < (-1*pixelProx) || Yoff > pixelProx || Yoff < (-1*pixelProx)){
                   
					//update servo locations to verge to target
                    int LxOld=Lx;
                    Lx=LxOld-Xoff*KPx; // roughly 300 servo offset = 320 [pixel offset]
                    int LyOld=Ly;
                    Ly=LyOld-Yoff; // roughly 300 servo offset = 320 [pixel offset]

                }

				//Check that the centre  of right eye is more than 10 pixels away. If less than 10 pixels then we've hit the target
                if(RXoff > pixelProx || RXoff < (-1*pixelProx) || RYoff > pixelProx || RYoff < (-1*pixelProx)){
					
					//update servo locations to verge to target
                    int RxOld=Rx;
                    Rx=RxOld-RXoff*KPx; // roughly 300 servo offset = 320 [pixel offset
                    int RyOld=Ry;
                    Ry=RyOld+RYoff; // roughly 300 servo offset = 320 [pixel offset]

                }
				
				//update servo positions with new locations
				CMDstream.str("");
				CMDstream.clear();
				CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
				CMD = CMDstream.str();
				RxPacket= OwlSendPacket (u_sock, CMD.c_str());

                //check if reached the current max val location we are trying to look at (both eyes)
                //only move to new position once both eyes have hit
                if(lMaxValLocations.size() > location) {
					//if both eyes are within 10 pixels of the target then we have "hit" the target
                    if(Xoff < pixelProx && Xoff > (-1*pixelProx) && Yoff < pixelProx && Yoff > (-1*pixelProx)){
                        if(RXoff < pixelProx && RXoff > (-1*pixelProx) && RYoff < pixelProx && RYoff > (-1*pixelProx)){
                            // Calculating eye servo angles
                            float pwmStep = 0.113;
-
                            int LxDelta = Lx - LxC;         // change in pos in pwmsteps from center
                            double L_A = pwmStep * LxDelta;    // Left angle


                            int RxDelta = RxC - Rx;         // flipped so that when both eyes are toe-in the angles are positive, making the math easier
                            double R_A = pwmStep * RxDelta;    // Right angle

                            // Calculating hypotenuse

							//Make sure both L_A and R_A are positive
							if(L_A < 0){
								L_A = L_A * -1;
							}
							if(R_A < 0){
								R_A = R_A * -1;
							}

							L_A = 90 - L_A;     //Needed as we have angles from the centre but need angles from perpendicular to the owl face
							R_A = 90 - R_A;

							printf("L_A = %f\n", L_A);
							printf("R_A = %f\n", R_A);
							
							//calculate third angle in the triangle
							double theta3 = 180 - (L_A + R_A);
							double rad_to_deg = 180.0 / PI; // radians to degrees
							double deg_to_rad = PI / 180.0; // degrees to radians

							printf("theta3 = %f\n", theta3);
							
							//convert degrees to radians
							double L_A_rad = L_A * deg_to_rad; // sin function takes radians as an input
							double R_A_rad = R_A * deg_to_rad ;
							
							//calculate sine of the servo angle and convert back to degrees
							double sin_L_A = sin(L_A_rad)* rad_to_deg; // The trig math requires degrees
							double sin_R_A = sin(R_A_rad)* rad_to_deg;

							//calculate hypotenuses
							double R_Hypot = (65*(sin(L_A_rad)*rad_to_deg)) / (sin(theta3*deg_to_rad)*rad_to_deg);
							printf("R_Hypot = %f\n", R_Hypot);

							double L_Hypot = (65*(sin(R_A_rad)*rad_to_deg)) / (sin(theta3*deg_to_rad)*rad_to_deg);
							printf("L_Hypot = %f\n", L_Hypot);
							
							//calculate distance estimates
							double distanceEst_R = (R_Hypot * (sin(R_A_rad)*rad_to_deg)) / (sin(90*deg_to_rad)*rad_to_deg);
							printf("Distance Estimate R = %f\n", distanceEst_R);

							double distanceEst_L = (L_Hypot * (sin(L_A_rad)*rad_to_deg)) / (sin(90*deg_to_rad)*rad_to_deg);
							printf("Distance Estimate L = %f\n", distanceEst_L);

							//calculate new distance estimate by transforming distances to a new scale
							//this was determined by comparing real distance to previously determined distance estimates
                            double newDistanceEst = (distanceEst_R - 32) / (121/20);

                            printf("New Distance Estimate = %f\n", newDistanceEst);
							
							//create stringstream to contain a string with the distance estimate
                            std::ostringstream oss;
                            oss << "Distance Estimate = " << newDistanceEst;
                            text = oss.str();
							
                            
                            Mat OWLtempl(blankLeft, Target);
                            OWL_Static = Owl_matchTemplate_L(blankRight, staticLeft, OWLtempl, Target);
							//draw green circle onto staticLeft image 
                            circle(staticLeft, Point( OWL_Static.Match.x + OWLtempl.cols/2 , OWL_Static.Match.y + OWLtempl.rows/2), 5, Scalar(0,255,0), 1);
							
							//show static left image
                            imshow("Static Left", staticLeft);
                            key=waitKey(10);

							//if looking at the third top location then reset
                            if(location == 2){
                                location = 0;
                            }else{		//else look at next target
                                location += 1;
                            }
                        }
                    }
                }
			//put text containing the distance estimate onto the left image
            putText(blankLeft, text, Point(60,60), FONT_HERSHEY_PLAIN, 1,  Scalar(0,0,0));
			//show the images
            imshow("left", blankLeft);
            imshow("right", blankRight);

            }
           
        }
        destroyAllWindows();


        } // end while outer loop
#ifdef __WIN32__
        closesocket(u_sock);
#else
        close(clientSock);
#endif
        exit(0); // exit here for servo testing only
    }
