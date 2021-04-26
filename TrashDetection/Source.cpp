//Includes for the project <librealsense2> and <opencv> are external libs, meaning property sheet is needed
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>


// Struct to store a set of frames from realsense
struct Frame {
    rs2::frameset frameset;
    rs2::frame colorFrame;
    rs2::frame depthFrame;
    uint height{};
    uint width{};
    cv::Mat matImage;
    uint32_t count = 0;
};

// Get a frame from realsense. Purpose of this function is to group the image collecting here.
void retrieveFrame(const rs2::pipeline& pipe, Frame* frame) {
    frame->frameset = pipe.wait_for_frames();                               // Get a frameset from realsense pipeline object. This object holds all data from the realsense camera
    frame->colorFrame = frame->frameset.get_color_frame();
    //frame->depthFrame = frame->frameset.get_depth_frame();                // We do not need depth frame for anything.. yet
    frame->width = frame->colorFrame.as<rs2::video_frame>().get_width();    // Width for OpenCV Mat object
    frame->height = frame->colorFrame.as<rs2::video_frame>().get_height();  // Height for OpenCV Mat object
    frame->matImage = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC3, (void*)frame->colorFrame.get_data(),
        cv::Mat::AUTO_STEP);                          // Construct openCV mat object used in zBar and homogryaphy


// Increment frame number
    frame->count++;
}


// Struct used to store information about each contour
struct features
{
    int contourIndex;
    int area; 
};


void FindGraspingPoint(cv::Mat1b bin) { //Receives grayscale image   

    // Find contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    //Creating vector<features> to store features
    std::vector<features> featVec;
    for (int i = 0; i < contours.size(); i++) {

        features f;
        f.contourIndex = i;

        //Store all features in featVec
        featVec.push_back(f);
        f.area = contourArea(contours[i]);
    }

    // Output image
    cv::Mat out;
    cv::cvtColor(bin, out, cv::COLOR_GRAY2BGR);
    
    //loop through contours
    for (int i = 0; i < contours.size(); i++) {
        if(featVec[i].area > 4000){
        // Draw on mask
        cv::Mat1b mask(bin.rows, bin.cols, uchar(0));
        cv::Mat1f dt;
        double max_val;
        cv::Point max_loc;
        cv::drawContours(mask, contours, featVec[i].contourIndex, cv::Scalar(255), cv::FILLED);

        // Distance Trasnsform
        cv::distanceTransform(mask, dt, cv::DIST_L2, 5, cv::DIST_LABEL_PIXEL);

        // Find max value
        cv::minMaxLoc(dt, nullptr, &max_val, nullptr, &max_loc);

        cv::circle(out, max_loc, max_val, cv::Scalar(0, 255, 0), 2);
        }
        std::cout<<featVec[i].area <<std::endl;
    }

    cv::imshow("Biggest Circles ", out);

}


int main()
{
    // -- REALSENSE SETUP --
   // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming from camera with default recommended configuration
    pipe.start();

    Frame frame;
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    cv::namedWindow("Image", cv::WINDOW_FREERATIO);
    printf("Start filming the scene\n");
    //We run a while loop to keep updating the depthImage
    while (cv::waitKey(1) < 0)
    {
        retrieveFrame(pipe, &frame);     // Get a set of frames from realsense Camera
        cvtColor(frame.matImage, frame.matImage, cv::COLOR_BGR2RGB);      // Convert to RGB to display correct colors
        cv::Mat detectGreen = frame.matImage;

        //Here we make the thresholds for the three colors
        cv::inRange(frame.matImage, cv::Scalar(0, 80, 0), cv::Scalar(90, 170, 90), detectGreen);
        cv::imshow("Green", detectGreen);
        //We create a structuring element with the shape of a ellipse as most of the objects have circular shapes
        cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

        //a morphology function to perform a CLOSE operaiton. This is a compound function 
        //that will first perform dilation followed by erosion
        cv::morphologyEx(detectGreen, detectGreen, cv::MORPH_OPEN, elem);
       
        FindGraspingPoint(detectGreen);
        
    }
}

