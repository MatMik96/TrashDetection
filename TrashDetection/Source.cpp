//Includes for the project <librealsense2> and <opencv> are external libs, meaning property sheet is needed
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
//#include <zbar.h>

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

// Struct to store the scanned QR codes
struct Object : public cv::_InputArray {
    std::string data;
    std::vector<cv::Point> location;
    cv::Point center;
};

// Struct used to store information about each contour
struct features {
    int contourIndex;
    int area;
};

// Add names for QR codes here if you add more QR codes to scene
const std::vector<std::string> qrCustomNames = { "ID_1", "ID_2", "ID_3", "ID_4" };

// ROS service client
//ros::ServiceClient client;


/*
// Find and decode barcodes and QR codes
void decode(cv::Mat& im, std::vector<Object>& decodedObjects) {

    // Create zbar scanner
    zbar::ImageScanner scanner;

    // Configure scanner
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Convert image to grayscale
    cv::Mat imGray;
    cvtColor(im, imGray, cv::
        COLOR_BGR2GRAY);

    // Wrap image data in a zbar image
    zbar::Image image(im.cols, im.rows, "GREY", (uchar*)imGray.data, im.cols * im.rows);

    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);

    // Print results
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        Object obj;
        obj.data = symbol->get_data();

        // Obtain location
        for (int i = 0; i < symbol->get_location_size(); i++) {
            obj.location.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
        }

        // calculate center coords
        int sumX = 0, sumY = 0;
        for (auto& i : obj.location) {
            sumX += i.x;
            sumY += i.y;
        }
        obj.center.x = sumX / obj.location.size();
        obj.center.y = sumY / obj.location.size();

        decodedObjects.push_back(obj);

    }
}*/

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



void FindGraspingPoint(cv::Mat1b bin, cv::Mat hMatrix) { //Receives grayscale image   

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

        //lego_throw::camera camSrv;
        //camSrv.request.x = point.x;
        //camSrv.request.y = point.y;
        //camSrv.request.z = 0.05;
        //camSrv.request.data = max_loc;
        //camSrv.request.time = time_stamp;
      
        //if (client.call(camSrv)) printf("Response status: %i\n", camSrv.response.status);
        }
        std::cout<<featVec[i].area <<std::endl;

    }

    cv::imshow("Biggest Circles ", out);

}



cv::Mat doHomography(const std::vector<Object> objects, cv::Mat* colorImage) {
    // Find homography matrix needs 8 points.
    std::vector<cv::Point2f> surfaceQR(4);     // Four corners of the real world plane
    std::vector<cv::Point2f> cameraQR(4);      // Four corners of the image plane


    // The QR codes that is scanned from zBar does not come ordered.
    // Thus we want to sort the corresponding points in SurfaceQR and cameraQR such that corresponding points is at the same index.
    int amountQRCornersFound = 0;

    //ROBOT TABLE POINTS
    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "00") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(0, 0);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "01") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(72.4, 0);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "02") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(0, 69.7);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "03") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(72.4, 69.7);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
    }

    if (amountQRCornersFound != 4){
        cv::Mat empty;
        return empty;
    }

    //calculate Homography matrix from 4 sets of corresponding points
    cv::Mat hMatrix = findHomography(cameraQR, surfaceQR);
    return hMatrix;

    //cv::Mat homographyImage;
    //warpPerspective(colorImage, homographyImage, hMatrix, cv::Size(800, 870));
}



int main()
{
    // -- REALSENSE SETUP --
    rs2::pipeline pipe;
    pipe.start(); 

    // --- ROS STUFF ---
    //ros::init(argc, argv, "realsenseVision");
    //ros::NodeHandle node_handle;

    // Creating the client
    //client = node_handle.serviceClient<lego_throw::camera>("camera");
    //client.waitForExistence();

    std::vector<Object> decodedObjects;

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
       // cv::Mat detectGreen = frame.matImage;
        cv::Mat detectRed, detectBlue, detectGreen = frame.matImage;

       // Here we make the thresholds for the three colors
        cv::inRange(frame.matImage, cv::Scalar(0, 0, 130), cv::Scalar(50, 50, 170), detectRed); //Rød
        cv::inRange(frame.matImage, cv::Scalar(80, 50, 10), cv::Scalar(100, 70, 25), detectBlue); //Blå
        cv::inRange(frame.matImage, cv::Scalar(40, 50, 15), cv::Scalar(60, 70, 40), detectGreen); //Grøn
        cv::imshow("Red", detectRed);
        cv::imshow("Blue", detectBlue);
        cv::imshow("Green", detectGreen);
       

        cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

        cv::morphologyEx(detectGreen, detectGreen, cv::MORPH_OPEN, elem);
        //decode(frame.matImage, decodedObjects);  
        cv::Mat hMatrix;
        if (decodedObjects.size() > 3) {                                         
           hMatrix = doHomography(decodedObjects, &frame.matImage);

           if (!hMatrix.empty()) {
               FindGraspingPoint(detectGreen, hMatrix);
           }
        }  

        if (cv::waitKey(25) == 27) break;  // If ESC is pushed then break loop
    }
}

