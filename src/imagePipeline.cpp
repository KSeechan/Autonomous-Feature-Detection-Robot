#include <imagePipeline.h>
#include <string>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

bool ImagePipeline::Duplicate(std::vector<int> &pastID, int template_ID){
    return (std::find(pastID.begin(), pastID.end(), template_ID) != pastID.end());
}

std::string ImagePipeline::tagIndexToString(int idx) {

    constexpr char prefix[] = "tag_";
    constexpr char suffix[] = ".jpg";
    constexpr char blankSuffix[] = "blank.jpg";

    // Check for the blank case
    if (idx == -1) {
        return std::string(prefix) + std::string(blankSuffix);
    }

    // Compute the required string length (not including null terminator)
    int numDigits = static_cast<int>(std::log10(idx + 1)) + 1;
    int length = sizeof(prefix) + numDigits + sizeof(suffix);

    // Allocate a buffer to hold the string
    std::string result;
    result.reserve(length);

    // Copy the prefix
    result.append(prefix, sizeof(prefix) - 1);

    // Copy the index value
    char indexStr[12]; // Enough space for a 32-bit integer
    int numChars = std::snprintf(indexStr, sizeof(indexStr), "%d", idx + 1);
    result.append(indexStr, numChars);

    // Copy the suffix
    result.append(suffix, sizeof(suffix) - 1);

    return result;
}

// Resize template image to match input image aspect ratio
double ImagePipeline::calculateSimilarityScore(Mat template_image) {
    // Convert input image to grayscale
    cv::Mat gray_image;
    cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
    cv::resize(template_image, template_image, cv::Size(500, 400));

    // Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint> keypoints_template, keypoints_input;
    Mat descriptors_template, descriptors_input;
    detector->detectAndCompute(template_image, Mat(), keypoints_template, descriptors_template);
    detector->detectAndCompute(gray_image, Mat(), keypoints_input, descriptors_input);

    // Lowe's ratio filer
    // Step 3: Matching descriptor vectors using FLANN matcher
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch(descriptors_template, descriptors_input, knn_matches, 2);

    // Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    // Localize the object
    std::vector<Point2f> template_points;
    std::vector<Point2f> input_points;

    for (int i = 0; i < good_matches.size(); i++) {
        // Get the keypoints from the good matches
        template_points.push_back(keypoints_template[good_matches[i].queryIdx].pt);
        input_points.push_back(keypoints_input[good_matches[i].trainIdx].pt);
    }

    if( input_points.empty()){
        std::cout << "Error: No keypoints detected in the captured image." << std::endl;
        return 0.0;  //set similarity score to zero.
    }

    Mat homography = findHomography(template_points, input_points, RANSAC);

    // Get the corners from the template image
    std::vector<Point2f> template_corners(4);
    template_corners[0] = cvPoint(0, 0);
    template_corners[1] = cvPoint(template_image.cols, 0);
    template_corners[2] = cvPoint(template_image.cols, template_image.rows);
    template_corners[3] = cvPoint(0, template_image.rows);
    std::vector<Point2f> input_corners(4);

    // Define input_corners using homography
    if (!homography.empty()) {
        perspectiveTransform(template_corners, input_corners, homography);
    } else {
        std::cout << "Error: Homography matrix is empty." << std::endl;
    }

    // Define contour using the input_corners
    std::vector<Point2f> contour;
    for (int i = 0; i < 4; i++) {
        contour.push_back(input_corners[i] + Point2f(template_image.cols, 0));
    }

    double area = contourArea(contour);
    double area_weight = 1.0;

    area_weight = (area > 1000 * 1000 || area < 5 * 5) ? 0.0 : 1.0;

    std::cout << "area: " << area << ", weight: " << area_weight << std::endl;

    double score;

    std::vector< DMatch > best_matches;

    // Check if the good match is inside the contour. If so, write in best_matches and multiply by area weight
    Point2f matched_point;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        matched_point = keypoints_input[ good_matches[i].trainIdx ].pt + Point2f( template_image.cols, 0);
        score = pointPolygonTest(contour, matched_point, false);
        if(score >= 0) best_matches.push_back( good_matches[i]);
    }
   
    cv::waitKey(10);

    double similarityScore = best_matches.size()*area_weight;
    return similarityScore;
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    double best_matches, matches;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } 
    else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } 
    else {
        //test value
        best_matches = 10;
        
        //loop through each box template
        for(size_t i = 0; i < boxes.templates.size(); ++i){
            matches = calculateSimilarityScore(boxes.templates[i]);

            if (matches > best_matches){
                best_matches = matches;
                template_id = i;
            }
        }
    } 
    //show grayscale image in Rviz
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    //display the scene image
    cv::imshow("view", img);
    cv::waitKey(10); 

    if (template_id == -1) {
        std::cout << "No matches found" << std::endl;
    } 
    else {
        std::cout << "The best template is " << template_id << " with " << best_matches << " matches" << std::endl;
    }

    return template_id;
}
