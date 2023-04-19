#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <ctime>
#include <locale>
#include <vector>
#include <algorithm>
#include <string>
#include <nav_msgs/GetPlan.h>

//to convert radians and degrees
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

const int numBoxes = 5;
const int numCoords = 3;
const int numPics = 3;
const int numBoxesXPics = numBoxes * numPics;

//sets angle to range of -Pi and Pi
float normalizeAngle(float angle){
    if (angle > M_PI){
        angle = angle -2*M_PI;
    }
    return angle;
}

float inbetweenDist (float x1, float y1, float x2, float y2){
    //gets the distanc between two points
    return sqrt(pow(x2-x1,2)+ pow(y2-y1,2));
}

void loadDriveCoords (float driveCoords[numBoxes][numCoords], Boxes* boxesPtr, float offset){
    //loads the coordinates of the boxes using boxesPtr -> coords
    //New x, y, phi are calculated to be slightly offset from the box such that the robot faces the front of the box

    Boxes boxes = *boxesPtr;

    for (int i = 0; i < boxes.coords.size(); i++){
        float boxX = boxes.coords[i][0];
        float boxY = boxes.coords[i][1];
        float boxAngle = boxes.coords[i][2];
        float driveX = boxX + offset * cosf(boxAngle);
        float driveY = boxY + offset * sinf(boxAngle);
        float driveAngle = normalizeAngle(boxAngle + M_PI);
        driveCoords[i][0] = driveX;
        driveCoords[i][1] = driveY;
        driveCoords[i][2] = driveAngle;
        //ROS_INFO("Cereal Box Coords %d: (%.3f,%.3f,%.3f)", i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
        //ROS_INFO("Drive Coords %d: (%.3f, %.3f, %.3f)", i, driveCoords[i][0], driveCoords[i][1], driveCoords[i][2]);
    }
}

//this function is similar to loadDriveCoords, however it forces the robot to drive to 3 spots in front of the robot
void loadMultDriveCoords (float driveMultCoords[numBoxesXPics][numCoords], Boxes* boxesPtr, float offset, float angleOffset){
    //loads the coordinates of the boxes using boxesPtr -> coords
    //New x, y, phi are calculated to be slightly offset from the box such that the robot faces the front of the box
    Boxes boxes = *boxesPtr;

    //fill driveCoords
    for (int i = 0; i < boxes.coords.size(); i++){
        float boxX = boxes.coords[i][0];
        float boxY = boxes.coords[i][1];
        float boxAngle = boxes.coords[i][2];
        float driveX = boxX + offset * cosf(boxAngle);
        float driveY = boxY + offset * sinf(boxAngle);
        float driveAngle = normalizeAngle(boxAngle + M_PI);

        // drive coordinates for directly in front of the box
        driveMultCoords[i*3][0] = driveX;
        driveMultCoords[i*3][1] = driveY;
        driveMultCoords[i*3][2] = driveAngle;

        // drive coordinates for slightly to the left of the box
        float leftAngle = normalizeAngle(boxAngle + angleOffset);
        driveMultCoords[i*3+1][0] = boxX + offset * cosf(leftAngle);
        driveMultCoords[i*3+1][1] = boxY + offset * sinf(leftAngle);
        driveMultCoords[i*3+1][2] = normalizeAngle(leftAngle + M_PI);

        // drive coordinates for slightly to the right of the box
        float rightAngle = normalizeAngle(boxAngle - angleOffset);
        driveMultCoords[i*3+2][0] = boxX + offset * cosf(rightAngle);
        driveMultCoords[i*3+2][1] = boxY + offset * sinf(rightAngle);
        driveMultCoords[i*3+2][2] = normalizeAngle(rightAngle + M_PI);

        //print out the drive instructions
        //ROS_INFO("Cereal Box Coords %d: (%.3f,%.3f,%.3f)", i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
        //ROS_INFO("Drive Coords %d - Directly in front: (%.3f, %.3f, %.3f)", i, driveMultCoords[i*3][0], driveMultCoords[i*3][1], driveMultCoords[i*3][2]);
        //ROS_INFO("Drive Coords %d - Left: (%.3f, %.3f, %.3f)", i, driveMultCoords[i*3+1][0], driveMultCoords[i*3+1][1], driveMultCoords[i*3+1][2]);
        //ROS_INFO("Drive Coords %d - Right: (%.3f, %.3f, %.3f)", i, driveMultCoords[i*3+2][0], driveMultCoords[i*3+2][1], driveMultCoords[i*3+2][2]);
    }
}


//this function fills a matrix with the distances between each box
void distanceStorage (float distMatrix[numBoxes][numBoxes], float driveCoords[numBoxes][numCoords]){
    for (int i=0; i < numBoxes; i++){
        for (int j=0; j < numBoxes; j++){
            distMatrix[i][j] = inbetweenDist(driveCoords[i][0], driveCoords[i][1], driveCoords[j][0], driveCoords[j][1]);
            //ROS_INFO("Distance between (%d, %d): %.3f",i, j, distMatrix[i][j]);
        }
    }
}

//finds the closest box to the robot at the start of the contest
int nearestBoxToStart (float driveCoords[numBoxes][numCoords]){
    float minDist = std::numeric_limits<float>::infinity();   //set to largest possible value
    float currentDist;
    int boxIndex = -1;    
    for (int i=0; i < numBoxes; i++){
        currentDist = inbetweenDist(0, 0, driveCoords[i][0], driveCoords[i][1]);
        //ROS_INFO("Distance to box %d: %.3f",i, currentDist);
        if (currentDist < minDist){
            minDist = currentDist;
            boxIndex = i;
        }
    }
    ROS_INFO("Nearest Box: %d", boxIndex);
    return boxIndex;
}

float findOptimalPath(float driveCoords[numBoxes][numCoords], float distMatrix[numBoxes][numBoxes], int startPoint, std::vector<int> &optimalPath){
     //calculates the optimal path distance and box order using the brute force algorithm
    //driveCoords: array of box coordinates
    //distStorage: array of distances between boxes
    //startPoint: start of box order
    //optimalPath: vector of box indices in order of the optimal path.
    std::vector<int> cerealBoxes;

    //create the cerelaBoxes vector without the start point
    for(int i = 0; i < numBoxes; i++){
        if(i != startPoint){
            cerealBoxes.push_back(i);
        }
    }

    float smallestPathDist = std::numeric_limits<float>::infinity();

    do {
        float currentPathDist = 0;
        std::vector<int> currentSequence = {startPoint};
        int j = startPoint;

        for (int cerealBox : cerealBoxes) {
            currentPathDist += distMatrix[j][cerealBox];
            currentSequence.push_back(cerealBox);
            j = cerealBox;

        }

        currentPathDist += distMatrix[j][startPoint];

        if (currentPathDist < smallestPathDist){
            smallestPathDist = currentPathDist;
            optimalPath = currentSequence;
        }

    } while (std::next_permutation(cerealBoxes.begin(), cerealBoxes.end()));

    //std::cout << " Distance: " << smallestPathDist << std::endl;
    std::cout << "Printing Visiting Order: ";

    for (int cerealBox : optimalPath) {
        std::cout << cerealBox << " ";

    }

    std::cout << std::endl;
    return smallestPathDist;
}


//we need to check whether the path to the boxes is valid
//to do this, we can check the current x,y,phi position and compare to the next x,y,phi recommend by the 'optimalPath' vector
bool isPathValid (ros::NodeHandle& nh, float curX, float curY, float curPhi, float nextX, float nextY, float nextPhi ){
    geometry_msgs::PoseStamped start, goal;
    start.header.frame_id = goal.header.frame_id = "map";
    start.header.stamp = goal.header.stamp = ros::Time::now();
    start.pose.position.x = curX;
    start.pose.position.y = curY;
    start.pose.position.z = goal.pose.position.z = 0.0;
    start.pose.orientation = tf::createQuaternionMsgFromYaw(curPhi);
    goal.pose.position.x = nextX;
    goal.pose.position.y = nextY;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(nextPhi);

    //call make_plan service
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = 0.0;

    if(nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan").call(srv)){
        ROS_INFO("Sending call to check if path is valid");
        return srv.response.plan.poses.size() > 0;
    }
    else{
        ROS_INFO("Failed to make call to check path validity");
        return false;
    }
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " phi: " << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    //initialize all variables
    float driveCoords[numBoxes][numCoords], driveMultCoords[numBoxesXPics][numCoords];
    float nextX, nextY, nextPhi, shortestPathDist, offset = 0.6, angleOffset = DEG2RAD(20);
    float distMatrix[numBoxes][numBoxes];
    int startPoint, template_ID, currentBox = 0;
    std::vector<int> optimalPath, pastID;
    std::array<float,3> origin;
    bool validPath, moveCompleted, dupTemplate = false, doneExploring = false;
    float changePhi = DEG2RAD(20);
    std::string filename, timeStr;
    int maxAngle = 61, angleInc = 15;
    int count = 0;
    std::string duplicateText;


    //writes the output file
    // Get timestamp for output file name
    time_t t = time(0);   // get time now
    struct tm * now = localtime(&t);
    char timestamp[150];
    strftime(timestamp, 150, "%Y-%m-%d %H-%M-%S", now);
    timeStr = std::string(timestamp);
    //use this filename when running on MIE443 Laptop
    filename = "/home/thursday2023/catkin_ws/src/mie443_contest2/Group 4 Output - " + timeStr + ".csv";

    //use this filename when running on Kevon Laptop
    //filename = "/home/turtlebot/catkin_ws/src/mie443_contest2/Group 4 Output - " + timeStr + ".csv";
    // Write output file if it doesn't exist yet with headers
    std::ofstream output(filename);
    output << "\"Path Taken (Box ID)\"" << ", " << "\"Image Tag\"" << ", " << "\"Is Duplicate\"" << ", " << "\"Location (x, y, angle)\"" << std::endl;
    
    //load the general driving coordinates
    loadDriveCoords (driveCoords, &boxes, offset);

    //load the specfic drive coordiantes
    loadMultDriveCoords (driveMultCoords, &boxes, offset, angleOffset);

    //calculate the distances between the cereal boxes
    distanceStorage (distMatrix, driveCoords);

    //Find the closest box to the robot at the beginning of the contest
    startPoint = nearestBoxToStart(driveCoords);

    //find the optimal path and shortest distance to navigate to the boxes.
    shortestPathDist = findOptimalPath (driveCoords, distMatrix, startPoint, optimalPath);

    //store origin
    origin = {robotPose.x, robotPose.y, robotPose.phi};
    ROS_INFO("Starting position:\n\tx: %5.3f\ty: %5.3f\tyaw: %5.3f", origin[0], origin[1], origin[2]);

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();

        if (currentBox <= numBoxes){
            while(count < numPics){
                ROS_INFO("Count: %d", count);
            //set the x,y,phi coordinates to go to three locations in front the box
                if(count == 0){
                    nextX = driveMultCoords[optimalPath[currentBox]*numPics+1][0];
                    nextY = driveMultCoords[optimalPath[currentBox]*numPics+1][1];
                    nextPhi = driveMultCoords[optimalPath[currentBox]*numPics+1][2];
                }
                if(count == 1){
                    nextX = driveMultCoords[optimalPath[currentBox]*numPics][0];
                    nextY = driveMultCoords[optimalPath[currentBox]*numPics][1];
                    nextPhi = driveMultCoords[optimalPath[currentBox]*numPics][2];
                }
                if(count == 2){
                    nextX = driveMultCoords[optimalPath[currentBox]*numPics+2][0];
                    nextY = driveMultCoords[optimalPath[currentBox]*numPics+2][1];
                    nextPhi = driveMultCoords[optimalPath[currentBox]*numPics+2][2];
                }
                //need to check if the set driving coordinates are actually valid
                validPath = isPathValid (n, robotPose.x, robotPose.y, robotPose.phi, nextX, nextY, nextPhi);

                //if the path is not valid, recalculate the new drive coordinates until there is a valid path:
                //the while loop varies the angle by 15 degrees and checks if the path is valid, if it is still not valid, it changes the angle again
                //the loop goes +-15,30,45,60 degrees
                if (!validPath) {
                    while (fabs(changePhi) <= DEG2RAD(maxAngle) && currentBox < numBoxes) {
                        nextX = boxes.coords[optimalPath[currentBox]][0] + offset * cosf(boxes.coords[optimalPath[currentBox]][2] + changePhi);
                        nextY = boxes.coords[optimalPath[currentBox]][1] + offset * sinf(boxes.coords[optimalPath[currentBox]][2] + changePhi);
                        nextPhi = normalizeAngle(boxes.coords[optimalPath[currentBox]][2] + changePhi + M_PI);

                        ROS_INFO("Testing new drive coords %d (original %d) with offset %.1f. (%.3f, %.3f, %.3f)", currentBox, optimalPath[currentBox], RAD2DEG(changePhi), nextX, nextY, nextPhi);
                        validPath = isPathValid(n, robotPose.x, robotPose.y, robotPose.phi, nextX, nextY, nextPhi);

                        if (validPath) {
                            break;
                        } 
                        changePhi = (changePhi > 0) ? -changePhi : -changePhi + DEG2RAD(angleInc);
                    }
                }

                //if the path is valid: execute the following
                if (validPath) {
                    moveCompleted = Navigation::moveToGoal(nextX, nextY, nextPhi);
                    ROS_INFO("Arrived at box");

                    if (moveCompleted && currentBox < numBoxes) {
                        // Wait to receive the picture from the camera
                        ros::Duration(0.10).sleep();
                        ros::spinOnce();
                        ros::Duration(0.60).sleep();
                        ros::spinOnce();
                        
                        // Get template ID and check for duplicates
                        template_ID = imagePipeline.getTemplateID(boxes);
                        dupTemplate = imagePipeline.Duplicate(pastID, template_ID);

                        duplicateText = (dupTemplate) ? "True" : "False";

                        if (!dupTemplate) {
                            pastID.push_back(template_ID);
                            ROS_INFO("Appended %i to pastID", template_ID);
                        }
                
                        // Append data to output file
                        ROS_INFO("Appending to output file");
                        output << "\"" << optimalPath[currentBox] << "\", \"" << imagePipeline.tagIndexToString(template_ID) << "\", \"" << duplicateText << "\", \"" << "("
                        << boxes.coords[optimalPath[currentBox]][0] << ", " << boxes.coords[optimalPath[currentBox]][1] << ", " << boxes.coords[optimalPath[currentBox]][2] <<  ")" << "\""
                        << std::endl;
                        count++;
                        
                    }
                    else{
                        if(currentBox < numBoxes){
                            ROS_INFO("PLAN VALID BUT NAVIGATION FAILED");
                            output << currentBox+1 << "\", \"" << optimalPath[currentBox] << "\", \"" << "" << "\", \"" << "" << "\", \"" << "\"" << std::endl;
                        }
                        count++;
                    }
                }
                if (count == 3){
                    currentBox ++;
                    if (currentBox == numBoxes){
                        doneExploring = true;
                    }
                    count = 0;
                    break;
                }   
            }
            if(fabs(changePhi) > DEG2RAD(maxAngle) || !validPath){
                ROS_INFO("Could Not Find Any Path to Node");
            } 
        }
        if (doneExploring){
            //all boxes have been explored
            ROS_INFO("Returning to Origin");
            Navigation::moveToGoal(origin[0], origin[1], origin[2]); 
            break;
        }
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        ros::Duration(0.01).sleep();
    }
    output.close();
    return 0;
}



    

