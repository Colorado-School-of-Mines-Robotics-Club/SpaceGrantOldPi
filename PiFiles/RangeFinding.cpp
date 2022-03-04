#include <iostream>
#include <list>
#include <vector>

#include "RangeFinding.h"

using namespace std;
//get array of values, find ones that are too far or too close than 1m, put them in an array of problem points
//input: array of structs with distance in mm and angle

//look for clumps of issues, return vector of Clumps (have start point and end point)

struct Point{
    int distance;
    float angle;
};
struct Clump{
    Point point1;
    Point point2;
};
vector<Clump>* RangeFinding (vector<Point> data){
    vector<Clump>* RangeFinding::RangeFinder (vector<RangeFinderPacket> data){
        vector<Clump> *clumps = new vector<Clump>;
        vector<Clump> *returnClumps = new vector<Clump>;
        const int threshold_low = 900;   //0.9m; Lower than this is concerningly close; replace with actual int later
        const int threshold_high = 1100;  //1.1m; Higher than this is concerningly high; replace with actual int later
        const int clumpAngle = 15; //This or larger angle between concerning points classifies it as a clump; replace with actual value later
        const int numPointsChecked=10; //Num of previous points to average and check that a new clump hasn't started; replace with actual value later
        const int changeInDistance = 50; //Greater distance than this results in new clump starting; replace with actual value later
        bool isClump = false;
        bool inThreshold = false;
        Clump clump;
        float average = 0; //keeps track of average value in clump
        int sum = 0; //keeps track of num points in clump
        for(int i=0; i<data.size(); i++){
            inThreshold = !(data[i].distance<threshold_low || data[i].distance>threshold_high);
            if(!inThreshold && !isClump){
                //if not in threshhold, point is starting clump
                clump.point1 = data[i];
                isClump = true;
                sum++;
                average = data[i].distance;
                cout << i << " is not in threshold and not currently a clump\n";
            }
            else if(inThreshold && isClump){
                //previous point is end of clump
                clump.point2 = data[i-1];
                clumps->push_back((clump));
                isClump = false;
                sum = 0;
                average = 0;
                cout << i << " is in threshold and is clump\n";
            }
            else if(isClump){
                sum++;
                average = (average+data[i].distance)/sum;
                if(abs(data[i].distance-average) > changeInDistance){
                    clump.point2=data[i-1];
                    clumps->push_back(clump);
                    isClump=false;
                    sum = 0;
                    average = 0;
                    if(!inThreshold){
                        cout << "new clump\n";
                        isClump=true;
                        clump.point1=data[i];
                        sum++;
                        average = data[i].distance;
                    }
                }
                cout << i << " is a clump and average is " << average << "\n";
            }
        }

        for (Clump c: *clumps){
            int difference = c.point2.angle - c.point1.angle;
            difference = abs(difference);
            cout << "This is a clump with difference " << difference << "\n";
            if(difference >= clumpAngle){
                returnClumps->push_back(c);
            }
        }

        return returnClumps;
    }

    int main() {
        //two normal points
        struct Point p1 = {950, 5.4};
        struct Point p2 = {910, 10.3};
        //six points too close
        struct Point p3 = {893, 11.4};
        struct Point p4 = {890, 14.3};
        struct Point p5 = {870, 15.6};
        struct Point p6 = {886, 22.3};
        struct Point p7 = {879, 24.5};
        struct Point p8 = {892, 28.2};
        //immediately switch to two points too far (should be separate clumps)
        struct Point p9 = {1203, 30.7};
        struct Point p10 = {1204, 45.9};
        //dip into threshold
        struct Point p11 = {1096, 56.7};

        //creates vector of points
        vector<Point> vect {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11};

        //calls rangeFinding
        vector<Clump>* clumps = RangeFinding(vect);
        for (Clump c: *clumps){
            cout << "Clump: Point 2: " << c.point2.distance << ", " << c.point2.angle << " Point 1: " << c.point1.distance << ", " << c.point1.angle;
        }
        return 0;
    }

    int tester() {
//    //two normal points
//    struct RangeFinderPacket p1 = {950, 5};
//    struct RangeFinderPacket p2 = {910, 10};
//    //six points too close
//    struct RangeFinderPacket p3 = {893, 11};
//    struct RangeFinderPacket p4 = {890, 14};
//    struct RangeFinderPacket p5 = {870, 15};
//    struct RangeFinderPacket p6 = {886, 22};
//    struct RangeFinderPacket p7 = {879, 24};
//    struct RangeFinderPacket p8 = {892, 28};
//    //immediately switch to two points too far (should be separate clumps)
//    struct RangeFinderPacket p9 = {1203, 30};
//    struct RangeFinderPacket p10 = {1204, 45};
//    //dip into threshold
//    struct RangeFinderPacket p11 = {1096, 56};
//
//    //creates vector of points
//    vector<RangeFinderPacket> vect {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11};
//
//    //calls rangeFinding
//    vector<Clump>* clumps = RangeFinding(vect);
//    for (Clump c: *clumps){
//        cout << "Clump: Point 2: " << c.point2.distance << ", " << c.point2.angle << " Point 1: " << c.point1.distance << ", " << c.point1.angle;
//    }
        return 0;
    }