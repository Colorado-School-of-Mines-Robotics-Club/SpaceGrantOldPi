#include <iostream>
#include <list>
#include <vector>

#include "RangeFinding.h"

using namespace std;
//get array of values, find ones that are too far or too close than 1m, put them in an array of problem points
//input: array of structs with distance in mm and angle

//look for clumps of issues, return vector of Clumps (have start point and end point)


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