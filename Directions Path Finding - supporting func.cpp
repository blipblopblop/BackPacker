/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m3_helper_functions.h"
//#include "m2_helper_functions.h"
#include "m3.h"

//prev file headers
#include "m1.h"
//#include "Node.h"
#include <cmath>
#include "StreetsDatabaseAPI.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <cctype>
#include <iostream>
#include <locale>
#include <vector>
#include <queue>
#include <string.h>
#include <map>
#include <iterator>
#include "OSMDatabaseAPI.h"
#include "LatLon.h"

//EZGL headers.
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m3_helper_functions.h"

//Useful DEFINE directives.
#define NO_CONNECTED_START_INTERSECTION -1

//Helper functions for find_path_with_walk_to_pickup.

//Reverses a vector of integers.
void reverseIntVector(std::vector<int>& vector)
{
    std::vector<int> reverseVector;
    reverseVector.clear();
    for (auto it = vector.rbegin(); it != vector.rend(); it++)
        reverseVector.push_back(*it);
    
    vector = reverseVector;
    return;
}

//Returns a map sorted by intersection indices and containing the
//shortest paths to those intersection indices by walking from the start_intersection.
std::map<IntersectionIndex, std::vector<StreetSegmentIndex>>
        find_intersections_within_walking_time(
                            const IntersectionIndex start_intersection,
                            const double turn_penalty,
                            const double walking_speed, 
                            const double walking_time_limit)
{
    //Map that gets built-up with the shortest paths to all
    //intersections within walkable distance. Intersection indices are used to get
    //the corresponding path to them, if one exists.
    std::map<IntersectionIndex, std::vector<StreetSegmentIndex>> pathsToWalkableIntersections;
    pathsToWalkableIntersections.clear();
    
    //Binary tree to keep track of nodes to visit (in closest-first order) as well
    //as the shortest paths to get to these nodes. The key of each node is the time
    //required to get to it from the start_intersection. It starts with just the start_intersection,
    //and new nodes are added for processing so long as the time to get to them does
    //not exceed the walking_time_limit.
    std::multimap<const double, QueueElement> intersectionsToVisit;
    intersectionsToVisit.clear();
    
    //The starting node (intersection) is added for processing.
    std::vector<StreetSegmentIndex> blankPath;
    blankPath.clear();
    QueueElement startingNode(start_intersection, blankPath);
    intersectionsToVisit.insert( std::make_pair(0, startingNode) );
    
    //The main processing loop of the algorithm. So long as there are entries
    //within intersectionsToVisit, this loop keeps running. As nodes are processed, 
    //they are removed from this map. New nodes to process are only added if the time
    //required to get to them does not exceed the walking_time_limit.
    while (!intersectionsToVisit.empty())
    {
        //Get the node in the queue that is closest to the starting intersection
        //in terms of travel time. Note that the queue, in this case a map,
        //is sorted according to travel time from the starting_intersection.
        //Hence, the closest node is always the first element in the queue.
        QueueElement currentIntersectionAndPath = intersectionsToVisit.begin()->second;
        
        //Check if the current node has already been visited.
        if ( pathsToWalkableIntersections.find(currentIntersectionAndPath.intersection) != pathsToWalkableIntersections.end() )
        {
            //The node has already been visited, so it should be removed from the queue
            //and the next iteration of the queue processing loop should begin.
            //Removing the first node equates to removing the node under inspection,
            //as the first node in the map data structure is always the one being processed.
            intersectionsToVisit.erase(intersectionsToVisit.begin());
            continue;
        }
        
        //Store the current node and its path, which constitutes the fastest
        //path required to reach it from the start, in the pathsToWalkableIntersections
        //map. Note that this data structure also functions as a way of ensuring
        //nodes are only added once to the queue.
        
        pathsToWalkableIntersections.insert( std::make_pair(currentIntersectionAndPath.intersection,
                                                            currentIntersectionAndPath.pathToIntersection) );
        
        //Add adjacent nodes from the current intersection to the processing queue,
        //along with the total time taken to get to them.
        
        //First, connected street segments to the current intersection are obtained.
        std::vector<StreetSegmentIndex> connectedStreetSegments = 
                find_street_segments_of_intersection(currentIntersectionAndPath.intersection);
        
        //Then, the intersection reached by traveling down each street segment is
        //found, assuming one is not traveling backwards, and the total time to get there
        //from the start_intersection calculated. The path to reach these intersections
        //from the start is also determined, and lastly these intersections are added to
        //the queue for processing along with their associated travel time key value.
        for (auto it = connectedStreetSegments.begin(); it != connectedStreetSegments.end(); it++)
        {
            //If the currently-analyzed street segment is the same as the one
            //most recently taken to reach the current intersection,
            //there is no point analyzing it as it has already been traversed.
            //This condition does not apply if the current intersection is the start.
            if (currentIntersectionAndPath.intersection != start_intersection)
                if ( *(it) == currentIntersectionAndPath.pathToIntersection.back() )
                    continue;
            
            //The travel time will equal the travel time to the current node
            //plus the additional time needed to get to the next intersection
            //under consideration.
            double travelTime = intersectionsToVisit.begin()->first;
            
            //A turn penalty is applied to the travel time if the most recently
            //traveled street segment and the one under consideration belong
            //to different streets.
            //Does not apply if current intersection is the start.
            InfoStreetSegment newSegmentInfo = getInfoStreetSegment(*it);
            if (currentIntersectionAndPath.intersection != start_intersection)
            {
                InfoStreetSegment previousSegmentInfo = 
                        getInfoStreetSegment( currentIntersectionAndPath.pathToIntersection.back() );

                if ( (previousSegmentInfo.streetID) != (newSegmentInfo.streetID) )
                    travelTime += turn_penalty;
            }
            
            //The intersection reached by traveling down the considered street segment
            //is determined, and travelTime is updated with the time needed to traverse it.
            IntersectionIndex newIntersection;
            if (newSegmentInfo.to == currentIntersectionAndPath.intersection)
                newIntersection = newSegmentInfo.from;
            else
                newIntersection = newSegmentInfo.to;
            
            //Update the travel time.
            std::vector<StreetSegmentIndex> newSegment;
            newSegment.clear();
            newSegment.push_back(*it);
            
            travelTime += compute_path_walking_time(newSegment, walking_speed, turn_penalty);
            
            //Assuming the travelTime to the new intersection does not exceed the
            //walking_time_limit, the path to the new intersection is determined,
            //and the new intersection is added to the queue for processing.
            if (travelTime > walking_time_limit)
                continue;   //Don't add this intersection for processing.
            
            //The path to the new intersection to be added is initialized with the
            //path to the current intersection.
            std::vector<StreetSegmentIndex> newPath = currentIntersectionAndPath.pathToIntersection;
            
            //The street segment under consideration is added to the path so it extends
            //to the new intersection.
            newPath.push_back(*it);
            
            //Combine the determined data into the format used by the queue and insert
            //the new intersection into the queue for processing.
            QueueElement newElement(newIntersection, newPath);
            intersectionsToVisit.insert( std::make_pair(travelTime, newElement) );
        }
            
        //Now that adjacent intersections have been added for processing and the current
        //intersection and path added to pathsToWalkableIntersections, this entry can be
        //removed from the queue and the next entries processed.
        intersectionsToVisit.erase(intersectionsToVisit.begin());
    }
    
         
    //Now that all walkable intersections have been visited and the shortest paths to them
    //determined, return the data structure organizing this information.
    return pathsToWalkableIntersections;
}

//Given a set of potential starting intersections and an ending intersection, returns the 
//quickest possible driving path between start and end. Starting intersections (along with the
//path to get to them) are passed by reference in the form of an unordered_map, so that this
//function can directly follow find_intersections_within_walking_time. Also returns the start
//intersection that was found to minimize travel time.
std::pair<IntersectionIndex, std::vector<StreetSegmentIndex>>
        find_quickest_path_many_points_to_start(
            std::map<IntersectionIndex, std::vector<StreetSegmentIndex>>& startingSet,
            const double turn_penalty,
            const IntersectionIndex end_intersection)
{
    ////////////////////////////////////////////////////////////////////////////////
    //A note about how this algorithm works:
    //Rather than construct paths from every start point to the end and then compare path
    //travel times, this algorithm starts from the end_intersection and works its way
    //backwards in closest-first order (based off of driving travel time required to reach a 
    //node from the end_intersection). As soon as a node in the set of starting intersections
    //is reached, the algorithm can stop as this is guaranteed to be the nearest start point
    //(nodes were visited closest-first). The backwards-path generated can then be reversed
    //to give the quickest possible path between an element of the starting set and the end.
    
    //map to keep track of visited nodes.
    std::map<IntersectionIndex, bool> alreadyVisited;
    alreadyVisited.clear();
    
    //Binary tree to keep track of nodes to visit (in closest-first order) as well
    //as the shortest paths to get to these nodes. The key of each node is the driving time
    //required to get to it from the end_intersection. It starts with just the end_intersection,
    //and is built up as nodes are marked for exploration.
    std::multimap<const double, QueueElement> intersectionsToVisit;
    intersectionsToVisit.clear();
    
    //The ending node (intersection) is added for processing.
    std::vector<StreetSegmentIndex> blankPath;
    blankPath.clear();
    QueueElement endingNode(end_intersection, blankPath);
    intersectionsToVisit.insert( std::make_pair(0, endingNode) );
    
    //The main processing loop of the algorithm. So long as there are entries
    //within intersectionsToVisit, this loop keeps running. As nodes are processed, 
    //they are removed from this map. This loop is exited when an element of the
    //starting_intersections set is found.
    while (!intersectionsToVisit.empty())
    {   
        //Get the node in the queue that is closest to the ending intersection
        //in terms of travel time. Note that the queue, in this case a map,
        //is sorted according to travel time from the end_intersection.
        //Hence, the closest node is always the first element in the queue.
        QueueElement currentIntersectionAndPath = intersectionsToVisit.begin()->second;
        
        //Check if the current node has already been visited.
        if ( alreadyVisited.find(currentIntersectionAndPath.intersection) != alreadyVisited.end() )
        {
            //The node has already been visited, so it should be removed from the queue
            //and the next iteration of the queue processing loop should begin.
            //Removing the first node equates to removing the node under inspection,
            //as the first node in the map data structure is always the one being processed.
            intersectionsToVisit.erase(intersectionsToVisit.begin());
            continue;
        }
        
        //See if the node under inspection is one of the possible start points.
        //map.find(keyValue) will not equal map.end() only if the key specified
        //by keyValue exists in the set.
        if ( startingSet.find(currentIntersectionAndPath.intersection) != startingSet.end() )
        {
            //At this point, we know that we have reached a possible start point.
            //The path taken from end to this node is the path we need to return, 
            //in reverse order.
            std::pair<IntersectionIndex, std::vector<StreetSegmentIndex>>
                    shortestStartAndPath;
            shortestStartAndPath.first = currentIntersectionAndPath.intersection;
            shortestStartAndPath.second = currentIntersectionAndPath.pathToIntersection;
            reverseIntVector(shortestStartAndPath.second);
            
            //Return the shortest path between any start and the end, along with the
            //determined start that minimizes this path.
            return shortestStartAndPath;
        }
        
        //Mark the current node as visited.
        alreadyVisited.insert( std::make_pair(currentIntersectionAndPath.intersection, true) );
        
        //Add adjacent nodes from the current intersection to the processing queue,
        //along with the total driving time taken to get to them.
        
        //First, connected street segments to the current intersection are obtained.
        std::vector<StreetSegmentIndex> connectedStreetSegments = 
                find_street_segments_of_intersection(currentIntersectionAndPath.intersection);
        
        //Then, the intersection reached by traveling down each street segment is
        //found, assuming one is not traveling backwards or the wrong way down one-way streets
        //(since the path is being made in a backwards order, the condition for one-way streets 
        //is flipped). The total time to get to the node from the end_intersection is calculated. 
        //The path to reach these intersections from the end is also determined, and lastly 
        //these intersections are added to the queue for processing along with their associated 
        //travel time key value.
        for (auto it = connectedStreetSegments.begin(); it != connectedStreetSegments.end(); it++)
        {
            //If the currently-analyzed street segment is the same as the one
            //most recently taken to reach the current intersection,
            //there is no point analyzing it as it has already been traversed.
            //This condition does not apply if the current intersection is the end.
            if (currentIntersectionAndPath.intersection != end_intersection)
                if ( *(it) == currentIntersectionAndPath.pathToIntersection.back() )
                    continue;
            
            //If it is not possible to travel down the street segment under consideration
            //because of one-way considerations, it need not be analyzed.
            InfoStreetSegment newSegmentInfo = getInfoStreetSegment(*it);
            if (newSegmentInfo.oneWay && (currentIntersectionAndPath.intersection == newSegmentInfo.from) )
                continue;
            
            //The travel time will equal the travel time to the current node
            //plus the additional time needed to get to the next intersection
            //under consideration.
            double travelTime = intersectionsToVisit.begin()->first;
            
            //A turn penalty is applied to the travel time if the most recently
            //traveled street segment and the one under consideration belong
            //to different streets.
            //Does not apply if current intersection is the end.
            if (currentIntersectionAndPath.intersection != end_intersection)
            {
                InfoStreetSegment previousSegmentInfo = 
                    getInfoStreetSegment( (currentIntersectionAndPath.pathToIntersection.back()) );

                if ( (previousSegmentInfo.streetID) != (newSegmentInfo.streetID) )
                    travelTime += turn_penalty;
            }
            
            //The intersection reached by traveling down the considered street segment
            //is determined, and travelTime is updated with the time needed to traverse it.
            IntersectionIndex newIntersection;
            if (newSegmentInfo.to == currentIntersectionAndPath.intersection)
                newIntersection = newSegmentInfo.from;
            else
                newIntersection = newSegmentInfo.to;

            travelTime += find_street_segment_travel_time(*it);
            
            
            //The path to the new intersection to be added is initialized with the
            //path to the current intersection.
            std::vector<StreetSegmentIndex> newPath = currentIntersectionAndPath.pathToIntersection;
            
            //The street segment under consideration is added to the path so it extends
            //to the new intersection.
            newPath.push_back(*it);
            
            //Combine the determined data into the format used by the queue and insert
            //the new intersection into the queue for processing.
            QueueElement newElement(newIntersection, newPath);
            intersectionsToVisit.insert( std::make_pair(travelTime, newElement) );
        }
            
        //Now that adjacent intersections have been added for processing and the current
        //intersection and path added to pathsToWalkableIntersections, this entry can be
        //removed from the queue and the next entries processed.
        intersectionsToVisit.erase(intersectionsToVisit.begin());
    }
    
    //At this point, no path was found from the end to any start intersection, so return
    //a value indicating this.
    std::pair<IntersectionIndex, std::vector<StreetSegmentIndex>>
            noStartNoPath;
    noStartNoPath.first = NO_CONNECTED_START_INTERSECTION;
    return noStartNoPath;
}
