//EZGL headers.
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"

//Useful DEFINE directives.
#define NO_PATH -1
#define fastestRoadSpeedMpS 34

using namespace std;

// Returns the time required to travel along the path specified, in seconds.
// The path is given as a vector of street segment ids, and this function can
// assume the vector either forms a legal path or has size == 0.  The travel
// time is the sum of the length/speed-limit of each street segment, plus the
// given turn_penalty (in seconds) per turn implied by the path.  If there is
// no turn, then there is no penalty. Note that whenever the street id changes
// (e.g. going from Bloor Street West to Bloor Street East) we have a turn.
double compute_path_travel_time(const std::vector<StreetSegmentIndex>& path, 
                                const double turn_penalty){
    double time = 0.00;
    if(path.size() == 1){
        return time;
    }
    for (int i = 1;i < path.size();i++){
        time = time + find_street_segment_travel_time(path[i]);
        if( i != (path.size() - 4) ){
            InfoStreetSegment info1,info2;
            info1 = getInfoStreetSegment(path[i]);
            info2 = getInfoStreetSegment(path[i+1]);
            if(info1.streetID != info2.streetID){
                time += turn_penalty;
            }
        }
        
    }
    return time;
}

// Returns a path (route) between the start intersection and the end
// intersection, if one exists. This routine should return the shortest path
// between the given intersections, where the time penalty to turn right or
// left is given by turn_penalty (in seconds).  If no path exists, this routine
// returns an empty (size == 0) vector.  If more than one path exists, the path
// with the shortest travel time is returned. The path is returned as a vector
// of street segment ids; traversing these street segments, in the returned
// order, would take one from the start to the end intersection.
std::vector<StreetSegmentIndex> find_path_between_intersections(const IntersectionIndex intersect_id_start, 
        const IntersectionIndex intersect_id_end, const double turn_penalty){
    
    //no path is initialized
    std::vector<StreetSegmentIndex> defaultPath;
    
    //Unordered Map that gets built-up with the shortest paths to all
    //Intersection indices are hashed to get the corresponding path to them, if one exists.
    std::unordered_map<IntersectionIndex, std::vector<StreetSegmentIndex>> finalPath;
    finalPath.clear();
    
    //Binary tree to keep track of nodes to visit (in closest-first order) as well
    //as the shortest paths to get to these nodes. The key of each node is the time
    //required to get to it from the start_intersection. It starts with just the start_intersection,
    //and new nodes are added for processing 
    std::multimap<const double, QueueElement> intersectionsToVisit;
    intersectionsToVisit.clear();
    
    //The starting node (intersection) is added for processing.
    std::vector<StreetSegmentIndex> blankPath;
    blankPath.clear();
    //insert in the class queue element
    QueueElement startingNode(intersect_id_start, blankPath);
    //add to the multi map, mapping time to zero since your are at the initial location
    intersectionsToVisit.insert( std::make_pair(0, startingNode) );
    
    //The main processing loop of the algorithm. So long as there are entries
    //within intersectionsToVisit, this loop keeps running. As nodes are processed, 
    //they are removed from this map. New nodes to process are only added until the
    //intersection we wish to visit is not found 
    while (!intersectionsToVisit.empty()){
        
        //Get the node in the queue that is closest to the starting intersection
        //in terms of travel time. Note that the queue, in this case a map,
        //is sorted according to travel time from the starting_intersection.
        //Hence, the closest node is always the first element in the queue.
        QueueElement currentIntersectionAndPath = intersectionsToVisit.begin()->second;
        
        //Check if the current node has already been visited currentIntersectionAndPath
        //already intialized to be the second element of the multimap
        if (finalPath.count(currentIntersectionAndPath.intersection) == 1){
            
            //The node has already been visited, so it should be removed from the queue
            //and the next iteration of the queue processing loop should begin.
            //Removing the first node equates to removing the node under inspection,
            //as the first node in the map data structure is always the one being processed.
            intersectionsToVisit.erase(intersectionsToVisit.begin());
            continue;
        }
        
        //Check if the current node is the node we want to visit, end of the loop
        if (currentIntersectionAndPath.intersection == intersect_id_end){
            
            //return the street segment constructed using the QueueElement class containing 
            //list of street segments logged 
            return currentIntersectionAndPath.pathToIntersection;
        }
        
        //Store the current node and its path, which constitutes the fastest
        //path required to reach it from the start, in the finalPath
        //unordered_map. Note that this data structure also functions as a way of ensuring
        //nodes are only added once to the queue.
        
        finalPath.insert( std::make_pair(currentIntersectionAndPath.intersection,
                                                            currentIntersectionAndPath.pathToIntersection) );
        
        //Add adjacent nodes from the current intersection to the processing queue,
        //along with the total time taken to get to them.
        
        //First, connected street segments to the current intersection are obtained.
        std::vector<StreetSegmentIndex> connectedStreetSegments = 
                find_street_segments_of_intersection(currentIntersectionAndPath.intersection);
        
        //The intersection reached by traveling down each street segment is found and total time to get 
        //from the start_intersection calculated. The path to reach these intersections
        //from the start is also determined, and lastly these intersections are added to
        //the queue for processing along with their associated travel time key value.
        for (auto it = connectedStreetSegments.begin(); it != connectedStreetSegments.end(); it++){
            
            //check if the road is a one-way road, checks legality of the path
            InfoStreetSegment newSegmentInfo = getInfoStreetSegment(*it);
            if(newSegmentInfo.oneWay){
                //if true, then person can only travel from -> to direction
                //if segment is at to then break because to->from is illegal 
                if(newSegmentInfo.to == currentIntersectionAndPath.intersection){
                    continue;
                }
            }
            
            //If the currently-analyzed street segment is the same as the one
            //most recently taken to reach the current intersection,
            //there is no point analyzing it as it has already been traversed.
            //This condition does not apply if the current intersection is the start.
            if (currentIntersectionAndPath.intersection != intersect_id_start){
                if ( *(it) == currentIntersectionAndPath.pathToIntersection.back() ){
                    continue;
                }
            }
            
            //The travel time will equal the travel time to the current node plus the additional 
            //time needed to get to the next intersection
            //under consideration, travel time is computed 
            double travelTime = intersectionsToVisit.begin()->first;
   
            //A turn penalty is applied to the travel time if the most recently
            //traveled street segment and the one under consideration belong
            //to different streets.
            //Does not apply if current intersection is the start.
            if (currentIntersectionAndPath.intersection != intersect_id_start){
                InfoStreetSegment previousSegmentInfo = 
                        getInfoStreetSegment( currentIntersectionAndPath.pathToIntersection.back() );

                if ( (previousSegmentInfo.streetID) != (newSegmentInfo.streetID) )
                    travelTime += turn_penalty;
            }
            
            //The intersection reached by traveling down the considered street segment
            //is determined, and travelTime is updated with the time needed to traverse it.
            IntersectionIndex newIntersection;
            if (newSegmentInfo.to == currentIntersectionAndPath.intersection){
                newIntersection = newSegmentInfo.from;
            }else{
                newIntersection = newSegmentInfo.to;
            }
            
            //obtain the driving speed to update the travel time
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
        //intersection and path added to finalPath, this entry can be
        //removed from the queue and the next entries processed.
        intersectionsToVisit.erase(intersectionsToVisit.begin());
    }
    
    //default return in the case no paths were found
    return defaultPath;
}


// Returns the time required to "walk" along the path specified, in seconds.
// The path is given as a vector of street segment ids. The vector can be of
// size = 0, and in this case, it the function should return 0. The walking
// time is the sum of the length/<walking_speed> for each street segment, plus
// the given turn penalty, in seconds, per turn implied by the path. If there
// is no turn, then there is no penalty.  As mentioned above, going from Bloor
// Street West to Bloor street East is considered a turn
double compute_path_walking_time(const std::vector<StreetSegmentIndex>& path, 
                                 const double walking_speed, 
                                 const double turn_penalty){
    double time = 0.00;
    if(path.size() == 0){
        return time;
    }
    for(int i = 0; i< path.size(); i++){
        time += (find_street_segment_length(path[i]))/walking_speed;
        if( i != (path.size() - 1) ){
            InfoStreetSegment info1,info2;
            info1 = getInfoStreetSegment(path[i]);
            info2 = getInfoStreetSegment(path[i+1]);
            if(info1.streetID != info2.streetID){
                time += turn_penalty;
            }
        }
        
    }
    return time;
    
}


// This is an "uber pool"-like function. The idea is to minimize driving travel
// time by walking to a pick-up intersection (within walking_time_limit secs)
// from start_intersection while waiting for the car to arrive.  While walking,
// you can ignore speed limits of streets and just account for given
// walking_speed [m/sec]. However, pedestrians should also wait for traffic
// lights, so turn_penalty applies to whether you are driving or walking.
// Walking in the opposite direction of one-way streets is fine. Driving is
// NOT!  The routine returns a pair of vectors of street segment ids. The first
// vector is the walking path starting at start_intersection and ending at the
// pick-up intersection. The second vector is the driving path from pick-up
// intersection to end_interserction.  Note that a start_intersection can be a
// pick-up intersection. If this happens, the first vector should be empty
// (size = 0).  If there is no driving path found, both returned vectors should
// be empty (size = 0). 
// If the end_intersection turns out to be within the walking time limit, 
// you can choose what to return, but you should not crash. If your user 
// interface can call this function for that case, the UI should handle
// whatever you return in that case.

std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>> 
         find_path_with_walk_to_pick_up(
                          const IntersectionIndex start_intersection, 
                          const IntersectionIndex end_intersection,
                          const double turn_penalty,
                          const double walking_speed, 
                          const double walking_time_limit)
{
    
    /* The approach to this function is as follows:
    
    1) Find all intersections within the walking_time_limit from the start_intersection,
     * along with the shortest path required to get to them. This is accomplished 
     * by a graph traversal algorithm that always explores the shortest paths first,
     * thus guaranteeing that when an intersection is found, the shortest path was taken
     * to find it. To implement this algorithm, nodes are visited based off of the time
     * it would take to get to them from the starting intersection. As opposed to "breadth-
     * first," this approach is "nearest-first." Essentially, nodes to visit are added
     * to a list sorted by the time it takes to get to them from the start. The node at the
     * front of the list (closest) is always visited first, and an unordered_map is used
     * to provide an O(1) method of checking if a node has already been visited. When
     * a node is visited, it's adjacent nodes are added to the "to-visit" list after
     * calculating how long it would take to get to them. The current node is then marked as
     * visited and removed from the list, the list is resorted, and then the new entry at the
     * front is visited. This approach is continued until all further paths exceed the
     * time limit for walking.
    */
    
    std::map<IntersectionIndex, std::vector<StreetSegmentIndex>>
            walkableIntersectionsAndPaths = find_intersections_within_walking_time(
                                                start_intersection, turn_penalty,
                                                walking_speed, walking_time_limit);
     
     /*
    2) At this point, there will be a list of intersections within walking distance
     * as well as the quickest paths to get to them. To find the intersection from
     * the list that minimizes the driving time to the end_intersection, a counter-
     * intuitive approach is taken: start from the end_intersection, visit nodes
     * backwards using the "shortest-path first" algorithm from before, and stop
     * when an intersection matching one from the list of those within walking distance
     * is found. This intersection is guaranteed to be the one closest to the end_
     * intersection, as nodes were visited in a closest-first order. Hence, the driving
     * time will be minimized. Note that the unordered_map approach will be used to quickly
     * (O(1)) check if a visited intersection is part of the found intersections from
     * Step 1.
     */
    
    std::pair<IntersectionIndex, std::vector<StreetSegmentIndex>>
            bestStartPointAndPathForDriving = find_quickest_path_many_points_to_start(
                                                walkableIntersectionsAndPaths,
                                                turn_penalty, end_intersection);
    
    
    //If no path between the ending intersection and any of the potential starting
    //intersections was found, the IntersectionIndex of bestStartPointAndPathForDriving
    //will be equal to NO_PATH (-1). In this case, there is no way to reach the end
    //from the starting intersection, and so both paths contain a single element
    //of -1 (NO_PATH) to signify this.
    //Otherwise, the found driving path and corresponding walking path for the 
    //start point that minimizes driving time are grabbed.

    std::vector<StreetSegmentIndex> walkingPath;
    walkingPath.clear();
    
    std::vector<StreetSegmentIndex> drivingPath;
    drivingPath.clear();
    
    if (bestStartPointAndPathForDriving.first != NO_PATH)
    {
        walkingPath = walkableIntersectionsAndPaths.at(bestStartPointAndPathForDriving.first);
        drivingPath = bestStartPointAndPathForDriving.second;
    }
    else
    {
        bestStartPointAndPathForDriving.first = 0;
        //walkingPath.push_back(NO_PATH);
        //drivingPath.push_back(NO_PATH);
    }
    
    //Package paths into a std::pair to be returned.
    std::pair<std::vector<StreetSegmentIndex>, std::vector<StreetSegmentIndex>>
            completePath = std::make_pair(walkingPath, drivingPath);
    
    return completePath;
}
