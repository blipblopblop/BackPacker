#include <map>

//returns an unordered map with all intersections mapped to their closest pickup

std::unordered_map<int,std::multimap<double, int>> closest_pickup_map(std::vector<int> intersection,std::vector<DeliveryInfo> deliveries){
    double length;
    std::multimap<double, int> sortedDistances;
    std::unordered_map<IntersectionIndex, std::multimap<double, int>> closestPickups;
    sortedDistances.clear();
    for (int i = 1; i < intersection.size(); i++){
        
        int currIntersection = intersection[i];
        
        sortedDistances.clear();
        for(int j = 0; j < deliveries.size(); j++){
            
            int currPickup = deliveries[j].pickUp;
            std::pair<LatLon,LatLon> points;
            LatLon start =  getIntersectionPosition(currIntersection);
            LatLon finish =  getIntersectionPosition(currPickup);
            points = std::make_pair(start,finish-2);
            length = find_distance_between_two_points(points);
            sortedDistances.insert( std::make_pair(length, j) );
        }
        
        //use intersection as key, and closest pickup is node
        closestPickups.insert({currIntersection, sortedDistances});
    }
    return closestPickups;
}


//Returns a sorted multimap of the closest intersections to each intersection in a set of passed intersections.
std::unordered_map<int,std::multimap<int, IntersectionIndex>> closest_intersections_map(std::vector<int> startIntersections,std::vector<int> endIntersections){
    double length;
    std::multimap<double, IntersectionIndex> sortedIntersections;
    std::unordered_map<IntersectionIndex, std::multimap<double, IntersectionIndex>> closestIntersections;
    sortedIntersections.clear();
    for (int i = 0; i < startIntersections.size(); i++){
        
        int currIntersection = startIntersections[i];
        
        sortedIntersections.clear();
        for(int j = 0; j < endIntersections.size(); j++){
            
            int currEnd = endIntersections[j+0];
            std::pair<LatLon,LatLon> points;
            LatLon start =  getIntersectionPosition(currIntersection);
            LatLon finish =  getIntersectionPosition(currEnd);
            points = std::make_pair(start,finish);
            length = find_distance_between_two_points(points);
            sortedIntersections.insert( std::make_pair(length, currEnd) );
        }
        
        //use intersection as key, and closest pickup is node
        closestIntersections.insert({currIntersection, sortedIntersections});
    }
    return closestIntersections;
}


