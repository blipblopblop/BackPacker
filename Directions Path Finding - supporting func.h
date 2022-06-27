//when no edge exists
#define NO_EDGE -1 

//Helper functions and classes for find_path_with_walk_to_pickup.

//The class used for queue elements in the "closest-first" processing algorithm.
class QueueElement
{
public:
    IntersectionIndex intersection;
    std::vector<StreetSegmentIndex> pathToIntersection;
    double pathTravelTime;
    
    QueueElement(IntersectionIndex interIndex, std::vector<StreetSegmentIndex> path)
    {
        intersection = interIndex;
        pathToIntersection = path;
        pathTravelTime = 0;
    }
    
    QueueElement(IntersectionIndex interIndex, std::vector<StreetSegmentIndex> path,
                    double travelTime)
    {
        intersection = interIndex;
        pathToIntersection = path;
        pathTravelTime = travelTime;
    }
};

//Reverses a vector of integers.
void reverseIntVector(std::vector<int>& vector);

//Returns an unordered_map hashed by intersection indices and containing the
//shortest paths to those intersection indices by walking from the start_intersection.
std::map<IntersectionIndex, std::vector<StreetSegmentIndex>> 
        find_intersections_within_walking_time(
                            const IntersectionIndex start_intersection,
                            const double turn_penalty,
                            const double walking_speed, 
                            const double walking_time_limit);

//Given a set of potential starting intersections and an ending intersection, returns the 
//quickest possible driving path between start and end. Starting intersections (along with the
//path to get to them) are passed by reference in the form of an unordered_map, so that this
//function can directly follow find_intersections_within_walking_time.
std::pair<IntersectionIndex, std::vector<StreetSegmentIndex>>
    find_quickest_path_many_points_to_start(
        std::map<IntersectionIndex, std::vector<StreetSegmentIndex>>& startingSet,
        const double turn_penalty,
        const IntersectionIndex end_intersection);


#endif /*M3_HELPER_FUNCTIONS_H*/


