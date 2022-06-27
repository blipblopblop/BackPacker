#include <omp.h>


std::vector<CourierSubpath> traveling_courier(
		            const std::vector<DeliveryInfo>& deliveries,
	       	        const std::vector<int>& depots, 
		            const float turn_penalty, 
		            const float truck_capacity){
    
    
    //std::vector<int> pickup_intersections;
    for(int i = 0; i < deliveries.size();i++){
        pickup_intersections.push_back(deliveries[i].pickUp);
    }
     
    std::vector<int> drop_off_intersections;
    for(int i = 0; i < deliveries.size();i++){
        drop_off_intersections.push_back(deliveries[i].dropOff);
    }
    //returns closest pickup to any other pickup
    //std::unordered_map<int,std::map<double, int>> closestPickupstoPickUps= closest_pickup_map(pickup_intersections, deliveries);
    
    //returns closest pickup to any other dropoff point
    std::unordered_map<int,std::multimap<double, int>> closestPickupstoDropoffs= closest_pickup_map(drop_off_intersections, deliveries);
    
    //return closest depot to pickups
    std::unordered_map<int,std::multimap<double, int>> closestPickupstoDepots= closest_pickup_map(depots, deliveries);
    //return closest depot to final drop off
    std::unordered_map<int,std::multimap<double, int>> closestDepotstoDropoffs= closest_intersections_map(drop_off_intersections,depots);
    
    std::unordered_map<IntersectionIndex, bool> completedDeliveryIndices;
    completedDeliveryIndices.clear();
    
    
    //copy truck capacity in order to work with value
    float truckCapacity = truck_capacity;
    //path with final answer
    //vector of a structs 
    std::vector<CourierSubpath> finalPath;
    //resize finalPath
    finalPath.clear();
    
    //how many deliveries have been made to iterate through the while loop
    int deliveryCount = 0;
    
    //The delivery index for the current delivery being completed.
    int closestDeliveryIndex;
    int oldClosestDeliveryIndex = 0;
    
    std::vector<std::vector<StreetSegmentIndex>> deliveryStartEndPath;
    deliveryStartEndPath.clear();
    deliveryStartEndPath.resize(deliveries.size());
    
    
    //Paths for deliveries are computed using multi-threading.
    
    #pragma omp parallel
    {
        #pragma omp parallel for
        for (int i = 0; i < deliveries.size() / 3; i++)
        {
            deliveryStartEndPath[i] = find_path_between_intersections(deliveries[i].pickUp, deliveries[i].dropOff, turn_penalty);
        }
        
        
        #pragma omp parallel for
        for (int j = deliveries.size() / 3; j < deliveries.size() * 2 / 3; j++)
        {
            deliveryStartEndPath[j] = find_path_between_intersections(deliveries[j].pickUp, deliveries[j].dropOff, turn_penalty);
        }
        
        #pragma omp parallel for
        for (int k = deliveries.size() * 2 / 3; k < deliveries.size(); k++)
        {
            deliveryStartEndPath[k] = find_path_between_intersections(deliveries[k].pickUp, deliveries[k].dropOff, turn_penalty);
        }
        
    }
     
    
    #pragma omp parallel for
    for (int i = 0; i < deliveries.size(); i++)
    {
        deliveryStartEndPath[i] = find_path_between_intersections(deliveries[i].pickUp, deliveries[i].dropOff, turn_penalty);
    }
    
   
    
    while(deliveryCount < deliveries.size()){
        
        //depending on whether a legal pickup or drop-off is closer, do either one
          
        //make delivery if corresponding pickup index has been visited
        
        
        //Go from the start depot, assumed to be depots[0], to the closest pickup
        //point for a package if this is the first iteration of the loop.
        if(deliveryCount == 0){
            CourierSubpath initialPath;
            //access unordered map to get closest pick up to first depot
            closestDeliveryIndex = closestPickupstoDepots[depots[0]].begin()->second;
            initialPath.start_intersection = depots[0];
            initialPath.end_intersection = deliveries[closestDeliveryIndex].pickUp;
            initialPath.subpath = find_path_between_intersections(
            initialPath.start_intersection, initialPath.end_intersection, turn_penalty);
            finalPath.push_back(initialPath);
        }
      
        //Assume you are starting at the pick-up point for a package.
        //Make the delivery.
        CourierSubpath deliveryPath;
        deliveryPath.start_intersection = deliveries[closestDeliveryIndex].pickUp;
        deliveryPath.end_intersection = deliveries[closestDeliveryIndex].dropOff;
        deliveryPath.pickUp_indices.clear();
        deliveryPath.pickUp_indices.push_back(closestDeliveryIndex);
        deliveryPath.subpath = deliveryStartEndPath[closestDeliveryIndex];
        finalPath.push_back(deliveryPath);
        
        completedDeliveryIndices[closestDeliveryIndex] = true;
        
        //Update truck capacity to reflect drop-off
        truckCapacity += deliveries[deliveryCount].itemWeight;
        
        
        //else a  drop off has been made so we can potential pickup more stuff
        // if truck capacity permits
        
        //Go to the closest pickup point for the next package.
        if (deliveryCount != (deliveries.size() - 1) && (truckCapacity >= deliveries[deliveryCount].itemWeight))
        {
            //Need to determine the delivery index of the next delivery to make.
            //Choose the delivery who's pick-up is closest.
            std::multimap<double, int> nearbyDeliveryIndices = closestPickupstoDropoffs[deliveries[closestDeliveryIndex].dropOff];
            for (auto it = nearbyDeliveryIndices.begin(); it != nearbyDeliveryIndices.end(); it++)
            {
                //Check the closest delivery index to see if it has not been delivered yet.
                //If this is the case, that is the next delivery to make.
                if (completedDeliveryIndices[it->second] == false)
                {
                    //The index of the new closest delivery to make.
                    oldClosestDeliveryIndex = closestDeliveryIndex;
                    closestDeliveryIndex = it->second;
                    break;
                } 
            }
            
            CourierSubpath pathToNextPickUp;
            pathToNextPickUp.start_intersection = deliveries[oldClosestDeliveryIndex].dropOff;
            pathToNextPickUp.end_intersection = deliveries[closestDeliveryIndex].pickUp;
            pathToNextPickUp.subpath = find_path_between_intersections(
            pathToNextPickUp.start_intersection, pathToNextPickUp.end_intersection, turn_penalty);
            finalPath.push_back(pathToNextPickUp);
            //Update truck capacity to reflect new pick-up
            truckCapacity -= deliveries[deliveryCount].itemWeight;
        }
        
        else if (deliveryCount == (deliveries.size() - 1)){
            CourierSubpath pathToEndDepot;
            pathToEndDepot.start_intersection = deliveries[closestDeliveryIndex].dropOff;
            
            
            int closestDepot = closestDepotstoDropoffs[pathToEndDepot.start_intersection].begin()->second;
            pathToEndDepot.end_intersection = closestDepot;
            pathToEndDepot.subpath = find_path_between_intersections(
            pathToEndDepot.start_intersection, pathToEndDepot.end_intersection, turn_penalty);
            finalPath.push_back(pathToEndDepot);
    
            
        }
        
        
        //Update delivery count to reflect drop off that was made
        deliveryCount += 1;
        
      
    }
    //all deliveries were finished inside the while loop
    
    CourierSubpath pathToEndDepot;
    pathToEndDepot.start_intersection = finalPath[deliveryCount].end_intersection;
    int closestDepot = closestDepotstoPickups[pathToEndDepot.start_intersection];
    pathToEndDepot.end_intersection = closestDepot;
    pathToEndDepot.subpath = find_path_between_intersections(
    pathToEndDepot.start_intersection, pathToEndDepot.end_intersection, turn_penalty);
    finalPath.push_back(pathToEndDepot);
    
    
    return finalPath;
}
