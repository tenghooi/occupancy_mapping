#include "occupancy_map.h"
#include <ctime>

double OccupancyMap::Logit(const double& prob) const
{
    return log(prob / (1 - prob));
}

bool OccupancyMap::Exist(const int& indx) 
{
    return occupancy_buffer_[indx] > occupancy_threshold_logit_;
}

