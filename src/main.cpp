#include <iostream>
#include "raycasting.h"
                   
int main()
{        
    Eigen::Vector2d ray_origin {-1.5, -1.5};
    Eigen::Vector2d ray_end {1, 1.3};

    std::vector<Eigen::Vector2i> traversed_voxels;

    RayCasting2D(ray_origin, ray_end, traversed_voxels);

    for(auto voxel:traversed_voxels)
    {
        std::cout << voxel.transpose() << std::endl;
    }

    std::cout << "\nTotal number of voxels traversed: " << \
                 traversed_voxels.size() << std::endl;

    return 0;
}