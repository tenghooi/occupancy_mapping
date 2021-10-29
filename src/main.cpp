#include <iostream>
#include "raycasting.h"
                   
int main()
{        
    Eigen::Vector3d ray_origin {0, 0, 0};
    Eigen::Vector3d ray_end {2.3, 1.21, 3};

    std::vector<Eigen::Vector3i> traversed_voxels;

    RayCasting3D(ray_origin, ray_end, traversed_voxels);

    for(auto voxel:traversed_voxels)
    {
        std::cout << voxel.transpose() << std::endl;
    }

    std::cout << "\nTotal number of voxels traversed: " << \
                 traversed_voxels.size() << std::endl;

    return 0;
}