#ifndef _RAYCASTING_H_
#define _RAYCASTING_H_

#include <Eigen/Eigen>
#include <vector>
#include <cmath>

/*  
    RayCasting2D and RayCasting3D are based on Amanatides & Woo ray tracing algorithm.
    Their paper "A Fast Voxel Traversal Algorithm for Ray Tracing" can be found here: 
    <https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.42.3443&rep=rep1&type=pdf>

    This algorithm uses a parameterized line equation for ray tracing.
    Ray equation:  r = r0 + t*v, where r0 is the ray origin, v is difference between ray end and ray origin.
    
    E.g: X0 = [0.5 0.5], X1 = [2.5 0]
         Then, v = [2 -0.5] 
         r = [0.5 0.5] + t * [2 -0.5]
*/

void RayCasting2D(const Eigen::Vector2d& ray_origin, const Eigen::Vector2d& ray_end,
                  std::vector<Eigen::Vector2i>& traversed_voxels)
{
// Initialization Step
    
    Eigen::Vector2i current_voxel(std::floor(ray_origin.x()), std::floor(ray_origin.y()));
    Eigen::Vector2i end_voxel(std::floor(ray_end.x()), std::floor(ray_end.y()));
    Eigen::Vector2d ray_direction = ray_end - ray_origin;

    double dx = ray_direction.x();
    double dy = ray_direction.y();

    // step* indicates wheter X or Y are incremented or decremented as the ray crosses voxel boundaries.
    int stepX = (dx == 0) ? 0 : (dx > 0) ? 1 : -1;
    int stepY = (dy == 0) ? 0 : (dy > 0) ? 1 : -1;
    
    // t breaks down the ray into intervals of t such that each interval spans one voxel.
    // t is the value which ray first crosses the vertical(horizontal) boundary inside the grid.
    // t is incremental from initial t where ray crossed the first boundary to 1 where 
    // it reaches ray end.
    //
    // tMax* >= 0 and tMax* <= 1. When tMax* > 1, it exceeds the ray length.
    // r = x0 + tMaxX * dx, where r is the x of the first crossed vertical boundary.
    // Hence, tMaxX = (r - x0) / dx       OR
    //        tMaxX = (current_voxel_x + voxel_width - x0) / dx for positive direction.
    // If dx = 0, that means ray will never travel horizontally. Set tMaxX to be large number(100).
    
    double tMaxX = (dx > 0) ? (current_voxel.x() + stepX - ray_origin.x()) / dx : \
                   (dx < 0) ? (current_voxel.x() - ray_origin.x()) / dx : 100;
    double tMaxY = (dy > 0) ? (current_voxel.y() + stepY - ray_origin.y()) / dy : \
                   (dy < 0) ? (current_voxel.y() - ray_origin.y()) / dy : 100;
    
    // tDeltaX indicates how far along the ray we must move (in units of t) for the horizontal 
    // component of such a movement to equal the width of a voxel.
    // In other words, divide the width of a voxel by dX or dY of the ray.
    // E.g. Voxel size = 1; dX = 5.0;
    //      Then, tDeltaX = 1 / 5;
    //
    // tDelta* must be positive like tMax*, i.e. tDelta* >= 0 and <= 1.

    double tDeltaX = (dx != 0) ? stepX / dx : 100;
    double tDeltaY = (dy != 0) ? stepY / dy : 100;
    
    // Add the ray origin to the traversed_voxels vector.
    traversed_voxels.push_back(current_voxel);

// Traversal Step
    while(current_voxel != end_voxel)
    {
        if(tMaxX < tMaxY)
        {
            tMaxX = tMaxX + tDeltaX;
            current_voxel[0] = current_voxel[0] + stepX;
        }
        else
        {
            tMaxY = tMaxY + tDeltaY;
            current_voxel[1] = current_voxel[1] + stepY;
        }

        traversed_voxels.push_back(current_voxel);
    }

    return;
}


void RayCasting3D(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_end,
                  std::vector<Eigen::Vector3i>& traversed_voxels)
{
// Initialization Step

    Eigen::Vector3i current_voxel(std::floor(ray_origin.x()),
                                  std::floor(ray_origin.y()),
                                  std::floor(ray_origin.z()));
    Eigen::Vector3i end_voxel(std::floor(ray_end.x()),
                              std::floor(ray_end.y()),
                              std::floor(ray_end.z()));
    Eigen::Vector3d ray_direction = ray_end - ray_origin;

    double dx = ray_direction.x();
    double dy = ray_direction.y();
    double dz = ray_direction.z();

    int stepX = (dx == 0) ? 0 : (dx > 0) ? 1 : -1;
    int stepY = (dy == 0) ? 0 : (dy > 0) ? 1 : -1;
    int stepZ = (dz == 0) ? 0 : (dz > 0) ? 1 : -1;

    double tMaxX = (dx > 0) ? (current_voxel.x() + stepX - ray_origin.x()) / dx : \
                   (dx < 0) ? (current_voxel.x() - ray_origin.x()) / dx : 100;
    double tMaxY = (dy > 0) ? (current_voxel.y() + stepY - ray_origin.y()) / dy : \
                   (dy < 0) ? (current_voxel.y() - ray_origin.y()) / dy : 100;
    double tMaxZ = (dz > 0) ? (current_voxel.z() + stepZ - ray_origin.z()) / dz : \
                   (dz < 0) ? (current_voxel.z() - ray_origin.z()) / dz : 100;

    double tDeltaX = (dx != 0) ? stepX / dx : 100;
    double tDeltaY = (dy != 0) ? stepY / dy : 100;
    double tDeltaZ = (dz != 0) ? stepZ / dz : 100;

    traversed_voxels.push_back(current_voxel);

// Traversal Step
    while(current_voxel != end_voxel)
    {
        if(tMaxX < tMaxY)
        {
            if(tMaxX < tMaxZ)
            {
               tMaxX = tMaxX + tDeltaX;
               current_voxel[0] = current_voxel[0] + stepX; 
            } 
            else
            {
                tMaxZ = tMaxZ + tDeltaZ;
                current_voxel[2] = current_voxel[2] + stepZ;
            }
        }
        else
        {
            if(tMaxY < tMaxZ)
            {
                tMaxY = tMaxY + tDeltaY;
                current_voxel[1] = current_voxel[1] + stepY;
            }
            else
            {
                tMaxZ = tMaxZ + tDeltaZ;
                current_voxel[2] = current_voxel[2] + stepZ;
            }
        }

        traversed_voxels.push_back(current_voxel);
    }
    
    return;
}


void RayCastingBresenham()
{
    //TODO
}

#endif //_RAYCASTING_H_