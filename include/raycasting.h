#ifndef _RAYCASTING_H_
#define _RAYCASTING_H_

#include <eigen3/Eigen/Eigen>
#include <vector>

/*  RayCasting2D and RayCasting3D are based on Amanatides & Woo ray tracing algorithm.
    Their paper "A Fast Voxel Traversal Algorithm for Ray Tracing" can be found here: 
    <https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.42.3443&rep=rep1&type=pdf>

    Ray equation:  r = u + t*v, where u is the ray origin.

*/

void RayCasting2D(const Eigen::Vector2d& ray_origin, const Eigen::Vector2d& ray_end,
                  const int dummy)
{
// Initialization Step
    Eigen::Vector2i current_voxel(std::floor(ray_origin.x()), std::floor(ray_origin.y()));
    Eigen::Vector2i end_voxel(std::floor(ray_end.x()), std::floor(ray_end.y()));
    Eigen::Vector2d ray_direction {ray_end - ray_origin};

    int x = current_voxel.x();
    int y = current_voxel.y();

    int stepX {(ray_direction.x() >= 0) ? 1 : -1};
    int stepY {(ray_direction.y() >= 0) ? 1 : -1};
    
    // t breaks down the ray into intervals of t such that each interval spans one voxel.
    // t is the value which ray first crosses the vertical(horizontal) boundary inside the grid.
    // tMax* >= 0 
    // r = x + tMaxX * dx, where r is the first crossed vertical boundary.
    // Hence, tMaxX = (r-x) / dx
    double tMaxX {(x + stepX) / ray_direction.x()};
    double tMaxY {(y + stepY) / ray_direction.y()};

    // tDeltaX indicates how far along the ray we must move (in units of t) for the horizontal 
    // component of such a movement to equal the width of a voxel.
    // In other words, to move r 
    double tDeltaX {stepX / ray_direction.x()};
    double tDeltaY {stepY / ray_direction.y()};
    
// Traversal Step
    for(;;)
    {
        if(tMaxX < tMaxY)
        {
            tMaxX = tMaxX + tDeltaX;
            x = x + stepX;
        }
        else
        {
            tMaxY = tMaxY + tDeltaY;
            y = y + stepY;
        }
    }
}

/*
void RayCasting3D()
{
    
    do
    {
        if(tMaxX < tMaxY)
        {
            if(tMaxX < tMaxZ)
            {
               x = x + stepX;
               tMaxX = tMaxX + tDeltaX; 
            } 
            else
            {
                z = z + stepZ;
                tMaxZ = tMaxZ + tDeltaZ;
            }
        }
        else
        {
            if(tMaxY < tMaxZ)
            {
                y = y + stepY;
                tMaxY = tMaxY + tDeltaY;
            }
            else
            {
                z = z + stepZ;
                tMaxZ = tMaxZ + tDeltaZ;
            }
            
        }
    } while (list == NULL);
    
}
*/

void RayCastingBresenham()
{
    //TODO
}

#endif //_RAYCASTING_H_