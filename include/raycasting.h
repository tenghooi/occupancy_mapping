#ifndef _RAYCASTING_H_
#define _RAYCASTING_H_

#include <eigen3/Eigen/Eigen>
#include <vector>

void RayCasting2D(const Eigen::Vector2d& ray_origin, const Eigen::Vector2d& ray_end,
                  const int dummy)
{
    // Initialization Step
    Eigen::Vector2i current_voxel(std::floor(ray_origin.x()), std::floor(ray_origin.y()));
    Eigen::Vector2i end_voxel(std::floor(ray_end.x()), std::floor(ray_end.y()));
    Eigen::Vector2d ray_direction {ray_end - ray_origin};

    int stepX {(ray_direction.x() >= 0) ? 1 : -1};
    int stepY {(ray_direction.y() >= 0) ? 1 : -1};
    
    double tMaxX;
    double tMaxY;

    double tDeltaX;
    double tDeltaY;
    
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

void RayCastingBresenham()
{
    //TODO
}

#endif //_RAYCASTING_H_