#ifndef _RAYCASTING_H_
#define _RAYCASTING_H_

#include <Eigen/Eigen>
#include <vector>

void RayCasting2D()
{
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