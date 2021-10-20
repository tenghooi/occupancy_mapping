#ifndef _RAYCASTING_H_
#define _RAYCASTING_H_

#include <Eigen/Eigen>
#include <vector>

void RayCasting()
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
                y = y + strpY;
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

#endif //_RAYCASTING_H_