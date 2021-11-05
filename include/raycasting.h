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
                  std::vector<Eigen::Vector2d>& traversed_voxels);


void RayCasting3D(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_end,
                  std::vector<Eigen::Vector3d>& traversed_voxels);


/*void RayCastingBresenham()
{
    //TODO
}
*/
#endif //_RAYCASTING_H_