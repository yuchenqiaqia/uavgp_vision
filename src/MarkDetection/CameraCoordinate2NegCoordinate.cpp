/*
 * @file	: CameraCoordinate2NegCoordinate.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"

static double distance_x = 0.695;
static double distance_y = 0;
static double distance_z = 0.15;

void CameraCoordinate2NegCoordinate( vector<VisionResult>& vision_results, const Attitude3D& attitude3d)
{
    double port_attitude_roll  = (double)attitude3d.roll;
    double port_attitude_pitch = (double)attitude3d.pitch;
    double port_attitude_yaw   = (double)attitude3d.yaw;

    for(int i=0;i<(int)vision_results.size();++i)
    {
        double cam_x = (double)vision_results[i].cameraPos3D.x;
        double cam_y = (double)vision_results[i].cameraPos3D.y;
        double cam_z = (double)vision_results[i].cameraPos3D.z;

        double uav_x = 0;
        double uav_y = 0;
        double uav_z = 0;

        uav_x=cam_z + distance_x;
        uav_y=cam_x + distance_y;
        uav_z=cam_y + distance_z;

        double sinx,siny,sinz,cosx,cosy,cosz;
        sinx=sin(port_attitude_roll); siny=sin(port_attitude_pitch); sinz=sin(port_attitude_yaw);
        cosx=cos(port_attitude_roll); cosy=cos(port_attitude_pitch); cosz=cos(port_attitude_yaw);
        double uav_neg_x,uav_neg_y,uav_neg_z;

        uav_neg_x = uav_x*cosy*cosz + uav_y*(sinx*siny*cosz - cosx*sinz) + uav_z*(cosx*siny*cosz + sinx*sinz);
        uav_neg_y = uav_x*cosy*sinz + uav_y*(sinx*siny*sinz + cosx*cosz) + uav_z*(cosx*siny*sinz - sinx*cosz);
        uav_neg_z = -uav_x*siny + uav_y*sinx*cosy + uav_z*cosx*cosy;

        vision_results[i].negPos3D.x = uav_neg_x;
        vision_results[i].negPos3D.y = uav_neg_y;
        vision_results[i].negPos3D.z = uav_neg_z;
    }
    return;
}
