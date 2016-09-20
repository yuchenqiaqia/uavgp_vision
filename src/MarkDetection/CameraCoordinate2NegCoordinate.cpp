/*
 * @file	: CameraCoordinate2NegCoordinate.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"

static double distance_x = 0.695;
static double distance_y = 0;
static double distance_z = 0.15;

void CameraCoordinate2NegCoordinate( vector<VisionResult>& vision_results, const Attitude3D& attitude3d) //从相机坐标系到飞机坐标系，再从飞机坐标系到NEG坐标系；
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
        //由相机坐标系到飞机坐标系,飞机前方为x轴正方向
        //cam.z - pix.x; cam.x - pix.y; cam.y - pix.(-z)
        uav_x=cam_z + distance_x;
        uav_y=cam_x + distance_y;
        uav_z=cam_y + distance_z;

        double sinx,siny,sinz,cosx,cosy,cosz;
        sinx=sin(port_attitude_roll); siny=sin(port_attitude_pitch); sinz=sin(port_attitude_yaw);
        cosx=cos(port_attitude_roll); cosy=cos(port_attitude_pitch); cosz=cos(port_attitude_yaw);
        double uav_neg_x,uav_neg_y,uav_neg_z;

/*
        double xt,yt,zt,_x,_y,_z;
        //飞机坐标系到NEG坐标系
        xt=uav_x;
        yt=uav_y*cosx-uav_z*sinx;
        zt=uav_y*sinx+uav_z*cosx;
        _x=xt;
        _y=yt;
        _z=zt;
        xt=_x*cosy+_z*siny;
        yt=_y;
        zt=-_x*siny+_z*cosy;
        _x=xt;
        _y=yt;
        _z=zt;
        uav_neg_x=_x*cosz-_y*sinz;
        uav_neg_y=_x*sinz+_y*cosz;
        uav_neg_z=_z;
*/

        uav_neg_x = uav_x*cosy*cosz + uav_y*(sinx*siny*cosz - cosx*sinz) + uav_z*(cosx*siny*cosz + sinx*sinz);
        uav_neg_y = uav_x*cosy*sinz + uav_y*(sinx*siny*sinz + cosx*cosz) + uav_z*(cosx*siny*sinz - sinx*cosz);
        uav_neg_z = -uav_x*siny + uav_y*sinx*cosy + uav_z*cosx*cosy;

        vision_results[i].negPos3D.x = uav_neg_x;
        vision_results[i].negPos3D.y = uav_neg_y;
        vision_results[i].negPos3D.z = uav_neg_z;

        //char transf_standard[50];
        //sprintf_s(transf_standard,"T_standard:[%0.3fm,%0.3fm,%0.3fm]",uav_standard_x,uav_standard_y,uav_standard_z);
        //Point T_stand_show_center;
        //T_stand_show_center=Point2d(2,45);
        //putText(show_color, transf_standard, T_stand_show_center,CV_FONT_HERSHEY_PLAIN,0.8,Scalar(0,0,255),1);//在图像中显示平移向量（x,y,z）

        //fprintf(fp,"pitch=%0.3f,roll=%0.3f,yaw=%0.3f;",port_attitude_pitch*180/3.14159,port_attitude_roll*180/3.14159,port_attitude_yaw*180/3.14159);
        //fprintf(fp,"uav_standard_T:(%0.3fm,%0.3fm,%0.3fm)\n",uav_standard_x,uav_standard_y,uav_standard_z);
        //fprintf(fp,"目标圆心位置，经度:%f,纬度:%f,高度:%f\n",target_jingdu,target_weidu,target_haiba);
    }
    return;
}
