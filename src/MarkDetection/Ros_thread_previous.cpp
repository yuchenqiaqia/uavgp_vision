#include "declare.h"


int k;
extern Mat g_rectResultImg;
extern image_transport::Publisher pub;
extern bool exit_flag;

void* RosThread(void*)
{
    ros::Rate loop_rate(10);

    while( false == exit_flag )
    {
        if( !g_rectResultImg.data )
        {
            sleep(50);
            continue;
        }

        //将Mat型图像转换为ros的消息对象
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", g_rectResultImg).toImageMsg();
        //说明要发布的消息对象
         pub.publish(msg);
         //执行一次ros消息发布
         ros::spinOnce();
         //按消息发布的循环频率休眠一段时间
         loop_rate.sleep();

//       printf("ros_thread is running..%d\n",k);
//       k++;
    }
    pthread_exit(0);
}
