#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry> // Eigen 几何模块
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

// 顶视
// -0.0121,   0.0362,    -1.6077  -- yaw pitch roll = 87.8628   179.158  178.231
// 侧视
// 0,         2.2214,    -2.2214  -- yaw pitch roll = 0.002376  179.998  90
// 顶视
// 0.01089,   0.06490,   -0.00386 -- yaw pitch roll = 179.799   176.28   -179.382

// 输入是欧拉角分别: X-Y-Z
// 旋转顺序是: X-Y-Z
// 旋转矩阵的公式为: Rz*Ry*Rx
Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d theta)
{
    // 计算旋转矩阵的X分量
    Eigen::Matrix3d R_x;
    R_x << 1,       0,               0,
           0,       cos(theta[0]),   -sin(theta[0]),
           0,       sin(theta[0]),   cos(theta[0]);

    // 计算旋转矩阵的Y分量
    Eigen::Matrix3d R_y;
    R_y << cos(theta[1]),    0,      sin(theta[1]),
           0,                1,      0,
           -sin(theta[1]),   0,      cos(theta[1]);

    // 计算旋转矩阵的Z分量
    Eigen::Matrix3d R_z;
    R_z << cos(theta[2]),    -sin(theta[2]),     0,
           sin(theta[2]),    cos(theta[2]),     0,
           0,                0,                 1;

    // 依次左乘，合并
    Eigen::Matrix3d R = R_z*R_y*R_x; /** @todo 顺序: X-Y-Z*/
    return R;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// 这里对应的旋转矩阵的公式为: Rz*Ry*Rx
/// 旋转矩阵转换为欧拉角时，如果出现`Y`方向旋转为`+/-90`度，则出现一个角度的自由度，导致结算结果不唯一，一般的做法是令另外一个角度为`0`
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // true: `Y`方向旋转为`+/-90`度
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

int main ( int argc, char** argv )
{
    ros::init(argc,argv,"odom_tf_pub");
    ros::NodeHandle n;
    tf::TransformBroadcaster broadcaster;
    ros::Rate spin_rate(20);

    //cout<<"Z--Y--X= "<<c_euler_zyx_.transpose()*180/M_PI<<endl;

    // ---<机体坐标系旋转>，即以机体坐标系为轴进行旋转，这里旋转顺序为:ZYX
    // ---<世界坐标系的旋转>，即以世界坐标系为轴进行旋转，这里旋转顺序为:XYZ
    // 二者旋转顺序不同，但值相等
    //【1】: 通过eigen生成旋转向量
    Eigen::AngleAxisd rotation_vector ( -M_PI/2, Eigen::Vector3d ( 0,0,1 ) );  //沿 Z 轴旋转
    Eigen::AngleAxisd rotation_vector_y ( 0, Eigen::Vector3d ( 0,1,0 ) );      //沿 Y 轴旋转
    Eigen::AngleAxisd rotation_vector_x ( -M_PI/2, Eigen::Vector3d ( 1,0,0 ) );//沿 X 轴旋转
    rotation_vector =                                            //1.对于机体坐标系，ZYX，右乘
            rotation_vector*rotation_vector_y*rotation_vector_x; //2.对于世界坐标系，XYZ，左乘

    //【2】: 欧拉角
    Eigen::Vector3d c_euler_zyx_;
    // 定义<机体坐标系旋转>
    c_euler_zyx_ << M_PI/2 , M_PI/2,  -M_PI/2; //ZYX

    Eigen::Vector3d w_euler_xyz_;
    // <机体坐标系旋转>转换到<世界坐标系的旋转>
    // 交换x,z的位置
    w_euler_xyz_ << c_euler_zyx_(2), c_euler_zyx_(1), c_euler_zyx_(0);
    cout<<"w_euler_xyz_: "<<w_euler_xyz_.transpose()*180/M_PI<<endl<<endl;

    Eigen::Matrix3d rotation_matrix;
    // <世界坐标系的旋转>，顺序为: XYZ
    // <旋转矩阵>为从<机体坐标系>到<世界坐标系的转换>
    //【3】: 欧拉角-->>旋转矩阵
    rotation_matrix = eulerAnglesToRotationMatrix(w_euler_xyz_);

//ds1-extrinc
//    Eigen::Matrix3d Ric, Rbi;
//    Ric << -0.006585518610541,   0.999854268284434,  -0.015750337711632,
//                  0.999635959715315,   0.006994564282106,   0.026058091153127,
//                  0.026164460412388,  -0.015572997909963,  -0.999536343885216;
//    Rbi << -0.056124481777875,   0.066089747039460,  -0.996234002572400,
//            -0.012596592290069,  -0.997774221731664,  -0.065482274780669,
//            -0.998344313555120,   0.008873994818207,   0.056832066724022;
//    rotation_matrix = Rbi * Ric;


    //【3-1】: 旋转向量-->>旋转矩阵
    //rotation_matrix = rotation_vector.matrix();

    cv::Mat Rbc = (cv::Mat_<double>(3,3) <<
            rotation_matrix(0,0), rotation_matrix(0,1), rotation_matrix(0,2),
            rotation_matrix(1,0), rotation_matrix(1,1), rotation_matrix(1,2),
            rotation_matrix(2,0), rotation_matrix(2,1), rotation_matrix(2,2));
    Rbc = (cv::Mat_<double>(3,3) <<
           0.999473,  0.0243328,  0.0215027,
           -0.0210329, -0.0194028,    0.99959,
              0.02474,  -0.999516, -0.0188808);
    cout<<"rotation matrix ="<<Rbc<<endl;
    cv::Mat rotation_Mat_vector;

    //【4】: 旋转矩阵-->>旋转向量
    cv::Rodrigues(Rbc, rotation_Mat_vector);
    cout<<"rotation vector ="<<rotation_Mat_vector.t()<<endl<<endl;

    //【4-1】: 自定义旋转向量
    if(argc==4)
    {
        cv::Mat Rcd,Rdb=Rbc;
        std::stringstream para_stream;
        Eigen::Vector3d rotation_vector_temp;
        para_stream<<argv[1]<<" "<<argv[2]<<" "<<argv[3];
        para_stream>>rotation_vector_temp(0)>>rotation_vector_temp(1)>>rotation_vector_temp(2);
        // 2.107961630209229 2.135572554932028 -0.1516360053404375
        rotation_Mat_vector = (cv::Mat_<double>(3,1) <<rotation_vector_temp(0),rotation_vector_temp(1),rotation_vector_temp(2));
        std::cout<<"rotation_vector_temp: "<<rotation_vector_temp<<std::endl;
        cv::Rodrigues(rotation_Mat_vector, Rcd);
        Rbc = Rcd;//(Rcd*Rdb).t();

        //-1.02701024274365, 1.018448485607195, -1.318042471155976
        cv::Rodrigues(Rbc, rotation_Mat_vector);
        cout<<"相机到里程计的rotation vector ="<<rotation_Mat_vector.t()<<endl<<endl;
        cout<<"相机到里程计的rotation matrix ="<<Rbc<<endl<<endl;
    }
    else
    {
        //【5】: 旋转向量-->>旋转矩阵
        rotation_Mat_vector = (cv::Mat_<double>(3,1) << -1.8026415, -0.036698326, -0.073075235);
        cv::Rodrigues(rotation_Mat_vector, Rbc);
    }

    //【6】: 旋转矩阵-->>欧拉角
    // 这里的旋转矩阵的公式为: Rz*Ry*Rx
    /// 旋转矩阵转换为欧拉角时，如果出现`Y`方向旋转为`+/-90`度，则出现一个角度的自由度，导致结算结果不唯一，一般的做法是令另外一个角度为`0`
    cv::Vec3f euler_xyz = rotationMatrixToEulerAngles(Rbc);
    cout<<"euler angles: X-Y-Z= "<<euler_xyz*180/M_PI<<endl<<endl;

    //【6-1】: 旋转矩阵-->>欧拉角(使用eigen公式)
    /** @todo debug ok*/
    /// 如果出现`Y`方向旋转为`+/-90`度，则出现一个角度的自由度，导致结算结果不唯一，一般的做法是令另外一个角度为`0`,
    /// 但这里不是这样，但不影响最后的位置
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles( 2,1,0 );//ZYX顺序
    cout<<"euler angles: Z-Y-X= "<<euler_angles.transpose()*180/M_PI<<endl<<endl;

    while(ros::ok())
    {
        geometry_msgs::TransformStamped camera_trans;
        camera_trans.header.frame_id = "base_footprint";
        camera_trans.child_frame_id = "camera_test";

        // update transform
        camera_trans.header.stamp = ros::Time::now();
        camera_trans.transform.translation.x = 0.0;
        camera_trans.transform.translation.y = 0;
        camera_trans.transform.translation.z = 0.0;

        // param roll The roll about the X axis
        // param pitch The pitch about the Y axis
        // param yaw The yaw about the Z axis
        /** @attention */
        // 按照自身坐标系是: Z-Y-X顺序，即YPR，
        // 按照世界坐标系是: X-Y-Z顺序，及RPY，(一般不使用，太绕了)
        /// 在描述统一姿态时，二者的是<等价的>，即<角度值是相等的>
        camera_trans.transform.rotation = //
                tf::createQuaternionMsgFromRollPitchYaw(euler_xyz[0],euler_xyz[1],euler_xyz[2]);

        broadcaster.sendTransform(camera_trans);
        ros::spinOnce();
        spin_rate.sleep();
    }

    return 0;
}




























