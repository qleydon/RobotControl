#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

#include<iostream>
#include<Eigen/Dense>
#include<math.h>
#include<cmath>
#include<vector>
#include<string>
#include<cstring>

#include <matplotlibcpp17/cm.h>
#include <matplotlibcpp17/axes.h>
#include <matplotlibcpp17/pyplot.h>
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/animation.h>

using namespace std;
using namespace matplotlibcpp17;
using matplotlibcpp17::animation::ArtistAnimation;

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define pi 3.141592653589



class Robot{
private:
    //ROS
    ros::NodeHandle *n;
    ros::Subscriber shoulder_pan_joint_sub;
    ros::Subscriber shoulder_lift_joint_sub;
    ros::Subscriber elbow_joint_sub;
    ros::Subscriber wrist_1_joint_sub;
    ros::Subscriber wrist_2_joint_sub;
    ros::Subscriber wrist_3_joint_sub;
    ros::Publisher joint_com_pub[6];
    std_msgs::Float64 jnt_pos_start[6];

    //controlable 
    double end_effector = 0.0;
    MatrixXd goal = MatrixXd::Zero(4,4);
    int steps = 100;
    double time_step = 0.1;
    int choice = 0;
    double floor = -0.3;
    double position_tolerence = 0.1;

    //DH table
    const vector<double> d{0.089159, 0, 0, 0.10915, 0.09465, 0.0823}; //ur5
    const vector<double> a{0 ,-0.425 ,-0.39225 ,0 ,0 ,0}; //ur5
    const vector<double> alph{pi/2, 0, 0, pi/2, -pi/2, 0};  //ur5
    //d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 mm
    //alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10
    //a =mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10 mm
    
    const double d1 = d[0];
    const double a2 = a[1];
    const double a3 = a[2];
    double a7 = end_effector;
    const double d4 =  d[3];
    const double d5 =  d[4];
    const double d6 =  d[5];


    //Dependents
    MatrixXd th = MatrixXd::Zero(6,8);
    MatrixXd angle = MatrixXd::Zero(6,1);
    MatrixXd theta = MatrixXd::Zero(6, steps);
    MatrixXd xt = MatrixXd::Zero(6, steps);
    MatrixXd yt = MatrixXd::Zero(6, steps);
    MatrixXd zt = MatrixXd::Zero(6, steps);

    void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
        jnt_pos_start[0].data = ctr_msg->process_value;
    }

    void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
        jnt_pos_start[1].data = ctr_msg->process_value;
    }

    void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
        jnt_pos_start[2].data = ctr_msg->process_value;
    }

    void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
        jnt_pos_start[3].data = ctr_msg->process_value;
    }

    void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
        jnt_pos_start[4].data = ctr_msg->process_value;
    }

    void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
        jnt_pos_start[5].data = ctr_msg->process_value;
    }

    void initializeSubscribers() {
        shoulder_pan_joint_sub = n->subscribe("/shoulder_pan_joint_position_controller/state", 1000, &Robot::get_shoulder_pan_joint_position, this);
        shoulder_lift_joint_sub = n->subscribe("/shoulder_lift_joint_position_controller/state", 1000, &Robot::get_shoulder_lift_joint_position, this);
        elbow_joint_sub = n->subscribe("/elbow_joint_position_controller/state", 1000, &Robot::get_elbow_joint_position, this);
        wrist_1_joint_sub = n->subscribe("/wrist_1_joint_position_controller/state", 1000, &Robot::get_wrist_1_joint_position, this);
        wrist_2_joint_sub = n->subscribe("/wrist_2_joint_position_controller/state", 1000, &Robot::get_wrist_2_joint_position, this);
        wrist_3_joint_sub = n->subscribe("/wrist_3_joint_position_controller/state", 1000, &Robot::get_wrist_3_joint_position, this);
    }

    void initializePublishers() {
        joint_com_pub[0] = n->advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
        joint_com_pub[1] = n->advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
        joint_com_pub[2] = n->advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
        joint_com_pub[3] = n->advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
        joint_com_pub[4] = n->advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
        joint_com_pub[5] = n->advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);
    }
    
    
public:
    Robot(int argc, char **argv) {
        ros::init(argc, argv, "IK");
        n = new ros::NodeHandle();
        initializeSubscribers();
        initializePublishers();

    }
    ~Robot(){
        delete n;
        cout<<endl<<endl<<"BEEEE BOOOOOOOOOPPP...."<<endl<<endl<<endl;
    }
   


    void run() {
        std_msgs::Float64 position[6];
            for(int i = 0; i < 6; i++) {
                position[i].data = -0.1;
                joint_com_pub[i].publish(position[i]);
            }
            ros::spinOnce();
            ros::Duration(3.0).sleep();

            for(int i = 0; i < 6; i++) {
                position[i].data = -1;
                joint_com_pub[i].publish(position[i]);
            }
            ros::spinOnce();
            ros::Duration(3.0).sleep();
    }

    MatrixXd rpy(double x, double y, double z){
        MatrixXd roll(4,4);
        roll << 1, 0, 0, 0, 
                0, cos(x), -sin(x), 0,
                0, sin(x), cos(x), 0,
                0, 0, 0, 1;
        
        MatrixXd pitch(4,4);
        pitch << cos(y), 0, sin(y), 0,
                0, 1, 0, 0,
                -sin(y), 0, cos(y), 0,
                0, 0, 0, 1;

        MatrixXd yaw(4,4);
        yaw << cos(z), -sin(z), 0, 0,
                sin(z), cos(z), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        return yaw * pitch * roll;
    }

    void pos(MatrixXd &g, double x, double y, double z){
        g(0,3) = x;
        g(1,3) = y;
        g(2,3) = z;
        return;
    }

    void set_goal(MatrixXd g){
        goal = g;
        return;
    }

    void set_goal(double roll, double pitch, double yaw, double x, double y, double z){
        goal = rpy(roll, pitch, yaw);
        pos(goal, x, y, z);
        return;
    }

    MatrixXd AH(int n, int c){ // C may change
        MatrixXd T_a = MatrixXd::Identity(4,4);
        T_a(0,3) = a[n-1];
        MatrixXd T_d = MatrixXd::Identity(4,4);
        T_d(2,3) = d[n-1];

        MatrixXd Rzt(4,4);
        Rzt << cos(th(n-1,c)), -sin(th(n-1,c)), 0 ,0,
                sin(th(n-1,c)),  cos(th(n-1,c)), 0, 0,
                0,0,1,0,
                0,0,0,1;

        MatrixXd Rxa(4,4);
        Rxa<< 1,0,0,0,
                0, cos(alph[n-1]), -sin(alph[n-1]), 0,
                0, sin(alph[n-1]), cos(alph[n-1]), 0,
                0,0,0,1;

        MatrixXd A_i = T_d * Rzt * T_a *Rxa;
        return A_i;
    }    

    MatrixXd HTrans(int c){
        MatrixXd A_1, A_2, A_3, A_4, A_5, A_6;
        A_1=AH(1,c);
        A_2=AH(2,c);
        A_3=AH(3,c);
        A_4=AH(4,c);
        A_5=AH(5,c);
        A_6=AH(6,c);

        MatrixXd T_06 = A_1*A_2*A_3*A_4*A_5*A_6;
        return T_06;
    }

    MatrixXd AH(int n, int c, MatrixXd th_L){
        MatrixXd T_a = MatrixXd::Identity(4,4);
        T_a(0,3) = a[n-1];
        MatrixXd T_d = MatrixXd::Identity(4,4);
        T_d(2,3) = d[n-1];

        MatrixXd Rzt(4,4);
        Rzt << cos(th_L(n-1,c)), -sin(th_L(n-1,c)), 0 ,0,
                sin(th_L(n-1,c)),  cos(th_L(n-1,c)), 0, 0,
                0,0,1,0,
                0,0,0,1;

        MatrixXd Rxa(4,4);
        Rxa<< 1,0,0,0,
                0, cos(alph[n-1]), -sin(alph[n-1]), 0,
                0, sin(alph[n-1]), cos(alph[n-1]), 0,
                0,0,0,1;

        MatrixXd A_i = T_d * Rzt * T_a *Rxa;
        return A_i;
    }    

    void Joint_positions(MatrixXd th_L){
        MatrixXd A_1, A_2, A_3, A_4, A_5, A_6;
        MatrixXd T_01, T_02, T_03, T_04, T_05, T_06; 
        
        for(int i=0; i<steps; i++){
            A_1=AH(1,i,th_L);
            A_2=AH(2,i,th_L);
            A_3=AH(3,i,th_L);
            A_4=AH(4,i,th_L);
            A_5=AH(5,i,th_L);
            A_6=AH(6,i,th_L);

            T_01 = A_1;
            T_02 = T_01 * A_2;
            T_03 = T_02 * A_3;
            T_04 = T_03 * A_4;
            T_05 = T_04 * A_5;
            T_06 = T_05 * A_6;

            xt(0,i) = T_01(0,3);
            xt(1,i) = T_02(0,3);
            xt(2,i) = T_03(0,3);
            xt(3,i) = T_04(0,3);
            xt(4,i) = T_05(0,3);
            xt(5,i) = T_06(0,3);

            yt(0,i) = T_01(1,3);
            yt(1,i) = T_02(1,3);
            yt(2,i) = T_03(1,3);
            yt(3,i) = T_04(1,3);
            yt(4,i) = T_05(1,3);
            yt(5,i) = T_06(1,3);

            zt(0,i) = T_01(2,3);
            zt(1,i) = T_02(2,3);
            zt(2,i) = T_03(2,3);
            zt(3,i) = T_04(2,3);
            zt(4,i) = T_05(2,3);
            zt(5,i) = T_06(2,3);
        }
        
        return;
    }

    void Joint_positions(MatrixXd &x, MatrixXd &y, MatrixXd &z){
        // x,y,z are (6,8)
        MatrixXd A_1, A_2, A_3, A_4, A_5, A_6;
        MatrixXd T_01, T_02, T_03, T_04, T_05, T_06; 
        
        for(int i=0; i<8; i++){
            A_1=AH(1,i);
            A_2=AH(2,i);
            A_3=AH(3,i);
            A_4=AH(4,i);
            A_5=AH(5,i);
            A_6=AH(6,i);

            T_01 = A_1;
            T_02 = T_01 * A_2;
            T_03 = T_02 * A_3;
            T_04 = T_03 * A_4;
            T_05 = T_04 * A_5;
            T_06 = T_05 * A_6;

            x(0,i) = T_01(0,3);
            x(1,i) = T_02(0,3);
            x(2,i) = T_03(0,3);
            x(3,i) = T_04(0,3);
            x(4,i) = T_05(0,3);
            x(5,i) = T_06(0,3);

            y(0,i) = T_01(1,3);
            y(1,i) = T_02(1,3);
            y(2,i) = T_03(1,3);
            y(3,i) = T_04(1,3);
            y(4,i) = T_05(1,3);
            y(5,i) = T_06(1,3);

            z(0,i) = T_01(2,3);
            z(1,i) = T_02(2,3);
            z(2,i) = T_03(2,3);
            z(3,i) = T_04(2,3);
            z(4,i) = T_05(2,3);
            z(5,i) = T_06(2,3);
        }
        return;
    }

    Eigen::MatrixXd invKine(Eigen::MatrixXd desired_pos){
        Eigen::MatrixXd P_05 = (desired_pos * Eigen::Vector4d(0, 0, -d6, 1)).transpose() - Eigen::Vector4d(0, 0, 0, 1).transpose();
        // theta1
        double psi = atan2(P_05(2-1), P_05(1-1));
        double phi = acos(d4/sqrt(P_05(2-1)*P_05(2-1) + P_05(1-1) * P_05(1-1)));

        th.block(0, 0, 1, 4).setConstant(pi / 2 + psi + phi);
        th.block(0, 4, 1, 4).setConstant(pi / 2 + psi - phi);
        th = th.real();
        
        // theta5
        vector<int> cl{0,4};
        for(int c:cl){
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_16 = T_10 * desired_pos;
            th.block(4, c, 1, 3).setConstant(acos((T_16(2,3)-d4)/d6));
            th.block(4, c + 2, 1, 2).setConstant(-acos((T_16(2,3)-d4)/d6));
        }
        th = th.real();

        // theta6;
        cl={0,2,4,6};
        for(int c:cl){
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_16 = (T_10 * desired_pos).inverse();
            th.block(5, c, 1, 2).setConstant(atan2((-T_16(1,2)/sin(th(4,c))), (T_16(0,2)/sin(th(4,c)))));
        }
        th = th.real();

        // theta3
        cl = {0,2,4,6};
        for(int i=0; i< cl.size(); i++){
            int c = cl[i];
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_65 = AH(6, c);
            Eigen::MatrixXd T_54 = AH(5, c);
            Eigen::MatrixXd T_14 = (T_10 * desired_pos) * (T_54 * T_65).inverse();
            Eigen::Vector4d P_13 = T_14 * Eigen::Vector4d(0, -d4, 0, 1) - Eigen::Vector4d(0, 0, 0, 1);
            
            double arg = (P_13.norm() * P_13.norm() - a[2-1] * a[2-1] - a[3-1] * a[3-1]) / (2 * a[2-1] * a[3-1]);
            double t3;

            if (arg >= -1.0 && arg <= 1.0) {
                t3 = std::acos(arg);
            } else {
                t3 = 0.0;
            }

            th(2, c) = t3;
            th(2, c + 1) = -t3;
        }

        // theta2, theta4
        for(int c=0; c<8; c++){
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_65 = AH(6, c).inverse();
            Eigen::MatrixXd T_54 = AH(5, c).inverse();

            Eigen::MatrixXd T_14 = (T_10 * desired_pos) * T_65 * T_54;
            Eigen::Vector4d P_13 = T_14 * Eigen::Vector4d(0, -d4, 0, 1) - Eigen::Vector4d(0, 0, 0, 1);

            // theta2
            th(1, c) = -std::atan2(P_13(1), -P_13(0)) + std::asin(a[3-1] * std::sin(th(2, c)) / P_13.norm());
            // theta4
            Eigen::MatrixXd T_32 = AH(3, c).inverse();
            Eigen::MatrixXd T_21 = AH(2, c).inverse();
            Eigen::MatrixXd T_34 = T_32 * T_21 * T_14;
            th(3, c) = std::atan2(T_34(1, 0), T_34(0, 0));
        }
        th = th.real();
        return(th);
    }

    void find_steps(){
        theta.col(steps - 1) = th.col(choice);
        MatrixXd dth = theta.col(steps-1) - theta.col(0);
        dth /=steps;

        // Fill theta
        for (int i = 1; i < steps - 1; i++) {
            theta.block<6, 1>(0, i) = theta.col(i) + i * dth;
        }

        //update x,y,z
        Joint_positions(theta);
        return;
    }

    void report_error(){
        for(int j =0; j <8; j++){
            cout << "error for option " << j << endl;
            cout << HTrans(0) - goal << endl;
        }
        return;
    }

    void demo(){
        pybind11::scoped_interpreter guard{};
        auto plt = matplotlibcpp17::pyplot::import();
        matplotlibcpp17::mplot3d::import();
        // Generate some sample data for 3D plot
        std::vector<double> x = {1.0, 2.0, 3.0, 4.0};
        std::vector<double> y = {1.0, 2.0, 3.0, 4.0};
        std::vector<double> z = {1.0, 2.0, 3.0, 4.0};

        // Create a 3D plot
        auto fig = plt.figure();
        auto ax =  fig.add_subplot(py::make_tuple(), Kwargs("projection"_a = "3d"));
        ax.plot(Args(x, y, z), Kwargs("color"_a = "blue", "linewidth"_a = 1.0));

        // Show the plot
        plt.show();
        
        return;
    }

    void show_end(int c){
        MatrixXd x(6,8);
        MatrixXd y(6,8);
        MatrixXd z(6,8);

        Joint_positions(x, y, z);

        vector<double> xs(6,0);
        vector<double> ys(6,0);
        vector<double> zs(6,0);
        for(int i = 0; i<6; i++){
            xs[i] = x(i,c);
            ys[i] = y(i,c);
            zs[i] = z(i,c);
        }

        auto plt = matplotlibcpp17::pyplot::import();
        matplotlibcpp17::mplot3d::import();

         // Create a 3D plot
        auto fig = plt.figure();
        auto ax =  fig.add_subplot(py::make_tuple(), Kwargs("projection"_a = "3d"));
        ax.plot(Args(xs, ys, zs), Kwargs("color"_a = "gray", "linewidth"_a = 20));
        ax.plot(Args(xs, ys, zs), Kwargs("linestyle"_a = "none", "color"_a = "blue", "marker"_a = "o", "markersize"_a = 20));
        ax.set_xlim(Args(-1, 1));
        ax.set_ylim(Args(-1, 1));
        ax.set_zlim(Args(0, 1));

        ax.set_xlabel(Args("X"));
        ax.set_ylabel(Args("Y"));
        ax.set_zlabel(Args("Z"));
        // Show the plot
        plt.show();
    }

    void show_end(){
        MatrixXd x(6,8);
        MatrixXd y(6,8);
        MatrixXd z(6,8);

        Joint_positions(x, y, z);

        vector<double> xs(6,0);
        vector<double> ys(6,0);
        vector<double> zs(6,0);

        auto plt = matplotlibcpp17::pyplot::import();
        matplotlibcpp17::mplot3d::import();
        auto [w, h] = plt.figaspect(Args(0.5));
        auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(w, h)));
        std::string str;
        for(int j=0; j<8; j++)
        {
            
            auto ax = fig.add_subplot(Args(2, 4, j+1), Kwargs("projection"_a = "3d"));
            for(int i = 0; i<6; i++){
                xs[i] = x(i,j);
                ys[i] = y(i,j);
                zs[i] = z(i,j);
            }

            ax.plot(Args(xs, ys, zs), Kwargs("color"_a = "gray", "linewidth"_a = 20));
            ax.plot(Args(xs, ys, zs), Kwargs("linestyle"_a = "none", "color"_a = "blue", "marker"_a = "o", "markersize"_a = 20));
            str = "Choice: " + std::to_string(j);
            ax.set_title(Args(str));
            ax.set_xlim(Args(-1, 1));
            ax.set_ylim(Args(-1, 1));
            ax.set_zlim(Args(-0.1, 1));

            ax.set_xlabel(Args("X"));
            ax.set_ylabel(Args("Y"));
            ax.set_zlabel(Args("Z"));
            
        }
        // Show the plot
        plt.show();
    }

    void show_movement(){
        auto plt_1 = matplotlibcpp17::pyplot::import();
        matplotlibcpp17::mplot3d::import();

        auto fig = plt_1.figure();
        auto ax =  fig.add_subplot(py::make_tuple(), Kwargs("projection"_a = "3d"));

        py::list artist_list;

        for (int j = 0; j < steps; j++) {
            // Clear the plot for the next frame
            ax.cla();
            

            // Get coordinates for the current time step
            std::vector<double> xs(6, 0);
            std::vector<double> ys(6, 0);
            std::vector<double> zs(6, 0);
            for (int i = 0; i < 6; i++) {
                xs[i] = double(xt(i, j));
                ys[i] = double(yt(i, j));
                zs[i] = double(zt(i, j));
            }

            // Plot the current arm
            ax.plot(Args(xs, ys, zs), Kwargs("color"_a = "gray", "linewidth"_a = 20));
            ax.plot(Args(xs, ys, zs), Kwargs("linestyle"_a = "none", "color"_a = "blue", "marker"_a = "o", "markersize"_a = 20));

            // Append the current frame to the artist list
            artist_list.append(ax.get_lines().unwrap());
            // Pause to create animation effect
            ax.set_xlim(Args(-1, 1));
            ax.set_ylim(Args(-1, 1));
            ax.set_zlim(Args(-0.1, 1));

            ax.set_xlabel(Args("X"));
            ax.set_ylabel(Args("Y"));
            ax.set_zlabel(Args("Z"));
            plt_1.pause(Args(0.1));
        }

        // Create the animation
        auto ani = ArtistAnimation(Args(fig.unwrap(), artist_list), Kwargs("interval"_a = 100));
        // Show the animation
        plt_1.show();
    }

    void report_stats(){
        // Compute the difference between consecutive columns
        MatrixXd vx = xt.block(0, 1, 6, 99) - xt.block(0, 0, 6, 99);
        MatrixXd vy = yt.block(0, 1, 6, 99) - yt.block(0, 0, 6, 99);
        MatrixXd vz = zt.block(0, 1, 6, 99) - zt.block(0, 0, 6, 99);

        MatrixXd axt = vx.block(0, 1, 6, 98) - vx.block(0, 0, 6, 98);
        MatrixXd ayt = vy.block(0, 1, 6, 98) - vy.block(0, 0, 6, 98);
        MatrixXd azt = vz.block(0, 1, 6, 98) - vz.block(0, 0, 6, 98);

        MatrixXd vt = 1/time_step * (vx.array().pow(2) + vy.array().pow(2) + vz.array().pow(2)).sqrt();
        MatrixXd at = 1/time_step * 1/time_step * (axt.array().pow(2) + ayt.array().pow(2) + azt.array().pow(2)).sqrt();  
        
        std::vector<double> temp_v(100, 0);
        std::vector<double> temp_a(100, 0);
        std::vector<double> time(100,0);

        for(int i =1; i<100; i++){
            time[i] = i;
        }

        auto plt = matplotlibcpp17::pyplot::import();

        auto [fig, axes] =
            plt.subplots(2,3,
                        Kwargs("figsize"_a = py::make_tuple(9, 6),
                                "subplot_kw"_a = py::dict("aspect"_a = "equal")));
        auto ax1 = axes[0], ax2 = axes[1], ax3 = axes[2];

        std::string str;
        for(int j=0; j<6; j++){
            for(int i =1; i<100; i++){
                temp_v[i] = vt(j,i-1);
                if(i!=1){
                    temp_a[i] = at(j,i-2);
                }
            }
            axes[j].plot(Args(time, temp_v), Kwargs("color"_a = "blue", "label"_a="velocity (m/s)"));
            axes[j].plot(Args(time, temp_a), Kwargs("color"_a = "red", "label"_a="acceleration (m/s^2)"));
            axes[j].set_xlabel(Args("time (s)"));
            str = "Joint " + std::to_string(j+1);
            axes[j].set_title(Args(str));
            axes[j].set_ylim(Args(0, 5));
        }
        axes[0].legend();
        
        plt.show();
        return;
    }

    bool validate_robot(){
        MatrixXd err = HTrans(0) - goal;

        if(err(0,3) >= position_tolerence || err(0,3) >= position_tolerence || err(0,3) >= position_tolerence){
            cout << "error out of tollerence" << endl;
            return false;
        }

        for(int j =0; j<6; j++){
            for(int i =0; i<steps; i++){
                if(zt(j,i) <= floor){
                    cout<<"joint "<< j<<" moves to invalid position ("<<xt(j,i)<<","<<yt(j,i)<<","<<zt(j,i)<<")"<<endl;
                    return false;
                }
                if(isnan(theta(j,i)) || isinf(theta(j,i))){
                    cout << "theta invalid for joint " << j << " theta " << i << ": " << theta(j,i) << endl;
                    return false;
                }
            }
        }

        
        return true;
    }

    void reset_robot(){
        std_msgs::Float64 position[6];
        ros::Duration(1).sleep();
        for(int j =0; j<20; j++){
            for(int i = 0; i < 6; i++) {
                position[i].data = 0.0;
                joint_com_pub[i].publish(position[i]);
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        
    }

    void move_robot(){
        std_msgs::Float64 position[6];
            for(int j =0; j<steps; j++){
                for(int i = 0; i < 6; i++) {
                    position[i].data = theta(i, j);
                    joint_com_pub[i].publish(position[i]);
                }
                ros::spinOnce();
                ros::Duration(time_step).sleep();
            }
        return;
    }

    bool unit_test_1(){ // should work fine
        reset_robot();
        set_goal(0,0,0, 0.5,0.5,0.5);
        invKine(goal);
        report_error();
        show_end();
        choice = 1;
        show_end(choice);
        find_steps();

        cout << "theta" << endl<< theta.transpose()<<endl;

        if(validate_robot()){
            show_movement();
            report_stats();
            move_robot();
        }
        else{
            cout <<endl<<endl<< "validation failed unit test 1 !!!"<<endl<<endl;
            return false;
        }
        return true;
    }

    bool unit_test_2(){ // Invalid postion, too low
        reset_robot();
        set_goal(0.0,0.0,0.0,0.5,0.5,-0.5);
        invKine(goal);
        report_error();
        show_end();
        choice = 1;
        show_end(choice);
        find_steps();
        if(validate_robot()){
            cout <<endl<<endl<< "validation should have failed, unit test 2 failure!!!"<<endl<<endl;
            return false;
        }
        return true;
    }

    bool unit_test_3(){ // Invalid postion, interior
        reset_robot();
        set_goal(0.5,0.5,0.5,0,0,0);
        invKine(goal);
        report_error();
        show_end();
        choice = 1;
        show_end(choice);
        find_steps();
        if(validate_robot()){
            cout <<endl<<endl<< "validation should have failed, unit test 3 failure!!!"<<endl<<endl;
            return false;
        }
        return true;
    }

    bool unit_test_4(){ // Invalid postion, exterior
        reset_robot();
        set_goal(0.5,0.5,0.5, 0.5,0.5,3.5);
        invKine(goal);
        report_error();
        show_end();
        choice = 1;
        show_end(choice);
        find_steps();
        if(validate_robot()){
            cout <<endl<<endl<< "validation should have failed, unit test 4 failure!!!"<<endl<<endl;
            return false;
        }
        return true;
    }

    bool unit_test(){
        bool t1 = unit_test_1();
        bool t2 = unit_test_2();
        bool t3 = unit_test_3();
        bool t4 = unit_test_4();

        if(t1 && t2 && t3 && t4){
            cout <<endl<<endl<< "Unit Test Passed"<<endl<<endl;
            return true;
        }
        return false;
    }

   void run_robot(){
        reset_robot();
        // -------------------------------------------------------------------------------------
        // set goal
        // -------------------------------------------------------------------------------------
        double x, y, z, roll, pitch, yaw;
        cout << "Enter desired x, y, z"<<endl;
        cin >> x>>y>>z;
        cout << "Enter desired roll, pitch, yaw"<<endl;
        cin >> roll>>pitch>>yaw;
        set_goal(roll, pitch, yaw, x, y, z);
        cout << "goal: "<< endl << goal << endl;
        // -------------------------------------------------------------------------------------
        // Inverse Kinematics
        // -------------------------------------------------------------------------------------
        MatrixXd th = invKine(goal);
        report_error();
        show_end();
        cout << endl << "choice: ";
        cin >> choice;
        cout << endl << "theta in degrees: " <<endl<<th.col(choice) * 180/pi;
        show_end(choice);
        // -------------------------------------------------------------------------------------
        // motion planning
        // -------------------------------------------------------------------------------------
        find_steps();
        show_movement();
        report_stats();
        // -------------------------------------------------------------------------------------
        // motion
        // -------------------------------------------------------------------------------------
        string str;
        cout << "move robot [y/n]"<<endl;
        cin >> str;
        transform(str.begin(), str.end(), str.begin(), ::tolower); 
        if(str == "y" || str == "yes"){
            cout << "moving robot"<<endl;
            move_robot();
        }
        return;
   }
};

int main(int argc, char **argv){
    py::scoped_interpreter guard{};
    Robot r1(argc, argv);
    //r1.unit_test();
    r1.run_robot();
    return 0; 
}

