#include<iostream>
#include<Eigen/Dense>
#include<math.h>
#include<vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


#define pi 3.141592653589

class Robot{
private:
    double d1 =  0.089159;
    double a2 = -0.425;
    double a3 = -0.39225;
    double a7 = 0.075;
    double d4 =  0.10915;
    double d5 =  0.09465;
    double d6 =  0.0823;
    vector<double> d;
    vector<double> a;
    vector<double> alph;
    Eigen::MatrixXd th; // 8 possible positions

public:
    Robot(){
        d={0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
        a={0 ,-0.425 ,-0.39225 ,0 ,0 ,0};
        alph={pi/2, 0, 0, pi/2, -pi/2, 0};
        th = MatrixXd::Zero(6,8);
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

    void HTrans(int c, MatrixXd &T_06){
        MatrixXd A_1, A_2, A_3, A_4, A_5, A_6;
        A_1=AH(1,c);
        A_2=AH(2,c);
        A_3=AH(3,c);
        A_4=AH(4,c);
        A_5=AH(5,c);
        A_6=AH(6,c);

        T_06 = A_1*A_2*A_3*A_4*A_5*A_6;
        return;
    }

    Eigen::MatrixXd invKine(Eigen::MatrixXd desired_pos){
        //P_05
        Eigen::MatrixXd P_05 = (desired_pos * Eigen::Vector4d(0, 0, -d6, 1)).transpose() - Eigen::Vector4d(0, 0, 0, 1).transpose();
        cout<<"P_05: "<<P_05<<endl<<endl;
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

    Eigen::MatrixXd getTheta(){
        return(th);
    }
};

int main(){
    Robot r1;
    Eigen::MatrixXd goal(4,4);
    goal<<-0.5, 0.5, 0.5, 0.5,
        0.5, -0.5, 0.5, 0.5,
        0.5, 0.5, -0.5, 0.5,
        0.0, 0.0, 0.0, 1.0;

    cout<<r1.invKine(goal)<<endl;
    return 0;
}

