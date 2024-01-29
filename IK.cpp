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
    vector<double> d;
    vector<double> a;
    vector<double> alph;
    MatrixXd th(6,8); // 8 possible positions

public:
    Robot(){
        ef = 0.05;
        d={0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
        a={0 ,-0.425 ,-0.39225 ,0 ,0 ,0};
        alph={pi/2, 0, 0, pi/2, -pi/2, 0};
        th = MatrixXd::zeros(4,4);
    }

    void AH(int n, int c, MatrixXd &A_i){ // C may change
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

        A_i = T_d * Rzt * T_a *Rxa;
        return;
    }    

    void HTrans(int c; MatrixXd &T_06){
        A_1=AH(1,c);
        A_2=AH(2,c);
        A_3=AH(3,c);
        A_4=AH(4,c);
        A_5=AH(5,c);
        A_6=AH(6,c);

        T_06 = A_1*A_2*A_3*A_4*A_5*A_6;
        return;
    }

    Eigen::MatrixXd invKine(Eigen::Vector4d desired_pos){
        //P_05
        Eigen::Vector4d P_05 = (desired_pos * Eigen::Vector4d(0, 0, -d[6-1], 1).transpose() - Eigen::Vector4d(0, 0, 0, 1)).transpose();
        
        // theta1
        double psi = atan2(p_05(2-1), P_05(1-1))
        double phi = acos(d[4-1]/sqrt(P_05(2-1)*P_05(2-1) + P_05(1-1) * P_05(1-1)))

        th(0,Eigen::seq(0,3)) = pi/2+psi+phi;
        th(0, Eigen::seq(4,7)) = pi/2+psi-phi;
        th = th.real;
        
        // theta5
        vector<int> cl{0,4};
        for(int c:cl){
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_16 = T_10 * desired_pos;
            th(4, Eigen::seq(c,c+2)) = acos((T_16(2,3)-d[4-1])/d[6-1]);
            th(4,Eigen::seq(c+2,c+4))=-acos((T_16[2,3]-d[4-1])/d[6-1]);
        }
        th = th.real;

        // theta6;
        cl={0,2,4,6};
        for(int c:cl){
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_16 = (T_10 * desired_pos).inverse;
            th(5, Eigen::seq(c, c+2)) = atan2((-T_16(1,2)/sin(th(4,c))), (T_16(0,2)/sin(th(4,c))));
        }
        th = th.real;

        // theta3
        cl = {0,2,4,6};
        for(int i=0; i< cl.size(); i++){
            int c = cl[i];
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_65 = AH(6, c);
            Eigen::MatrixXd T_54 = AH(5, c);
            Eigen::MatrixXd T_14 = (T_10 * desired_pos) * (T_54 * T_65).inverse();
            Eigen::Vector4d P_13 = T_14 * Eigen::Vector4d(0, -d[4-1], 0, 1) - Eigen::Vector4d(0, 0, 0, 1);
            
            double t3 = std::acos((P_13.norm() * P_13.norm() - a[2-1] * a[2-1] - a[3-1] * a[3-1]) / (2 * a[2-1] * a[3-1]));
            th(2, c) = t3;
            th(2, c + 1) = -t3;
        }

        // theta2, theta4
        for(int c=0; i<8; i++){
            Eigen::MatrixXd T_10 = AH(1, c).inverse();
            Eigen::MatrixXd T_65 = AH(6, c).inverse();
            Eigen::MatrixXd T_54 = AH(5, c).inverse();
            Eigen::MatrixXd T_32 = AH(3, c).inverse();
            Eigen::MatrixXd T_21 = AH(2, c).inverse();

            Eigen::MatrixXd T_14 = (T_10 * desired_pos) * T_65 * T_54;
            Eigen::Vector4d P_13 = T_14 * Eigen::Vector4d(0, -d[4-1], 0, 1) - Eigen::Vector4d(0, 0, 0, 1);

            // theta2
            th(1, c) = -std::atan2(P_13(1), -P_13(0)) + std::asin(a3 * std::sin(th(2, c)) / P_13.norm());
            Eigen::MatrixXd T_34 = T_32 * T_21 * T_14;
            th(3, c) = std::atan2(T_34(1, 0), T_34(0, 0));
        }
        th = th.real;
    }

    Eigen::MatrixXd getTheta(){
        return(th);
    }
}

int main(){
    Robot r1();

}

