#include<iostream>
#include<Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(){
    Eigen::MatrixXd tst(4,4);
    tst<< 1,0,0,1,
        0,1,0,2,
        0,0,0,5,
        0,0,0,1;
    
    Eig::MatrixXd tst_inv = tst.inverse();

    cout<<test<<endl;
    cout<<test_inv<<endl;

}