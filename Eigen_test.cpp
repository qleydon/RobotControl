#include<iostream>
#include<Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
//using Eigen::Tensor;


int main(){
    Eigen::MatrixXd tst(4,4);
    tst<< 1,0,0,1,
        0,1,0,2,
        0,0,1,5,
        0,0,0,1;
    
    Eigen::MatrixXd tst_inv = tst.inverse();

    cout<<tst<<endl;
    cout<<tst_inv<<endl;
    
    // Create a tensor of zeros with dimensions 4x4x100
    Eigen::Tensor<double, 3> tensor(4, 4, 100);
    tensor.setZero();

    std::cout << "Tensor dimensions: " << tensor.dimension(0) << "x" << tensor.dimension(1) << "x" << tensor.dimension(2) << std::endl;


    return 0;
}