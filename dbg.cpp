#include<iostream>
#include<Eigen/Dense>
#include<math.h>
#include<vector>

#include <matplotlibcpp17/axes.h>
#include <matplotlibcpp17/pyplot.h>
#include <matplotlibcpp17/mplot3d.h>


using namespace std;

int main() {
    Eigen::MatrixXd tst(4,4);
    tst<< 1,0,0,1,
        0,1,0,2,
        0,0,1,5,
        0,0,0,1;
    
    Eigen::MatrixXd tst_inv = tst.inverse();

    cout<<tst<<endl;
    cout<<tst_inv<<endl;




    pybind11::scoped_interpreter guard{};
    virtual auto plt = matplotlibcpp17::pyplot::import();
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
    
    return 0;
}