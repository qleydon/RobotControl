#include <matplotlibcpp17/axes.h>
#include <matplotlibcpp17/pyplot.h>
#include <matplotlibcpp17/mplot3d.h>
#include <vector>

using namespace std;

int main() {
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
    
    return 0;
}