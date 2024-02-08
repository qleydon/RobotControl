#include <matplotlibcpp17/axes.h>
#include <matplotlibcpp17/pyplot.h>
#include <matplotlibcpp17/mplot3d.h>
#include <vector>


class Robot {
private:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

public:
    Robot(const std::vector<double>& x_values, const std::vector<double>& y_values, const std::vector<double>& z_values)
        : x(x_values), y(y_values), z(z_values) {}

    void plot() {
        pybind11::scoped_interpreter guard{};
        auto plt = matplotlibcpp17::pyplot::import();
        matplotlibcpp17::mplot3d::import();

        // Create a 3D plot
         auto fig = plt.figure();
    auto ax =  fig.add_subplot(py::make_tuple(), Kwargs("projection"_a = "3d"));
    ax.plot(Args(x, y, z), Kwargs("color"_a = "blue", "linewidth"_a = 1.0));

        // Show the plot
        plt.show();
    }
};

int main() {
    std::vector<double> x_values = {1.0, 2.0, 3.0, 4.0};
    std::vector<double> y_values = {1.0, 2.0, 3.0, 4.0};
    std::vector<double> z_values = {1.0, 2.0, 3.0, 4.0};

    Robot r1(x_values, y_values, z_values);
    r1.plot();
    r1.plot();

    return 0;
}
