#include "svg.cpp"
#include <chrono>
#include <iostream>

static std::default_random_engine engine(std::random_device{}());
static std::uniform_real_distribution<double> uniform(0, 1);

void lab_6_1() {
    int K = 1000;
    Polygon point_set;
    for (int i = 0; i < K; ++i) {
        double x = uniform(engine);
        double y = uniform(engine);
        point_set.vertices.push_back(Vector(x, y));
    }
    point_set.triangulate();
    std::vector<Polygon> polygons;
    for (int i = 0; i < point_set.triangulation.size(); ++i) {
        if (point_set.triangulation[i].valid) {
            polygons.push_back(Polygon({point_set.triangulation[i].A, point_set.triangulation[i].B, point_set.triangulation[i].C}));
        }
    }
    save_svg(polygons, "_results/lab_6_1.svg");
}


int main() {
    std::chrono::time_point<std::chrono::steady_clock> start, end;
    std::chrono::seconds::rep duration;

    std::cout << "Delaunay triangulation" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_6_1();
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " seconds" << std::endl;
    std::cout << "Output stored in _results/lab_6_1.svg" << std::endl << std::endl;

    return 0;
}
