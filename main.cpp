#include "svg.cpp"
#include <chrono>
#include <iostream>

int K = 1000;
int W = 512;
int H = 512;

static std::default_random_engine engine(std::random_device{}());
static std::uniform_real_distribution<double> uniform(0, 1);

void lab_6_1(Polygon& point_set) {
    point_set.triangulate();
    std::vector<Polygon> polygons;
    for (int i = 0; i < point_set.triangulation.size(); ++i) {
        if (point_set.triangulation[i].valid) {
            polygons.push_back(Polygon({point_set.triangulation[i].A, point_set.triangulation[i].B, point_set.triangulation[i].C}));
        }
    }
    save_polygons_svg(polygons, W, H, "_results/lab_6_1_1.svg");
    point_set.compute_voronoi();
    save_segments_svg(point_set.voronoi, W, H, "_results/lab_6_1_2.svg");
}

void lab_6_2(Polygon& point_set) {
    std::vector<int> curIter;
    std::vector<double> distance;
    point_set.compute_voronoi2(W, H, curIter, distance);
    save_ints_svg(curIter, W, H, "_results/lab_6_2_1.svg");
    save_doubles_svg(distance, W, H, "_results/lab_6_2_2.svg");
}

void lab_6_3(Polygon& point_set) {
    std::vector<Polygon> voronoi_cells = point_set.compute_voronoi3(W, H);
    save_polygons_svg(voronoi_cells, W, H, "_results/lab_6_3.svg");
}

void lab_6_4(Polygon& point_set) {
    std::vector<Polygon> voronoi_cells = point_set.compute_voronoi4(W, H);
    save_polygons_svg(voronoi_cells, W, H, "_results/lab_6_4.svg");
}


int main() {
    std::chrono::time_point<std::chrono::steady_clock> start, end;
    std::chrono::seconds::rep duration;

    Polygon point_set;
    for (int i = 0; i < K; ++i) {
        double x = W * uniform(engine);
        double y = H * uniform(engine);
        point_set.vertices.push_back(Vector(x, y));
    }

    std::cout << "Delaunay Triangulation" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_6_1(point_set);
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " milliseconds" << std::endl;
    std::cout << "Output stored in _results/lab_6_1_1.svg and _results/lab_6_1_2.svg" << std::endl << std::endl;

    std::cout << "Jump Flooding" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_6_2(point_set);
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " milliseconds" << std::endl;
    std::cout << "Output stored in _results/lab_6_2_1.svg and _results/lab_6_2_2.svg" << std::endl << std::endl;

    std::cout << "Voronoi Parallel Linear Enumeration" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_6_3(point_set);
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " milliseconds" << std::endl;
    std::cout << "Output stored in _results/lab_6_3.svg" << std::endl << std::endl;

    std::cout << "K-nearest Neighbor Search" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_6_4(point_set);
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " milliseconds" << std::endl;
    std::cout << "Output stored in _results/lab_6_4.svg" << std::endl << std::endl;

    return 0;
}
