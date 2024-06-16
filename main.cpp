#include "svg.cpp"
#include <chrono>
#include <iostream>

int K1 = 1000;
int K2 = 100;
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


void lab_7_1(Polygon point_set, int max_iterations=1000) {
    std::vector<Polygon> voronoi_cells;
    voronoi_cells = point_set.compute_voronoi3(W, H);
    save_centers_and_polygons_svg(point_set.vertices, voronoi_cells, W, H, "_results/lab_7_1_1.svg");
    for (int i = 0; i < max_iterations; ++i) {
        if (point_set.lloyd_iteration(W, H)) {
            break;
        }
    }
    voronoi_cells = point_set.compute_voronoi3(W, H);
    save_centers_and_polygons_svg(point_set.vertices, voronoi_cells, W, H, "_results/lab_7_1_2.svg");
}

void lab_7_2(Polygon& point_set) {
    Vector center = Vector(W/2, H/2);
    double sigma = sqrt(W * W + H * H) / 4;
    std::vector<double> weights(point_set.vertices.size());
    std::vector<double> radii(point_set.vertices.size());
    for (int i = 0; i < point_set.vertices.size(); ++i) {
        weights[i] = 2 * sqrt(W*H) * exp(-(point_set.vertices[i] - center).norm2() / (2 * sigma * sigma));
        radii[i] = sqrt(weights[i]);
    }
    std::vector<Polygon> voronoi_cells = point_set.compute_voronoi3(W, H);
    std::vector<Polygon> power_diagram_cells = point_set.compute_power_diagram(W, H, weights);
    save_circles_and_polygons_svg(point_set.vertices, radii, voronoi_cells, W, H, "_results/lab_7_2_1.svg");
    save_circles_and_polygons_svg(point_set.vertices, radii, power_diagram_cells, W, H, "_results/lab_7_2_2.svg");
}

void lab_7_3(Polygon& point_set) {
    Vector center = Vector(W/2, H/2);
    double sigma = sqrt(W * W + H * H) / 4;
    std::vector<double> weights(point_set.vertices.size(), 1);
    std::vector<double> target_areas(point_set.vertices.size());
    for (int i = 0; i < point_set.vertices.size(); ++i) {
        target_areas[i] = 2 * sqrt(W*H) * exp(-(point_set.vertices[i] - center).norm2() / (2 * sigma * sigma));
    }
    Data data;
    data.polygon = point_set;
    data.target_areas = target_areas;
    data.weights = weights;
    data.W = W;
    data.H = H;
    objective_function obj(data);
    weights = obj.run(K2);
    std::vector<Polygon> power_diagram_cells = point_set.compute_power_diagram(W, H, weights);
    save_centers_and_polygons_svg(point_set.vertices, power_diagram_cells, W, H, "_results/lab_7_3.svg");
}


int main() {
    std::chrono::time_point<std::chrono::steady_clock> start, end;
    std::chrono::seconds::rep duration;

    Polygon point_set_1;
    for (int i = 0; i < K1; ++i) {
        double x = W * uniform(engine);
        double y = H * uniform(engine);
        point_set_1.vertices.push_back(Vector(x, y));
    }

    /*std::cout << "Delaunay Triangulation" << std::endl;
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
    std::cout << "Output stored in _results/lab_6_4.svg" << std::endl << std::endl;*/

    Polygon point_set_2;
    for (int i = 0; i < K2; ++i) {
        double x = W * uniform(engine);
        double y = H * uniform(engine);
        point_set_2.vertices.push_back(Vector(x, y));
    }

    std::cout << "Centroidal Voronoi Tessellation" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_7_1(point_set_2);
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " milliseconds" << std::endl;
    std::cout << "Output stored in _results/lab_7_1_1.svg and _results/lab_7_1_2.svg" << std::endl << std::endl;

    std::cout << "Power Diagram" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_7_2(point_set_2);
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " milliseconds" << std::endl;
    std::cout << "Output stored in _results/lab_7_2_1.svg and _results/lab_7_2_2.svg" << std::endl << std::endl;

    std::cout << "Semi-discrete Optimal Transport" << std::endl;
    start = std::chrono::steady_clock::now();
    lab_7_3(point_set_2);
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time elapsed: " << duration << " milliseconds" << std::endl;
    std::cout << "Output stored in _results/lab_7_3.svg" << std::endl << std::endl;

    return 0;
}
