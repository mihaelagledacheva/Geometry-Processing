#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include "vector.cpp"
#include "liblbfgs/lib/lbfgs.c"
#include "nanoflann/include/nanoflann.hpp"

struct PointCloud {
    std::vector<Vector> points;

    inline size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_distance(const double *p1, const size_t idx_p2, size_t) const {
        const double d0 = p1[0] - points[idx_p2][0];
        const double d1 = p1[1] - points[idx_p2][1];
        return d0 * d0 + d1 * d1;
    }

    inline double kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0) return points[idx][0];
        else return points[idx][1];
    }

    template<class BBOX>
    bool kdtree_get_bbox(BBOX &) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud, 2> my_kd_tree_t;


class Segment {
public:
    Vector A, B;
    Segment(Vector& A, Vector& B) : A(A), B(B) {}
};


class Triangle {
private:
    void compute_barycenter() {
        double x = (A[0] + B[0] + C[0]) / 3;
        double y = (A[1] + B[1] + C[1]) / 3;
        G = Vector(x, y);
    }

    void compute_circumcenter() {
        double D = 2 * (A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1]));
        double Ox = ((A[0] * A[0] + A[1] * A[1]) * (B[1] - C[1]) +
                     (B[0] * B[0] + B[1] * B[1]) * (C[1] - A[1]) +
                     (C[0] * C[0] + C[1] * C[1]) * (A[1] - B[1])) / D;
        double Oy = ((A[0] * A[0] + A[1] * A[1]) * (C[0] - B[0]) +
                     (B[0] * B[0] + B[1] * B[1]) * (A[0] - C[0]) +
                     (C[0] * C[0] + C[1] * C[1]) * (B[0] - A[0])) / D;
        O = Vector(Ox, Oy);
        R = (O - A).norm();
    }

    bool ccw(Vector& X, Vector& Y, Vector& Z) {
        return (Z[1]-X[1])*(Y[0]-X[0]) > (Y[1]-X[1])*(Z[0]-X[0]);
    }

    bool intersect(Vector& M, Vector& N, Vector& X, Vector& Y) {
        return ccw(M,X,Y) != ccw(N,X,Y) && ccw(M,N,X) != ccw(M,N,Y);
    }

public:
    Vector A, B, C;
    Vector O, G;
    double R;
    bool valid;
    int na, nb, nc;

    Triangle(Vector& A, Vector& B, Vector& C) : A(A), B(B), C(C) {
        valid = true;
        na = nb = nc = -1;
        compute_circumcenter();
        compute_barycenter();
    }

    bool has_vertex(Vector& P) {
        if (P == A || P == B || P == C) {
            return true;
        }
        return false;
    }

    bool in_circumcircle(Vector& P) {
        return (O - P).norm() <= R;
    }

    void update_neighbor(int index, int new_index) {
        if (na == index) {
            na = new_index;
        } else if (nb == index) {
            nb = new_index;
        } else {
            nc = new_index;
        }
    }

    int locate_point(Vector& P) {
        if (intersect(A, B, P, G)) {
            return nc;
        } else if (intersect(B, C, P, G)) {
            return na;
        } else if (intersect(C, A, P, G)) {
            return nb;
        } else {
            return -1;
        }
    }
};


class Polygon {
private:
    void bound() {
        double min_x = vertices[0][0];
        double max_x = vertices[0][0];
        double min_y = vertices[0][1];
        double max_y = vertices[0][1];
        for (int i = 1; i < vertices.size(); ++i) {
           min_x = std::min(min_x, vertices[i][0]);
           max_x = std::max(max_x, vertices[i][0]);
           min_y = std::min(min_y, vertices[i][1]);
           max_y = std::max(max_y, vertices[i][1]);
        }
        double W = max_x - min_x;
        double H = max_y - min_y;
        Vector A = Vector(min_x - (H / sqrt(3)) - 1, min_y - (sqrt(3) / 2));
        Vector B = Vector(max_x + (H / sqrt(3)) + 1, min_y - (sqrt(3) / 2));
        Vector C = Vector(min_x + (W / 2), max_y + (W * sqrt(3) / 2) + 1);
        triangulation.push_back(Triangle(A, B, C));
    }

    int choose_random_triangle() {
        static std::default_random_engine gen(std::random_device{}());
        std::uniform_int_distribution<> dis(0, triangulation.size()-1);
        while (true) {
            int index = dis(gen);
            if (triangulation[index].valid) {
                return index;
            }
        }
    }

    int locate_triangle(Vector& vertex) {
        int index = choose_random_triangle();
        std::vector<int> seen = {index};
        while (true) {
            int n = triangulation[index].locate_point(vertex);
            if (n == -1 || std::find(seen.begin(), seen.end(), n) != seen.end()) {
                break;
            }
            index = n;
            seen.push_back(n);
        }
        return index;
    }

    void remove_area(Vector& v, std::vector<int>& indices, std::vector<Vector>& area, std::vector<int>& neighbors) {
        while (true) {
            std::vector<int> indices_new, neighbors_new;
            std::vector<Vector> area_new;
            for (int i = 0; i < area.size(); ++i) {
                area_new.push_back(area[i]);
                if (neighbors[i] != -1) {
                    if (triangulation[neighbors[i]].in_circumcircle(v)) {
                        triangulation[neighbors[i]].valid = false;
                        if (triangulation[neighbors[i]].A == area[i]) {
                            area_new.push_back(triangulation[neighbors[i]].B);
                            neighbors_new.push_back(triangulation[neighbors[i]].nc);
                            neighbors_new.push_back(triangulation[neighbors[i]].na);
                        } else if (triangulation[neighbors[i]].B == area[i]) {
                            area_new.push_back(triangulation[neighbors[i]].C);
                            neighbors_new.push_back(triangulation[neighbors[i]].na);
                            neighbors_new.push_back(triangulation[neighbors[i]].nb);
                        } else {
                            area_new.push_back(triangulation[neighbors[i]].A);
                            neighbors_new.push_back(triangulation[neighbors[i]].nb);
                            neighbors_new.push_back(triangulation[neighbors[i]].nc);
                        }
                        indices_new.push_back(neighbors[i]);
                        indices_new.push_back(neighbors[i]);
                    } else {
                        neighbors_new.push_back(neighbors[i]);
                        indices_new.push_back(indices[i]);
                    }
                } else {
                    neighbors_new.push_back(neighbors[i]);
                    indices_new.push_back(indices[i]);
                }
            }
            if (area == area_new) {
                break;
            }
            area = std::move(area_new);
            indices = std::move(indices_new);
            neighbors = std::move(neighbors_new);
        }
    }

    void JFA(int W, int H, int step, const int* prevIter, int* curIter, double* distance) {
        #pragma omp parallel for
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                Vector p = Vector(j, i);
                double minDist2 = std::numeric_limits<double>::max();
                int bestSite = -1;
                for (int k = -1; k <= 1; k++) {
                    for (int l = -1; l <= 1; l++) {
                        int i2 = i + k*step, j2 = j + l*step;
                        if (i2 < 0 || j2 < 0 || i2 >= H || j2 >= W || prevIter[i2*W + j2] < 0 ) {
                            continue;
                        }
                        double dist2 = (vertices[prevIter[i2*W + j2]] - p).norm2();
                        if (dist2 < minDist2) {
                            minDist2 = dist2;
                            bestSite = prevIter[i2*W + j2];
                        }
                    }
                }
                curIter[i*W + j] = bestSite;
                distance[i*W + j] = sqrt(minDist2);
            }
        }
    }

    bool intersection(const Vector& A, const Vector& B, const Vector& C, const Vector& D, Vector& P) {
        Vector AB = B - A;
        Vector CD = D - C;
        Vector AC = C - A;
        double det = - AB[0] * CD[1] + AB[1] * CD[0];
        if (std::abs(det) < std::numeric_limits<double>::epsilon()) {
            return false;
        }
        double t = (- AC[0] * CD[1] + AC[1] * CD[0]) / det;
        double u = (AB[0] * AC[1] - AB[1] * AC[0]) / det;
        if (t < 0 || t > 1 || u < 0 || u > 1) {
            return false;
        }
        P = A + AB * t;
        return true;
    }
    
    bool clip_by_line(Vector& C, Vector& u, Vector& v, std::vector<Vector>& clipped_vertices, double &max_dist) {
        bool clipped = false;
        double dist = 0;
        for (int i = 0; i < vertices.size(); ++i) {
            Vector prev = vertices[(i == 0) ? vertices.size()-1 : i-1];
            Vector vertex = vertices[i];
            Vector next = vertices[(i == vertices.size()-1) ? 0 : i+1];
            Vector P;
            if (intersection(vertex, C, u, v, P)) {
                if (intersection(vertex, prev, u, v, P)) {
                    clipped_vertices.push_back(P);
                    dist = std::max(dist, (C-P).norm());
                    clipped = true;
                }
                if (intersection(vertex, next, u, v, P)) {
                    clipped_vertices.push_back(P);
                    dist = std::max(dist, (C-P).norm());
                    clipped = true;
                }
            } else {
                clipped_vertices.push_back(vertex);
                dist = std::max(dist, (C-vertex).norm());
            }
        }
        if (dist > 0) {
            max_dist = dist;
        }
        return clipped;
    }

    void knn(size_t i, const PointCloud &cloud, size_t k, std::vector<size_t> &indices) {
        my_kd_tree_t index(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        index.buildIndex();
        indices.resize(k);
        std::vector<double> dists(k);
        nanoflann::KNNResultSet<double> resultSet(k);
        resultSet.init(&indices[0], &dists[0]);
        double query_pt[2] = {cloud.points[i][0], cloud.points[i][1]};
        index.findNeighbors(resultSet, &query_pt[0], {10});
    }

public:
    std::vector<Vector> vertices;
    std::vector<Triangle> triangulation;
    std::vector<Segment> voronoi;

    Polygon() {}
    Polygon(std::vector<Vector> vertices) : vertices(vertices) {}
    
    Vector compute_barycenter() {
        double C_x = 0, C_y = 0;
        if (vertices.size() >= 1) {
            for (int i = 0; i < vertices.size()-1; ++i) {
                C_x += (vertices[i][0] + vertices[i+1][0]) * (vertices[i][0]*vertices[i+1][1] - vertices[i+1][0]*vertices[i][1]);
                C_y += (vertices[i][1] + vertices[i+1][1]) * (vertices[i][0]*vertices[i+1][1] - vertices[i+1][0]*vertices[i][1]);
            }
            C_x += (vertices.back()[0] + vertices[0][0]) * (vertices.back()[0]*vertices[0][1] - vertices[0][0]*vertices.back()[1]);
            C_y += (vertices.back()[1] + vertices[0][1]) * (vertices.back()[0]*vertices[0][1] - vertices[0][0]*vertices.back()[1]);
            double A = area();
            C_x /= 6*A;
            C_y /= 6*A;
        }
        return Vector(C_x, C_y);
    }
    
    double area() {
        double A = 0;
        if (vertices.size() >= 1) {
            for (int i = 0; i < vertices.size()-1; ++i) {
                A += vertices[i][0]*vertices[i+1][1] - vertices[i+1][0]*vertices[i][1];
            }
            A += vertices.back()[0]*vertices[0][1] - vertices[0][0]*vertices.back()[1];
        }
        return A / 2;
    }
    
    void triangulate() {
        bound();
        for (int i = 0; i < vertices.size(); ++i) {
            int index = locate_triangle(vertices[i]);
            triangulation[index].valid = false;
            std::vector<int> indices = {index, index, index};
            std::vector<Vector> area = {triangulation[index].A, triangulation[index].B, triangulation[index].C};
            std::vector<int> neighbors = {triangulation[index].nc, triangulation[index].na, triangulation[index].nb};
            remove_area(vertices[i], indices, area, neighbors);
            int n = triangulation.size();
            int k = area.size();
            for (int j = 0; j < k; ++j) {
                Triangle t = Triangle(area[j], area[(j+1)%k], vertices[i]);
                t.na = n + ((j + 1) % k);
                t.nb = n + ((k + j - 1) % k);
                t.nc = neighbors[j];
                if (neighbors[j] != -1) {
                    triangulation[neighbors[j]].update_neighbor(indices[j], n+j);
                }
                triangulation.push_back(t);
            }
        }
        for (int i = 0; i < triangulation.size(); ++i) {
            if (triangulation[i].has_vertex(triangulation[0].A) || 
                triangulation[i].has_vertex(triangulation[0].B) || 
                triangulation[i].has_vertex(triangulation[0].C))  {
                triangulation[i].valid = false;
            }
        }
    }

    void compute_voronoi() {
        for (int i = 0; i < triangulation.size(); ++i) {
            if (triangulation[i].valid) {
                if (triangulation[i].na != -1) {
                    voronoi.push_back(Segment(triangulation[i].O, triangulation[triangulation[i].na].O));
                }
                if (triangulation[i].nb != -1) {
                    voronoi.push_back(Segment(triangulation[i].O, triangulation[triangulation[i].nb].O));
                }
                if (triangulation[i].nc != -1) {
                    voronoi.push_back(Segment(triangulation[i].O, triangulation[triangulation[i].nc].O));
                }
            }
        }
    }

    void compute_voronoi2(int W, int H, std::vector<int>& curIter, std::vector<double>& distance) {
        std::vector<int> prevIter(W*H, -1);
        curIter.resize(W*H);
        distance.resize(W * H, std::numeric_limits<double>::max());
        for (int i = 0; i < vertices.size(); i++) {
            prevIter[((int)vertices[i][1])*W + (int)(vertices[i][0])] = i;
        }
        for (int k = W/2; k >= 1; k/=2) {
            JFA(W, H, k, &prevIter[0], &curIter[0], distance.data());
            prevIter.swap(curIter);
        }
        if ((int)(log2(W))%2 == 1) {
            prevIter.swap(curIter);
        }
    }

    std::vector<Polygon> compute_voronoi3(int W, int H) {
        std::vector<Polygon> voronoi_cells(vertices.size());
        #pragma omp parallel for
        for (int i = 0; i < vertices.size(); ++i) {
            Polygon bounding_box = Polygon({Vector(0, 0), Vector(W, 0), Vector(W, H), Vector(0, H)});
            for (int j = 0; j < vertices.size(); ++j) {
                if (i != j) {
                    Vector M = (vertices[i] + vertices[j]) / 2;
                    Vector u = M + ortho(vertices[i] - vertices[j]) * sqrt(W*W + H*H);
                    Vector v = M - ortho(vertices[i] - vertices[j]) * sqrt(W*W + H*H);
                    std::vector<Vector> clipped_vertices;
                    double max_dist;
                    bounding_box.clip_by_line(vertices[i], u, v, clipped_vertices, max_dist);
                    bounding_box = Polygon(clipped_vertices);
                }
            }
            voronoi_cells[i] = bounding_box;
        }
        return voronoi_cells;
    }

    std::vector<Polygon> compute_voronoi4(int W, int H) {
        std::vector<Polygon> voronoi_cells(vertices.size());
        PointCloud cloud;
        cloud.points = vertices;
        #pragma omp parallel for
        for (int i = 0; i < vertices.size(); ++i) {
            Polygon bounding_box = Polygon({Vector(0, 0), Vector(W, 0), Vector(W, H), Vector(0, H)});
            double max_dist = sqrt(W*W + H*H);
            int k = vertices.size() / 8;
            bool clipped = false;
            while (!clipped && k < 2*vertices.size()) {
                std::vector<size_t> indices;
                knn(i, cloud, k, indices);
                for (int j = 1; j < k; ++j) {
                    if ((vertices[i] - vertices[indices[j]]).norm() < 2*max_dist) {
                        Vector M = (vertices[i] + vertices[indices[j]]) / 2;
                        Vector u = M + ortho(vertices[i] - vertices[indices[j]]) * sqrt(W*W + H*H);
                        Vector v = M - ortho(vertices[i] - vertices[indices[j]]) * sqrt(W*W + H*H);
                        std::vector<Vector> clipped_vertices;
                        if (bounding_box.clip_by_line(vertices[i], u, v, clipped_vertices, max_dist)) {
                            bounding_box = Polygon(clipped_vertices);
                        }
                    } else {
                        clipped = true;
                        break;
                    }
                }
                k *= 2;
            }
            voronoi_cells[i] = bounding_box;
        }
        return voronoi_cells;
    }

    bool lloyd_iteration(int W, int H) {
        bool convergence = true;
        std::vector<Polygon> voronoi_cells = compute_voronoi3(W, H);
        #pragma omp parallel for
        for (int i = 0; i < vertices.size(); ++i) {
            Vector barycenter = voronoi_cells[i].compute_barycenter();
            if (!(barycenter==vertices[i])) {
                vertices[i] = barycenter;
                convergence = false;
            }
        }
        return convergence;
    }

    std::vector<Polygon> compute_power_diagram(int W, int H, std::vector<double>& weights) {
        std::vector<Polygon> voronoi_cells(vertices.size());
        #pragma omp parallel for
        for (int i = 0; i < vertices.size(); ++i) {
            Polygon bounding_box = Polygon({Vector(0, 0), Vector(W, 0), Vector(W, H), Vector(0, H)});
            double max_dist = sqrt(W*W + H*H);
            for (int j = 0; j < vertices.size(); ++j) {
                if (i != j) {
                    Vector M = (vertices[i] + vertices[j]) / 2;
                    double scaled_weight_diff = (weights[i] - weights[j]);
                    M = M + (scaled_weight_diff * (vertices[j] - vertices[i]) / (2 * (vertices[i] - vertices[j]).norm2()));
                    Vector u = M + ortho(vertices[i] - vertices[j]) * sqrt(W*W + H*H);
                    Vector v = M - ortho(vertices[i] - vertices[j]) * sqrt(W*W + H*H);
                    std::vector<Vector> clipped_vertices;
                    bounding_box.clip_by_line(vertices[i], u, v, clipped_vertices, max_dist);
                    bounding_box = Polygon(clipped_vertices);
                }
            }
            voronoi_cells[i] = bounding_box;
        }
        return voronoi_cells;
    }
};


struct Data {
    Polygon polygon;
    std::vector<double> target_areas;
    std::vector<double> weights;
    double W, H;
};

class objective_function {
protected:
    lbfgsfloatval_t *m_x;
    Data data;

    double compute_integral(std::vector<Vector>& vertices, Vector P_i) {
        P_i = Vector(P_i[0], P_i[1]);
        double integral = 0;
        for (int i = 0; i < vertices.size(); ++i) {
            double x1 = vertices[i][0];
            double y1 = vertices[0][1];
            double x2 = (i < vertices.size()-1) ? vertices[i+1][0]: vertices[0][0];
            double y2 = (i < vertices.size()-1) ? vertices[i+1][1]: vertices[0][1];
            integral += (x1 * y2 - x2 * y1) * 
                        (x1 * x1 + x1 * x2 + x2 * x2 + y1 * y1 + y1 * y2 + y2 * y2 
                        - 4 * (P_i[0] * (x1 + x2) + P_i[1] * (y1 + y2)) + 6 * (P_i.norm2()));
        }
        integral /= 12;
        return integral;
    }

    static lbfgsfloatval_t _evaluate(void *instance, const lbfgsfloatval_t *x, lbfgsfloatval_t *g,
                                     const int n, const lbfgsfloatval_t step) {
        return reinterpret_cast<objective_function*>(instance)->evaluate(x, g, n, step);
    }

    lbfgsfloatval_t evaluate(const lbfgsfloatval_t *x, lbfgsfloatval_t *g,
                             const int n, const lbfgsfloatval_t step) {
        for (int i = 0; i < n; ++i) {
            data.weights[i] = x[i];
        }
        std::vector<Polygon> power_diagram_cells;
        power_diagram_cells = data.polygon.compute_power_diagram(data.W, data.H, data.weights);
        lbfgsfloatval_t fx = 0;
        for (int i = 0; i < n; ++i) {
            double area = power_diagram_cells[i].area();
            double integral = compute_integral(power_diagram_cells[i].vertices, data.polygon.vertices[i]);
            g[i] = data.target_areas[i] - area;
            fx += integral + data.weights[i] * (data.target_areas[i] - area);
        }
        return fx;
    }

public:
    objective_function(Data data) : m_x(NULL), data(data) {}

    virtual ~objective_function() {
        if (m_x != NULL) {
            lbfgs_free(m_x);
            m_x = NULL;
        }
    }

    std::vector<double> run(int N) {
        lbfgsfloatval_t fx;
        lbfgsfloatval_t *m_x = lbfgs_malloc(N);
        for (int i = 0;i < N; ++i) {
            m_x[i] = data.weights[i];
        }
        lbfgs(N, m_x, &fx, _evaluate, NULL, this, NULL);
        return data.weights;
    }
};
