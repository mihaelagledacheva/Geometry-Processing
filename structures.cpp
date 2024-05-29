#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include "vector.cpp"

class Triangle {
private:
    void compute_barycenter() {
        double x = (A[0] + B[0] + C[0]) / 3;
        double y = (A[1] + B[1] + C[1]) / 3;
        G = Vector(x, y);
    }

    bool ccw(Vector& X, Vector& Y, Vector& Z) {
        return (Z[1]-X[1])*(Y[0]-X[0]) > (Y[1]-X[1])*(Z[0]-X[0]);
    }

    bool intersect(Vector& M, Vector& N, Vector& X, Vector& Y) {
        return ccw(M,X,Y) != ccw(N,X,Y) && ccw(M,N,X) != ccw(M,N,Y);
    }

public:
    Vector A, B, C;
    Vector G;
    bool valid;
    int na, nb, nc;

    Triangle(Vector& A, Vector& B, Vector& C) : A(A), B(B), C(C) {
        valid = true;
        na = nb = nc = -1;
        compute_barycenter();
    }

    bool has_vertex(Vector& P) {
        if (P == A || P == B || P == C) {
            return true;
        }
        return false;
    }

    bool in_circumcircle(Vector& P) {
        double D = 2 * (A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1]));
        double Ox = ((A[0] * A[0] + A[1] * A[1]) * (B[1] - C[1]) +
                     (B[0] * B[0] + B[1] * B[1]) * (C[1] - A[1]) +
                     (C[0] * C[0] + C[1] * C[1]) * (A[1] - B[1])) / D;
        double Oy = ((A[0] * A[0] + A[1] * A[1]) * (C[0] - B[0]) +
                     (B[0] * B[0] + B[1] * B[1]) * (A[0] - C[0]) +
                     (C[0] * C[0] + C[1] * C[1]) * (B[0] - A[0])) / D;
        double OA = (Vector(Ox, Oy) - A).norm();
        double OP = (Vector(Ox, Oy) - P).norm();
        return OP <= OA;
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

    void remove_area(Triangle& t, int index, Vector& v, std::vector<int>& indices, Polygon& area, std::vector<int>& neighbors) {
        t.valid = false;
        area.vertices.push_back(t.A);
        if (t.nc != -1) {
            if (triangulation[t.nc].in_circumcircle(v)) {
                triangulation[t.nc].valid = false;
                if (triangulation[t.nc].A == t.A) {
                    area.vertices.push_back(triangulation[t.nc].B);
                    neighbors.push_back(triangulation[t.nc].nc);
                    neighbors.push_back(triangulation[t.nc].na);
                } else if (triangulation[t.nc].B == t.A) {
                    area.vertices.push_back(triangulation[t.nc].C);
                    neighbors.push_back(triangulation[t.nc].na);
                    neighbors.push_back(triangulation[t.nc].nb);
                } else {
                    area.vertices.push_back(triangulation[t.nc].A);
                    neighbors.push_back(triangulation[t.nc].nb);
                    neighbors.push_back(triangulation[t.nc].nc);
                }
                indices.push_back(t.nc);
                indices.push_back(t.nc);
            } else {
                neighbors.push_back(t.nc);
                indices.push_back(index);
            }
        } else {
            neighbors.push_back(-1);
            indices.push_back(index);
        }
        area.vertices.push_back(t.B);
        if (t.na != -1) {
            if (triangulation[t.na].in_circumcircle(v)) {
                triangulation[t.na].valid = false;
                if (triangulation[t.na].A == t.B) {
                    area.vertices.push_back(triangulation[t.na].B);
                    neighbors.push_back(triangulation[t.na].nc);
                    neighbors.push_back(triangulation[t.na].na);
                } else if (triangulation[t.na].B == t.B) {
                    area.vertices.push_back(triangulation[t.na].C);
                    neighbors.push_back(triangulation[t.na].na);
                    neighbors.push_back(triangulation[t.na].nb);
                } else {
                    area.vertices.push_back(triangulation[t.na].A);
                    neighbors.push_back(triangulation[t.na].nb);
                    neighbors.push_back(triangulation[t.na].nc);

                }
                indices.push_back(t.na);
                indices.push_back(t.na);
            } else {
                neighbors.push_back(t.na);
                indices.push_back(index);
            }
        } else {
            neighbors.push_back(-1);
            indices.push_back(index);
        }
        area.vertices.push_back(t.C);
        if (t.nb != -1) {
            if (triangulation[t.nb].in_circumcircle(v)) {
                triangulation[t.nb].valid = false;
                if (triangulation[t.nb].A == t.C) {
                    area.vertices.push_back(triangulation[t.nb].B);
                    neighbors.push_back(triangulation[t.nb].nc);
                    neighbors.push_back(triangulation[t.nb].na);
                } else if (triangulation[t.nb].B == t.C) {
                    area.vertices.push_back(triangulation[t.nb].C);
                    neighbors.push_back(triangulation[t.nb].na);
                    neighbors.push_back(triangulation[t.nb].nb);
                } else {
                    area.vertices.push_back(triangulation[t.nb].A);
                    neighbors.push_back(triangulation[t.nb].nb);
                    neighbors.push_back(triangulation[t.nb].nc);
                }
                indices.push_back(t.nb);
                indices.push_back(t.nb);
            } else {
                neighbors.push_back(t.nb);
                indices.push_back(index);
            }
        } else {
            neighbors.push_back(-1);
            indices.push_back(index);
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

public:
    std::vector<Vector> vertices;
    std::vector<Triangle> triangulation;

    Polygon() {}
    Polygon(std::vector<Vector> vertices) : vertices(vertices) {}

    Polygon clip_by_line(Vector& u, Vector& v) {
        std::vector<Vector> cliped_vertices;
        Vector N = Vector(v[1]-u[1], u[0]-v[0]);
        for (int i = 0; i < vertices.size(); ++i) {
            Vector A = vertices[std::fmod(i-1, vertices.size())];
            Vector B = vertices[i];
            if (dot(B-u, N) <= 0) {
                if (dot(A-u, N) <= 0) {
                    cliped_vertices.push_back(B);
                } else {
                    Vector P = A + (B-A) * dot(u-A, N) / dot(B-A, N);
                    cliped_vertices.push_back(P);
                }
            } else if (dot(A-u, N) <= 0) {
                Vector P = A + (B-A) * dot(u-A, N) / dot(B-A, N);
                cliped_vertices.push_back(P);
            }
        }
        return Polygon(cliped_vertices);
    }
    
    void compute_voronoi(int W, int H, std::vector<int>& curIter, std::vector<double>& distance) {
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

    void triangulate() {
        bound();
        for (int i = 0; i < vertices.size(); ++i) {
            int index = locate_triangle(vertices[i]);
            std::vector<int> indices;
            Polygon area;
            std::vector<int> neighbors;
            remove_area(triangulation[index], index, vertices[i], indices, area, neighbors);
            int n = triangulation.size();
            int k = area.vertices.size();
            for (int j = 0; j < k; ++j) {
                Triangle t = Triangle(area.vertices[j], area.vertices[(j+1)%k], vertices[i]);
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
};
