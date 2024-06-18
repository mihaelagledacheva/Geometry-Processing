#include "svg.cpp"

const double REST_DENSITY = 1000;
const double GAS_CONSTANT = 2000;
const double R = 0.0457;
const double R2 = R * R;
const double VISCOSITY = 3.5;
const double TIME_STEP = 0.1;
const double GRAVITY = -9.8;
const double EPSILON = 0.0001;

struct Particle {
    Vector position;
    Vector velocity;
    Vector force;
    double mass;
    double density;
    double pressure;
};


class Simulation_Particles {
private:
    std::vector<Particle> particles;
    int W , H;

    void update() {
        for (auto& pi : particles) {
            pi.density = 0;
            for (const auto& pj : particles) {
                Vector rij = pi.position - pj.position;
                double r2 = rij.norm2();
                double poly6Kernel = (r2 > R2) ? 0 : 315 / (64 * M_PI * std::pow(R, 9)) * std::pow(R2 - r2, 3);
                pi.density += pi.mass * poly6Kernel;
            }
            pi.pressure = GAS_CONSTANT * (pi.density - REST_DENSITY);
        }

        for (auto& pi : particles) {
            Vector pressureForce = Vector(0, 0);
            Vector viscosityForce = Vector(0, 0);
            for (const auto& pj : particles) {
                if (&pi != &pj) {
                    Vector rij = pi.position - pj.position;
                    double r_length = rij.norm();
                    Vector gradient = (r_length > R || r_length < EPSILON) ? Vector(0, 0) :
                                    -45 * rij * (M_PI * std::pow(R, 6)) * std::pow(R - r_length, 2 * r_length);
                    pressureForce = pressureForce - gradient * (pi.pressure + pj.pressure) / (2 * pj.density);
                    double viscosityLaplacian = (r_length > R) ? 0 : 45 / (M_PI * std::pow(R, 6)) * (R - r_length);
                    viscosityForce = viscosityForce + (pj.velocity - pi.velocity) * (viscosityLaplacian / pj.density);
                }
            }
            pi.force = pressureForce + viscosityForce * VISCOSITY + Vector{0, GRAVITY} * pi.density;
        }

        for (auto& p : particles) {
            p.velocity = p.velocity + p.force * (TIME_STEP / p.density);
            p.position = p.position + p.velocity * TIME_STEP;
        }
        for (auto& p : particles) {
            p.velocity = p.velocity + p.force * (TIME_STEP / p.density);
            p.position = p.position + p.velocity * TIME_STEP;
            if (p.position[0] < 0 || p.position[0] > W) {
                p.position[0] = (p.position[0] < 0) ? 0 : W;
                p.velocity[0] *= -0.8;
            }
            if (p.position[1] < 0 || p.position[1] > H) {
                p.position[1] = (p.position[1] < 0) ? 0 : H;
                p.velocity[1] *= -0.8;
            }
        }
    }

    void save_frame(const std::string& filename) {
        std::ofstream file(filename);
        file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        file << "<rect width=\"100%\" height=\"100%\" fill=\"white\" />\n";
        for (const auto& p : particles) {
            file << "<circle cx=\"" << p.position[0] << "\" cy=\"" << H-p.position[1] << "\" r=\"" << p.mass << "\" fill=\"black\" />\n";
        }
        file << "</svg>\n";
        file.close();
    }

public:
    Simulation_Particles(std::vector<Vector> points, int W, int H) : particles(points.size()), W(W), H(H) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(-1.0, 1.0);
        for (int i = 0; i < particles.size(); ++i) {
            particles[i].position = points[i];
            particles[i].velocity = Vector(dis(gen), dis(gen));
            particles[i].force = Vector(dis(gen), dis(gen));
            particles[i].mass = (1 + dis(gen)) * 5;
            particles[i].density = (1 + dis(gen)) / 2;
            particles[i].pressure = (1 + dis(gen)) / 2;
        }
    }

    void run(const std::string& filename, int frames=200) {
        for (int i = 0; i < frames; ++i) {
            update();
            save_frame(filename + "_" + std::to_string(i) + ".svg");
        }
    }
};


class Simulation_Laguerre {
private:
    std::vector<Vector> points;
    std::vector<Vector> velocities;
    std::vector<double> masses;
    double fluid_volume;
    int N;
    int W, H;
    std::vector<Polygon> voronoi;
    
    void update(std::vector<double> weights) {
        Polygon polygon = Polygon(points);
        voronoi = polygon.compute_power_diagram(W, H, weights);
        std::vector<Vector> points_prime(points.size());
        std::vector<Vector> velocities_prime(N);
        for (int i = 0; i < N; ++i) {
            Vector F_spring = (voronoi[i].compute_barycenter() - points[i]);
            Vector F = F_spring + masses[i] * Vector(0, -9.8);
            velocities_prime[i] = velocities[i] + TIME_STEP * F / masses[i];
            points_prime[i] = points[i] + TIME_STEP * velocities[i];
            if (points_prime[i][0] < 0 || points_prime[i][0] > W) {
                points_prime[i][0] = (points_prime[i][0] < 0) ? 0 : W;
                velocities_prime[i][0] *= -0.8;
            }
            if (points_prime[i][1] < 0 || points_prime[i][1] > H) {
                points_prime[i][1] = (points_prime[i][1] < 0) ? 0 : H;
                velocities_prime[i][1] *= -0.8;
            }
        }
        for (int i = N; i < points.size(); ++i) {
            points_prime[i] = points[i];
        }
        points = points_prime;
        velocities = velocities_prime;
    }

    void save_frame(const std::string& filename) {
        std::ofstream file(filename);
        file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        file << "<rect width=\"100%\" height=\"100%\" fill=\"white\" />\n";
        for (int i = 0; i < N; ++i) {
            file << "<polygon points=\"";
            for (const auto& vertex : voronoi[i].vertices) {
                file << vertex[0] << "," << H-vertex[1] << " ";
            }
            file << "\" stroke=\"black\" fill=\"blue\" stroke-width=\"1\" />\n";
        }
        for (int i = N; i < points.size(); ++i) {
            file << "<polygon points=\"";
            for (const auto& vertex : voronoi[i].vertices) {
                file << vertex[0] << "," << H-vertex[1] << " ";
            }
            file << "\" stroke=\"black\" fill=\"white\" stroke-width=\"1\" />\n";
        }
        file << "</svg>";
        file.close();
    }

public:
    Simulation_Laguerre(std::vector<Vector> points, 
                        std::vector<Vector> velocities, 
                        std::vector<double> masses,
                        double fluid_volume, int N, 
                        int W, int H) : points(points), velocities(velocities), masses(masses), 
                                        fluid_volume(fluid_volume), N(N), W(W), H(H) {}

    void run(const std::string& filename, int frames=200) {
        std::vector<double> weights(points.size(), 1);
        std::vector<double> target_areas(points.size());
        for (int i = 0; i < N; ++i) {
            target_areas[i] = fluid_volume / N;
        }
        for (int i = N; i < points.size(); ++i) {
            target_areas[i] = (W * H - fluid_volume) / (points.size() - N);
        }
        Data data;
        data.polygon = points;
        data.target_areas = target_areas;
        data.weights = weights;
        data.W = W;
        data.H = H;
        objective_function obj(data);
        weights = obj.run(points.size());

        for (int i = 0; i < frames; ++i) {
            update(weights);
            save_frame(filename + "_" + std::to_string(i) + ".svg");
        }
    }
};


class Simulation_Voronoi {
private:
    std::vector<Vector> points;
    std::vector<Vector> velocities;
    std::vector<double> masses;
    int N;
    int W, H;
    std::vector<Polygon> voronoi;
    
    void update() {
        Polygon polygon = Polygon(points);
        polygon.lloyd_iteration(W, H);
        voronoi = polygon.compute_voronoi3(W, H);
        std::vector<Vector> points_prime(points.size());
        std::vector<Vector> velocities_prime(N);
        for (int i = 0; i < N; ++i) {
            Vector F_spring = (voronoi[i].compute_barycenter() - points[i]);
            Vector F = F_spring + masses[i] * Vector(0, -9.8);
            velocities_prime[i] = velocities[i] + TIME_STEP * F / masses[i];
            points_prime[i] = points[i] + TIME_STEP * velocities[i];
            if (points_prime[i][0] < 0 || points_prime[i][0] > W) {
                points_prime[i][0] = (points_prime[i][0] < 0) ? 0 : W;
                velocities_prime[i][0] *= -0.8;
            }
            if (points_prime[i][1] < 0 || points_prime[i][1] > H) {
                points_prime[i][1] = (points_prime[i][1] < 0) ? 0 : H;
                velocities_prime[i][1] *= -0.8;
            }
        }
        for (int i = N; i < points.size(); ++i) {
            points_prime[i] = points[i];
        }
        points = points_prime;
        velocities = velocities_prime;
    }

    void save_frame_1(const std::string& filename) {
        std::ofstream file(filename);
        file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        file << "<rect width=\"100%\" height=\"100%\" fill=\"white\" />\n";
        for (int i = 0; i < N; ++i) {
            file << "<polygon points=\"";
            for (const auto& vertex : voronoi[i].vertices) {
                file << vertex[0] << "," << H-vertex[1] << " ";
            }
            file << "\" stroke=\"black\" fill=\"blue\" stroke-width=\"1\" />\n";
        }
        for (int i = N; i < points.size(); ++i) {
            file << "<polygon points=\"";
            for (const auto& vertex : voronoi[i].vertices) {
                file << vertex[0] << "," << H-vertex[1] << " ";
            }
            file << "\" stroke=\"black\" fill=\"white\" stroke-width=\"1\" />\n";
        }
        file << "</svg>";
        file.close();
    }

    void save_frame_2(const std::string& filename) {
        std::ofstream file(filename);
        file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        file << "<rect width=\"100%\" height=\"100%\" fill=\"white\" />\n";
        for (int i = 0; i < N; ++i) {
            file << "<circle cx=\"" << points[i][0] << "\" cy=\"" << H-points[i][1] << "\" r=\"" << masses[i] << "\" fill=\"black\" />\n";
        }
        file << "</svg>";
        file.close();
    }

public:
    Simulation_Voronoi(std::vector<Vector> points, 
                       std::vector<Vector> velocities, 
                       std::vector<double> masses,
                       int N, int W, int H) : points(points), velocities(velocities), masses(masses), 
                                              N(N), W(W), H(H) {}

    void run_1(const std::string& filename, int frames=200) {
        for (int i = 0; i < frames; ++i) {
            update();
            save_frame_1(filename + "_" + std::to_string(i) + ".svg");
        }
    }

    void run_2(const std::string& filename, int frames=200) {
        for (int i = 0; i < frames; ++i) {
            update();
            save_frame_2(filename + "_" + std::to_string(i) + ".svg");
        }
    }
};
