#include <fstream>
#include "structures.cpp"

void save_ints_svg(const std::vector<int>& ints, int W, int H, const std::string& filename) {
    std::ofstream file(filename);
    file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int site = ints[y * W + x];
            std::string color = "rgb(" + std::to_string(site * 30 % 256) + "," +
                                         std::to_string(site * 60 % 256) + "," +
                                         std::to_string(site * 90 % 256) + ")";
            file << "<rect x=\"" << x << "\" y=\"" << y << "\" width=\"1\" height=\"1\" fill=\"" << color << "\" />\n";
        }
    }
    file << "</svg>";
    file.close();
}

void save_doubles_svg(const std::vector<double>& doubles, int W, int H, const std::string& filename) {
    std::ofstream file(filename);
    double max_double = *std::max_element(doubles.begin(), doubles.end());
    file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            double dist = doubles[y*W + x];
            int gray = static_cast<int>(255 * dist / max_double);
            std::string color = "rgb(" + std::to_string(gray) + "," +
                                         std::to_string(gray) + "," +
                                         std::to_string(gray) + ")";
            file << "<rect x=\"" << x << "\" y=\"" << y << "\" width=\"1\" height=\"1\" fill=\"" << color << "\" />\n";
        }
    }
    file << "</svg>";
    file.close();
}

void save_segments_svg(const std::vector<Segment>& segments, int W, int H, const std::string& filename) {
    std::ofstream file(filename);
    file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    for (const auto& segment : segments) {
        file << "<line x1=\"" << segment.A[0] << "\" y1=\"" << segment.A[1]
             << "\" x2=\"" << segment.B[0] << "\" y2=\"" << segment.B[1]
             << "\" stroke=\"black\" stroke-width=\"1\" />\n";
    }
    file << "</svg>";
    file.close();
}

void save_polygons_svg(const std::vector<Polygon>& polygons, int W, int H, const std::string& filename) {
    std::ofstream file(filename);
    file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    for (const auto& polygon : polygons) {
        file << "<polygon points=\"";
        for (const auto& vertex : polygon.vertices) {
            file << vertex[0] << "," << vertex[1] << " ";
        }
        file << "\" stroke=\"black\" fill=\"none\" stroke-width=\"1\" />\n";
    }
    file << "</svg>";
    file.close();
}

void save_centers_and_polygons_svg(const std::vector<Vector>& centers, const std::vector<Polygon>& polygons, int W, int H, const std::string& filename) {
    std::ofstream file(filename);
    file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    for (const auto& polygon : polygons) {
        file << "<polygon points=\"";
        for (const auto& vertex : polygon.vertices) {
            file << vertex[0] << "," << vertex[1] << " ";
        }
        file << "\" stroke=\"black\" fill=\"none\" stroke-width=\"1\" />\n";
    }
    for (const auto& center : centers) {
        file << "<circle cx=\"" << center[0] << "\" cy=\"" << center[1] << "\" r=\"2\" fill=\"red\" />\n";
    }
    file << "</svg>";
    file.close();
}

void save_circles_and_polygons_svg(const std::vector<Vector>& centers, const std::vector<double>& radii, 
                                   const std::vector<Polygon>& polygons, int W, int H, const std::string& filename) {
    std::ofstream file(filename);
    file << "<svg width=\"" << W << "\" height=\"" << H << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    for (const auto& polygon : polygons) {
        file << "<polygon points=\"";
        for (const auto& vertex : polygon.vertices) {
            file << vertex[0] << "," << vertex[1] << " ";
        }
        file << "\" stroke=\"black\" fill=\"none\" stroke-width=\"1\" />\n";
    }
    for (size_t i = 0; i < centers.size(); ++i) {
        file << "<circle cx=\"" << centers[i][0] << "\" cy=\"" << centers[i][1] << "\" r=\"" << radii[i]
           << "\" fill=\"blue\" fill-opacity=\"0.5\" stroke=\"black\" stroke-width=\"1\" />\n";
    }
    file << "</svg>";
    file.close();
}
