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

void save_svg_animated(const std::vector<Polygon> &polygons, std::string filename, int frameid, int nbframes) {
    FILE* f;
    if (frameid == 0) {
        f = fopen(filename.c_str(), "w+");
        fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");
        fprintf(f, "<g>\n");
    } else {
        f = fopen(filename.c_str(), "a+");
    }
    fprintf(f, "<g>\n");
    for (int i = 0; i < polygons.size(); i++) {
        fprintf(f, "<polygon points = \"");
        for (int j = 0; j < polygons[i].vertices.size(); j++) {
            fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000-polygons[i].vertices[j][1] * 1000));
        }
        fprintf(f, "\"\nfill = \"none\" stroke = \"black\"/>\n");
    }
    fprintf(f, "<animate\n");
    fprintf(f, "    id = \"frame%u\"\n", frameid);
    fprintf(f, "    attributeName = \"display\"\n");
    fprintf(f, "    values = \"");
    for (int j = 0; j < nbframes; j++) {
        if (frameid == j) {
            fprintf(f, "inline");
        } else {
            fprintf(f, "none");
        }
        fprintf(f, ";");
    }
    fprintf(f, "none\"\n    keyTimes = \"");
    for (int j = 0; j < nbframes; j++) {
        fprintf(f, "%2.3f", j / (double)(nbframes));
        fprintf(f, ";");
    }
    fprintf(f, "1\"\n   dur = \"5s\"\n");
    fprintf(f, "    begin = \"0s\"\n");
    fprintf(f, "    repeatCount = \"indefinite\"/>\n");
    fprintf(f, "</g>\n");
    if (frameid == nbframes - 1) {
        fprintf(f, "</g>\n");
        fprintf(f, "</svg>\n");
    }
    fclose(f);
}
