/*
 * Example: visualize_projection
 * Loads an image, projects a lidar point (Pose) into the camera image using
 * provided extrinsic and intrinsic matrices, draws a small marker, and saves
 * the output image to disk.
 *
 * This example embeds STB image reader/writer as single-file implementations
 * so it does not add external dependencies.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <iostream>
#include <cmath>

#include "helpers.hpp"
#include "transformers.hpp"

using namespace AdasTools;

// Very small PPM (P6) image loader/writer to avoid external deps.
static unsigned char *load_ppm(const char *path, int *out_w, int *out_h)
{
    FILE *f = std::fopen(path, "rb");
    if (!f) return nullptr;
    char magic[3] = {0};
    if (fscanf(f, "%2s", magic) != 1) { fclose(f); return nullptr; }
    if (std::strcmp(magic, "P6") != 0) { fclose(f); return nullptr; }
    int w, h, maxv;
    // skip whitespace/comments
    int c = fgetc(f);
    while (c == '\n' || c == '\r' || c == ' ' || c == '\t') c = fgetc(f);
    ungetc(c, f);
    // read width height maxval
    if (fscanf(f, "%d %d %d", &w, &h, &maxv) != 3) { fclose(f); return nullptr; }
    // consume single whitespace char after header
    fgetc(f);
    size_t sz = (size_t)w * (size_t)h * 3;
    unsigned char *buf = (unsigned char*)std::malloc(sz);
    if (!buf) { fclose(f); return nullptr; }
    size_t read = fread(buf, 1, sz, f);
    fclose(f);
    if (read != sz) { std::free(buf); return nullptr; }
    *out_w = w; *out_h = h;
    return buf;
}

static bool write_ppm(const char *path, const unsigned char *data, int w, int h)
{
    FILE *f = std::fopen(path, "wb");
    if (!f) return false;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    size_t sz = (size_t)w * (size_t)h * 3;
    size_t wrote = fwrite(data, 1, sz, f);
    fclose(f);
    return wrote == sz;
}

static bool draw_marker_and_save(const char *inpath, const char *outpath,
                                 const Pose &p_local,
                                 const double extrinsic[16],
                                 const double K[9])
{
    int w,h;
    unsigned char *data = load_ppm(inpath, &w, &h);
    if (!data) {
        std::cerr << "Failed to load PPM image: " << inpath << "\n";
        return false;
    }

    // Project point
    Pose pix = projectPointCamera(p_local, extrinsic, K);

    // Check depth
    if (pix.z <= 0.0) {
        std::cerr << "Point is behind the camera (depth=" << pix.z << ").\n";
        std::free(data);
        return false;
    }

    int u = static_cast<int>(std::round(pix.x));
    int v = static_cast<int>(std::round(pix.y));

    // Draw a simple 5x5 red square center at (u,v)
    const int radius = 2;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int yy = v + dy;
            int xx = u + dx;
            if (xx < 0 || xx >= w || yy < 0 || yy >= h) continue;
            int idx = (yy * w + xx) * 3;
            data[idx+0] = 255; // R
            data[idx+1] = 0;   // G
            data[idx+2] = 0;   // B
        }
    }

    bool ok = write_ppm(outpath, data, w, h);
    std::free(data);
    if (!ok) {
        std::cerr << "Failed to write output image: " << outpath << "\n";
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: visualize_projection <input_image> <output_image>\n";
        std::cout << "This example uses a hard-coded lidar pose and camera intrinsics/extrinsics for demo.\n";
        return 0;
    }

    const char *inpath = argv[1];
    const char *outpath = argv[2];

    // Hard-coded lidar local point (Pose: x,y,z,roll,pitch,yaw)
    Pose p_local = {1.0, 0.5, 0.2, 0.0, 0.0, 0.0};

    // For this example we construct a simple extrinsic that maps lidar->camera
    // We'll use identity extrinsic (point already expressed in camera coords)
    double extrinsic[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    // Simple intrinsics: fx=fy=800, cx=320, cy=240
    double K[9] = {800.0, 0.0, 320.0,
                   0.0, 800.0, 240.0,
                   0.0, 0.0, 1.0};

    bool ok = draw_marker_and_save(inpath, outpath, p_local, extrinsic, K);
    if (!ok) return 2;

    std::cout << "Wrote " << outpath << " with projected marker.\n";
    return 0;
}
