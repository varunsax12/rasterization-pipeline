#ifndef __RASTER_H__
#define __RASTER_H__

#include <fstream> 
#include <string.h>
#include <iostream>
#include <stdint.h>

#include "fixed.h"

#define FIXED_POINT_SIZE 8

typedef cocogfx::Fixed<FIXED_POINT_SIZE> fixed_t;

struct rgb_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    rgb_t() : r(0), g(0), b(0) {}
    rgb_t(const int red, const int green, const int blue) : r(red), g(green), b(blue) {}
};

static rgb_t color_red(255, 0, 0);
static rgb_t color_green(0, 255, 0);
static rgb_t color_blue(0, 0, 255);

struct vertex_t {
    float x;
    float y;
    rgb_t color;
    vertex_t() : x(0), y(0)
    {
        color.r = 0; color.g = 0; color.b = 0;
    }
    vertex_t(const float inX, const float inY, const rgb_t inColor) : x(inX), y(inY), color(inColor) {}
};

struct vertex_fixed_t {
    fixed_t x;
    fixed_t y;
    rgb_t color;
};

class FrameBuffer {

private:
    uint32_t width_;
    uint32_t height_;
    rgb_t* pixels_;
public:
    FrameBuffer(uint32_t width, uint32_t height) : width_(width), height_(height)
    {
        pixels_ = new rgb_t[width * height];
        rgb_t black;
        for (uint32_t i = 0; i < height; ++i)
        {
            for (uint32_t j = 0; j < width; ++j)
            {
                this->pixels_[i * width + j] = black;
            }
        }
        //memset(pixels_, 0x0, width * height * sizeof(rgb_t));
    }

    void setPixelColor(vertex_fixed_t currentPixel)
    {
        uint32_t pixelLoc = (uint32_t)currentPixel.y * width_ + (uint32_t)currentPixel.x;
        pixels_[pixelLoc] = currentPixel.color;
    }

    ~FrameBuffer() {
        delete[] pixels_;
    }

    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }
    rgb_t* pixels() const { return pixels_; }

    void saveImage(const std::string& filename) {
        std::ofstream ofs;
        ofs.open(filename);
        ofs << "P6\n" << width_ << " " << height_ << "\n255\n";
        ofs.write((char*)pixels_, width_ * height_ * sizeof(rgb_t));
        ofs.close();
    }
};

/*
* Function to convert number to Fixed point format
* @param inVal -> value to convert
* @return Fixed point format of inVal
*/
template <typename T>
fixed_t convert_to_fixed(T inVal);

/*
* Function to generate vectors (A->B)
* @param A -> origin of vector
* @param B -> destination of vector
* @return Position type for vector
*/
vertex_fixed_t generate_vector(const vertex_fixed_t& A, const vertex_fixed_t& B);


/*
* Function to compute edge function value
* @param A -> vertex A
* @param B -> vectex B
* @param P -> position of pixel to check
* @return float of the edge value
*
* Logic: The cross product of (A->P)x(A->B) should be negative
* (IMP): Recomputes the edge value on each call
*/
fixed_t edge_bounds_value(const vertex_fixed_t& A, const vertex_fixed_t& B, const vertex_fixed_t& P);
/*
* Function to calculate the edge equations
* using matrices
*
* Reference: https://www.youtube.com/watch?v=RjGUV6iua10
*
* Matirx:
*   x   y   1
*   A.x A.y 1
*   B.x B.y 1
*
* Line equation: coeffX * x + coeffY * y + C
*/
void edge_calculation(const vertex_fixed_t A, const vertex_fixed_t B,
    fixed_t line_eq[3]);

/*
* Function to check point within edge (<0)
* @param line_eq: Line equation
* @param P: vertex to check
*/
bool pixel_within_edge(const fixed_t line_eq[3], const int x, const int y);

/*
* Function to check whether tile on the same side of the edge
* @param bottomLeft
* @param bottomRight
* @param topLeft
* @param topRight
* @param line_eq
* @param check -> result of checks
*
* Logic: To check whether all vertices on the same side of the edge
*/
void tile_edge_check(const int bottom, const int top, const int left, const int right, const fixed_t line_eq[3],
    std::pair<bool, bool>& check);

/*
* Function to implement simple rasterization for
* proof of concept
*
* Assumptions:
*   1. The coordinates V0,V1,V2 are in the required order V0->V1->V2 in counter-clockwise direction
*
* Supports 2 implementations:
*   1. Absolute calculation for each pixel
*   2. Incremental calculation using scan-tile
*
* @param edge1 -> edge equation of line 1
* @param edge2 -> edge equation of line 2
* @param edge3 -> edge equation of line 3
* @param bottomLeft -> bottom left of the tile
* @param topRight -> top right of the tile
* @param frameBuffer -> image to be updated
*/

void raster_implementation(const vertex_fixed_t edge1, const vertex_fixed_t edge2, const vertex_fixed_t edge3, const vertex_fixed_t bottomLeft,
    const vertex_fixed_t topLeft, FrameBuffer & frameBuffer);

/*
* Top draw call
* Assumptions:
*   1. The coordinates V0,V1,V2 are in the required order V0->V1->V2 in counter-clockwise direction
*/
void drawTriangle(const vertex_t& v0, const vertex_t& v1, const vertex_t& v2, FrameBuffer& frameBuffer);

#endif 