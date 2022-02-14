#ifndef __RASTER_H__
#define __RASTER_H__

#include <fstream> 
#include <string.h>
#include <iostream>
#include <stdint.h>

#include "fixed.h"

#define FIXED_POINT_SIZE 8

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
    cocogfx::Fixed<FIXED_POINT_SIZE> x;
    cocogfx::Fixed<FIXED_POINT_SIZE> y;
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
cocogfx::Fixed<FIXED_POINT_SIZE> convert_to_fixed(T inVal);

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
cocogfx::Fixed<FIXED_POINT_SIZE> edge_bounds_value(const vertex_fixed_t& A, const vertex_fixed_t& B, const vertex_fixed_t& P);

/*
* Function to compute edge function
* @param A -> vertex A
* @param B -> vectex B
* @param P -> position of pixel to check
* @return bool whether the pixel is within the triangle
*
* Logic: The cross product of (A->P)x(A->B) should be negative
* (IMP): Recomputes the edge value on each call
*/
bool edge_bounds_absolute(const vertex_fixed_t& A, const vertex_fixed_t& B, const vertex_fixed_t& P);

/*
* Function to compute edge function
* @param A -> vertex A
* @param B -> vectex B
* @param P -> position of pixel to check
* @param edgeIndex -> edge number by index being computed of 3 in triangle
* @param firstCal -> whether the computation is being done for the first corner pixel
* @param colIncrementSign -> to decide whether to add or subtract the X direction increment
* @param increaseRow -> to tell program to increase the value in Y direction
* @return bool whether the pixel is within the triangle
*
* Logic: The cross product of (A->P)x(A->B) should be negative
* (IMP): Incrementally changes the edgeVal post first computation
*/
cocogfx::Fixed<FIXED_POINT_SIZE> edge_bounds_check_incremental(const vertex_fixed_t & A, const vertex_fixed_t & B,
    const vertex_fixed_t & P, int edgeIndex, bool firstCalc = true, int colIncrementSign = 1, bool increaseRow = false);

/*
* Function to generate vectors (A->B)
* @param A -> origin of vector
* @param B -> destination of vector
* @return Position type for vector
*/
vertex_fixed_t generate_vector(const vertex_fixed_t& A, const vertex_fixed_t& B);

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
* @param V0 -> position of 1st vertex
* @param V1 -> position of 2nd vertex
* @param V2 -> position of 3rd vertex
* @param rowMin -> minimum row of tile
* @param rowMax -> maximum row of tile
* @param colMin -> minimum column of tile
* @param colMax -> maxmimum column of tile
* @param frameBuffer -> image to be updated
*/

void raster_implementation(const vertex_t & V0, const vertex_t & V1, const vertex_t & V2, const int rowMin, const int rowMax,
    const int colMin, const int colMax, FrameBuffer & frameBuffer);

// wrapper to call raster_implementation
void drawTriangle(const vertex_t& v0, const vertex_t& v1, const vertex_t& v2, FrameBuffer& frameBuffer);

#endif 