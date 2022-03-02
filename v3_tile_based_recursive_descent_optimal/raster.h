#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include "fixed.h"

typedef cocogfx::Fixed<16> fixed16_t;

struct rgb_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

static rgb_t color_black{0,    0,    0};
static rgb_t color_red  {0xff, 0,    0};
static rgb_t color_green{0,    0xff, 0};
static rgb_t color_blue {0,    0,    0xff};
static rgb_t color_white{0xff, 0xff, 0xff};

template <typename T>
struct vec2d_t {
    T x;
    T y;
};

template <typename T>
struct vec3d_t {
    T x;
    T y;
    T z;
};

template <typename T>
struct rect_t {
    T left;
    T right;
    T top;    
    T bottom;
};

using vec2d_fx_t = vec2d_t<fixed16_t>;
using vec3d_fx_t = vec3d_t<fixed16_t>;
using rect_u_t   = rect_t<uint32_t>;

struct posf_t {
    float x;
    float y;
    float z;
    float w;
};

struct vertex_t {
    posf_t pos;
    rgb_t  color;
};

class FrameBuffer {
private:
    uint32_t width_;
    uint32_t height_;
    std::vector<rgb_t> pixels_;
    
public:
    FrameBuffer(uint32_t width, uint32_t height);
    ~FrameBuffer();

    uint32_t width() const;    
    uint32_t height() const;

    void setPixel(uint32_t x, uint32_t y, const rgb_t& color);

    const rgb_t* pixels() const;

    void saveImage(const std::string& filename);
};

class Rasterizer {
private:

    FrameBuffer* frameBuffer_;
    uint32_t    logTileSize_;
    uint32_t    logBlockSize_;
    uint32_t    max_level_;

    float setup(vec3d_fx_t edges[3], const posf_t& p0, const posf_t& p1, const posf_t& p2);

    void calcBoundingBox(rect_u_t* bbox, const posf_t& p0, const posf_t& p1, const posf_t& p2);

    void drawBlock(const vec3d_fx_t edges[3], uint32_t x, uint32_t y, fixed16_t e0, fixed16_t e1, fixed16_t e2);

    void drawTile(uint32_t level, const vec3d_fx_t edges[3], const fixed16_t extents[3], uint32_t x, uint32_t y, fixed16_t e0, fixed16_t e1, fixed16_t e2);

public:
    Rasterizer(FrameBuffer* frameBuffer, 
               uint32_t tileSize,
               uint32_t blockSize);

    ~Rasterizer();

    void drawTiangle(const vertex_t& v0, const vertex_t& v1, const vertex_t& v2);
};