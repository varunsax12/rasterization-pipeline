#include "raster.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream> 
#include <string.h>
#include <iostream>
#include <vector>

using namespace cocogfx;

static fixed16_t fxZero(0);
static fixed16_t fxHalf(0.5f);

template <typename R, uint32_t F1, uint32_t F2, typename T1, typename T2>
inline R Mul(Fixed<F1, T1> lhs, Fixed<F2, T2> rhs) {
  assert((static_cast<int64_t>(lhs.data()) * rhs.data()) == (lhs.data() * rhs.data()));
  int FRAC = Fixed<F1, T1>::FRAC + Fixed<F2, T2>::FRAC - R::FRAC;
  return R::make((lhs.data() * rhs.data()) >> FRAC);
}

constexpr uint32_t count_leading_zeros(uint32_t value) {
  return value ? __builtin_clz(value) : 32;
}

constexpr uint32_t log2ceil(uint32_t value) {
  return 32 - count_leading_zeros(value - 1);
}

constexpr bool ispow2(uint32_t value) {
  return value && !(value & (value - 1));
}

///////////////////////////////////////////////////////////////////////////////

FrameBuffer::FrameBuffer(uint32_t width, uint32_t height) 
    : width_(width)
    , height_(height)
    , pixels_(width * height) {
    for (uint32_t i = 0; i < height; ++i) {
        for (uint32_t j = 0; j < width; ++j) {
            pixels_.at(i * width + j) = color_black;
        }
    }
}

FrameBuffer::~FrameBuffer() {
    //--
}

void FrameBuffer::setPixel(uint32_t x, uint32_t y, const rgb_t& color) {
    uint32_t index = y * width_ + x;
    pixels_.at(index) = color;
}

uint32_t FrameBuffer::width() const { 
    return width_; 
}

uint32_t FrameBuffer::height() const { 
    return height_; 
}

const rgb_t* FrameBuffer::pixels() const { 
    return pixels_.data(); 
}

void FrameBuffer::saveImage(const std::string& filename) {
    std::ofstream ofs;
    ofs.open(filename);
    ofs << "P6\n" << width_ << " " << height_ << "\n255\n";
    // image is stored in memory from bottom to top.
    uint32_t pitch = width_ * sizeof(rgb_t);
    for (int32_t row = height_-1; row >= 0; --row) {
        ofs.write((char*)pixels_.data() + row * pitch, pitch);
    }    
    ofs.close();
}

///////////////////////////////////////////////////////////////////////////////

// Convert position from clip to 2D homogenous device space
static void clip_to_2DH(posf_t* out, const posf_t& in, uint32_t width, uint32_t height) {    
    out->x = width * (in.x + in.w) / 2;
    out->y = height * (in.y + in.w) / 2;
    out->z = in.z;
    out->w = in.w;
}

// Convert position from clip to screen space
static void clip_to_screen(posf_t* out, const posf_t& in, uint32_t width, uint32_t height) {
    // Clip to NDC (normalized device-space coordinates)
    auto rhw   = 1.0f / in.w;
    auto ndc_x = in.x * rhw;
    auto ndc_y = in.y * rhw;
    auto ndc_z = in.z * rhw;
    auto ndc_w = in.w;

    // NDC to screen
    out->x = width * (ndc_x + 1.0f) / 2;
    out->y = height * (ndc_y + 1.0f) / 2;
    out->z = ndc_z;
    out->w = ndc_w;
}

// Evaluate edge function
static fixed16_t evalEdgeFunction(const vec3d_fx_t& e, uint32_t x, uint32_t y) {
    return (e.x * x) + (e.y * y) + e.z;
}

// Calculate the edge extents for tile corners
static fixed16_t calcEdgeExtents(const vec3d_fx_t& e, uint32_t logTileSize) {
    vec2d_fx_t corners[4] = {{fxZero, fxZero},  // 00
                             {e.x,    fxZero},  // 10
                             {fxZero, e.y},     // 01
                             {e.x,    e.y}};    // 11
    auto i = (e.y >= fxZero) ? ((e.x >= fxZero) ? 3 : 2) : (e.x >= fxZero) ? 1 : 0;
    return (corners[i].x + corners[i].y) << logTileSize;
}

///////////////////////////////////////////////////////////////////////////////

Rasterizer::Rasterizer(FrameBuffer* frameBuffer, 
                       uint32_t tileSize,  
                       uint32_t blockSize)
    : frameBuffer_(frameBuffer)
    , logTileSize_(log2ceil(tileSize))
    , logBlockSize_(log2ceil(blockSize))
{
    assert(ispow2(tileSize));
    assert(ispow2(blockSize));
    assert(0 == (tileSize % blockSize));
    assert(0 == (frameBuffer->width() % tileSize));
    assert(0 == (frameBuffer->width() % tileSize));
}

Rasterizer::~Rasterizer() {
    //--
}

void Rasterizer::drawTiangle(const vertex_t& v0, const vertex_t& v1, const vertex_t& v2) {
    // Perform triangle setup
    vec3d_fx_t edges[3];
    auto det = this->setup(edges, v0.pos, v1.pos, v2.pos);
    if (det <= 0) {
        // reject back-facing or degenerate triangles
        return;
    } 

    // Calculate triangle bounding box
    rect_u_t bbox;
    this->calcBoundingBox(&bbox, v0.pos, v1.pos, v2.pos);

    // Calculate min/max tile positions
    auto tileSize = 1 << logTileSize_;
    auto minTileX = bbox.left >> logTileSize_;
    auto minTileY = bbox.top >> logTileSize_;
    auto maxTileX = (bbox.right + tileSize - 1) >> logTileSize_;
    auto maxTileY = (bbox.bottom + tileSize - 1) >> logTileSize_;

    // Starting tile coordinates
    auto X = minTileX << logTileSize_;
    auto Y = minTileY << logTileSize_;

    // Add tile corner edge offsets
    fixed16_t extents[3];
    extents[0] = calcEdgeExtents(edges[0], logTileSize_);
    extents[1] = calcEdgeExtents(edges[1], logTileSize_);
    extents[2] = calcEdgeExtents(edges[2], logTileSize_);

    // Evaluate edge equation for the starting tile
    auto E0 = evalEdgeFunction(edges[0], X, Y);
    auto E1 = evalEdgeFunction(edges[1], X, Y);
    auto E2 = evalEdgeFunction(edges[2], X, Y);

    /*printf("X=%d, Y=%d, E0=%f, E1=%f, E2=%f, x0=%f, x1=%f, x2=%f\n", 
             X, Y, float(E0), float(E1), float(E2), float(extents[0]), float(extents[1]), float(extents[2]));*/

    // Raster each covered tile
    for (uint32_t j = minTileY; j < maxTileY; ++j) {
        auto e0 = E0;
        auto e1 = E1;
        auto e2 = E2;
        for (uint32_t i = minTileX; i < maxTileX; ++i) {
            // draw the tile
            auto tx = i << logTileSize_;
            auto ty = j << logTileSize_;
            this->drawTile(0, edges, extents, tx, ty, e0, e1, e2);

            // update edge equation x components
            e0 += edges[0].x << logTileSize_;
            e1 += edges[1].x << logTileSize_;
            e2 += edges[2].x << logTileSize_;
        }
        // update edge equation y components
        E0 += edges[0].y << logTileSize_;
        E1 += edges[1].y << logTileSize_;
        E2 += edges[2].y << logTileSize_;
    }
}

void Rasterizer::drawTile(uint32_t level, 
                          const vec3d_fx_t edges[3], 
                          const fixed16_t extents[3],                           
                          uint32_t x, 
                          uint32_t y, 
                          fixed16_t e0, 
                          fixed16_t e1, 
                          fixed16_t e2) {
    // check if tile overlap triangle    
    if ((e0 + extents[0] >> level) < fxZero 
     || (e1 + extents[1] >> level) < fxZero
     || (e2 + extents[2] >> level) < fxZero)
        return; 
    
    ++level;
    auto logSubTileSize = logTileSize_ - level;   
    auto subTileSize = 1 << logSubTileSize;
    if (logSubTileSize >= logBlockSize_) {
        // draw top-left subtile
        {
            auto sx  = x;
            auto sy  = y;
            auto se0 = e0;
            auto se1 = e1;
            auto se2 = e2;
            this->drawTile(level, edges, extents, sx, sy, se0, se1, se2);
        }

        // draw top-right subtile
        {
            auto sx  = x + subTileSize;
            auto sy  = y;
            auto se0 = e0 + (edges[0].x << logSubTileSize);
            auto se1 = e1 + (edges[1].x << logSubTileSize);
            auto se2 = e2 + (edges[2].x << logSubTileSize);
            this->drawTile(level, edges, extents, sx, sy, se0, se1, se2);
        }

        // draw bottom-left subtile
        {
            auto sx  = x;
            auto sy  = y + subTileSize;
            auto se0 = e0 + (edges[0].y << logSubTileSize);
            auto se1 = e1 + (edges[1].y << logSubTileSize);
            auto se2 = e2 + (edges[2].y << logSubTileSize);
            this->drawTile(level, edges, extents, sx, sy, se0, se1, se2);
        }

        // draw bottom-right subtile
        {
            auto sx  = x + subTileSize;
            auto sy  = y + subTileSize;
            auto se0 = e0 + (edges[0].x << logSubTileSize) + (edges[0].y << logSubTileSize);
            auto se1 = e1 + (edges[1].x << logSubTileSize) + (edges[1].y << logSubTileSize);
            auto se2 = e2 + (edges[2].x << logSubTileSize) + (edges[2].y << logSubTileSize);
            this->drawTile(level, edges, extents, sx, sy, se0, se1, se2);
        }
    } else {
        // draw low-level block
        this->drawBlock(edges, x, y, e0, e1, e2);
    }
}

void Rasterizer::drawBlock(const vec3d_fx_t edges[3], uint32_t x, uint32_t y, fixed16_t e0, fixed16_t e1, fixed16_t e2) {
    auto blockSize = 1 << logBlockSize_;

    for (uint32_t j = 0; j < blockSize; ++j) {
        auto be0 = e0;
        auto be1 = e1;
        auto be2 = e2;
        for (uint32_t i = 0; i < blockSize; ++i) {
            auto bx = x + i;
            auto by = y + j;

            // test if pixel overlaps triangle
            if (be0 >= fxZero && be1 >= fxZero && be2 >= fxZero) {
                frameBuffer_->setPixel(bx, by, color_red);
            }

            // update edge equation x components
            be0 += edges[0].x;
            be1 += edges[1].x;
            be2 += edges[2].x;
        }
        // update edge equation y components
        e0 += edges[0].y;
        e1 += edges[1].y;
        e2 += edges[2].y;
    }
}

float Rasterizer::setup(vec3d_fx_t edges[3], const posf_t& p0, const posf_t& p1, const posf_t& p2) {    
    auto width = frameBuffer_->width();
    auto height = frameBuffer_->height();

    // Convert position from clip to 2D homogenous device space
    posf_t v0, v1, v2;
    clip_to_2DH(&v0, p0, width, height);
    clip_to_2DH(&v1, p1, width, height);
    clip_to_2DH(&v2, p2, width, height);

    // Calculate edge equation matrix
    auto a0 = (v1.y * v2.w) - (v2.y * v1.w);
    auto a1 = (v2.y * v0.w) - (v0.y * v2.w);
    auto a2 = (v0.y * v1.w) - (v1.y * v0.w);

    auto b0 = (v2.x * v1.w) - (v1.x * v2.w);
    auto b1 = (v0.x * v2.w) - (v2.x * v0.w);
    auto b2 = (v1.x * v0.w) - (v0.x * v1.w);

    auto c0 = (v1.x * v2.y) - (v2.x * v1.y);
    auto c1 = (v2.x * v0.y) - (v0.x * v2.y);
    auto c2 = (v0.x * v1.y) - (v1.x * v0.y);

    // Normalize the matrix
    #define NORMALIZE(x, y, z) { auto t = 1.0 / (std::abs(x) + std::abs(y) + std::abs(z)); x *= t; y *= t; z *= t; }
    NORMALIZE(a0, b0, c0)
    NORMALIZE(a1, b1, c1)
    NORMALIZE(a2, b2, c2)

    // Convert the edge equation to fixedpoint
    edges[0] = {fixed16_t(a0), fixed16_t(b0), fixed16_t(c0)};
    edges[1] = {fixed16_t(a1), fixed16_t(b1), fixed16_t(c1)};
    edges[2] = {fixed16_t(a2), fixed16_t(b2), fixed16_t(c2)};

    /*printf("E0.x=%f, E0.y=%f, E0.z=%f, E1.x=%f, E1.y=%f, E1.z=%f, E2.x=%f, E2.y=%f, E2.z=%f\n", 
        float(edges[0].x), float(edges[0].y), float(edges[0].z),
        float(edges[1].x), float(edges[1].y), float(edges[1].z),
        float(edges[2].x), float(edges[2].y), float(edges[2].z));*/

    auto det = c0 * v0.w + c1 * v1.w + c2 * v2.w;

    return det;
}

void Rasterizer::calcBoundingBox(rect_u_t* bbox, const posf_t& p0, const posf_t& p1, const posf_t& p2) {
    auto width = frameBuffer_->width();
    auto height = frameBuffer_->height();

    // Convert position from clip to screen space
    posf_t v0, v1, v2;
    clip_to_screen(&v0, p0, width, height);
    clip_to_screen(&v1, p1, width, height);
    clip_to_screen(&v2, p2, width, height);

    // Find min/max 
    auto xmin = static_cast<int32_t>(std::min(v0.x, std::min(v1.x, v2.x)));
    auto xmax = static_cast<int32_t>(std::max(v0.x, std::max(v1.x, v2.x)));
    auto ymin = static_cast<int32_t>(std::min(v0.y, std::min(v1.y, v2.y)));
    auto ymax = static_cast<int32_t>(std::max(v0.y, std::max(v1.y, v2.y)));

    // Clamp to screen
    bbox->left   = std::max<int32_t>(0, xmin);
    bbox->right  = std::min<int32_t>(width, xmax);
    bbox->top    = std::max<int32_t>(0, ymin);
    bbox->bottom = std::min<int32_t>(height, ymax);
}