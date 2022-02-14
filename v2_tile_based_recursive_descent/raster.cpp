#include <iostream>
#include <chrono>
#include <cmath>

#include "raster.h"

#define MAX_DESCENT 8

fixed_t area(0);
fixed_t lineV0V1[3], lineV1V2[3], lineV2V0[3];

/*
* Function to convert number to Fixed point format
* @param inVal -> value to convert
* @return Fixed point format of inVal
*/
template <typename T>
fixed_t convert_to_fixed(T inVal)
{
    fixed_t retVal(inVal);
    return retVal;
}

/*
* Function to generate vectors (A->B)
* @param A -> origin of vector
* @param B -> destination of vector
* @return Position type for vector
*/
vertex_fixed_t generate_vector(const vertex_fixed_t& A, const vertex_fixed_t& B)
{
    vertex_fixed_t AB;
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;
    return AB;
}

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
fixed_t edge_bounds_value(const vertex_fixed_t& A, const vertex_fixed_t& B, const vertex_fixed_t& P)
{
    fixed_t edgeVal;
    edgeVal = (A.x - B.x) * (P.y - A.y) - (A.y - B.y) * (P.x - A.x);
    return edgeVal;
}

/*
* Function to calculate the edge equations
* using matrices
* 
* @param A: vertex A
* @param B: vertex B
* @param line_eq: line equation [x, y ,c]
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
    fixed_t* line_eq)
{
    fixed_t coeffX = A.y - B.y;
    fixed_t coeffY = B.x - A.x;
    fixed_t C = A.x * B.y - B.x * A.y;
    line_eq[0] = coeffX;
    line_eq[1] = coeffY;
    line_eq[2] = C;
}

/*
* Function to check point within edge (<0)
* @param line_eq: Line equation
* @param P: vertex to check
*/
bool pixel_within_edge(const fixed_t line_eq[3], const int x, const int y)
{
    fixed_t x_fixed = convert_to_fixed(x);
    fixed_t y_fixed = convert_to_fixed(y);
    fixed_t weight = (line_eq[0] * x_fixed + line_eq[1] * y_fixed + line_eq[2]);
    return weight >= convert_to_fixed(0);
}

/*
* Function to check whether tile on the same side of the edge
* @param bottomLeft
* @param bottomRight
* @param topLeft
* @param topRight
* @param line_eq
* @param check -> result of checks
* @param weight -> of the edge equations
* 
* Logic: To check whether all vertices on the same side of the edge
*/
void tile_edge_check(const int bottom, const int top, const int left, const int right, const fixed_t line_eq[3],
    std::pair<bool, std::pair<bool, bool>> &check)
{
    bool flag1 = pixel_within_edge(line_eq, left, bottom);
    bool flag2 = pixel_within_edge(line_eq, right, bottom);
    bool flag3 = pixel_within_edge(line_eq, left, top);
    bool flag4 = pixel_within_edge(line_eq, right, top);

    // Flag to check if tile is completely on the - of edge
    check.first = (!flag1 && !flag2 && !flag3 && !flag4);
    // Flag to check if tile is completely on the + of edge
    check.second.first = (flag1 && flag2 && flag3 && flag4);
    // Flag to check if tile is completely on on side of edge => line does not pass through it
    check.second.second = check.first || check.second.first;
}

/*
* Function to implement simple rasterization for
* proof of concept
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

bool raster_implementation(fixed_t edge1[3], fixed_t edge2[3], fixed_t edge3[3], const int top, const int bottom,
    const int left, const int right, FrameBuffer& frameBuffer, int descentCount,
    vertex_fixed_t V0_fixed, vertex_fixed_t V1_fixed, vertex_fixed_t V2_fixed)
{

    // Evaluate whether current tile contains triangle
    // NOTE: Trivial accept/reject not implemented as might be slowed in h/w (TODO)
    // Logic: If all 4 vertices of all edges have the same sign => outside triangle

    // Pair for flags
    std::pair<bool, std::pair<bool, bool>> check1, check2, check3;
    tile_edge_check(bottom, top, left, right, edge1, check1);
    tile_edge_check(bottom, top, left, right, edge2, check2);
    tile_edge_check(bottom, top, left, right, edge3, check3);


    if (
        // If the complete tile within triangle, then accept it
        (!(check1.second.first && check2.second.first && check3.second.first)) &&
        (
            // If the complete tile on one side of triangle
            (check1.second.second && check2.second.second && check3.second.second) ||
            // If the tile complete on + or - side of 2 edges, but passes through 3rd
            // Logic => + - on line combinations
            (check1.first        && check2.second.first && !check3.second.second) ||
            (check1.second.first && check2.first        && !check3.second.second) ||
            (check2.first        && check3.second.first && !check1.second.second) ||
            (check2.second.first && check3.first        && !check1.second.second) ||
            (check3.first        && check1.second.first && !check2.second.second) ||
            (check3.second.first && check1.first        && !check2.second.second)
        )
       )
    {
        // reject the tile
        return false;
    }

    if (descentCount == MAX_DESCENT)
    {
        // Color tile
        for (int i = left; i <= right; ++i)
        {
            for (int j = bottom; j <= top; ++j)
            {
                // Color computation logic
                vertex_fixed_t currentPixel; currentPixel.x = convert_to_fixed(i); currentPixel.y = convert_to_fixed(j);
                // Applying test color to make the triangle visible
                // Idea is to handle his in the shader computation
                currentPixel.color.r = 75; currentPixel.color.g = 50; currentPixel.color.b = 50;
                frameBuffer.setPixelColor(currentPixel);
            }
        }
        return true;
    }

    // Tile contains triangle
    // Break triangle and recurse
    //  tile1   tile2
    //  tile3   tile4

    int midPointX, midPointY;
    midPointX = ((right - left) >> 1) + left;
    midPointY = ((top - bottom) >> 1) + bottom;

    // Recurse the tiles
    //fixed_t newEdge1[3] = { lineV0V1[0], lineV0V1[1], lineV0V1[2] };
    //fixed_t newEdge2[3] = { lineV1V2[0], lineV1V2[1], lineV1V2[2] };
    //fixed_t newEdge3[3] = { lineV2V0[0], lineV2V0[1], lineV2V0[2] };
    descentCount++;
    //edge1[2] = lineV0V1[2] + (lineV0V1[0] << descentCount) + (lineV0V1[1] << descentCount);
    //edge2[2] = lineV1V2[2] + (lineV1V2[0] << descentCount) + (lineV1V2[1] << descentCount);
    //edge3[2] = lineV2V0[2] + (lineV2V0[0] << descentCount) + (lineV2V0[1] << descentCount);
    bool flag1 = raster_implementation(edge1, edge2, edge3, midPointY,
        bottom, left, midPointX, frameBuffer, descentCount,
        V0_fixed, V1_fixed, V2_fixed);
    bool flag2 = raster_implementation(edge1, edge2, edge3, midPointY,
        bottom, midPointX, right, frameBuffer, descentCount,
        V0_fixed, V1_fixed, V2_fixed);
    bool flag3 = raster_implementation(edge1, edge2, edge3, top, midPointY,
        left, midPointX, frameBuffer, descentCount,
        V0_fixed, V1_fixed, V2_fixed);
    bool flag4 = raster_implementation(edge1, edge2, edge3, top, midPointY,
        midPointX, right, frameBuffer, descentCount,
        V0_fixed, V1_fixed, V2_fixed);
    return flag1 | flag2 | flag3 | flag4;
}

/*
* Top draw call
* Assumptions:
*   1. The coordinates V0,V1,V2 are in the required order V0->V1->V2 in counter-clockwise direction
*/
void drawTriangle(const vertex_t& v0, const vertex_t& v1, const vertex_t& v2, FrameBuffer& frameBuffer)
{
    // Input tile size
    fixed_t xmin(0), ymin(0), xmax(255), ymax(255);
    vertex_fixed_t topRight; topRight.x = xmax; topRight.y = ymax;
    vertex_fixed_t bottomLeft; bottomLeft.x = xmin; bottomLeft.y = ymin;

    // Convert to fixed point number system for vertex data
    vertex_fixed_t V0_fixed;
    V0_fixed.x = convert_to_fixed(v0.x); V0_fixed.y = convert_to_fixed(v0.y); V0_fixed.color = v0.color;
    vertex_fixed_t V1_fixed;
    V1_fixed.x = convert_to_fixed(v1.x); V1_fixed.y = convert_to_fixed(v1.y); V1_fixed.color = v1.color;
    vertex_fixed_t V2_fixed;
    V2_fixed.x = convert_to_fixed(v2.x); V2_fixed.y = convert_to_fixed(v2.y); V2_fixed.color = v2.color;

    // Setup line equations
    edge_calculation(V0_fixed, V1_fixed, lineV0V1);
    edge_calculation(V1_fixed, V2_fixed, lineV1V2);
    edge_calculation(V2_fixed, V0_fixed, lineV2V0);

    // Calculate the area of triangle to be used to color computation using bayescentric
    // Compute area of triangle using the parallogram formation logic
    // Using vertices V0V1 and V0V2
    vertex_fixed_t V0V1 = generate_vector(V0_fixed, V1_fixed);
    vertex_fixed_t V0V2 = generate_vector(V0_fixed, V2_fixed);
    area = (V0V1.x * V0V2.y) - (V0V1.y * V0V2.x);
    if (area < convert_to_fixed(0))
    {
        area = area * (convert_to_fixed(-1));
    }

    raster_implementation(lineV0V1, lineV1V2, lineV2V0, 255, 0, 0, 255, frameBuffer, 0, V0_fixed, V1_fixed, V2_fixed);
}