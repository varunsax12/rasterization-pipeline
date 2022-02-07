#include <iostream>
#include <chrono>
#include <cmath>

#include "raster.h"

using namespace cocogfx;

/*
* Function to convert number to Fixed point format
* @param inVal -> value to convert
* @return Fixed point format of inVal
*/
template <typename T>
Fixed<FIXED_POINT_SIZE> convert_to_fixed(T inVal)
{
    Fixed<FIXED_POINT_SIZE> retVal = Fixed<FIXED_POINT_SIZE>::make(inVal);
    return retVal;
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
Fixed<FIXED_POINT_SIZE> edge_bounds_value(const vertex_fixed_t& A, const vertex_fixed_t& B, const vertex_fixed_t& P)
{
    Fixed<FIXED_POINT_SIZE> edgeVal;
    edgeVal = (A.x - B.x) * (P.y - A.y) - (A.y - B.y) * (P.x - A.x);
    return edgeVal;
}

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
bool edge_bounds_absolute(const vertex_fixed_t& A, const vertex_fixed_t& B, const vertex_fixed_t& P)
{
    return (edge_bounds_value(A, B, P) <= convert_to_fixed(0));
}

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
Fixed<FIXED_POINT_SIZE> edge_bounds_check_incremental(const vertex_fixed_t& A, const vertex_fixed_t& B,
    const vertex_fixed_t& P, int edgeIndex, bool firstCalc, int colIncrementSign, bool increaseRow)
{
    static Fixed<FIXED_POINT_SIZE> edgeVal[3] = {
        convert_to_fixed(0),
        convert_to_fixed(0),
        convert_to_fixed(0)
    };
    // if-else statements are set are per priority of logic
    if (firstCalc)
    {
        edgeVal[edgeIndex] = convert_to_fixed((A.x - B.x).data() * (P.y - A.y).data() - (A.y - B.y).data() * (P.x - A.x).data());
    }
    else if (increaseRow)
    {
        edgeVal[edgeIndex] += (A.x - B.x);
    }
    else
    {
        edgeVal[edgeIndex] -= convert_to_fixed((A.y - B.y).data() * colIncrementSign);
    }
    return edgeVal[edgeIndex];
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

void raster_implementation(const vertex_t& V0, const vertex_t& V1, const vertex_t& V2, const int rowMin, const int rowMax,
    const int colMin, const int colMax, FrameBuffer& frameBuffer)
{

    // Convert to fixed point number system for vertex data
    vertex_fixed_t V0_fixed;
    V0_fixed.x = convert_to_fixed(V0.x); V0_fixed.y = convert_to_fixed(V0.y); V0_fixed.color = V0.color;
    vertex_fixed_t V1_fixed;
    V1_fixed.x = convert_to_fixed(V1.x); V1_fixed.y = convert_to_fixed(V1.y); V1_fixed.color = V1.color;
    vertex_fixed_t V2_fixed;
    V2_fixed.x = convert_to_fixed(V2.x); V2_fixed.y = convert_to_fixed(V2.y); V2_fixed.color = V2.color;

    // IMPLEMENTATION #1: Iterating over all pixels and recomputing edge function
    // Check if pixel is within triangle
    // for (int row = FRAME_SIZE - 1; row >= 0; --row)
    // {
    //     for (int col = 0; col < FRAME_SIZE; ++col)
    //     {
    //         bool withinBounds = true;
    //         Position P; P.x = col; P.y = row;
    //         // Compute edge equations for each to see if point within bounds of triangle formed
    //         withinBounds &= edge_bounds_absolute(V0, V1, P);
    //         withinBounds &= edge_bounds_absolute(V1, V2, P);
    //         withinBounds &= edge_bounds_absolute(V2, V0, P);
    //         std::cout << withinBounds << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // IMPLEMENTATION #2: Using scan line based approach to update the edge value
    int row = rowMin, col = colMin;
    bool increaseColIndex = true, increaseRow = false, firstCalc = true;

    // Compute area of triangle using the parallogram formation logic
    // Using vertices V0V1 and V0V2
    vertex_fixed_t V0V1 = generate_vector(V0_fixed, V1_fixed);
    vertex_fixed_t V0V2 = generate_vector(V0_fixed, V2_fixed);
    Fixed<FIXED_POINT_SIZE> area = convert_to_fixed((V0V1.x.data() * V0V2.y.data()) - (V0V1.y.data() * V0V2.x.data()));
    if (area < convert_to_fixed(0))
    {
        area = area * (convert_to_fixed(-1));
    }
    while (row <= rowMax)
    {
        vertex_fixed_t P; P.x = convert_to_fixed(col); P.y = convert_to_fixed(row);
        // Compute edge equations for each to see if point within bounds of triangle formed
        Fixed<FIXED_POINT_SIZE> edgeVal01 = edge_bounds_check_incremental(V0_fixed, V1_fixed, P, 0, firstCalc, increaseColIndex ? 1 : -1, increaseRow);
        Fixed<FIXED_POINT_SIZE> edgeVal12 = edge_bounds_check_incremental(V1_fixed, V2_fixed, P, 1, firstCalc, increaseColIndex ? 1 : -1, increaseRow);
        Fixed<FIXED_POINT_SIZE> edgeVal20 = edge_bounds_check_incremental(V2_fixed, V0_fixed, P, 2, firstCalc, increaseColIndex ? 1 : -1, increaseRow);
        // Reset increaseY and firstCalc
        increaseRow = false;
        firstCalc = false;
        if (edgeVal01 <= convert_to_fixed(0) &&
            edgeVal12 <= convert_to_fixed(0) &&
            edgeVal20 <= convert_to_fixed(0))
        {
            // Compute color intensity based on the Barycentric coordinates of points in the 3 parallelograms
            // formed from the 2 vertices taken at a time
            // NOTE: Take absolute of each edge value as in the current direction system the edgevalues are
            // negative if the point is inside the triangle
            float w1 = float(edgeVal01.data())*(-1)/area.data();
            float w2 = float(edgeVal12.data())*(-1)/area.data();
            float w3 = float(edgeVal20.data())*(-1)/area.data();
            P.color.r = (int)(w1*V0.color.r + w2*V1.color.r + w3*V2.color.r);
            P.color.g = (int)(w1*V0.color.g + w2*V1.color.g + w3*V2.color.g);
            P.color.b = (int)(w1*V0.color.b + w2*V1.color.b + w3*V2.color.b);
            frameBuffer.setPixelColor(P);
        }
        if (increaseColIndex)
        {
            ++col;
        }
        else
        {
            --col;
        }
        if (col < colMin || col > colMax)
        {
            ++row;
            increaseColIndex = !increaseColIndex;
            increaseRow = true;
            if (col < colMin)
            {
                ++col;
            }
            else
            {
                --col;
            }
        }
    }
}

// wrapper to call raster_implementation
void drawTriangle(const vertex_t& v0, const vertex_t& v1, const vertex_t& v2, FrameBuffer& frameBuffer)
{
    // calculate the max height and width of row
    int width = std::ceil(std::max(std::max(v0.x, v1.x), v2.x));
    int heigth = std::ceil(std::max(std::max(v0.y, v1.y), v2.y));
    raster_implementation(v0, v1, v2, 0, heigth, 0, width, frameBuffer);
}