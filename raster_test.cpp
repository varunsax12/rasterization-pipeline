
#include <iostream>
#include <chrono>

#define FRAME_SIZE 16
#define TILE_SIZE FRAME_SIZE/4

/*
* Struct to represent anything which needs an (x,y) representation
* which could be a coordinate in cartesian plane or vector
*/
struct Position
{
    float x;
    float y;
    int color; // considering only 0->255 for color intensity, can be scaled to hold RGB data
};

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
float edge_bounds_value(Position A, Position B, Position P)
{
    float edgeVal;
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
bool edge_bounds_absolute(Position A, Position B, Position P)
{
    return (edge_bounds_value(A, B, P) <= 0);
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
float edge_bounds_check_incremental(Position A, Position B, Position P, int edgeIndex,
    bool firstCalc = true, int colIncrementSign = 1, bool increaseRow = false)
{
    static float edgeVal[3] = {0, 0, 0};
    // if-else statements are set are per priority of logic
    if (firstCalc)
    {
        edgeVal[edgeIndex] = (A.x - B.x) * (P.y - A.y) - (A.y - B.y) * (P.x - A.x);
    }
    else if (increaseRow)
    {
        edgeVal[edgeIndex] += (A.x - B.x); 
    }
    else
    {
        edgeVal[edgeIndex] -= (A.y - B.y) * colIncrementSign;
    }
    // for (int i = 0; i < 3; ++i)
    // {
    //     std::cout << edgeVal[i] << " ";
    // }
    // std::cout << std::endl;
    return edgeVal[edgeIndex];
}

/*
* Function to generate vectors (A->B)
* @param A -> origin of vector
* @param B -> destination of vector
* @return Position type for vector
*/
Position generate_vector(Position A, Position B)
{
    Position AB;
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
void raster_implementation(Position V0, Position V1, Position V2, int rowMin, int rowMax, int colMin, int colMax, int frameBuffer[FRAME_SIZE][FRAME_SIZE])
{
    // IMPLEMENTATION #1: Iterating over all pixels and recomputing edge function
    // Check if pixel is within triangle
    for (int row = FRAME_SIZE - 1; row >= 0; --row)
    {
        for (int col = 0; col < FRAME_SIZE; ++col)
        {
            bool withinBounds = true;
            Position P; P.x = col; P.y = row;
            // Compute edge equations for each to see if point within bounds of triangle formed
            withinBounds &= edge_bounds_absolute(V0, V1, P);
            withinBounds &= edge_bounds_absolute(V1, V2, P);
            withinBounds &= edge_bounds_absolute(V2, V0, P);
            std::cout << withinBounds << " ";
        }
        std::cout << std::endl;
    }

    // IMPLEMENTATION #2: Using scan line based approach to update the edge value
    int row = rowMin, col = colMin;
    bool increaseColIndex = true, increaseRow = false, firstCalc = true;

    // Compute area of triangle using the parallogram formation logic
    // Using vertices V0V1 and V0V2
    Position V0V1 = generate_vector(V0, V1);
    Position V0V2 = generate_vector(V0, V2);
    float area = std::abs(V0V1.x * V0V2.y - V0V1.y * V0V2.x);
    while (row <= rowMax)
    {
        Position P; P.x = col; P.y = row;
        // Compute edge equations for each to see if point within bounds of triangle formed
        float edgeVal01 = edge_bounds_check_incremental(V0, V1, P, 0, firstCalc, increaseColIndex ? 1 : -1, increaseRow);
        float edgeVal12 = edge_bounds_check_incremental(V1, V2, P, 1, firstCalc, increaseColIndex ? 1 : -1, increaseRow);
        float edgeVal20 = edge_bounds_check_incremental(V2, V0, P, 2, firstCalc, increaseColIndex ? 1 : -1, increaseRow);
        // Reset increaseY and firstCalc
        increaseRow = false;
        firstCalc = false;
        if (edgeVal01 <= 0 && edgeVal12 <= 0 && edgeVal20 <= 0)
        {
            // Compute color intensity based on the Barycentric coordinates of points in the 3 parallelograms
            // formed from the 2 vertices taken at a time
            // NOTE: Take absolute of each edge value as in the current direction system the edgevalues are
            // negative if the point is inside the triangle
            float w1 = std::abs(edgeVal01)/area;
            float w2 = std::abs(edgeVal12)/area;
            float w3 = std::abs(edgeVal20)/area;
            int colorIntesity = w1*V0.color + w2*V1.color + w3*V2.color;
            frameBuffer[row][col] = colorIntesity;
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

int main()
{
    int frameBuffer[FRAME_SIZE][FRAME_SIZE];
    memset(frameBuffer, 0, sizeof(frameBuffer[0][0]) * FRAME_SIZE * FRAME_SIZE);
    Position V0, V1, V2;
    V0.x = 1; V0.y = 2; V1.x = 12; V1.y = 5; V2.x = 5; V2.y = 12;
    V0.color = 255; V1.color = 0; V2.color = 120;
    //V0.x = 1; V0.y = 2; V1.x = 12; V1.y = 2; V2.x = 1; V2.y = 12;
    Position P; P.x = 5; P.y = 5;
    //Position P; P.x = 0; P.y = 0;
    auto start = std::chrono::high_resolution_clock::now();
    raster_implementation(V0, V1, V2, 0, FRAME_SIZE - 1, 0, FRAME_SIZE - 1, frameBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  
    std::cout << "Time taken by function: "
         << duration.count() << " microseconds" << std::endl;
    for (int i = FRAME_SIZE - 1; i >= 0; --i)
    {
        for (int j = 0; j < FRAME_SIZE; ++j)
        {
            std::cout << frameBuffer[i][j] << "\t";
        }
        std::cout << std::endl;
    }
    return 0;
}