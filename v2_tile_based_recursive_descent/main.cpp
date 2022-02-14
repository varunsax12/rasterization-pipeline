
#include "raster.h"

int main(int argc, char** argv) {

    // initialize frame buffer
    FrameBuffer frameBuffer(256, 256);

    // triangle description
    //vertex_t v0(8.2, 80.5, color_blue);
    //vertex_t v1(15.2, 10.5, color_green);
    //vertex_t v2(120, 130.5, color_red);
    //vertex_t v0(1, 2, color_blue);
    //vertex_t v1(100, 10.5, color_green);
    //vertex_t v2(50, 50, color_red);
    //vertex_t v0(50, 50, color_blue);
    //vertex_t v1(150, 50, color_green);
    //vertex_t v2(50, 150, color_red);
    //vertex_t v0(100, 100, color_blue);
    //vertex_t v1(200, 100, color_green);
    //vertex_t v2(100, 200, color_red);
    int min = 5, max = 255;
    vertex_t v0(min, min, color_blue);
    vertex_t v1(max, min, color_green);
    vertex_t v2(min, max-100, color_red);

    // draw the triangle
    drawTriangle(v0, v1, v2, frameBuffer);

    // save framebuffer to image file
    frameBuffer.saveImage("drawtri.ppm");

    //fixed_t line_eq[3];
    //vertex_fixed_t A; A.x = 2; A.y = 3;
    //vertex_fixed_t B; B.x = -4; B.y = 1;
    //edge_calculation(A, B, line_eq);
    //std::cout << line_eq[0] << " " << line_eq[1] << " " << line_eq[2] << std::endl;

    return 0;
}
