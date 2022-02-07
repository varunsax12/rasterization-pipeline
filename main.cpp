
#include "raster.h"

int main(int argc, char** argv) {

    // initialize frame buffer
    FrameBuffer frameBuffer(256, 256);

    // triangle description
    //vertex_t v0(8.2, 80.5, color_blue);
    //vertex_t v1(15.2, 10.5, color_green);
    //vertex_t v2(120, 130.5, color_red);
    vertex_t v0(1, 2, color_blue);
    vertex_t v1(100, 10.5, color_green);
    vertex_t v2(50, 50, color_red);

    // draw the triangle
    drawTriangle(v0, v1, v2, frameBuffer);

    // save framebuffer to image file
    frameBuffer.saveImage("drawtri.ppm");

    return 0;
}
