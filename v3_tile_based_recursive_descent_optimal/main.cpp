#include "raster.h"

int main(int argc, char** argv) {
    // initialize frame buffer
    FrameBuffer frameBuffer(1024, 1024);
    
    // initialize rasterizer
    Rasterizer raster(&frameBuffer, 64, 8);
    
    // draw a triangle (position defined in clip space)
    raster.drawTiangle(
        {{-15.0f,-0.5f, 0.0f, 1.0f}, color_blue},
        {{ 0.5f,-0.5f, 0.0f, 1.0f}, color_green},
        {{ 0.0f, 0.5f, 0.0f, 1.0f}, color_red}
    );

    // save framebuffer to image file
    frameBuffer.saveImage("drawtri.ppm");

    return 0;
}
