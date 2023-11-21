#include <string.h>
#include <M5Unified.h>
#include "m5backend.h"

#include "renderer.h"
#include "texture.h"
#include "pixel.h"
#include "depth.h"

#include "freertos/task.h"

// Global sprite objects for rendering
LGFX_Sprite sprite1(&M5.Display);
extern LGFX_Sprite sprite;

// Function to convert RGB color to 16-bit color (565 format)
inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Initializes the M5Stack backend for the renderer
void _M5STACKBackendInit(Renderer *ren, BackEnd *backEnd, Vec4i _rect) {
    // Add initialization code here if needed
}

// Function called before rendering process starts
void _M5STACKBackendBeforeRender(Renderer *ren, BackEnd *backEnd) {
    // Add pre-render actions here
}

// Function called after the rendering process is complete
void _M5STACKBackendAfterRender(Renderer *ren, BackEnd *backEnd) {
    // Push the rendered sprite to the display and reset the sprite
    sprite1.pushSprite(&sprite, 0, 0);
    sprite1.fillSprite(0);
}

// Retrieves the framebuffer for rendering
Pixel * _M5STACKBackendGetFrameBuffer(Renderer *ren, BackEnd *backEnd) {
    return (Pixel *)sprite1.getBuffer();
}

// Retrieves the z-buffer for depth calculations
Depth * _M5STACKBackendGetZetaBuffer(Renderer *ren, BackEnd *backEnd) {
    return ((M5STACKBackend *) backEnd)->zetaBuffer;
}

// Function to draw a single pixel on the texture
void _M5STACKDrawPixel(Texture *f, Vec2i pos, Pixel color, float illumination) {
    // Calculate illuminated RGB values
    float r = color.g * illumination * 3;
    float g = color.g * illumination * 3 + 0.33;
    float b = color.g * illumination * 3 + 0.66;
    // Draw the pixel on the sprite
    sprite1.drawPixel(pos.x, pos.y, color565(r, g, b));
}

// Initializes the M5Stack backend
void M5STACKBackendInit(M5STACKBackend *backend, Vec2i size) {
    // Set function pointers for backend operations
    backend->backend.init = &_M5STACKBackendInit;
    backend->backend.beforeRender = &_M5STACKBackendBeforeRender;
    backend->backend.afterRender = &_M5STACKBackendAfterRender;
    backend->backend.getFrameBuffer = &_M5STACKBackendGetFrameBuffer;
    backend->backend.getZetaBuffer = &_M5STACKBackendGetZetaBuffer;
    backend->backend.drawPixel = &_M5STACKDrawPixel;

    // Allocate memory for the zeta buffer
    backend->zetaBuffer = (Depth*)heap_caps_malloc(size.x*size.y*sizeof(Depth), MALLOC_CAP_SPIRAM);
    if (backend->zetaBuffer == NULL) {
        printf("Failed to allocate zeta buffer\n");
        return;
    }

    // Initialize sprite for rendering
    sprite1.initDMA();
    sprite1.createSprite(size.x, size.y);
}

void texture_draw(Texture *f, Vec2i pos, Pixel color)
{
    sprite1.drawPixel(pos.x, pos.y, color565(color.g,color.g,color.g));
}