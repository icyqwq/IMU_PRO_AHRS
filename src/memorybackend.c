
#include "memorybackend.h"

#include "renderer.h"
#include "texture.h"
#include "pixel.h"
#include "depth.h"


void memoryBackendinit( Renderer * ren, BackEnd * backEnd, Vec4i _rect) {

}

void memoryBackendbeforeRender( Renderer * ren, BackEnd * backEnd) {
}

void memoryBackendafterRender( Renderer * ren,  BackEnd * backEnd) {

}

Pixel * memoryBackendgetFrameBuffer( Renderer * ren,  BackEnd * backEnd) {
    return ((MemoryBackend *) backEnd) -> frameBuffer;
}

Depth * memoryBackendgetZetaBuffer( Renderer * ren,  BackEnd * backEnd) {
    return ((MemoryBackend *) backEnd) -> zetaBuffer;
}

void memoryBackendInit( MemoryBackend * this, Pixel * buf, Vec2i size) {

    this->backend.init = &memoryBackendinit;
    this->backend.beforeRender = &memoryBackendbeforeRender;
    this->backend.afterRender = &memoryBackendafterRender;
    this->backend.getFrameBuffer = &memoryBackendgetFrameBuffer;
    this->backend.getZetaBuffer = &memoryBackendgetZetaBuffer;

    this -> zetaBuffer = malloc(size.x*size.y*sizeof (Depth));
    this -> frameBuffer = buf;
}
