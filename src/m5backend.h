#pragma once
#if defined(__cplusplus)
extern "C" {
#endif
#include "backend.h"
#include "vec2.h"
#include "vec2.h"
#include "mesh.h"
#include "teapot.h"
#include "cube.h"
#include "pingo_mesh.h"
#include "renderer.h"
#include "texture.h"
#include "sprite.h"
#include "scene.h"
#include "object.h"
#include "mat3.h"

typedef struct Pixel Pixel;
typedef struct Depth Depth;
typedef struct Texture Texture;

typedef  struct {
    BackEnd backend;
    Depth * zetaBuffer;
    Vec2i size;
} M5STACKBackend;



void M5STACKBackendInit(M5STACKBackend * backend, Vec2i size);

void texture_draw(Texture *f, Vec2i pos, Pixel color);

#ifdef __cplusplus
}
#endif
