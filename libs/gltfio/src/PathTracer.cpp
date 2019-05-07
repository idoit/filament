/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "PathTracer.h"

#include <math/mat4.h>
#include <math/vec2.h>

#include <cgltf.h>

#include <utils/JobSystem.h>

#include <chrono> // TODO remove
#include <thread> // TODO remove

#ifdef FILAMENT_HAS_EMBREE
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#endif

using namespace filament::math;
using namespace std::chrono_literals; // TODO remove

static constexpr uint16_t TILE_SIZE = 32;
static constexpr float inf = std::numeric_limits<float>::infinity();

struct PixelRectangle {
    ushort2 topLeft;
    ushort2 bottomRight;
};

// http://www.altdevblogaday.com/2012/05/03/generating-uniformly-distributed-points-on-sphere/
static void random_direction(float* result) {
    float z = 2.0f * rand() / RAND_MAX - 1.0f;
    float t = 2.0f * rand() / RAND_MAX * 3.14f;
    float r = sqrt(1.0f - z * z);
    result[0] = r * cos(t);
    result[1] = r * sin(t);
    result[2] = z;
}

namespace gltfio {

PathTracer::Builder::Builder() {
    mPathTracer = new PathTracer();
}

PathTracer::Builder::~Builder() {
    delete mPathTracer;
}

PathTracer::Builder& PathTracer::Builder::renderTarget(image::LinearImage target) {
    mPathTracer->mRenderTarget = target;
    return *this;

}

PathTracer::Builder& PathTracer::Builder::sourceAsset(AssetHandle asset) {
    mPathTracer->mSourceAsset = asset;
    return *this;

}

PathTracer::Builder& PathTracer::Builder::filmCamera(const SimpleCamera& filmCamera) {
    mPathTracer->mFilmCamera = filmCamera;
    return *this;

}

PathTracer::Builder& PathTracer::Builder::uvCameraAttribute(const char* uvAttribute) {
    mPathTracer->mUvCameraAttribute = uvAttribute;
    return *this;

}

PathTracer::Builder& PathTracer::Builder::tileCallback(TileCallback onTile, void* userData) {
    mPathTracer->mTileCallback = onTile;
    mPathTracer->mTileUserData = userData;
    return *this;
}

PathTracer::Builder& PathTracer::Builder::doneCallback(DoneCallback onDone, void* userData) {
    mPathTracer->mDoneCallback = onDone;
    mPathTracer->mDoneUserData = userData;
    return *this;
}

PathTracer PathTracer::Builder::build() {
    return *mPathTracer;
}

#ifndef FILAMENT_HAS_EMBREE

bool PathTracer::render() {
    puts("Embree is not available.");
    return false;
}

#else

struct EmbreeContext {
    // PathTracer fields
    image::LinearImage renderTarget;
    const cgltf_data* sourceAsset;
    SimpleCamera filmCamera;
    const char* uvCameraAttribute;
    PathTracer::TileCallback tileCallback;
    void* tileUserData;
    PathTracer::DoneCallback doneCallback;
    void* doneUserData;

    // Bookkeeping data
    std::atomic<int> numRemainingTiles;

    // Embree objects
    RTCDevice embreeDevice;
    RTCScene embreeScene;
};

static void renderTile(EmbreeContext* context, PixelRectangle rect) {
    image::LinearImage& image = context->renderTarget;

    // Precompute some camera parameters.
    const SimpleCamera& camera = context->filmCamera;
    const float iw = 1.0f / image.getWidth();
    const float ih = 1.0f / image.getHeight();
    const float theta = camera.vfovDegrees * M_PI / 180;
    const float f = tanf(theta / 2);
    const float a = camera.aspectRatio;
    const float3 org = camera.eyePosition;
    const uint16_t hm1 = image.getHeight() - 1;

    // The camera basis: View / Right / Up.
    const float3 v = normalize(camera.targetPosition - org);
    const float3 r = normalize(cross(v, camera.upVector));
    const float3 u = cross(r, v);

    // Given a pixel row and column, generate a ray from the eye to the film.
    auto generateCameraRay = [=] (uint16_t row, uint16_t col) {
        const uint16_t x = col;
        const uint16_t y = hm1 - row;
        const float s = (2.0f * (x + 0.5f) * iw - 1.0f);
        const float t = (2.0f * (y + 0.5f) * ih - 1.0f);
        const float3 dir = normalize(a * f * s * r - f * t * u + v);
        return RTCRay {
            .org_x = org.x,
            .org_y = org.y,
            .org_z = org.z,
            .tnear = 0,
            .dir_x = dir.x,
            .dir_y = dir.y,
            .dir_z = dir.z,
            .time = 0,
            .tfar = inf,
            .mask = 0xffffffff
        };
    };

    // Loop over all pixels in the tile.
    for (uint16_t row = rect.topLeft.y, len = rect.bottomRight.y; row < len; ++row) {
        for (uint16_t col = rect.topLeft.x, len = rect.bottomRight.x; col < len; ++col) {
            RTCIntersectContext intersector;
            rtcInitIntersectContext(&intersector);

            RTCRay ray = generateCameraRay(row, col);
            rtcOccluded1(context->embreeScene, &intersector, &ray);

            float* dst = image.getPixelRef(col, row);
            *dst = (ray.tfar == -inf) ? 1.0f : 0.0f;
        }
    }

    // Signal that we're done with the tile and possibly the entire image.
    context->tileCallback(image, rect.topLeft, rect.bottomRight, context->tileUserData);
    if (--context->numRemainingTiles == 0) {
        context->doneCallback(image, context->doneUserData);
        rtcReleaseScene(context->embreeScene);
        rtcReleaseDevice(context->embreeDevice);
        delete context;
    }
}

bool PathTracer::render() {
    auto sourceAsset = (const cgltf_data*) mSourceAsset;
    EmbreeContext* context = new EmbreeContext {
        .renderTarget = mRenderTarget,
        .sourceAsset = sourceAsset,
        .filmCamera = mFilmCamera,
        .uvCameraAttribute = mUvCameraAttribute,
        .tileCallback = mTileCallback,
        .tileUserData = mTileUserData,
        .doneCallback = mDoneCallback,
        .doneUserData = mDoneUserData
    };

    // Create the embree device and scene.
    auto device = context->embreeDevice = rtcNewDevice(nullptr);
    assert(device && "Unable to create embree device.");
    auto scene = context->embreeScene = rtcNewScene(device);
    assert(scene);

    auto addEmbreeGeometry = [scene, device](const cgltf_accessor* positions,
            const cgltf_accessor* indices) {
        assert(positions->component_type == cgltf_component_type_r_32f);
        assert(positions->type == cgltf_type_vec3);
        assert(indices->component_type == cgltf_component_type_r_32u);
        assert(indices->type == cgltf_type_scalar);

        auto geo = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

        rtcSetSharedGeometryBuffer(geo, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
                positions->buffer_view->buffer->data,
                positions->offset + positions->buffer_view->offset,
                positions->stride,
                positions->count);

        rtcSetSharedGeometryBuffer(geo, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
                indices->buffer_view->buffer->data,
                indices->offset + indices->buffer_view->offset,
                indices->stride * 3,
                indices->count / 3);

        rtcCommitGeometry(geo);
        rtcAttachGeometry(scene, geo);
        rtcReleaseGeometry(geo);
    };

    // Populate the embree mesh from a flattened glTF asset.
    cgltf_node** nodes = sourceAsset->scene->nodes;
    for (cgltf_size i = 0, len = sourceAsset->scene->nodes_count; i < len; ++i) {
        assert(nodes[i]->mesh);
        assert(nodes[i]->mesh->primitives_count == 1);
        const cgltf_primitive& prim = nodes[i]->mesh->primitives[0];
        for (cgltf_size j = 0, len = prim.attributes_count; j < len; ++j) {
            const cgltf_attribute& attr = prim.attributes[j];
            if (attr.type == cgltf_attribute_type_position) {
                addEmbreeGeometry(attr.data, prim.indices);
            }
        }
    }
    rtcCommitScene(scene);

    // Compute the number of jobs by pre-running the loop.
    const uint16_t width = mRenderTarget.getWidth();
    const uint16_t height = mRenderTarget.getHeight();
    int numTiles = 0;
    for (uint16_t row = 0; row < height; row += TILE_SIZE) {
        for (uint16_t col = 0; col < width; col += TILE_SIZE, ++numTiles);
    }
    context->numRemainingTiles = numTiles;

    // Kick off one job per tile.
    utils::JobSystem* js = utils::JobSystem::getJobSystem();
    utils::JobSystem::Job* parent = js->createJob();
    for (uint16_t row = 0; row < height; row += TILE_SIZE) {
        for (uint16_t col = 0; col < width; col += TILE_SIZE) {
            PixelRectangle rect;
            rect.topLeft = {col, row};
            rect.bottomRight = {col + TILE_SIZE, row + TILE_SIZE};
            rect.bottomRight.x = std::min(rect.bottomRight.x, width);
            rect.bottomRight.y = std::min(rect.bottomRight.y, height);
            utils::JobSystem::Job* tile = utils::jobs::createJob(*js, parent, [context, rect] {
                renderTile(context, rect);
            });
            js->run(tile);
        }
    }
    js->run(parent);

    return true;
}

#endif // FILAMENT_HAS_EMBREE

} // namespace gltfio
