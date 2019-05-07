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

#define GLTFIO_SIMPLEVIEWER_IMPLEMENTATION

#include "app/Config.h"
#include "app/FilamentApp.h"
#include "app/IBL.h"

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/View.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/AssetPipeline.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/SimpleViewer.h>

#include <getopt/getopt.h>

#include <utils/NameComponentManager.h>

#include <fstream>
#include <string>

#include "generated/resources/gltf.h"

using namespace filament;
using namespace gltfio;
using namespace utils;

enum AppState {
    EMPTY,
    LOADED,
    PREPPING,
    PREPPED,
    BAKING,
    BAKED,
};

struct App {
    Engine* engine;
    SimpleViewer* viewer;
    Config config;
    AssetLoader* loader;
    FilamentAsset* asset = nullptr;
    NameComponentManager* names;
    MaterialProvider* materials;
    bool actualSize = false;
    AppState state = EMPTY;
    utils::Path filename;
};

static const char* DEFAULT_IBL = "envs/venetian_crossroads";

static const char* INI_FILENAME = "gltf_baker.ini";

static void printUsage(char* name) {
    std::string exec_name(Path(name).getName());
    std::string usage(
        "SHOWCASE can perform AO baking on the specified glTF file\n"
        "Usage:\n"
        "    SHOWCASE [options] <gltf path>\n"
        "Options:\n"
        "   --help, -h\n"
        "       Prints this message\n\n"
        "   --actual-size, -s\n"
        "       Do not scale the model to fit into a unit cube\n\n"
    );
    const std::string from("SHOWCASE");
    for (size_t pos = usage.find(from); pos != std::string::npos; pos = usage.find(from, pos)) {
        usage.replace(pos, from.length(), exec_name);
    }
    std::cout << usage;
}

static int handleCommandLineArguments(int argc, char* argv[], App* app) {
    static constexpr const char* OPTSTR = "ha:i:us";
    static const struct option OPTIONS[] = {
        { "help",       no_argument,       nullptr, 'h' },
        { "actual-size", no_argument,      nullptr, 's' },
        { nullptr, 0, nullptr, 0 }
    };
    int opt;
    int option_index = 0;
    while ((opt = getopt_long(argc, argv, OPTSTR, OPTIONS, &option_index)) >= 0) {
        std::string arg(optarg ? optarg : "");
        switch (opt) {
            default:
            case 'h':
                printUsage(argv[0]);
                exit(0);
            case 's':
                app->actualSize = true;
                break;
        }
    }
    return optind;
}

static std::ifstream::pos_type getFileSize(const char* filename) {
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    return in.tellg();
}

static void saveIniFile(App& app) {
    std::ofstream out(INI_FILENAME);
    out << "[recent]\n";
    out << "filename=" << app.filename.c_str() << "\n";
}

static void loadIniFile(App& app) {
    utils::Path iniPath(INI_FILENAME);
    if (!app.filename.isEmpty() || !iniPath.isFile()) {
        return;
    }
    std::ifstream infile(INI_FILENAME);
    std::string line;
    while (std::getline(infile, line)) {
        size_t sep = line.find('=');
        if (sep != std::string::npos) {
            std::string lhs = line.substr(0, sep);
            std::string rhs = line.substr(sep + 1);
            if (lhs == "filename") {
                utils::Path gltf = rhs;
                if (gltf.isFile()) {
                    app.filename = rhs;
                }
            }
        }
    }
}

static void loadAsset(App& app) {
    std::cout << "Loading " << app.filename << "..." << std::endl;

    if (app.filename.getExtension() == "glb") {
        std::cerr << "GLB files are not yet supported." << std::endl;
        exit(1);
    }

    // Peek at the file size to allow pre-allocation.
    long contentSize = static_cast<long>(getFileSize(app.filename.c_str()));
    if (contentSize <= 0) {
        std::cerr << "Unable to open " << app.filename << std::endl;
        exit(1);
    }

    // Consume the glTF file.
    std::ifstream in(app.filename.c_str(), std::ifstream::in);
    std::vector<uint8_t> buffer(static_cast<unsigned long>(contentSize));
    if (!in.read((char*) buffer.data(), contentSize)) {
        std::cerr << "Unable to read " << app.filename << std::endl;
        exit(1);
    }

    // Parse the glTF file and create Filament entities.
    app.asset = app.loader->createAssetFromJson(buffer.data(), buffer.size());
    buffer.clear();
    buffer.shrink_to_fit();

    if (!app.asset) {
        std::cerr << "Unable to parse " << app.filename << std::endl;
        exit(1);
    }

    // Load external textures and buffers.
    gltfio::ResourceLoader({
        .engine = app.engine,
        .gltfPath = app.filename.getAbsolutePath(),
        .normalizeSkinningWeights = true,
        .recomputeBoundingBoxes = false
    }).loadResources(app.asset);

    // Load animation data then free the source hierarchy.
    app.asset->getAnimator();
    app.asset->releaseSourceData();

    // Add the renderables to the scene.
    app.viewer->setAsset(app.asset, app.names, !app.actualSize);

    app.viewer->setIndirectLight(FilamentApp::get().getIBL()->getIndirectLight());

    app.state = LOADED;
}

static void prepAsset(App& app) {
    app.state = PREPPING;
    std::cout << "Prepping..." << std::endl;

    gltfio::AssetPipeline pipeline;
    gltfio::AssetPipeline::AssetHandle asset = pipeline.load(app.filename);
    if (!asset) {
        std::cerr << "Unable to read " << app.filename << std::endl;
        exit(1);
    }

    asset = pipeline.flatten(asset);
    if (!asset) {
        std::cerr << "Unable to flatten " << app.filename << std::endl;
        exit(1);
    }

    asset = pipeline.parameterize(asset);
    if (!asset) {
        std::cerr << "Unable to parameterize " << app.filename << std::endl;
        exit(1);
    }

    const utils::Path folder = app.filename.getAbsolutePath().getParent();
    const utils::Path binPath = folder + "baked.bin";
    const utils::Path outPath = folder + "baked.gltf";

    pipeline.save(asset, outPath, binPath);
    std::cout << "Generated " << outPath << " and " << binPath << std::endl;

    app.filename = outPath;
    loadAsset(app);

    app.state = PREPPED;
}

static void bakeAsset(App& app) {
    app.state = BAKING;
    std::cout << "Baking..." << std::endl;
}

int main(int argc, char** argv) {
    App app;

    app.config.title = "gltf_baker";
    app.config.iblDirectory = FilamentApp::getRootPath() + DEFAULT_IBL;

    int option_index = handleCommandLineArguments(argc, argv, &app);
    int num_args = argc - option_index;
    if (num_args >= 1) {
        app.filename = argv[option_index];
        if (!app.filename.exists()) {
            std::cerr << "file " << app.filename << " not found!" << std::endl;
            return 1;
        }
        if (app.filename.isDirectory()) {
            auto files = app.filename.listContents();
            for (auto file : files) {
                if (file.getExtension() == "gltf") {
                    app.filename = file;
                    break;
                }
            }
            if (app.filename.isDirectory()) {
                std::cerr << "no glTF file found in " << app.filename << std::endl;
                return 1;
            }
        }
    }

    loadIniFile(app);

    auto setup = [&](Engine* engine, View* view, Scene* scene) {
        app.engine = engine;
        app.names = new NameComponentManager(EntityManager::get());
        app.viewer = new SimpleViewer(engine, scene, view, SimpleViewer::FLAG_COLLAPSED);
        app.materials = createMaterialGenerator(engine);
        app.loader = AssetLoader::create({engine, app.materials, app.names });

        if (!app.filename.isEmpty()) {
            loadAsset(app);
            saveIniFile(app);
        }

        app.viewer->setUiCallback([&app, scene] () {

            const ImVec4 disabled = ImGui::GetStyle().Colors[ImGuiCol_TextDisabled];
            const ImVec4 enabled = ImGui::GetStyle().Colors[ImGuiCol_Text];

            ImGui::PushStyleColor(ImGuiCol_Text, app.state == LOADED ? enabled : disabled);
            if (ImGui::Button("Prep", ImVec2(100, 50)) && app.state == LOADED) {
                prepAsset(app);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Flattens the asset and generates a new set of UV coordinates.");
            }
            ImGui::PopStyleColor();

            ImGui::SameLine();

            ImGui::PushStyleColor(ImGuiCol_Text, app.state == PREPPED ? enabled : disabled);
            if (ImGui::Button("Bake", ImVec2(100, 50)) && app.state == PREPPED) {
                bakeAsset(app);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Invokes an embree-based pathtracer.");
            }
            ImGui::PopStyleColor();
        });

        // Leave FXAA enabled but we also enable MSAA for a nice result. The wireframe looks
        // much better with MSAA enabled.
        view->setSampleCount(4);
    };

    auto cleanup = [&app](Engine* engine, View*, Scene*) {
        Fence::waitAndDestroy(engine->createFence());
        delete app.viewer;
        app.loader->destroyAsset(app.asset);
        app.materials->destroyMaterials();
        delete app.materials;
        AssetLoader::destroy(&app.loader);
        delete app.names;
    };

    auto animate = [&app](Engine* engine, View* view, double now) {
        if (app.state != EMPTY) {
            app.viewer->applyAnimation(now);
        }
    };

    auto gui = [&app](filament::Engine* engine, filament::View* view) {
        app.viewer->updateUserInterface();
        FilamentApp::get().setSidebarWidth(app.viewer->getSidebarWidth());
    };

    FilamentApp& filamentApp = FilamentApp::get();
    filamentApp.animate(animate);

    filamentApp.setDropHandler([&] (std::string path) {
        app.viewer->removeAsset();
        app.loader->destroyAsset(app.asset);
        app.filename = path;
        loadAsset(app);
        saveIniFile(app);
    });

    filamentApp.run(app.config, setup, cleanup, gui);

    return 0;
}
