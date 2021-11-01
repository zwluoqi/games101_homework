//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

void ThreadRender(const Scene& scene, std::vector<Vector3f>& framebuffer) {

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 8;
    //std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++) {
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            }
            //framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0);
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
}


void foo(const int& x, char* mychar)
{
    std::cout << &x << "   " << &mychar << std::endl;
    std::cout << "正在运行的线程为：" << std::this_thread::get_id() << "线程的参数为： " << x << "  " << mychar << std::endl;
    return;
}


// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer((scene.width * scene.height));

    const int THREAD_COUNT = 8;
    std::cout << "threadCount: " << THREAD_COUNT << "\n";

    std::thread threads[THREAD_COUNT];
    std::vector<Vector3f> frameBuffers[THREAD_COUNT];


    for (int i = 0; i < THREAD_COUNT; i++) {
        frameBuffers[i] = std::vector<Vector3f>((scene.width * scene.height));
        threads[i] = std::thread(ThreadRender, std::ref(scene), std::ref(frameBuffers[i]));
    }

    for (int i = 0; i < THREAD_COUNT; i++) {
        threads[i].join();
    }

    for (int threadCount = 0; threadCount < THREAD_COUNT; threadCount++) {
        int m = 0;
        for (uint32_t j = 0; j < scene.height; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                framebuffer[m] += frameBuffers[threadCount][m] / 16.0;
                m++;
            }
        }
    }


    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* thread_fp = fopen("binary_thread.ppm", "wb");
    (void)fprintf(thread_fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, frameBuffers[0][i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, frameBuffers[0][i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, frameBuffers[0][i].z), 0.6f));
        fwrite(color, 1, 3, thread_fp);
    }
    fclose(thread_fp);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
