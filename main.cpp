
#include "SDL2/SDL.h"
#include <Eigen/Dense>
#include "stdio.h"
#include <iostream>
#include <vector>
#include <cmath>
// SDL_Window *window;

// lldb (print pow correctly): expr -l objective-c -- @import Darwin

struct Ray
{
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
};

struct RayHitResult
{
  Eigen::Vector3d hitPosition;
  bool hit;
};

class Object
{
  public:
  virtual RayHitResult raytrace(Ray ray) = 0;
};

class Sphere : public Object
{
  float radii;
  Eigen::Vector3d position;

  public:
  Sphere ()
  {
    radii = 1; // 1 meter radii
    position = Eigen::Vector3d(0, 0, -3);
  }

  virtual RayHitResult raytrace(Ray ray)
  {
    // print the closest colision point
    // see equation at https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // - (l * ( o - c) ) +- sqrt( (l * ( o - c))^2 - (o-c)^2 + r^2 )
    double l_o_c =
      ray.direction.x() * ( ray.origin.x() - position.x() ) +
      ray.direction.y() * ( ray.origin.y() - position.y() ) +
      ray.direction.z() * ( ray.origin.z() - position.z() );

    double sqrtValue =
      pow( l_o_c, 2)
      - ( pow(ray.origin.x() - position.x(), 2) +
          pow(ray.origin.y() - position.y(), 2) +
          pow(ray.origin.z() - position.z(), 2)
        )
      + pow(radii, 2);

    // std::cout << "sqrt val: " << sqrtValue << std::endl;

    RayHitResult hitResult;

    if (sqrtValue < 0)
    {
      hitResult.hit = false;

      return hitResult; //  missed the sphere
    }
    else
    {
      double t = - l_o_c - sqrtValue;

      if (t < 0)
      {
        // it's being the ray
        hitResult.hit = false;
        return hitResult;
      }

      // ray equation
      // R(t) = StartPos + Direction * t
      Eigen::Vector3d hitPosition = ray.origin + ray.direction * t;

      hitResult.hit = true;
      hitResult.hitPosition = hitPosition;
      return hitResult;
    }
  }
};

class World
{
  public:
  std::vector<Object*> sceneObjects;

  void spawnObject()
  {
    Sphere* sphere = new Sphere();
    sceneObjects.push_back(sphere);
  }
};

class Camera
{
  public:
  Eigen::Vector3d pos;
  Eigen::Vector3d dir;
  double viewPlaneXsize;  // the size of the rendering place, in meters
  double viewPlaneYsize;  // the size of the rendering place, in meters

  Camera(double pScreenWidth, double pScreenHeight)
  {
    viewPlaneYsize = 0.1;
    viewPlaneXsize = (pScreenWidth/pScreenHeight) * viewPlaneYsize;
  }

  Ray RayAtScreenSpace(double x, double y)
  {
    Ray ray;

    ray.origin = Eigen::Vector3d(0, 0, 0); // TODO change this to use the camera position
    ray.direction = Eigen::Vector3d(x * viewPlaneXsize, y * viewPlaneYsize, -0.15);
    ray.direction.normalize();

    return ray;
  }
};

class Renderer
{
  public:

  SDL_Window *window;
  int windowIndex;
  SDL_Renderer* sdl_renderer;

  World* world;

  Renderer(SDL_Window *window, World *pWorld)
  {
    this->window = window;
    world = pWorld;

    windowIndex = -1; // the index of the rendering driver to initialize, or -1 to initialize the first one supporting the requested flags
    int flags = 0;
    sdl_renderer = SDL_CreateRenderer(window, windowIndex, flags);
  }

  void render()
  {
    SDL_SetRenderDrawColor(sdl_renderer, 255, 0, 0, 1); // If something is FULL red on the screen, it means that pixel was not rendered
    SDL_RenderClear(sdl_renderer);
    // SDL_SetRenderDrawColor(sdl_renderer, 0xFF, 0, 0, 0xFF);
    // SDL_RenderDrawPoint(sdl_renderer, 30, 30);
    // SDL_RenderPresent( sdl_renderer );

    int windowWidth = 640;
    int windowHeight = 480;
    Camera camera(windowWidth, windowHeight);

    // Ray ray;
    // ray.origin = Eigen::Vector3d(0, 0, 0);

    // this defines the size of the camera view
    double screenSpaceXRatio = 1.0 / windowWidth;
    double screenSpaceYRatio = 1.0 / windowHeight;

    for (int row = 0; row < windowHeight; ++row)
    {
      for (int col = 0; col < windowWidth; ++col)
      {
        double screenSpaceX = screenSpaceYRatio * row;
        double screenSpaceY = screenSpaceXRatio * col;
        Ray ray = camera.RayAtScreenSpace(screenSpaceX, screenSpaceY);
        // std::cout << "("<< screenSpaceX << ";" << screenSpaceY << ") \n";

        // ray.direction = Eigen::Vector3d(col * renderPlaneXRatio, row * renderPlaneYRatio, -0.15); // render plane, pixel position
        // ray.direction.normalize();
        // std::cout << ray.direction << "\n ----" << std::endl;

        RayHitResult hitResult = findClosestHit(world->sceneObjects, ray);
        if (hitResult.hit)
        {
          // std::cout << "hit(r:"<<row<< ";col:"<<col<<";pos:"<<hitResult.hitPosition.size()<<") " << std::endl;
          SDL_SetRenderDrawColor(sdl_renderer, 255, 255, 255, 1);
        }
        else
        {
          SDL_SetRenderDrawColor(sdl_renderer, 0, 0, 0, 1);
        }

        // std::cout << rowPos << std::endl;
        SDL_RenderDrawPoint(sdl_renderer, col, row);
      }
    }

    SDL_RenderPresent( sdl_renderer );
    std::cout << "done" << std::endl;
  }

  RayHitResult findClosestHit(std::vector<Object*> worldObjects, Ray ray)
  {
    RayHitResult closestHitResult;
    closestHitResult.hitPosition = Eigen::Vector3d(9999, 9999, 9999);
    closestHitResult.hit = false;

    for (int i = 0; i < worldObjects.size(); ++i)
    {
      Object* sceneObject = world->sceneObjects[i];

      RayHitResult hitResult;
      hitResult = sceneObject->raytrace(ray);

      if (hitResult.hit && hitResult.hitPosition.norm() < closestHitResult.hitPosition.norm())
      {
        closestHitResult = hitResult;
      }
    }

    // if (closestHitResult.hit)
      // std::cout << ".";

    return closestHitResult;
  }
};

void waitUntilQuit()
{
  // A basic main loop to prevent blocking
  bool is_running = true;
  SDL_Event event;
  while (is_running) {
      while (SDL_PollEvent(&event)) {
          if (event.type == SDL_QUIT) {
              is_running = false;
          }
      }
      SDL_Delay(16);
  }
}

int main(int argc, char* argv[])
{
  SDL_Init(SDL_INIT_VIDEO);

  SDL_Window* window = SDL_CreateWindow(
      "An SDL2 window",                  // window title
      SDL_WINDOWPOS_CENTERED,           // initial x position
      SDL_WINDOWPOS_CENTERED,           // initial y position
      640,                               // width, in pixels
      480,                               // height, in pixels
      SDL_WINDOW_MAXIMIZED | SDL_WINDOW_SHOWN                  // flags - see below
  );

  if (window == NULL)
  {
      printf("Could not create window: %s\n", SDL_GetError());
      return 1;
  }

  SDL_Delay(1);

  World world;
  world.spawnObject();

  Renderer render(window, &world);
  render.render();

  waitUntilQuit();

  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}



