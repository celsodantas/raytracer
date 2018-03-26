
#include "SDL2/SDL.h"
#include <Eigen/Dense>
#include "stdio.h"
#include <iostream>
#include <vector>
#include <cmath>
// SDL_Window *window;

// lldb (print pow correctly): expr -l objective-c -- @import Darwin

class GlobalSettings
{
  public:
  const static int ScreenResolutionX = 640;
  const static int ScreenResolutionY = 480;
};

struct Ray
{
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
};

class Object;

struct RayHitResult
{
  Eigen::Vector3d hitPosition;
  bool hit;
  Object* hitObject;
};

class Object
{
  public:
  Eigen::Vector3d color;

  virtual RayHitResult raytrace(Ray ray) = 0;
  virtual const Eigen::Vector3d normalAt(const Eigen::Vector3d pos) = 0;
};

class Sphere : public Object
{
  float radii;
  Eigen::Vector3d position;

  public:
  Sphere (double pRadii, Eigen::Vector3d pPosition, Eigen::Vector3d pColor)
  {
    radii = pRadii; // in meters
    position = pPosition;
    color = pColor;
  }

  virtual const Eigen::Vector3d normalAt(const Eigen::Vector3d pPos)
  {
    return pPos - position;
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

struct Light
{
  Eigen::Vector3d pos;
  Eigen::Vector3d color;
  double intensity = 70;
};

class World
{
  public:
  std::vector<Object*> sceneObjects;
  Light light;

  void spawnObject()
  {
    light.pos = Eigen::Vector3d(0, -2, 0);
    light.color = Eigen::Vector3d(255, 255, 255);

    sceneObjects.push_back(new Sphere(0.5, Eigen::Vector3d(0.5, 0.8, -8), Eigen::Vector3d(100, 100, 0)));
    sceneObjects.push_back(new Sphere(0.5, Eigen::Vector3d(1.9, 0.3, -9.8), Eigen::Vector3d(0, 100, 0)));
    sceneObjects.push_back(new Sphere(0.5, Eigen::Vector3d(0.9, 0.8, -7.5), Eigen::Vector3d(0, 100, 55)));
  }
};

class Camera
{
  public:
  Eigen::Vector3d pos;
  Eigen::Vector3d dir;
  double viewPlaceDist = -0.5;
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

    ray.origin = Eigen::Vector3d(0, 0, 0.5); // TODO change this to use the camera position
    ray.direction = Eigen::Vector3d(x * viewPlaneXsize, y * viewPlaneYsize, viewPlaceDist);
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
    SDL_RenderPresent( sdl_renderer );

    Camera camera(GlobalSettings::ScreenResolutionX, GlobalSettings::ScreenResolutionY);

    double screenSpaceXRatio = 1.0 / GlobalSettings::ScreenResolutionX;
    double screenSpaceYRatio = 1.0 / GlobalSettings::ScreenResolutionY;

    for (int y = 0; y < GlobalSettings::ScreenResolutionY; ++y)
    {
      for (int x = 0; x < GlobalSettings::ScreenResolutionX; ++x)
      {
        double screenSpaceX = screenSpaceXRatio * x;
        double screenSpaceY = screenSpaceYRatio * y;
        Ray ray = camera.RayAtScreenSpace(screenSpaceX, screenSpaceY);

        RayHitResult hitResult = findClosestHit(world->sceneObjects, ray);

        if (hitResult.hit)
        {
          // calculating pixel color (SHADER!)
          Eigen::Vector3d normal = hitResult.hitObject->normalAt(hitResult.hitPosition);

          Eigen::Vector3d lightNormalToHitPos = world->light.pos - hitResult.hitPosition;
          double distanceFromLight = lightNormalToHitPos.norm();
          double lightAttenuation = (1/ (1 + 0.1 * distanceFromLight + 0.1 * distanceFromLight * distanceFromLight ));

          lightNormalToHitPos.normalize();
          double lightAngle = lightNormalToHitPos.dot(normal);

          if (lightAngle > 0)
          {
            int r = hitResult.hitObject->color.x() * lightAngle * lightAttenuation * world->light.intensity;
            int g = hitResult.hitObject->color.y() * lightAngle * lightAttenuation * world->light.intensity;
            int b = hitResult.hitObject->color.z() * lightAngle * lightAttenuation * world->light.intensity;
            SDL_SetRenderDrawColor(sdl_renderer, r, g, b, 1);
          }
          else
          {
            SDL_SetRenderDrawColor(sdl_renderer, 0, 0, 0, 1);
          }
        }
        else
        {
          SDL_SetRenderDrawColor(sdl_renderer, 50, 50, 50, 1);
        }

        SDL_RenderDrawPoint(sdl_renderer, x, y);
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
    closestHitResult.hitObject = nullptr;

    for (int i = 0; i < worldObjects.size(); ++i)
    {
      Object* sceneObject = world->sceneObjects[i];

      RayHitResult hitResult;
      hitResult = sceneObject->raytrace(ray);

      if (hitResult.hit && hitResult.hitPosition.norm() < closestHitResult.hitPosition.norm())
      {
        closestHitResult = hitResult;
        closestHitResult.hitObject = sceneObject;
      }
    }

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
      "Raytracer",
      SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED,
      GlobalSettings::ScreenResolutionX,
      GlobalSettings::ScreenResolutionY,
      SDL_WINDOW_MAXIMIZED | SDL_WINDOW_SHOWN
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



