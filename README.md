# RayTracer

A lightweight C++ ray tracing implementation that renders 3D scenes defined in XML format. The project includes a custom parser, PPM image writer, geometric primitives, shading models, and support for multiple camera configurations. It is designed to simulate fundamental light–object interactions and produce high-quality RGB images using classical ray tracing techniques.

##  Overview

The ray tracer renders a scene defined in an XML file by simulating light rays traveling from the camera, through the image plane, and into the scene. Key features implemented include:

* **Ray-Object Intersections:** Calculating intersections with **spheres**, **triangles**, and **meshes**.
* **Surface Shading:** Using the **Blinn-Phong shading model** for diffuse and specular reflection.
* **Light Sources:** Handling **Ambient** and **Point** light sources with distance-dependent intensity falloff ($E(d) = I/d^2$).
* **Shadows:** Implementing **shadow rays** to determine visibility from light sources, utilizing a `ShadowRayEpsilon` to prevent self-intersection.
* **Reflections:** Supporting **recursive ray tracing** for **mirror-like** materials up to a specified `MaxRecursionDepth`.
* **Camera Model:** Correctly setting up the camera coordinate frame and projecting rays through the **NearPlane**. 

---

##  Build and Run


The project includes a `Makefile` to simplify the build process.

```bash
make
```

This command will compile the source files and create the executable named raytracer

The executable takes a single argument: the path to the XML scene description file.

```bash
./raytracer <scene_file.xml>
```
