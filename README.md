## What is Ferrum 3d

The Ferrum 3d is a simple 3d physics engine built in Rust. 
It was created to help me further my Rust abilities 
as well as learn Physics and Maths necessary for my future education.
It is a 3d rewrite of my first engine [Ferrum Engine](https://github.com/LukaJakimovski/Ferrum-Engine)
which I learned a lot from which now permits me to create a much better project but now in 3d.

Ferrum 3d currently does fast and physically accurate simulations of gravity and rigid-body dynamics.

## Build Instructions
Clone the repository and run the following command in the root of the project:

``cargo run --package ferrum_app --bin ferrum``

> [!IMPORTANT]
> Currently I have only verified the proper functioning of the code on Linux and it **may**
> not work properly on other systems

You can also download the most recent precompiled binary for your platform in 
[releases](https://github.com/LukaJakimovski/Ferrum-3d/releases)

## Controls
- WASD -> Move Forward, Left, Down, Right
- Space -> Go Up
- Shift -> Go Down
- Arrow Keys -> Move Camera
- Left Click Drag -> Move Camera

## Future Plans
(In order)
* Collision
* Constraints (Springs, hinges, joints)
* Reusability
* Parallelism
* Electromagnetism
* Optics (Raytracing)
* Acoustics
* Fluid Dynamics

## Design Goals

* **Fast**: Memory efficient, cache friendly, parallel, scalable, real-time capable uses fast solving methods.
* **Accurate**: Simulates physics as close to reality as possible
* **Stable**: Does not explode velocities, no tunneling
* **Deterministic**: For any given time step and start state always give the same result
* **Simple**: Code that is easy to read, understand, and navigate for those curious
* **Modular**: Clean architecture, swappable components (integrators, solvers, and features) as needed
* **Reusable**: The components of this project should be easily usable in other codebases


## Attributions
* [Hansen B. (2025)](https://github.com/sotrh/learn-wgpu)  
  *learn-wgpu*  
  GitHub

    For teaching me wgpu and being the reference for the wgpu code

* [Mirtich, B. (1996)](https://www.cs.upc.edu/~virtual/SGI/docs/3.%20Further%20Reading/Fast%20and%20accurate%20computation%20of%20polyhedral%20mass%20properties.pdf).  
  *Fast and Accurate Computation of Polyhedral Mass Properties*
  Journal of Graphics Tools, 1(2), 31–50.

  Which was my reference for calculating the center of mass, density, volume, and moment of inertia of rigid-bodies

* [Wei, X., Liu, M., Ling, Z., & Su, H. (2022)](https://doi.org/10.1145/3528223.3530077).
  *Approximate convex decomposition for 3D meshes with collision-aware concavity and tree search* 
  ACM Transactions on Graphics, 41(4), 1–18.

  Used for creating accurate collision meshes