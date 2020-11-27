# VorteGrid: Interactive Fluid Simulation for Games and Movies

**Many fluid simulations run slowly. Games need fast solutions, even if at the cost of accuracy.**

Fluid simulations can be slow for many reasons, including these:

* Flow treated as incompressible imposes a divergence-free condition which requires a global solution to enforce that condition.
* Solutions can be **unstable**.
Making them stable requires either explicitly high viscosity or unintentionally incorporates very high **"numerical" viscosity** from implicit solvers,
which degrades details and makes the simulation look muddy.
  * Techniques that try to compensate by artificially injecting detail just looks like noise instead of the beautiful filamentary wisps that characterize fluid flow.
* Traditional approaches to satisfying **boundary conditions** require global solutions and often entail imposing constraints on the shape of the simulation domain.
Simulations that can handle complicated boundary conditions (such as arbitrary shapes or expanding boxes) typically employ sophisticated and computationally expensive techniques, such as dynamic regridding or level set methods.

These problems have solutions. **Mesh-free** algorithms simulate interactions between **particles**.
These techniques lack numerical diffusion and preserve detail. Naive direct computations of such interactions require a computational complexity of O(N^2).
Approximate methods such as fast-multipole and treecode can consume O(N log N) operations. But games need faster techniques.

Real-time simulations exploit the embarrassingly parallel nature of the fluid simulation algorithms.
Some use massively parallel *graphics processing units* (GPU's).
While these simulations to indeed run in real time, they rob video games of an important resource, which is the ability to render complex and compelling virtual environments and characters.
Games require solutions that use CPU resources for simulation and leave GPU resources for rendering.

**This vortex particle simulation solves these problems.**

It's used in professional software like **[Red Giant Trapcode Suite 15](https://www.redgiant.com/products/trapcode-suite/)**,
including [Red Giant Trapcode Particular](https://www.redgiant.com/products/trapcode-particular/)
and [Red Giant Trapcode Form](https://www.redgiant.com/products/trapcode-form/).

I explain below how it works, in 21 articles, with demo videos and source code, all free.

## Objectives

A fluid simulation for video games should satisfy these requirements:

* **Fast**; The simulation should run in real-time: 30 frames per second or faster.
* **Controllable**; Effects artists should be able to author effects, and influence or steer flow.
* **Detailed**; Flow should retain fine details such as wisps since they are aesthetically pleasing.
* **Flexible**; Fluid region should be able to expand to any size and any shape, with open or closed boundaries.
* **Scalable**; The simulation should work for large and small effects.
* **Plausible**; Results should have the ability to appear natural and not appear distractingly artificial.
* **Beautiful**; Results should appear aesthetically pleasing.
* **Accessible**; The implementation should be affordable to write and easy to understand.
* **Robust**; The implementation should be easy to modify and tune without introducing numerical instabilities.
* **Interactive**; The fluid should interact with other entities, including user-controlled, moving or stationary.
* **Parallelizeable**; The simulation code should exploit multiple cores when they are available.
* The simulation should handle **multiple immiscible fluids** such as water with air, **combustion** (fuel, plasma and exhaust).
* The simulation should work with 2D or 3D flows.

# Method

My choice of simulation uses a **vortex particle method**:
Particles representing small vortices (called "**vortons**") induce a velocity field which in turn moves the vortons.

Tracking vortons means the fluid lacks **numerical viscosity** that erodes detail in simulations that use grid-based methods.
Also, a relatively small number of vortons induces a pleasingly chaotic velocity field which drives a much larger number of passive tracer particles used for rendering.
Effects authors can control the number of vortons independently from the number of tracers, thereby separately controlling simulation and rendering resolution.

Grid-free particle methods such as the one these simulations employ only need particles in regions where the flow behaves "interestingly", which is near vortices.
Vortices induce fluid motion.
They also represent a (potentially very high resolution) velocity gradient which implies that a grid-based solution would require twice or more the resolution in each direction so for example to simulate the same flow in a grid-based simulation would require 2x2x2=8 times as many grid points -- and that assumes vortices densely populate the entire grid, which is unlikely.
Furthermore, grid-free methods can conform to any shape without expensive regridding techniques.

This simulation provides multiple velocity-from-vorticity solvers:
A **direct** solver provides a gold-standard exact (but slow) velocity calculation for comparison with other methods.
Some simplify far-field interactions by using integral methods, either **treecode** or a novel "**monopole**" approach.
Another uses a **multigrid Poisson** differential solver.
Yet another uses a treecode to compute vector potential at boundaries and a Poisson solver elsewhere, yielding the best of both approaches: speed, high quality and flexibility.

In one incarnation, the algorithm exploits **nearest-neighbor** information, and incidentally includes a new approximate dynamic nearest neighbor tracking algorithm that runs in linear time.
In the nearest-neighbor incarnation, the simulation approximates spatial gradients by comparing properties of each particle with that of its neighbors.

In another incarnation, the simulation transfers quantities from particles to a grid where spatial gradients are computed, then updates the source particles.

Vortex **stretching** and **tilting** plays a crucial role in the cascade from **laminar** to **turbulent** flow.
For visual effects, authors can inject pseudo-turbulence *a priori*.
This simulation includes vortex stretching and tilting by computing spatial gradients of velocity, but games may elect to omit those terms if they inject turbulence artificially.

To handle **stratified** fluids and multiple fluids with different densities the simulation must handle **baroclinic** generation of vorticity.
In principle that requires knowing both density and pressure gradients, but this simulation assumes pressure gradients come from hydrostatic equilibrium.
The simulation includes a **combustion model** which generates heat that feeds the baroclinic generator to create fire and smoke plumes.

A **particle strength exchange** (PSE) approach facilitates computing viscous and thermal diffusion, thereby avoiding a need to compute high-order spatial gradients using a grid.

Effects authors can control flows using two methods: They can **tune fluid flow parameters** and they can use **traditional particle operations** such as commonly occur in visual effects packages, like emit, wind, grow and kill.

## Results

* [My real-time fluid simulations](https://www.youtube.com/playlist?list=PL5F7B0289179F4297)
* [Fluid simulation of smoldering using vector potential from vortex particles](https://youtu.be/NthHWI6SsgA)
* [Vorton Combustion](https://youtu.be/Mdzq6Y2IP2c)
* [Red Giant Trapcode Suite 15 with NEW fluid dynamics](https://www.youtube.com/watch?v=5gZDfPyhrNQ)
  * [Red Giant Trapcode Particular 4](https://www.youtube.com/watch?v=xKnqYg5SxaM)
  * [Red Giant Trapcode Form](https://youtu.be/rigV1lhsJps)
  * [Red Giant Trapcode Suite 15](https://www.youtube.com/watch?v=Sx8AeiqR5fk)
  * [Red Giant Trapcode Fluid Physics First Look](https://youtu.be/J7IBmcOMSdM)
  * [Red Giant Form After Effects Fluid Sim Tutorial](https://youtu.be/8ioNFESLDko)
* [Curve ball](https://youtu.be/2f4zp6Kh7Yg)
* [Blue sinks, red floats](https://youtu.be/5b2Fm0R3hGA)
* [Explanation / Campfire / Airfoil / Dragon, Bunny, Skull, Beach ball / Turbine](https://youtu.be/MlFAzXeCB80)
* [Galaxy of Dots](https://youtu.be/rsJrJ-bzVSw)
* [Ballistic Mayhem](https://youtu.be/LdxJ_nfWt0M)
* [Atomic Vortex Ring](https://youtu.be/G9E8xEjGzk0)

## Publications

Intel Software Network and Gamasutra: Fluid Simulation for Video Games (series of articles)

1. Part 1: [Introduction to Fluid Dynamics](https://github.com/mijagourlay/VorteGrid/blob/master/Documents/FluidsForGames_Pt01-IntroToFluidDynamics.pdf) ([Intel archive](https://web.archive.org/web/20200426024331/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/)) ([Gamasutra](https://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php)) ([Venture Beat](https://venturebeat.com/2017/07/19/fluid-simulation-for-video-games-part-1/))
1. Part 2: Simulation Techniques ([Intel archive](https://web.archive.org/web/20190816113346/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2)) ([Gamasutra](https://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php))
1. Part 3: Vorton Simulation ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/))
1. Part 4: Fluid-Body Interaction ([Intel archive](https://web.archive.org/web/20191006005910/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4))
1. Part 5: Profiling and Optimization ([Intel archive](https://web.archive.org/web/20191005235439/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5))
1. Part 6: Differential Velocity Solvers ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/))
1. Part 7: Particle Operations ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/))
1. Part 8: Baroclinicity - Fluid Buoyancy ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/))
1. Part 9: Body Buoyancy ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/))
1. Part 10: Thermal Effects ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/))
1. Part 11: Combustion ([Intel archive](https://web.archive.org/web/20190816113345/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/))
1. Part 12: Jerkstrophy ([Intel archive](https://web.archive.org/web/20191006000816/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12))
1. Part 13: Convex Obstacles ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/))
1. Part 14: Convex Containers
1. Part 15: Smoothed Particle Hydrodynamics
1. Part 16: Hybrid VPM-SPH
1. Part 17: Fluid Surface Identification ([Intel archive](https://web.archive.org/web/20190816113349/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-17))
1. Part 18: Fluid Surface Rendering ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-18/))
1. Part 19: From Vorticity through Vector Potential to Velocity ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-19))
1. Part 20: Assigning Vector Potential at Boundaries ([Intel archive](https://web.archive.org/web/20190816113338/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-20))
1. Part 21: Recapitulation ([Intel archive](https://web.archive.org/web/20190816113337/https://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-21))

## Presentations

* 2010 July: [Fluid-body Simulation using vortex particle operations: International Conference on Computer Graphics and Interactive Techniques, Los Angeles, California, Session: Animation, article 8, 2010.](http://www.google.com/url?q=http%3A%2F%2Fportal.acm.org%2Fcitation.cfm%3Fid%3D1836845.1836854&sa=D&sntz=1&usg=AFQjCNE5400ftFSspvQfdUeRZN9EQzWL5g)
* 2009 August: High Performance Graphics (HPG) 2009, New Orleans, Louisiana
* 2009 March: Game Developers Conference (GDC) 2009, San Francisco, California
* 2008 August: [International Conference on Computer Graphics and Interactive Techniques archive, ACM SIGGRAPH 2008, Los Angeles, California, SESSION: Animation, Article No. 5.](https://dl.acm.org/doi/10.1145/1400885.1400891)

## News

* 2019 Dec 5:	[Frozen II VFX](https://youtu.be/M78pkQNeBbo) that uses my fluid sim in Red Giant Trapcode Suite 15 Particular 4
* 2019 Apr 27:	Port of [VorteGrid to WebAssembly and WebGL](https://www.buildingphysicsonline.com/MjgIntelFluidDemo/webgl.html) by Julien de Charentenay
* 2019 Jan 16:	[Red Giant Trapcode Suite 15.0.1 update with Particular 4.0.1 and Form 4.0.1](https://www.redgiant.com/blog/2019/01/16/free-update-trapcode-15-0-1/) with improvements to fluid simulation.
* 2019 Jan 15:	[AQUAMAN Underwater Effects using Red Giant Trapcode Suite with Particular](https://youtu.be/x4XjNWUXUd8) made with my fluid simulation. "The speed is phenomenal," says Daniel Hashimoto, who also gushes about the various vortex operators.

* 2018 Oct 30:	Red Giant adopted my fluid model into their Trapcode Suite 15 , Particular 4 and Form 4 products for Adobe After Effects.

* 2016 Nov 08:	Fluid Simulation for Video Games, part 21: Recapitulation .
* 2016 Jun 22:	Fluid Simulation for Video Games, part 20: Assigning Vector Potential at Boundaries .
* 2016 Apr 22:	Fluid Simulation for Video Games, part 19: From Vorticity through Vector Potential to Velocity . Video .

* 2014 Sep 9:	Fluid Simulation for Video Games, part 18: Fluid Surface Rendering

* 2013 Oct 8:	Fluid Simulation for Video Games, part 17: Fluid Surface Identification
* 2013 May 1:	Fluid Simulation for Video Games, part 16: VPM-SPH Hybrid
* 2013 Jan 30:	Fluid Simulation for Video Games, part 15: Smoothed Particle Hydrodynamics

* 2012 Aug 15:	Fluid Simulation for Video Games, part 14: Containers
* 2012 May 16:	Fluid Simulation for Video Games, part 13: Convex Obstacles
* 2012 Mar 15:	Fluid Simulation for Video Games, part 12: Jerkstrophy

* 2011 Dec 15:	Fluid Simulation for Video Games, part 11: Combustion
* 2011 July 8:	Fluid Simulation for Video Games, part 10: Thermal effects
* 2011 Mar 22:	Fluid Simulation for Video Games, part 9: Body Buoyancy
* 2011 Jan 28:	[Reddit linked to Intel vorton fluid sim articles](https://www.reddit.com/r/gamedev/comments/f9g8r/cool_article_on_fluid_simulation_for_games_a_bit/).
* 2011 Jan 25:	Farsthary (Raul Fernandez Hernandez) added a vorton fluid particle simulation to Blender .

* 2010 Nov 5:	Fluid Simulation for Video games: part 8 - Baroclinicity: Fluid Buoyancy
* 2010 July-Sept:	Fluid videos: Vorton combustion . Curve ball . Blue sinks, red floats .
* 2010 Aug :	Jayeson Lee-Steere ported the vorton fluid sim to iPhone. Runs in real time (by SIGGRAPH standards). First 3D fluid-body simulation on an iPhone? .
* 2010 July:	Fluid-body Simulation using vortex particle operations: International Conference on Computer Graphics and Interactive Techniques, Los Angeles, California, Session: Animation, article 8, 2010.
* 2010 Apr 20:	Interview on Intel Software Network TV show called Visualize This!
* 2010 Feb 10:	Fluid Simulation for Video games: part 7 - Particle Operations for Fluid Motion
* 2010 Jan 21:	Fluid Simulation for Video games: part 6 - Differential Velocity Solvers

* 2009 Dec 7:	Fluid Simulation for Video games: part 5 - Profiling and Optimization
* 2009 Oct 28:	Fluid Simulation for Video games: part 4 - Two-way Fluid-body Interaction
* 2009 Oct 28:	Fluid Simulation for Video games on Gamasutra . Part 2
* 2009 Oct 15:	Fluid Simulation for Video games on Gamasutra . Part 1
* 2009 Oct 2:	Fluid Simulation for Video games: part 3 - Vortex Particle Fluid Simulation
* 2009 July 21:	Fluid Simulation for Video games: part 2 - Fluid Simulation Techniques (PDF)
* 2009 July 21:	Fluid videos: Atomic vortex ring . Ballistic mayhem . Galaxy of Dots .
* 2009 July 7:	[Fluid Simulation for Video games: part 1 - Introduction to Fluid Dynamics](https://github.com/mijagourlay/VorteGrid/blob/master/Documents/FluidsForGames_Pt01-IntroToFluidDynamics.pdf) (Visual Adrenaline) (PDF)
* 2009 Jan 20:	Fluid Video. Click on this link, or choose "HQ" in the menu bar below, to see a higher quality version.

* 2008 Aug 11:	Fluid Simulation for Video games:
International Conference on Computer Graphics and Interactive Techniques ACM SIGGRAPH 2008 posters.
My poster at SIGgraph 2008. See A104, near the top.
Poster materials
