# CFD setup (OpenFOAM in WSL2)

- Distro: `MMAV-CFD` (Ubuntu 24.04, WSL2). Open with `wsl -d MMAV-CFD` (logs in as user `cfd`, passwordless sudo inside this distro only).
- OpenFOAM: ESI/openfoam.com v2606 (`openfoam2606-default` + tutorials), from the vendor apt repo
  (https://dl.openfoam.com/repos/deb noble main, key in /usr/share/keyrings/openfoam.gpg). Loaded automatically via ~/.bashrc.
- Also installed: gmsh 4.12, python3, numpy, matplotlib. No ParaView.
- Cases live in `~/run` inside the distro (/home/cfd/run). Not on OneDrive. To reach from Windows: `\wsl.localhost\MMAV-CFD\home\cfd\run`.
- Resources seen: 24 threads, ~15 GB RAM (WSL default is half of 31 GB; `.wslconfig` was not changed), 4 GB swap.
- Verified (2026-10-12): blockMesh, snappyHexMesh, pimpleFoam, checkMesh all run; icoFoam cavity serial ran to end (0.5 s) in ~0.1 s;
  cavity parallel (decomposePar, mpirun --oversubscribe -np 4 icoFoam -parallel, reconstructPar) ran to end in ~0.7 s.
  These are tiny cases; they prove the toolchain, not performance.
- Remove: `wsl --unregister MMAV-CFD` (deletes everything in it, including cases).
- The existing `QuadFlightStack` distro was not touched and is still the default.

Overnight AoA sweep of the Ananda AR 3 plate (vp_sweep_aXX cases in MMAV-CFD): see `validation_sweep/README.md`.
