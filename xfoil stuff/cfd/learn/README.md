# OpenFOAM first lessons: commands only

Read `index.html` in this folder for the explanations (open it in a browser yourself).

```
wsl -d MMAV-CFD                                   # open the OpenFOAM distro (never the QuadFlightStack distro)
cd ~/run/learn/01_cavity                          # lesson 1: lid-driven cavity (stock tutorial)
blockMesh && checkMesh && icoFoam > log.icoFoam   # about 0.3 s
tail log.icoFoam
rm -rf 0.[1-9]* VTK postProcessing log.* constant/polyMesh   # back to un-run

cd ~/run/learn/02_plate2d                         # lesson 2: 2D flat plate, TEACHING TOY, no accuracy claimed
./Allrun                                          # about 85 s, one core
python3 plot_forces.py                            # forces.png
python3 plot_slice.py                             # speed.png (no ParaView needed)
tail postProcessing/forceCoeffs1/0/coefficient.dat   # Time Cd Cd(f) Cd(r) Cl ... CmPitch
./Allclean                                        # back to un-run
```

Windows Explorer: `\wsl.localhost\MMAV-CFD\home\cfd\run\learn`
ParaView (not installed yet): open `...\learn\<case>\case.foam`.
If OpenFOAM commands are not found in a script: `source /usr/lib/openfoam/openfoam2606/etc/bashrc`.
Do not touch `~/run/vp_*` or `~/run/sweep/` while the sweep runs. Progress: `wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/sweep/sweep_status.sh`

## ParaView (installed on Windows, 5.12.0, `C:\Program Files\ParaView 5.12.0`)
Open `\wsl.localhost\MMAV-CFD\home\cfdun\learn_plate2d\case.foam` (run the case first).
Rebuild the pictures (no window opens): `"C:\Program Files\ParaView 5.12.0in\pvbatch.exe" --force-offscreen-rendering make_images.py`
(`make_images_sweep.py` needs the copy folder described at its top.) Rebuild the page: `python build_learn.py`
(reads `learn_src/` and `../validation_sweep/sweep_results.csv`).
