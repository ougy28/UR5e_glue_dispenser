Revision 4: upper-arm and forearm dark housing regions now use the same
near-black RGB [0.035 0.035 0.035] as the previously updated housings.
Silver tube and blue cover regions are unchanged.

Revision 3: three circled housings (shoulder_link, wrist_1_link,
wrist_2_link) are near-black. Blue covers retained. Other meshes unchanged.

Revision 2: corrected elbow cover to the positive-Y outer cap of the
upper-arm mesh. Removed blue coloring on the connection side.
Other six link color datasets are unchanged.

UR5e visual color pack — MATLAB R2025b

1. Stop Simulink (disable Fast Restart if enabled).
2. Extract this folder into your existing project.
3. Add this folder to the MATLAB path, or make it the Current Folder.
4. With the original robotUE in the workspace, run:
   robotUEColored = UR5e_Apply_Colors(robotUE);
   figure; show(robotUEColored,qUE,'Visuals','on','Collisions','off');
5. In Simulation 3D Robot, set Rigid body tree to robotUEColored.
   Keep the Constant input qUE and all other settings unchanged.
6. Run the simulation again to rebuild the visual actors.

Keep UR5e_Color_Data.mat beside UR5e_Apply_Colors.m. The function generates
colored_meshes_v4/*.stl on first use and overwrites those generated files on rerun.
Run after your normal initialization; rebuilding robotUEColored from scratch
without applying the colors would restore the original gray visuals.
To undo: set the block's Rigid body tree back to robotUE and restart.

Appearance: approximate UR-style blue covers, silver links, dark housings.
Color boundaries were designed from the supplied link geometry; this is not
manufacturer-supplied material segmentation or a calibrated metallic shader.
The preview shows separate meshes in their local coordinates, not an assembly.

Geometry checks performed outside MATLAB: every original surface is retained
through triangle subdivision; surface areas and coordinate bounds match.
MATLAB/Simulink execution is not available in the authoring environment.
The supplied function checks visual transforms/bounds before replacing visuals,
and joint fixed transforms and collision-object counts after replacement.
Only visuals in a copy are edited. Glue dispenser, TCP, joints and collision
geometry are not edited. Prior collision checks are not Unreal physics checks.
