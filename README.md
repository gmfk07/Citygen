Linux build
===========

```shell
sudo apt install mono-mcs
```

Download Unity from https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

Use version 2020.1.

Open as a Project in Unity Hub. You may need to edit `ProjectSettings/ProjectVersion.txt` beforehand to make it go through.

Once open in Unity, press the Play button while the SampleScene is open to generate a set of roadmaps. A roadmap on the Pareto front will be displayed. Press the "N" key to move to the next roadmap on the front. Press the "P" key to toggle the population display on/off. You can move around the scene using WASD, Shift, and the mouse.

The RoadSpawner.cs script contains the logic for our algorithm. Adjust the variables at the top of the script as desired. Most notably, PERLIN_OFFSET randomly seeds the population map.
