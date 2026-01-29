# ArUco Markers in Gazebo Classic - Issue Report

**Date**: 29 January 2026  
**Status**: RESOLVED ✅

---

## Solution Found!

The root cause of the gzclient crash was **modifying `os.environ` for `GAZEBO_RESOURCE_PATH` and `GAZEBO_MODEL_PATH` in the launch file**. This corrupted Gazebo's rendering context and caused the Camera pointer assertion failure.

### The Fix:
1. **Do NOT modify environment variables in the launch file**
2. **Install materials to Gazebo's default path** (`/usr/share/gazebo-11/media/materials/`)
3. **Use `__default__` URI** in SDF to reference materials from the default path

### Working Configuration:

**Material location:**
```
/usr/share/gazebo-11/media/materials/
├── scripts/
│   └── aruco.material
└── textures/
    ├── agent_0.png
    ├── agent_1.png
    ├── agent_2.png
    └── agent_3.png
```

**SDF material definition:**
```xml
<material>
  <script>
    <uri>__default__</uri>
    <name>Aruco/Marker0</name>
  </script>
</material>
```

---

## Original Problem Statement
We are trying to display ArUco marker PNG textures on TurtleBot3 robots in **Gazebo Classic (gazebo-11)** simulation. The goal is to replace solid color markers with actual ArUco pattern textures for robot identification.

---

## Environment
| Component | Version/Details |
|-----------|-----------------|
| Gazebo | Gazebo Classic 11 (uses Ogre3D rendering engine) |
| ROS | ROS 2 (Humble/Iron) |
| Robot | TurtleBot3 Burger |
| OS | Linux (Ubuntu) |

---

## What We Have

### Texture Files
- **Location**: `/home/gaurav/ros2_ws/src/turtlebot3/turtlebot3_description/media/materials/textures/`
- **Files**: `agent_0.png`, `agent_1.png`, `agent_2.png`, `agent_3.png`
- **Format**: 378x378 RGBA PNG files with ArUco patterns

### Material Script
- **Location**: `/home/gaurav/ros2_ws/src/turtlebot3/turtlebot3_description/media/materials/scripts/aruco.material`
- **Content**:
```ogre
material Aruco/Marker0
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture agent_0.png
      }
    }
  }
}

material Aruco/Marker1
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture agent_1.png
      }
    }
  }
}

material Aruco/Marker2
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture agent_2.png
      }
    }
  }
}

material Aruco/Marker3
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture agent_3.png
      }
    }
  }
}
```

### Launch File
- **Location**: `/home/gaurav/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/spawn_4_bots_aruco.launch.py`
- **Function**: `add_aruco_marker_to_sdf()` - dynamically adds marker visual to robot SDF

---

## Attempts Made

### Attempt 1: Inline PBR Material in SDF
**Approach**: Used `<pbr>` tags with `<albedo_map>` for texture path
```xml
<material>
  <pbr>
    <metal>
      <albedo_map>/path/to/agent_0.png</albedo_map>
    </metal>
  </pbr>
</material>
```
**Result**: ❌ Did not work - PBR materials are only supported in Gazebo Sim (Ignition), not Gazebo Classic

---

### Attempt 2: Ogre Material Script with model:// URI
**Approach**: Used `<script>` tags with model:// URI
```xml
<material>
  <script>
    <uri>model://turtlebot3_description/media/materials/scripts</uri>
    <uri>model://turtlebot3_description/media/materials/textures</uri>
    <name>Aruco/Marker0</name>
  </script>
</material>
```
**Result**: ❌ gzclient crashed - Material not found

---

### Attempt 3: Ogre Material Script with file:// URI
**Approach**: Used absolute file:// paths
```xml
<material>
  <script>
    <uri>file:///home/gaurav/ros2_ws/src/turtlebot3/turtlebot3_description/media/materials/scripts/aruco.material</uri>
    <uri>file:///home/gaurav/ros2_ws/src/turtlebot3/turtlebot3_description/media/materials/textures</uri>
    <name>Aruco/Marker0</name>
  </script>
</material>
```
**Result**: ❌ gzclient crashed with assertion error

---

### Attempt 4: SetEnvironmentVariable in Launch
**Approach**: Set `GAZEBO_RESOURCE_PATH` and `GAZEBO_MODEL_PATH` via `SetEnvironmentVariable` action in ROS 2 launch
```python
SetEnvironmentVariable(
    name='GAZEBO_RESOURCE_PATH',
    value='/path/to/media:' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
)
```
**Result**: ❌ Environment variables not set early enough before Gazebo starts

---

### Attempt 5: os.environ at Python Process Level
**Approach**: Set environment variables directly in Python before LaunchDescription
```python
import os
media_path = '/home/gaurav/ros2_ws/src/turtlebot3/turtlebot3_description/media'
os.environ['GAZEBO_RESOURCE_PATH'] = media_path + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
os.environ['GAZEBO_MODEL_PATH'] = '/home/gaurav/ros2_ws/src/turtlebot3/turtlebot3_description:' + os.environ.get('GAZEBO_MODEL_PATH', '')
```
**Result**: ❌ gzclient still crashes

---

## Error Observed
```
gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: 
typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const 
[with T = gazebo::rendering::Camera; ...]: 
Assertion `px != 0' failed.
```

This is a null pointer dereference in Gazebo's Camera/rendering subsystem when it tries to load materials.

**Note**: gzserver works fine and spawns all 4 robots successfully. Only gzclient (the GUI) crashes.

---

## Root Cause Analysis

1. **Box primitives lack proper UV mapping**: Gazebo's `<box>` geometry has basic UV coordinates, but custom texture mapping may not work as expected

2. **Ogre material path resolution**: Gazebo Classic's Ogre3D integration has specific requirements for finding material scripts and textures that may not be documented well

3. **Timing of resource loading**: Environment variables must be set BEFORE gzclient initializes its Ogre rendering context

4. **Material script format**: The Ogre .material format may need specific syntax or additional parameters for Gazebo Classic compatibility

5. **Resource group initialization**: Ogre may need materials registered in a specific resource group before they can be used

---

## What Works
✅ **Solid color markers** using inline `<ambient>`/`<diffuse>`/`<specular>` tags work perfectly because they don't require external texture files or material scripts.

```xml
<material>
  <ambient>1 0 0 1</ambient>   <!-- Red -->
  <diffuse>1 0 0 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
</material>
```

---

## Current Workaround
Using colored markers instead of ArUco textures:
- `robot_0` → 🔴 Red
- `robot_1` → 🟢 Green  
- `robot_2` → 🔵 Blue
- `robot_3` → 🟡 Yellow

Marker position: `-0.03 0 0.178` (relative to base_link)

---

## Possible Solutions to Investigate

1. **Create a proper Gazebo model** with `model.config` and `model.sdf` in `~/.gazebo/models/`
2. **Use a textured mesh (DAE/Collada)** instead of box primitive - mesh can have proper UV mapping
3. **Copy materials to Gazebo's default media path** (`/usr/share/gazebo-11/media/materials/`)
4. **Migrate to Gazebo Sim (Ignition)** which has better PBR material support
5. **Use decal/overlay approach** if supported

---

## Prompt for AI Research (Gemini Pro / ChatGPT / Claude)

```
I'm trying to apply custom PNG textures (ArUco markers) to box geometry in Gazebo Classic 11 (gazebo-11) which uses Ogre3D for rendering. I'm working with ROS 2 and TurtleBot3 simulation.

**My Setup:**
- Gazebo Classic 11 (NOT Ignition/Gazebo Sim)
- Ogre3D rendering engine
- PNG texture files (378x378 RGBA)
- Ogre .material script file defining materials

**What I'm trying to do:**
Add a small box visual to a robot model via SDF and apply a custom texture to it using Ogre material scripts.

**The Error:**
gzclient crashes with: "Assertion 'px != 0' failed" in boost shared_ptr related to gazebo::rendering::Camera

**Questions:**
1. What is the correct way to reference custom Ogre materials in SDF for Gazebo Classic 11?
2. How should I structure the .material file for Gazebo Classic compatibility?
3. What environment variables (GAZEBO_RESOURCE_PATH, GAZEBO_MODEL_PATH, OGRE_RESOURCE_PATH) need to be set, and what directories should they point to?
4. Does Gazebo Classic's box geometry support texture mapping, or do I need a mesh (DAE/STL) with proper UV coordinates?
5. Is there a working example of applying custom PNG textures to SDF visuals in Gazebo Classic 11?
6. Could the crash be related to material script syntax, missing textures, or Ogre resource group configuration?
7. How does Gazebo Classic register custom Ogre materials - does it need a resources.cfg file?
8. What is the correct directory structure for custom materials in Gazebo Classic?

Please provide specific code examples for SDF material definitions and Ogre .material script syntax that work with Gazebo Classic 11.
```

---

## File References
- Launch file: `src/turtlebot3_simulations/turtlebot3_gazebo/launch/spawn_4_bots_aruco.launch.py`
- Material script: `src/turtlebot3/turtlebot3_description/media/materials/scripts/aruco.material`
- Textures: `src/turtlebot3/turtlebot3_description/media/materials/textures/agent_*.png`
- Documentation: `TURTLEBOT3_CUSTOMIZATIONS.md`
