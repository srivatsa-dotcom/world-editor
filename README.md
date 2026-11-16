# World Editor: visual editor for Gazebo worlds

Open a Gazebo world (`.world` / `.sdf`), view it in 3D, move, rotate, resize, recolour, add and delete models,
and have every change written straight back into the SDF file.

There are two versions; use whichever suits you:

| | `gazebo_world_editor.html` | `sdf_world_editor.py` |
|---|---|---|
| How to run | Double-click it to open in Chrome or Edge | `python3 sdf_world_editor.py my.world` |
| Install | Nothing | Python 3.9+ (standard library only) |
| Finds `model://` / `package://` | Folders you drop in | `GAZEBO_MODEL_PATH`, `GZ_SIM_RESOURCE_PATH`, ROS packages, `--model-path` |
| Saves | Back into the dropped file (Chrome/Edge), or Download | Back into the file, keeps a `.bak` |

Both need internet once, to load three.js from a CDN.

## Features

- 3D view of models, includes, lights, primitives (box, cylinder, sphere, capsule, ellipsoid, plane) and meshes (STL, DAE, OBJ, glTF)
- Click to select; drag the gizmo to move (**W**) or rotate (**E**); optional snapping (0.1 m / 15°)
- Exact pose editing (x y z, roll pitch yaw in degrees)
- Change geometry sizes and colour; the matching `<collision>` is updated along with the `<visual>`
- Add box / cylinder / sphere (with inertia), duplicate (**Ctrl+D**), delete (**Del**), rename, toggle static
- Raw XML editor for anything else (plugins, sensors, joints…)
- Undo / redo (**Ctrl+Z / Ctrl+Y**), Live save toggle, Save (**Ctrl+S**)
- Keeps your comments and formatting; reloads automatically if the file changes on disk
- Handles SDF 1.4–1.11 poses (`degrees="true"`, `rotation_format="quat_xyzw"`)

## Using the HTML version

1. Open `gazebo_world_editor.html` in Chrome or Edge.
2. Drop your `.world` or `.sdf` file on it (or click **Open world**).
3. Optional: drop your workspace or models folder (or click **Add models folder**) so meshes and `model://` includes show up instead of red placeholder boxes.
4. Edit. With **Live save** on, the file is updated on every change. The browser asks once for permission to edit the file.

### Ubuntu: "read 0 bytes" or red boxes everywhere

Chromium and Firefox on Ubuntu are Snaps and can't read `/opt`, `/usr/share` or hidden folders such as `~/.gazebo` or `~/.gz`.
Copy the world, and the models it uses, into your home folder:

```bash
mkdir -p ~/worlds/models
cp /opt/ros/humble/share/nav2_simple_commander/warehouse.world ~/worlds/
chmod u+w ~/worlds/warehouse.world

# copy every model:// the world needs (and their own dependencies)
need=$(grep -ohE 'model://[A-Za-z0-9_.-]+' ~/worlds/warehouse.world | sed 's|model://||' | sort -u)
while [ -n "$need" ]; do
  next=""
  for m in $need; do
    [ -d ~/worlds/models/$m ] && continue
    src=$(find /opt /usr/share $HOME -type d -name "$m" -path "*models*" -not -path "$HOME/worlds/*" 2>/dev/null | head -1)
    if [ -n "$src" ]; then
      cp -rL "$src" ~/worlds/models/ && echo "copied  $m"
      next="$next $(grep -rhoE 'model://[A-Za-z0-9_.-]+' ~/worlds/models/$m 2>/dev/null | sed 's|model://||')"
    else echo "NOT FOUND: $m"; fi
  done
  need=$(echo $next | tr ' ' '\n' | sort -u)
done
```

Then drop `~/worlds/warehouse.world` and add `~/worlds/models` as a models folder.
For Fuel models in Gazebo Sim worlds: `cp -r ~/.gz/fuel ~/fuel_models` and add that folder.

## Using the Python version

```bash
python3 sdf_world_editor.py path/to/my.world            # opens the browser at http://localhost:8765
python3 sdf_world_editor.py my.world --model-path ~/my_models --port 9000 --no-browser
python3 sdf_world_editor.py new.world                   # creates a blank world (ground + sun)
```

Run it from a terminal where ROS/Gazebo is sourced so `model://` and `package://` resolve.
A copy of the original is kept as `my.world.bak`. If the world is a colcon `--symlink-install` link, the real file in `src/` is edited.

## Limitations

- `<include>`d models can be moved, renamed and set static, but their shapes live in their own model files, so they are read-only here (use Edit XML on the include, or edit the model's own SDF).
- Heightmaps and polylines aren't drawn.
- URDF / xacro are not SDF and aren't supported.
- Gazebo doesn't pick up file changes while running: restart it or reload the world.
