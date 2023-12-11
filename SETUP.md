## Prerequisites

### GCC 7

Required for SMPL (`smpl_test` won't compile with newer versions).

```bash
sudo apt install gcc-7 g++-7
export CC="$(which gcc-7)"
export CXX="$(which g++-7)"
unset CMAKE_GENERATOR  # to use makefiles
```

### Build & install the `epase` branch of `sbpl`

```bash
git clone -b epase https://github.com/shohinm/sbpl \
    && cd sbpl
mkdir build \
    && cd build
cmake -Wno-dev .. \
    && make \
    && sudo make install
```

You might get this warning several times: `warning: throw will always call terminate() [-Wterminate]` but it is no issue.

## Prepare workspace

```bash
workspace=~/smpl_ws
mkdir -p "${workspace}"/src \
    && cd "${workspace}"/src
git clone -b base-u18 https://github.com/shohinm/leatherman
git clone -b add_fetch https://github.com/gsotirchos/smpl  # my fork OR
#git clone -b epase https://github.com/shohinm/smpl        # Shohin's branch
git clone --depth 1 https://github.com/KavrakiLab/robowflex \
    && env mv robowflex/robowflex_library . \
    && env rm robowflex_library/CMakeModules \
    && env mv robowflex/CMakeModules robowflex_library \
    && env rm -rf robowflex
rosdep install --from-paths "${workspace}"/src -i -r -y
```

You might get the following warning about `orocos_kdl` not having a rosdep definition:

```
sbpl_pr2_robot_model: Cannot locate rosdep definition for [orocos_kdl]
smpl_test: Cannot locate rosdep definition for [orocos_kdl]
sbpl_kdl_robot_model: Cannot locate rosdep definition for [orocos_kdl]
leatherman: Cannot locate rosdep definition for [orocos_kdl]
```

but it is no issue.

## Build workspace

```bash
cd "${workspace}"
catkin config --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -Wno-dev
catkin build -j 3
```

💡 Compile for Release with Debug messages: `RelWithDebInfo`

In case there are dependencies still missing  you can install them as requested by each error.

## Get the pre-generated problems dataset

```bash
roscd robowflex_library \
    && mkdir problems \
    && cd problems
for robot_name in "panda" "fetch"; do
    mkdir "${robot_name}" \
        && cd "${robot_name}"
    source <(curl -s https://raw.githubusercontent.com/KavrakiLab/motion_bench_maker/4115132bb55a9334529294643d6054471210a184/problems/download.sh) \
        "${robot_name}"
done
```

## Run example

```bash
source "${workspace}"/devel/setup.bash
roslaunch smpl_test goal_fetch_benchmarking.launch debug:=true profile:=false rviz:=true reverse:=false
```

- RViz will start in full-screen unless otherwise specified with `fullscreen:=false`.
- The outputs are saved in the files `~/occupied_voxels.csv` and `~/states_and_costs.csv`.
- You might have to press Ctrl+C and then 2× Ctrl+D to exit the debugger.
