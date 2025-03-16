# `var_n7k_beadando` package
ROS 2 python package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/samuvarga/var_n7k_beadando
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select var_n7k_beadando --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 launch var_n7k_beadando archery_target_launch.py
```
## Visualization

### Archery Target Visualization

![Archery Target Visualization](img/ezgif.com-optimize.gif)

### Graph

![Graph](img/graph.png)

## Challenges

A turtlesim alkalmazásban a teknősök textúráját módosítani kellett, mert az eredeti 48×48 pixeles képek helyett 45×45 pixeles textúrákra volt szükség. A méretkülönbség miatt a teknősök mozgásuk során nem rajzoltak koncentrikus köröket.

A probléma megoldásához a galactic és rolling nevű teknős textúrákat ardent nevű textúrára cseréltem, amely megfelelő méretű, így biztosítva a pontos rajzolást.

In the turtlesim application, the turtle textures had to be modified because the original 48×48 pixel images needed to be replaced with 45×45 pixel textures. Due to the size difference, the turtles did not draw concentric circles during their movement.

To solve this issue, I replaced the galactic and rolling turtle textures with the ardent texture, which has the correct size, ensuring accurate drawing.

<div style="display: flex; justify-content: space-around;">
    <div style="text-align: center;">
        <img src="img/galactic.png" alt="Image 1" width="150" height="150"/>
        <h5>Galactic</h5>
    </div>
    <div style="text-align: center;">
        <img src="img/rolling.png" alt="Image 2" width="150" height="150"/>
        <h5>Rolling</h5>
    </div>
    <div style="text-align: center;">
        <img src="img/ardent.png" alt="Image 3" width="150" height="150"/>
        <h5>Ardent</h5>
    </div>
</div>