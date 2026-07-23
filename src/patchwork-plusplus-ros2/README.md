# patchwork-plusplus-ros2

ROS2 implementation of Patchwork++.
Tested with Ros2 Humble and Ubuntu 22.04.

This is ROS package of Patchwork++ (@ IROS'22), which is a fast and robust ground segmentation method.

<p align="center"><img src=pictures/patchwork++.gif alt="animated" /></p>

> Original codebase [[repository](https://github.com/url-kaist/patchwork-plusplus)]

> ROS1 implementation [repository](https://github.com/url-kaist/patchwork-plusplus-ros)

## :gear: How to build and run Patchwork++

To build Patchwork++, you can follow below codes.

```bash
$ mkdir -p ~/patchworkpp_ws/src
$ cd ~/patchworkpp_ws/src
$ git clone https://github.com/kaancolak/patchwork-plusplus-ros2.git
$ cd ..
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ colcon build 
$ ros2 run patchwork_plusplus patchwork_plusplus_exe --ros-args --params-file ~/patchworkpp_ws/src/patchwork-plusplus-ros2/patchwork_plusplus/config/patchwork_plusplus.param.yaml
```

Run your bag file :

```bash
$ ros2 bag play kitti_00_sample.bag
```

## Citation

If you use our codes, please cite our [paper][patchwork++arXivLink].

In addition, you can also check the paper of our baseline(Patchwork) [here][patchworkarXivlink].

[patchwork++arXivLink]: https://arxiv.org/abs/2207.11919

[patchworkarXivlink]: https://arxiv.org/abs/2108.05560

```
@inproceedings{lee2022patchworkpp,
    title={{Patchwork++: Fast and robust ground segmentation solving partial under-segmentation using 3D point cloud}},
    author={Lee, Seungjae and Lim, Hyungtae and Myung, Hyun},
    booktitle={Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.},
    year={2022},
    note={{Submitted}} 
}
```

```
@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
}
```


