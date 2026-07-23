# patchwork-plusplus-ros2

ROS2 implementation of Patchwork++.
Tested with Ros2 Humble and Ubuntu 22.04. 

This is ROS package of Patchwork++ (@ IROS'22), which is a fast and robust ground segmentation method.

<p align="center"><img src=pictures/patchwork++.gif alt="animated" /></p>

> Original codebase [[repository](https://github.com/url-kaist/patchwork-plusplus)]

> ROS1 implementation [repository](https://github.com/url-kaist/patchwork-plusplus-ros)


## :gear: How to build Patchwork++
To build Patchwork++, you can follow below codes.

```bash
$ mkdir -p ~/patchworkpp_ws/src
$ cd ~/patchworkpp_ws
$ git clone 

$ catkin build # or catkin_make
```

## :runner: To run the demo codes
There is a demo which executes Patchwork++ with sample rosbag file. You can download a sample file with the following command.

> For the sample rosbag data, I utilizes [semantickitti2bag](https://github.com/amslabtech/semantickitti2bag) package.

```bash
$ wget https://urserver.kaist.ac.kr/publicdata/patchwork++/kitti_00_sample.bag
```
> If you have any trouble to download the file by the above command, please click [here][kitti_sample_link] to download the file directly.

[kitti_sample_link]: https://urserver.kaist.ac.kr/publicdata/patchwork++/kitti_00_sample.bag

> The rosbag file is based on the [KITTI][kittilink] dataset. The bin files are merged into the rosbag file format.

> The sample file contains LiDAR sensor data only.

[kittilink]: http://www.cvlibs.net/datasets/kitti/raw_data.php

Then, you can run demo as follows.

```bash
# Start Patchwork++
$ roslaunch patchworkpp demo.launch
# Start the bag file
$ rosbag play kitti_00_sample.bag
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

