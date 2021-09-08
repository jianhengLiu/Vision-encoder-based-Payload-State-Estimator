# Vision-encoder-based Payload State Estimation for Autonomous MAV With a Suspended Payload

![image-20210908104823823](README.assets/image-20210908104823823.png)

Autonomous delivery of suspended payloads with MAVs has many applications in rescue and logistics transportation. Robust and online estimation of the payload status is important but challenging especially in outdoor environments. The paper develops a novel real-time system for estimating the payload position; the system consists of a monocular fisheye camera and a novel encoder-based device. A Gaussian fusion-based estimation algorithm is developed to obtain the payload state estimation. Based on the robust payload position estimation, a payload controller is presented to ensure the reliable tracking performance on aggressive trajectories. Several experiments are performed to validate the high performance of the proposed method

**Authors:** [Jianheng Liu](https://github.com/jianhengLiu), [Yunfan Ren](https://github.com/RENyunfan), [Haoyao Chen](faculty.hitsz.edu.cn/chenhaoyao), and [Yunhui Liu](ri.cuhk.edu.hk/yhliu) from the [Networked Robotics and Systems Lab in Harbin Institute of Technology Shenzhen](http://nrs-lab.com/)



Due to the unique mechanism design, it it recommended to try our [Simulation](./Simulation) on simulator platform [CoppeliaSim](https://www.coppeliarobotics.com/coppeliaSim). The real world estimator is also provided in [Estimator](./Estimator)



**Related Paper**

- **Vision-encoder-based Payload State Estimation for Autonomous MAV With a Suspended Payload**, Jianheng Liu, Yunfan Ren, Haoyao Chen, and Yunhui Liu, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2021)[pdf](./paper/iros2021.pdf)



**Video**

<video src="README.assets/IROS2021-Video_x264.mp4"></video>

<video src="README.assets/drop_load-2020-09-11_15.55.29.mp4"></video>



<video src="README.assets/payload_trajectory_tracking-2020-08-10_17.26.17.mp4"></video>

