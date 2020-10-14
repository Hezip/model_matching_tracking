# v_hezhenpeng-truck_pose_estimation
Match the object(truck) in the scene through by given template, and output **T: object --- > template**


## Getting Started

### Prerequisites

make install
```
PCL >= 1.7
yaml-cpp (https://github.com/jbeder/yaml-cpp)
teaser++ (https://github.com/MIT-SPARK/TEASER-plusplus)
```

### Building

```
mkdir build && cd build && cmake .. && make -j4
```


## Running the tests


Download the sample [data](https://ecloud.baidu.com?t=9fe0c59838eb6a938522bb0bd65e6016)


**Config.yaml** 

first read the config.yaml in *config/* !!! 


**For Match** you should run
```
./match model_left.pcd scene_left.pcd
```
```
Enter **SPACE** to show/hide the model
```

---

**For Track** you should run

```
./track r_model_s.pcd {.pcd folder} r_model_s.pcd
```

r_model_s.pcd -> the model used to match the point cloud

{.pcd folder} -> the data you want to track

truck_r.pcd   -> the model you want to show(you can repalce it with **r_model_s.pcd**)

```
Enter **Q** to quit the first window which show the initial guess
Enter **SPACE** to start tracking
```



## Built With

* [Zhenpeng](v_zhenpeng@baidu.com) - The Email Adress

## Versioning
Ground Segmentation ---> RANSAC ---> ICP ---> Tracking

## Acknowledgments

* Only **simple** tmplate && **uncalibrated** T: lidar--->template available!
* T: lidar--->template could be thinking as **Identity**(0,0,0,0,0,0,1)
  
![tf](./img/center.png)






