# Drone-Delivery-Landing-Zone-Detection Algorithms and Benchmarking Strategies

Work In Progress!

This repository consists of two things mainly as title suggests.
- Drone-Delivery-Landing-Zone-Detection Algorithms
- Proposed Benchmarking Strategy for these algorithms

Configuring the benchmarking framework:
The .yaml file present in lib/config/monte_carlo_benchmarking_config.yaml allows to configure the benchmarking parameters like pre-set heuristic hazard metrics specific to the UAV and your use case, algorithms to be benchmarked etc (will be elaborated as needed with addition of new algorithms).

Currently implemented algorithms can be found at lib/include/architecture.h under the commented line // ALGORITHMS ..............

To start the benchmarking do the following:

- ensure the nvida drivers and container toolkit are installed. 

- use the provided docker image that has all the required libraries installed.

    docker pull giri6937/lam:latest


- Clone this repo to a workspace that can be mounted into docker container

    cd to/your/desired/workspace
    git clone https://github.com/EXPX3/Drone-Delivery-Landing-Zone-Detection.git

- run the docker container and mount the repo into it with the below command

    sudo docker run --name DDLZD --rm -it --privileged -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY -v /tmp/.X11-unix:/tmp/.X11-unix -v /PATH_TO_YOUR_CLONED_REPO_DIR/Drone-Delivery-Landing-Zone-Detection:/home/airsim_user/Drone-Delivery-Landing-Zone-Detection --gpus all --network host --entrypoint /bin/bash giri6937/lam:latest

- inside the docker container run the below command to enter the cloned repo

    cd /home/airsim_user/Drone-Delivery-Landing-Zone-Detection

- to build the lib inside the cloned repo do the following

    cd lib  
    mkdir build
    cd build
    cmake ..
    make 
- once the build is successfull you should be able to see a executable file named main
- It is time to configure the .yaml file, for initial test set number of iterations to 1, ensure visualization is set to true to see the generated pointcloud for each run with varying parameter values within a provided range. 
- once configured execute the generated main executable as follows ...

    ./main

- with poppedup window with generated pointcloud active use mouse wheel to zoom in and out, press q to exit the window and continue the simulation to next step.
- with num of iterations set to 1, in total you will be pressing q for 125 times, given 5 hazardmetrics with varying each metric over a range of 5 values for 5 different algorithms..
- Ofcourse to get valid benchmarking results you should set the vizualization to false and num of iterations to at least 20 or more.


To test the algorithms implemented on your own pointcloud do the following:

- exit the container started before as we will be mounting your .pcd while starting it.
- change the config_path value to point to algo_testing_config.yaml instead of monte_carlo_benchmarking_config.yaml

- use the provide test.pcd file or mount the directory containing your .pcd while running the docker container as follows.  
    
    sudo docker run --name DDLZD --rm -it --privileged -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY -v /tmp/.X11-unix:/tmp/.X11-unix -v /PATH_TO_YOUR_CLONED_REPO_DIR/Drone-Delivery-Landing-Zone-Detection:/home/airsim_user/Drone-Delivery-Landing-Zone-Detection -v /PATH_TO_YOUR_OWN_PCD_FILES/pcds:/home/airsim_user/Drone-Delivery-Landing-Zone-Detection/pcds --gpus all --network host --entrypoint /bin/bash giri6937/lam:latest

- set your .pcd file path inside algo_testing_config.yaml relative to the docker container filesystem
- enable viszualization
- when a window pops out to vizualize the loaded pointcloud at the moment the window origin is not set to the load pointcloud origin, so you need to zoom out a couple of wheels to find the pointcloud.
. press q to close the window. This lets the process continue. You will be shown the results specific to specific algorithms. you will also see xz projections of potential landing delivery zone candidates, followd by the final selected candidates if the algorithm finds any. Note that there are some pre-set threshold values in place specific to an UAV and usecase so make sure you tune them to your specifc need when testing on your own .pcd.


## Simulation framework
![Simulation framework](media/simulation_framework.drawio_color.png)


Implemented algorithms to date and included in the Monte-carlo based benchmarking strategy.

### Algorithms Implemented

1. **Traditional Rule-Based Mathematical Models with Preset Heuristic Rules**:
    - Region_Growing_Segmentation
    - seq_overlap
    - kdtree_InflatingCircles
    - sequentialApproachKdtree
    - sequentialApproach

---


## Visualizations

### Algorithm Visualizations

| Algorithm                     | Visualization                              |
|-------------------------------|--------------------------------------------|
| **Region_Growing_Segmentation** | ![Region_Growing_Segmentation](media/regiongrowingseg.drawio.png) |
| **seq_overlap**               | ![seq_overlap](media/seq_throry_imple_resul.drawio.png)          |
| **kdtree_InflatingCircles**   | ![kdtree_InflatingCircles](media/inflating_circleskdtree.drawio.png) |

### Synthetic Point Clouds

Synthetic point clouds generated:

![synthetic_pointcloud](media/synthetic_pointcloud.drawio.png)

> **Note**: For detailed explanation of implementation with flow charts and pseudo code, refer to the paper published here (published by end of Aug 2025).

---

## Configuration

We use a `.yaml` file to configure algorithm selection and their parameters. You will find the self-explanatory file at the following path:

Drone-Delivery-Landing-Zone-Detection/lib/config/monte_carlo_benchmarking_config.yaml


> **Note**: More details will be added soon.

---