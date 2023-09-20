# UAV_data_collection

**Project Setup**


### If you are using router as communication between VOXL drone and PC for Data Saving
Generate an SSH key pair on the local machine (if you haven't already):
```
ssh-keygen
```

Copy the public key to the remote server:

```
ssh-copy-id root@172.16.0.33 # your local machine ip will be different so replace accordingly
```

```password: oelinux123```

```
git clone https://github.com/gaudelbijay/UAV_data_collection.git

cd UAV_data_collection/src

git clone https://github.com/ros-perception/vision_opencv.git -b melodic

cd ..

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

source  devel/setup.bash
```

```
sudo apt-get install expect

```

Open up 2 terminal 

```
# terminal 1

./adb_script.sh
```

```
# terminal 2
./data_collection.sh
```

### Dockerfile (creating docker image)

```
export DOCKER_BUILDKIT=0
docker buildx build --platform linux/arm64 -t data_collection -f Dockerfile . --load
```
### Save Docker Image to Tarball

```
docker save -o <path_for_tar_file> <image_name>:<tag>
```
Example: 

```docker save -o data_collection.tar data_collection:latest
```

### Transfering Docker Image (Tarball) to VOXL Drone
```
adb push data_collection.tar /root
```

### Load a Docker Image from a Tarball

```
docker load -i <path_to_tar_file>
```

Example:

```docker load -i data_collection.tar
```

### Run Docker image inside voxl drone:
```docker run -v /<path_in_the_drone>/:/root/data/ --privileged -it --rm data_collection:latest```

Example: 
```docker run -v /collectedData/:/root/data/ --privileged -it --rm data_collection:latest ```

```
    docker run \
    -v /collectedData/:/root/data/ \
    --network="host" \
    -e ROS_IP=172.16.0.33 \
    -e ROS_MASTER_URI=http://172.16.0.33:11311 \
    --privileged \
    -it --rm \
    data_collection:latest
```


### Pull collected Data into PC:

```my@pc:$ adb pull <source-path-on-drone> <destination-path-on-pc>```
Example: 

```
adb pull /collectedData ~/Desktop/ 
```