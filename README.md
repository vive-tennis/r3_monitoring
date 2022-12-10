# Vive R3 Monitoring 
(Realtime Remote Robot Monitoring Toolbox based on ROS)

<details>

<summary>Setup Docker for ROS (click to expand)</summary>


### Install docker-compose

```
apt install docker-ce docker-ce-cli
sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
```

### Run a docker interactively and access it through another terminal
`docker run -it -v "/home/ubuntu/.ros/:/root/.ros/" ros:XXX`

```
docker exec -it ros_docker /bin/bash
source /opt/ros/melodic/setup.bash
...
```

* No need to move the code inside the docker => mount the source code dir on the docker


</details>



