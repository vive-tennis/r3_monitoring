
# Vive R3 Monitoring 
(Realtime Remote Robot Monitoring Toolbox based on ROS)

<details>

<summary>Setup Docker for ROS (click to expand)</summary>


### Install docker-compose

```
sudo apt install ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
```

</details>

### Dockerfile
`docker pull ros:noetic-ros-base`

#### Install dependencies (temp)

```
apt update
apt install python3-pip
pip3 install paramiko simplejpeg psutil paho-mqtt mock
```

### Run docker
* interactively and access it through another terminal

`docker run -it -v "/home/ubuntu/.ros/:/root/.ros/" ros:XXX`

```
docker exec -it ros_docker /bin/bash
source /opt/ros/melodic/setup.bash
...
```

* No need to move the code inside the docker => mount the source code dir on the docker


### Get remote topics on local system
change inputs in ```/core/user_config.py``` file. CLIENT_ID should be match with access_token in remote system. then run ```r3_monotoing_user.py``` on your local system.
* rosocre should be running on your local system.
* mosquitto must be running on server on defined port in user_config file.

## TODO

- [x] Support ROS1
- [x] ~~Appear Program in Tray Bar~~
- [x] ~~Switch to PyQt for smoother integration~~
- [x] Add Client RQT Plugin
- [x] Show List of topics by name/type to include/exclude
- [ ] Make it a linux daemon
- [x] Start/Stop ROS Core
- [ ] Add REPL Terminal
- [ ] Support ROS2 
  - [ ] change docker base to `ros:foxy-ros1-bridge`
  - [ ] update r3_client.py script





