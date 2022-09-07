## ROS Installation

This instruction will help you install ROS with Docker.


### References:

If you're just getting started using Docker with ROS, you are encouraged to make use of the following resources:

- [What is Docker](https://www.docker.com/whatisdocker/) - A page that will give you excellent high level overview of Docker and its purpose. 
- [Documentation](https://docs.docker.com/) - Browse the online documentation and references to find information about Docker's various functions and tools. 
- [Using Docker with ROS](http://wiki.ros.org/docker) - Official ROS wiki for using Docker with ROS.
---


### 0. Prerequisite

* macOS, Linux and Windows 11
  * You are ready

* Windows 10
  * **Home or Pro** 2004 (build 19041) or higher / **Enterprise or Education** 1909 (build 18363) or higher

    > To check your version, select **Win+ R**, type `winver`, select **OK**

  * BIOS-level hardware virtualization support enabled.

    > For more information, see [Virtualization](https://docs.docker.com/desktop/windows/troubleshoot/#virtualization-must-be-enabled).

  * WSL 2 feature enabled.

    > For more information, see [WSL 2](https://docs.microsoft.com/en-us/windows/wsl/install-win10).


### 1. Install Docker Desktop

<https://www.docker.com/get-started>


### 2. Run a ROS Container

In Terminal

* X64

  ```sh
  docker run -itd -v ~/ros_shared:/root/shared --name ros osrf/ros:melodic-desktop-full /bin/bash
  ```

* ARM

  ```sh
  docker run -itd -v ~/ros_shared:/root/shared --name ros ros:melodic /bin/bash
  ```


### 3.  Setup

#### 3.1. Packages

  ```sh
  apt update
  apt upgrade
  apt install python3-pip
  pip3 install pyyaml rospkg
  ```


#### 3.2. Environment setup

```sh
echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

There will be no output for the above two lines


#### 3.4 Testing

Try `roscore` in the container

You should see the following output:

```
root@cac940d2ee67:/# roscore
... logging to /root/.ros/log/81e96716-12d5-11ec-809d-0242ac110003/roslaunch-cac940d2ee67-37.log

(omitted)

setting /run_id to 81e96716-12d5-11ec-809d-0242ac110003
process[rosout-1]: started with pid [58]
started core service [/rosout]

```


#### Congratulations, you've successfully installed ROS

---


### 4. Enable GUI

Optional. ~~GUI is for losers.~~

Sometimes GUI tools can be really helpful, but when you are directly operating robots, GUI is difficult to access.

* macOS

  In Container:

  ```sh
  echo "export DISPLAY=172.17.0.1:0" >> ~/.bashrc
  source ~/.bashrc
  ```

  There will be no output for the above two lines


  On Host:

  * install HomeBrew
  ```sh
  /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
  echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
  eval "$(/opt/homebrew/bin/brew shellenv)"
  ```

  * install socat and xquartz with HomeBrew
  ```sh
  brew install socat xquartz
  ```


  **RESTART YOUR COMPUTER** after installations are complted.

  On Host, every time before starting GUI in container:

  ```sh
  socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
  ```

  This will run in foreground, so leave running and open another terminal to continue.

In Container, try `xeyes`

You should see a GUI window popup.

