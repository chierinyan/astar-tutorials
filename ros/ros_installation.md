## ROS Installation

This instruction will help you install ROS with Docker.


### References:

If you're just getting started with Docker, you are encouraged to make use of the following resources:

* [What is Docker](https://www.docker.com/whatisdocker/) - A page that will give you excellent high level overview of Docker and its purpose.
* [Docker Docs](https://docs.docker.com/) - Browse the online documentation and references to find information about Docker's various functions and tools.
---


### 0. Prerequisite

* Linux
* macOS
* Windows 11
    * You are ready

* Windows 10
    * **Home or Pro** 2004 (build 19041) or higher / **Enterprise or Education** 1909 (build 18363) or higher
        > To check your version, select **Win+ R**, type `winver`, select **OK**

    * [BIOS-level hardware virtualization](https://docs.docker.com/desktop/windows/troubleshoot/#virtualization-must-be-enabled)
    support enabled.

    * [WSL 2](https://docs.microsoft.com/en-us/windows/wsl/install-win10) feature enabled.


### 1. Install Docker Desktop

<https://www.docker.com/get-started>


### 2. Create a ROS container

```sh
docker run -d \
    --name astar_ros \
    gwentgod/astar-ros:melodic
```


### 3. Enter the container

```sh
docker exec -it astar_ros bash
```

You should see the shell prompt (the `user@host:dir$` at the beginning of each line in terminal)
changes to somethin like `root@d8c1817653d3:~#`.

Try
```sh
rosversion -d
```
The output should be
```
melodic
```


##### Congratulations, you've successfully installed ROS
---


### 4. Enable GUI

Optional. ~~GUI is for losers.~~

Sometimes GUI tools can be really helpful, but when you are directly operating robots, GUI is difficult to access.

* macOS

    On Host (which means the shell prompt should be `your_mac_username@your_computer_name...`,
    not `root@some_messy_code...`):

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

    On Host, every time before starting GUI

    ```sh
    socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
    ```

    This will run in foreground, so leave running and open another terminal to continue.

In Container, try `rosrun turtlesim turtlesim_node`

You should see a GUI window popup.

