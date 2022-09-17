## Linux

* GNU: GNU's Not Unix
* Linux: Linux Is Not UniX

* Distributions

    ![dist](https://s1.ax1x.com/2022/09/14/vxnpy8.jpg)

    * *debian* (*debian*, *ubuntu*, ...) / *redhat* (*RHEL*, *CentOS*, *fedora*, ...)
    * Astar: *ubuntu* 18.04 (ROS1, OPENCV3)


* Install

    * Dual boot (Asahi for Apple Silicon)
    * Win: [WSL](https://docs.microsoft.com/en-us/windows/wsl/install)
    * VM
        * X64: VMWare / VirtualBox
        * Apple Silicon: [Parallels Desktop](https://www.parallels.com)
    * Cloud
        * AWS
        * MS Azure
        * Google
        * Oracle
    * [Docker](https://www.docker.com/products/docker-desktop/)
        ```sh
        docker run -itd --name ubuntu ubuntu:bionic bash  # Create container
        docker start ubuntu                               # Strat container
        docker exec -it ubuntu bash                       # Enter container
        ```


* Shell Prompt / Bash prompt

    Indicates where you are
    ```
    <user>@<host>:<dir>$
    trainees@micrl-327-02:~$
    ```


* SSH && SFTP

    * `ssh user@host`
        * -p server port (default 22)
        * -X ForwardX11 (GUI)
            > macOS: require *XQuartz*
            >
            > Win: require *putty* + *Xming*
            >
            > **Reboot your computer** after installing the above dependency!
        * -J ProxyJump
    * `ssh-keygen, ssh-copy-id`
        > `id_rsa.pub`: public key
        >
        > `id_rsa`: private key, never leaves your device, trate it as your personal passwords.
    * config file
        ```
        # ~/.ssh/config
        Host <friendly_name>
            HostName <host_domain/ip>
            User <username>
        ```
    * *termius* - Cross platform SSH client
    * SFTP: SSH File Transfer Protocol
        > For transfer file between hosts
        ```
        you@local:~$  sftp user@host
        sftp>         get <remote file>
        sftp>         put <local file>
        ```


* Commands

    * `cmd [sub_cmd] [--option/-o] [target]`
    * Commonly used cmds
        * `man`
        * `echo, cat`
        * `cd, ls, pwd, mv, cp, touch, rm, mkdir, rmdir`
        * `wget`
        * `tar -xzvf`
        * `grep, diff`


* File, dir

    * Anything is file to Linux
    * `~`: home dir
    * `.`: current dir
    * `..`: parent dir


* File permissions && User permissions

    * `ls -l`
        ```
        d      rwx     r-x     r-x
        type   owner   group   others
        ```
    * `sudo`: run command as root


* Envionment variables

    * naming: SCREAMING_SNAKE
    * get variable list with `env`
    * get one value with `echo $VAR_NAME`
    * set with `export NAME=value`
        > Do **NOT** put spaces around `=`.


* Shortcuts

    * `<Ctrl-c>` Terminate current process
    * `<Ctrl-d>` Exit interactive process
    * `<Ctrl-z>` Suspended current process
        > Bring it back with `fg <No.>`


* *apt* - package manager for *debian* systems

    ```sh
    apt update
    apt upgrade
    apt install <package>
    ```
    For interactive envionment, or
    ``` sh
    apt-get update
    apt-get upgrade
    apt-get install <package>
    ```
    for scripts


* *pip* - package manager for python

    Install *pip*
    ```sh
    apt install python3-pip
    python3 -m pip install --upgrade pip
    ```

    Install new packages
    ```sh
    python3 -m pip install <package>
    # pip3 install <package> (Discouraged)
    ```


* *conda* - environment management system for python


* Editors
    * VIM
        * In normal mode
            * press `i` to enter insert mode
            * press `:wq<return>` to save and quit
            * press `:q<return>` to quit
            * press `:q!<return>` to force quit with out save
        * In any other modes
            * press `<Esc>` multiple times always brings you back to normal mode
    * nano

