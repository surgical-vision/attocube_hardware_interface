# Attocube Hardware Interface
Hardware interface to run attocube stages with the EC100 controller through ROS.

## Structure
There are 2 main classes to this interface: AttocubeActor and AttocubeDeviceManager  
Attocube actor holds all the actor information and provides member functions for appropriate ECC calls (anything actor 
specific).  
Attocube Device Manager manages the communication to the actual conntroller which is mainly opening and closing the 
connection but also polling the controllers to see what is connected. 


## Docker
### Build
There is a docker file that contains the packages and all the dependencies. 
Where the general workflow to build the image:  
- Clone the repo
- Build the docker image

```
git clone git@github.com:surgical-vision/attocube_hardware_interface.git
cd attocube_hardware_interface
docker build --pull --rm -f ./.docker/Dockerfile  -t attocube_hardware_interface:latest .
```
If you are changing the Dockerfile remove the `--rm` tag to keep your intermediate builds. 

### Running
My approach (2.3 from the [ROS guide](http://wiki.ros.org/docker/Tutorials/GUI))
```
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add dialout --group-add sudo \
    --env="DISPLAY" \
    --workdir="/catkin_ws/src" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    attocube_hardware_interface:latest
```
The container needs to be privileged to run as it needs access to the USB, the user added also needs to be in dialout and sudo.
This should be logging your host user in the container, mounting your home directory within the image and other things like x server info and sudo access.  
The other perk of this is your ssh keys are hopefully in `~/.ssh` so you can then push your changes. 

if you are going to use this container for a while then give it name with: `--name attocube_hardware_interface_dev`

Lastly the repo has been added in the docker process and is owned by root so the user id you've added won't be able to use it.
Change ownership to the user with 
`sudo chown -R $UID /catkin_ws/`