# ROS Docker

## usage

```
git clone https://github.com/masachika-kamada/ros-docker.git
cd ros-docker/[desired directory]
sh build.sh
sh run.sh
```

## add function

By adding the following to `.bashrc`, you can execute `docker exec -it [container] bash` with `de [container]`

```bash:.bashrc
function de() {
    CONTAINER="$1"
    docker exec -it $CONTAINER bash
}
```
