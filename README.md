# navigation2-docker

Building a Docker image with Navigation 2.

Available for ROS distros:
- ROS 2 galactic
- ROS 2 humble

Using in Docker Compose:

```yaml
  nav2:
    image: husarion/navigation2:galactic
    restart: unless-stopped
    volumes: 
      - ./config/nav2_params.yaml:/nav2_params.yaml
    command: >
      ros2 launch nav2_bringup navigation_launch.py    
        params_file:=/nav2_params.yaml
```
