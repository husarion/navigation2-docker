# navigation2-docker

Building a Docker image with Navigation 2.

Available for ROS distros:
- ROS 2 Humble

Demo in [rosbot-autonomy](https://github.com/husarion/rosbot-autonomy) repo.


## Building locally for multiple architectures

```bash
sudo apt install -y qemu-user-static binfmt-support
docker buildx build --platform linux/arm64,linux/amd64 -t nav2-test --build-arg PREFIX=vulcanexus- .
```
