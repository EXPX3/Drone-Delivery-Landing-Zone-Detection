FROM yolo-lidar-fusion:live-ros

USER root
RUN /app/venv/bin/pip install --no-cache-dir \
    "git+https://github.com/ultralytics/CLIP.git@7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40"
RUN printf '%s\n' \
    /opt/ros/humble/local/lib/python3.10/dist-packages \
    /opt/ros/humble/lib/python3.10/site-packages \
    > /app/venv/lib/python3.10/site-packages/ros2_humble.pth

