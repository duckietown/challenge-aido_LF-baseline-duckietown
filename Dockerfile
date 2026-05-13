# syntax=docker/dockerfile:1.4

# Definition of Submission container
ARG DOCKER_REGISTRY=docker.io
ARG ARCH=amd64
ARG DISTRO=ente
ARG BASE_TAG=${DISTRO}-${ARCH}

FROM ${DOCKER_REGISTRY}/duckietown/challenge-aido_lf-template-ros:${BASE_TAG}

ARG PIP_INDEX_URL="https://pypi.org/simple/"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}

WORKDIR /code

COPY ./dependencies.* ./
RUN dt-pip3-install "./dependencies.*"

COPY assets/calibrations /tmp/runtime-calibrations
COPY ./scripts/install-runtime-calibrations.sh /usr/local/bin/install-runtime-calibrations
RUN chmod +x /usr/local/bin/install-runtime-calibrations && \
    /usr/local/bin/install-runtime-calibrations \
        /tmp/runtime-calibrations \
        /data/config/calibrations \
        map_0/vehicle_0

COPY ./solution/. "${DT_PROJECT_PATH}/packages/agent"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build --workspace "${CATKIN_WS_DIR}/"

COPY ./launchers/. /code/launchers/

CMD ["bash", "/code/launchers/run_and_start.sh"]
