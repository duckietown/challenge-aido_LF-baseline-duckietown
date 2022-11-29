# Definition of Submission container

ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}
ARG DOCKER_REGISTRY=docker.io

FROM ${DOCKER_REGISTRY}/duckietown/dt-car-interface:${BASE_TAG} AS dt-car-interface

FROM ${DOCKER_REGISTRY}/duckietown/challenge-aido_lf-template-ros:${BASE_TAG} AS template

FROM ${DOCKER_REGISTRY}/duckietown/dt-core:${BASE_TAG} AS base

WORKDIR /code

COPY --from=dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface

COPY --from=template /data/config /data/config

# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt

ARG PIP_INDEX_URL="https://pypi.org/simple/"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}

RUN python3 -m pip install -U   pipdeptree
COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN python3 -m pip install  -r .requirements.txt


RUN echo PYTHONPATH=$PYTHONPATH
RUN pipdeptree
RUN python3 -m pip list

RUN mkdir /code/solution
RUN mkdir /code/launchers
COPY solution /code/solution
COPY launchers/. /code/launchers

RUN mkdir /code/submission_ws
COPY --from=template /code/submission_ws /code/submission_ws

ENV HOSTNAME=agent
ENV VEHICLE_NAME=agent
ENV ROS_MASTER_URI=http://localhost:11311

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build --workspace ${CATKIN_WS_DIR}

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${CATKIN_WS_DIR}/devel/setup.bash  && \
    catkin build --workspace /code/solution

ENV DISABLE_CONTRACTS=1
CMD ["bash", "/code/launchers/run_and_start.sh"]
