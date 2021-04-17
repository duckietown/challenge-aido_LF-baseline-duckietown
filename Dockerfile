# Definition of Submission container

ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}
ARG AIDO_REGISTRY=docker.io

FROM ${AIDO_REGISTRY}/duckietown/dt-car-interface:${BASE_TAG} AS dt-car-interface

FROM ${AIDO_REGISTRY}/duckietown/challenge-aido_lf-template-ros:${BASE_TAG} AS template

FROM ${AIDO_REGISTRY}/duckietown/dt-core:${BASE_TAG} AS base

WORKDIR /code

COPY --from=dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface

COPY --from=template /data/config /data/config

# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt

ARG PIP_INDEX_URL
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}

RUN pip install -U "pip>=20.2" pipdeptree
COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN  pip3 install --use-feature=2020-resolver -r .requirements.txt


RUN echo PYTHONPATH=$PYTHONPATH
RUN pipdeptree
RUN pip list

RUN mkdir submission_ws
COPY submission_ws/src submission_ws/src
COPY launchers .

COPY --from=template /code/submission_ws/src/agent /code/submission_ws/src/agent

ENV HOSTNAME=agent
ENV VEHICLE_NAME=agent
ENV ROS_MASTER_URI=http://localhost:11311

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build --workspace ${CATKIN_WS_DIR}

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${CATKIN_WS_DIR}/devel/setup.bash  && \
    catkin build --workspace /code/submission_ws

ENV DISABLE_CONTRACTS=1
CMD ["bash", "run_and_start.sh"]
