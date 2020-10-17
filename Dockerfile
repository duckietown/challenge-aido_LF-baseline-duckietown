# Definition of Submission container

ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}

FROM duckietown/dt-car-interface:${BASE_TAG} AS dt-car-interface

FROM duckietown/dt-core:${BASE_TAG} AS base

COPY --from=dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python3-tk \
         python3-wheel \
         python3-pip  \
         software-properties-common && \
     rm -rf /var/lib/apt/lists/*

# RUN apt-get update -y && \
#   add-apt-repository ppa:deadsnakes/ppa -y && \
#   apt-get update -y && \
#   apt-get install -y python3.7-dev && \
#   ln -sf /usr/bin/python3.7 /usr/bin/python3

#WORKDIR /workspace
WORKDIR /code

RUN mkdir -p /data/config
# TODO this is just for the default.yamls - these should really be taken from init_sd_card
RUN git clone https://github.com/duckietown/duckiefleet.git /data/config


# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt

# Before installing
RUN echo PYTHONPATH=$PYTHONPATH
RUN pip install pipdeptree
RUN pipdeptree
RUN pip list


ARG PIP_INDEX_URL
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}

COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN  pip3 install --use-feature=2020-resolver -r .requirements.txt


RUN echo PYTHONPATH=$PYTHONPATH
RUN pipdeptree
RUN pip list

RUN mkdir /code/submission_ws

COPY submission_ws/src /code/submission_ws/src
COPY launchers /code

# let's copy all our solution files to our workspace
# if you have more file use the COPY command to move them to the workspace
COPY solution.py ./

# For ROS Agent - Additional Files
COPY rosagent.py ./

# FIXME: what is this for? envs are not persisted
RUN /bin/bash -c "export PYTHONPATH="/usr/local/lib/python3.7/dist-packages:$PYTHONPATH""


RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build --workspace ${CATKIN_WS_DIR}

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${CATKIN_WS_DIR}/devel/setup.bash  && \
    catkin build --workspace /code/submission_ws


# Note: here we try to import the solution code
# so that we can check all of the libraries are imported correctly
RUN /bin/bash -c "source ${CATKIN_WS_DIR}/devel/setup.bash && python3 -c 'from solution import *'"

ENV DISABLE_CONTRACTS=1
CMD ["bash", "run_and_start.sh"]
