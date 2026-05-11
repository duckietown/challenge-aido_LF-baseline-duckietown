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

COPY --from=duckietown-sdk src/duckietown/sdk/__init__.py /tmp/sdk_python38_compat/__init__.py
COPY --from=duckietown-sdk src/duckietown/sdk/compat.py /tmp/sdk_python38_compat/compat.py
RUN python3 -c "import pathlib, shutil, site; site_dir = next(pathlib.Path(path) for path in site.getsitepackages() if path.endswith(('dist-packages', 'site-packages'))); sdk_dir = site_dir / 'duckietown' / 'sdk'; sdk_dir.mkdir(parents=True, exist_ok=True); shutil.copy2('/tmp/sdk_python38_compat/__init__.py', sdk_dir / '__init__.py'); shutil.copy2('/tmp/sdk_python38_compat/compat.py', sdk_dir / 'compat.py')"

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
