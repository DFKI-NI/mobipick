FROM ros:noetic-ros-core
ARG GITLAB_USER=gitlab-ci-token
ARG CI_JOB_TOKEN

ENV CATKIN_WS=/root/catkin_ws
ENV CATKIN_WS_SRC=${CATKIN_WS}/src
ENV CATKIN_PROJECT_DIR=$CATKIN_WS_SRC/mobipick
ENV DEBIAN_FRONTEND=noninteractive

# Enable system-wide authentication for git cloning via https
# This will leak the CI_JOB_TOKEN into the history and the intermediate
# containers, but that should be fine because it's only valid as long as the job is running.
RUN echo -e "machine git.ni.dfki.de\nlogin ${GITLAB_USER}\npassword ${CI_JOB_TOKEN}" > /root/.netrc

# Enable caching of APT packages (only works for packages.ubuntu.com and packages.ros.org)
RUN echo "Acquire::http::Proxy \"http://apt-cache.ni.dfki:8000\";" > /etc/apt/apt.conf.d/00proxy

# Install pre-commit hooks to /root/.cache/pre-commit/
RUN apt-get update -qq \
    && apt-get install -y -qq --no-install-recommends git python3-pip ruby shellcheck clang-format-10 python3-catkin-lint \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install pre-commit
RUN mkdir -p /tmp/pre-commit
COPY .pre-commit-config.yaml /tmp/pre-commit/
RUN cd /tmp/pre-commit \
    && git init \
    && pre-commit install-hooks \
    && rm -rf /tmp/pre-commit

# install dependencies
COPY mobipick_bringup/package.xml       $CATKIN_PROJECT_DIR/mobipick_bringup/package.xml
COPY mobipick_description/package.xml   $CATKIN_PROJECT_DIR/mobipick_description/package.xml
COPY mobipick_gazebo/package.xml        $CATKIN_PROJECT_DIR/mobipick_gazebo/package.xml
COPY mobipick_moveit_config/package.xml $CATKIN_PROJECT_DIR/mobipick_moveit_config/package.xml
COPY mobipick_pick_n_place/package.xml  $CATKIN_PROJECT_DIR/mobipick_pick_n_place/package.xml
COPY pose_teacher/package.xml           $CATKIN_PROJECT_DIR/pose_teacher/package.xml
COPY dependencies.rosinstall            $CATKIN_PROJECT_DIR/
COPY install-deps.sh                    $CATKIN_PROJECT_DIR/

WORKDIR $CATKIN_WS_SRC
RUN /bin/bash $CATKIN_PROJECT_DIR/install-deps.sh \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# build workspace
COPY . $CATKIN_PROJECT_DIR
RUN /bin/bash $CATKIN_PROJECT_DIR/build.sh

# cleanup
RUN rm /etc/apt/apt.conf.d/00proxy /root/.netrc
