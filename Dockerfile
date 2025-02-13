#  Copyright (C) 2024 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

ARG DOCKER_ORG="usdotfhwastol"
ARG DOCKER_TAG="carma-system-4.9.0"
FROM usdotfhwastol/carma-base:carma-system-4.5.0
ENV GIT_BRANCH="master"
USER carma
WORKDIR /home/carma
COPY --chown=carma /docker ./docker
COPY /msger_mosaic_bridge ./msger_mosaic_bridge
RUN /home/carma/docker/install.sh

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="messenger-mosaic-bridge"
LABEL org.label-schema.description="Simulated communications driver for usage with CARMA Platform and CDASim"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-messenger-bridge"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch msger_mosaic_bridge mosaic_adapter_launch.py"]
