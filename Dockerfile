FROM ripl/libbot2:latest

# arguments
ARG INSTALL_DIR=/usr/local

# environment
ENV RRTSTAR_INSTALL_DIR $INSTALL_DIR

ENV WORKING_DIR /root

# Clone and build dependencies

# rrtstar lcmtypes
RUN git clone https://github.com/ripl-ttic/rrtstar-lcmtypes ${WORKING_DIR}/rrtstar-lcmtypes
RUN cd ${WORKING_DIR}/rrtstar-lcmtypes/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# gridmap lcmtypes
RUN git clone https://github.com/ripl-ttic/gridmap-lcmtypes ${WORKING_DIR}/gridmap-lcmtypes
RUN cd ${WORKING_DIR}/gridmap-lcmtypes/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# obstacle-detector lcmtypes
RUN git clone https://github.com/ripl-ttic/obstacle-detector-lcmtypes ${WORKING_DIR}/obstacle-detector-lcmtypes
RUN cd ${WORKING_DIR}/obstacle-detector-lcmtypes/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# map lcmtypes
RUN git clone https://github.com/ripl-ttic/map-lcmtypes ${WORKING_DIR}/map-lcmtypes
RUN cd ${WORKING_DIR}/map-lcmtypes/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# geom-utils
RUN git clone https://github.com/ripl-ttic/geom-utils ${WORKING_DIR}/geom-utils
RUN cd ${WORKING_DIR}/geom-utils/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# gridmap-utils
RUN git clone https://github.com/ripl-ttic/gridmap-utils ${WORKING_DIR}/gridmap-utils
RUN cd ${WORKING_DIR}/gridmap-utils/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# clone carmen-utils
RUN git clone https://github.com/ripl-ttic/carmen-utils ${WORKING_DIR}/carmen-utils
RUN cd ${WORKING_DIR}/carmen-utils/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# clone carmen-map-utils
RUN git clone https://github.com/ripl-ttic/carmen-map-utils ${WORKING_DIR}/carmen-map-utils
RUN cd ${WORKING_DIR}/carmen-map-utils/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j

# clone check-gridmap
RUN git clone https://github.com/ripl-ttic/check-gridmap ${WORKING_DIR}/check-gridmap
RUN cd ${WORKING_DIR}/check-gridmap/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j


# copy source code
COPY ./ ${WORKING_DIR}/rrtstar/

# build rrtstar
RUN cd ${WORKING_DIR}/rrtstar/ && BUILD_PREFIX=$RRTSTAR_INSTALL_DIR make -j
