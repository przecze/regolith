FROM ubuntu:20.04
RUN apt update && DEBIAN_FRONTEND="noninteractive" apt install -y cmake git build-essential

# Install bullet3
WORKDIR /third_party
RUN git clone --depth 1 --branch 3.06 --single-branch https://github.com/bulletphysics/bullet3.git
WORKDIR /third_party/bullet3/cmake_build
# Enable multithreading
RUN cmake -DBULLET2_MULTITHREADING=on ..
# Enable profiler
RUN make -j4 install

# Install yaml-cpp
WORKDIR /third_party
RUN git clone --depth 1 --branch yaml-cpp-0.6.3 --single-branch https://github.com/jbeder/yaml-cpp.git
WORKDIR /third_party/yaml-cpp/cmake_build
RUN cmake ..
RUN make -j4 install

# Install json
WORKDIR /third_party
RUN git clone --depth 1 --branch v3.9.1 --single-branch https://github.com/nlohmann/json.git
WORKDIR /third_party/json/cmake_build
RUN cmake ..
RUN make -j4 install

# Install packgen
WORKDIR /third_party
# Note: we use a custom fork of packgen with cmake support added
RUN git clone --depth 1 --branch master --single-branch https://github.com/przecze/packgen.git
WORKDIR /third_party/packgen/cmake_build
RUN cmake ..
RUN make -j4 install

# Build the app
ADD ./ /app/
WORKDIR /app/build_docker
RUN cmake .. && make -j4
RUN ln -s ../simulations/ConePenetrationTest/config.example.yaml config.yaml
CMD ./simulations/ConePenetrationTest/ConePenetrationTest
