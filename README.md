# The regolith project
Exploring Bullet Physics as a tool to simulate behavior of regolith and granular soils.  
![cpt_picture](cpt_example.png)

## Note about requirements and build steps
Complete set-up and build instructions for Ubuntu 20.04 are provided in [the Dockerfile](Dockerfile).
Dockerfile should be treated as reference for setting up and building the project on your machine.
You can verify the correctness of the steps in Dockerfile by:
```
docker build -t regolith .
docker run regolith
```
Note, that these commands will take long time and won't install anything on your local machine.  
They will only allow you to run one example from _simulations_ directory without gui.  
They will not provide an easy way to modify and rebuild code or modify the configuration.  
Execute the commands used inside Dockerfile on your local machine to have fully prepared environment.  

In case of any problems with the installation, feel free to open an issue in Github

## Requirements
All packages listed below must be installed in a fashion that they are
discoverable by cmake `find_package` function.
For example installation steps, refer to [the Dockerfile](Dockerfile)
### Bullet Physics
Bullet Physics installed from source
Note: bullet installed via `apt install libbullet-dev` **will not work**  
because CommonInterfaces (used for examples) are not included in the package  

### yaml-cpp
Required for configuration files
https://github.com/przecze/packgen


### packgen
Packgen is a library created by authors of the "[An efficient algorithm to
generate random sphere packs in arbitrary
domains](https://www.sciencedirect.com/science/article/pii/S0898122116300864#!)"
paper. Original code is here: https://git.tecgraf.puc-rio.br/elozano/packgen
However to install it use this fork: https://github.com/przecze/pacgen where cmake support is added
Packgen is used for generating the samples for simulations.

## Build instructions
```
mkdir build
cd build
cmake ..
make
```

## Run simulations
For ConePenetrationTest, link example config to current directory:
```
ln -s simulations/ConePenetrationTest/config.example.yaml build/config.yaml
```
Executables for simulations will be in `build/simulations` subdirectories
