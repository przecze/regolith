# The regolith project
Exploring Bullet Physics as a tool to simulate behavior of regolith and granular soils.  
![cpt_picture](cpt_example.png)

## Requirements
Bullet Physics installed from source (preferably in /usr/local)  
Note: bullet installed via `apt install libbullet-dev` **will not work**  
because CommonInterfaces (used for examples) are not included in the package  

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
