# The regolith project
Exploring Bullet Physics as a tool to simulate behavior of regolith and granular soils.  

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
Executables for simulations will be in `build/simulations` subdirectories
