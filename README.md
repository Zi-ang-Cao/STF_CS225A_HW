# cs225a
This repository will contain the homeworks and demos for the class cs225a.

## Dependencies
The project depends on the sai2 libraries. You have received instructions to install sai2 in class.

## Build and make
In the main directory, create a build directory and build from that folder:
```Shell
cd /Users/zi-angcao/03_ResearchRepo/SAI2/OpenSai/STF_CS225A_HW
# mkdir -p homework/hw0/data_files

rm -rf build bin
mkdir build
cd build
cmake .. && make -j4
cd ..
```

## Run
Go to the bin folder and then to the folder of the application you want to run.
For hw0 for example:
```Shell
cd bin/hw0
./hw0

./hw0-viz
```

### hw0
You have 2 programs there. A visualizer and the actual homework file.
The visualizer is here to help you make sure you are doing what you think you are doing.
To run it, go to bin/hw0 and run `./hw0-viz`. You will see a window appear with the robot from hw0 in a configuration close to the one drawn on the pdf.
When you run hw0 and modify the values for the joints, it will modify the position of the visualized robot as long as you publish the new joint values to redis from hw0
```Shell
redis_client.setEigen(JOINT_ANGLES_KEY, robot->q());
```

### hw1
You have 2 programs there. A visualizer and the actual homework file.
```Shell
# First
sai_hw1_vis

# Second
sai_hw1 1   # sai_hw1 x, where x is the question id
```

