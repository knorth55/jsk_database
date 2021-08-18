# jsk_database

## Installation & Build

```bash
mkdir catkin_ws/src -p
cd c/src
wget https://raw.githubusercontent.com/knorth55/jsk_database/master/fc.rosinstall -o .rosinstall
wstool update -j 2
rosdep install --ignore-src --from-path . -y -r -c
cd ..
catkin build
```

## gdrive_recorder

See [./gdrive_recorder/README.md](./gdrive_recorder/README.md)

## jsk_database_scripts

See [./jsk_database_scripts/README.md](./jsk_database_scripts/README.md)
