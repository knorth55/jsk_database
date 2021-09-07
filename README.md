# jsk_database

[![main](https://github.com/knorth55/jsk_database/actions/workflows/main.yml/badge.svg)](https://github.com/knorth55/jsk_database/actions/workflows/main.yml)
[![linter](https://github.com/knorth55/jsk_database/actions/workflows/linter.yaml/badge.svg)](https://github.com/knorth55/jsk_database/actions/workflows/linter.yaml)

Long-term robot logging

## Installation & Build

```bash
mkdir catkin_ws/src -p
cd c/src
wget https://raw.githubusercontent.com/knorth55/jsk_database/main/fc.rosinstall -o .rosinstall
wstool update -j 2
rosdep install --ignore-src --from-path . -y -r -c
cd ..
catkin build
```

## gdrive_recorder

See [./gdrive_recorder/README.md](./gdrive_recorder/README.md)

## jsk_database_scripts

See [./jsk_database_scripts/README.md](./jsk_database_scripts/README.md)
