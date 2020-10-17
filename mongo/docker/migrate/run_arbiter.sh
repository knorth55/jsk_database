#!/bin/bash

PORT=30017
NAME=mongodb-arbiter
VERSION=3.4.17
LOG_PATH=/var/log/mongodb/mongod.log
REPLSET=jsk_robot_lifelog_set

docker run \
-p $PORT:27017 \
--name $NAME \
-d mongo:$VERSION \
mongod --replSet $REPLSET
