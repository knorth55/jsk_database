#!/bin/bash

_THISDIR=$(cd $(dirname $0);pwd)

#
# veriables
PORT=27017
VERSION=3.4.17
NAME=mongo2-$VERSION
DB_PATH=/media/mongo2/mongodb_store
LOG_PATH=/var/log/mongodb/mongod.log
CONF_PATH=$_THISDIR/mongod.conf

if [ "`docker ps -a -q -f name=$NAME`" != "" ]; then
    if [ "`docker ps -q -f name=$NAME`" = "" ]; then
        docker start $NAME
    else
        echo "already exists"
        exit 1
    fi
else
    # run server
    docker run \
        --name $NAME \
        -p $PORT:27017 \
        -v $DB_PATH:/data/db \
        -v $CONF_PATH:/etc/mongod.conf \
        -d mongo:$VERSION \
        --config /etc/mongod.conf
fi


docker exec -it $NAME /usr/bin/tail -f $LOG_PATH
