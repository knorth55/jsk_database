# jsk_database

## gdrive_recorder

## jsk_database_scripts

#### mongodb

JSK mongodb database server config

This project is originally created by Yuki Furuta (@furushchev) and modified by Shingo Kitagawa (@knorth55)

This project is moved from [knorth55/toolbox](https://github.com/knorth55/toolbox.git)

#### Installation

Please see [./jsk_database_scripts/mongodb/docker](./jsk_database_scripts/mongodb/docker) for more information.

#### Backup mongodb to QNAP

```
cd .jsk_database_scripts/mongodb
sudo bash ./backup_to_qnap.sh
```

### influxdb 

JSK influxdb database server config 

#### Systemctl service installation

```
sudo cp ./jsk_database_scripts/influxdb/systemd/* /etc/systemd/system
sudo systemctl daemon-reload
# enable your service
sudo systemctl enable jsk-pr1040-influxdb.service
```

### gdrive 

JSK gdrive server config 

#### Systemctl service installation

```
sudo cp ./jsk_database_scripts/gdrive/systemd/* /etc/systemd/system
sudo systemctl daemon-reload
# enable your service
sudo systemctl enable jsk-pr1040-gdrive.service
```

### grafana

JSK grafana server config

#### Grafana models for InfluxDB

You can find models JSON file in [./jsk_database_scripts/grafana/models](./grafana/models).
