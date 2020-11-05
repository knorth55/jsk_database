# jsk_database

## mongodb

JSK mongodb database server config

This project is originally created by Yuki Furuta (@furushchev) and modified by Shingo Kitagawa (@knorth55)

This project is moved from [knorth55/toolbox](https://github.com/knorth55/toolbox.git)

## influxdb 

JSK influxdb database server config 

### Systemctl service

#### Installation

```
sudo cp ./influxdb/systemd/* /etc/systemd/system
sudo systemctl daemon-reload
# enable your service
sudo systemctl enable jsk-pr1040-influxdb.service
```

## Grafana

JSK grafana server config

### Grafana models for InfluxDB

You can find models JSON file in [./grafana/models](./grafana/models).
