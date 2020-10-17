# mongo docker

## start all containers

```bash
docker-compose up
```

## get log of all containers

```bash
docker-compose logs -f --tail="100"
```

## initializae containers

First,  remove all docker images and volumes.

Then,  remove all `mongodb_store` dirs in `/media/mongo` and `/media/mongo2`.

```bash
# docker-compose up mongo-primary
mongo --port 27017
rs.initiate()
# docker-compose up mongo-secondary
rs.add("musca:28017")
# docker-compose up mongo-arbiter
rs.addArb("musca:29017")
cfg = rs.config()
cfg.members[0].host = "musca:27017"
cfg.members[0].priority = 100
rs.reconfig(cfg)
```
