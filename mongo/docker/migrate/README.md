migrate from MongoDB 2.6 to 3.0

## Changes

- Use WiredTiger for storage engine
  - Fast
  - Less file size
- Support Compression of index / databases
- Manage memory usage (Enable more on-memory cache)

## Scripts

1. first run mongodb with replication set

- run_src.sh: script to run mongodb 3.0 with the existing database
  - Use 27017,28017 port
- run_dst.sh: script to run mongodb 3.0 with the new database
  - Use 29017 port

2. run arbiter which is need to run with replication set enabled

- run_arbiter.sh: script to run mongodb3.0 with virtual database
  - Use 30017 port

3. Start replication

```
$ mongo localhost:27017
> rs.initialize()
> rs.add("musca:29017")
> rs.add("musca:30017")
```

4. Wait for synchronization
