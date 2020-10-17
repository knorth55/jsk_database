
var conn = new Mongo("localhost");
var db = conn.getDB("jsk_robot_lifelog");

var res = db.pr1012.aggregate([
  // {"$limit": 100},
  {"$project": {"_id": 0,
                "_meta.stored_type": 1}},
  {"$group": {"_id": "$_meta.stored_type",
              "count": {"$sum": 1}}}
  ], {
    allowDiskUse: true
  })

printjson(res);
