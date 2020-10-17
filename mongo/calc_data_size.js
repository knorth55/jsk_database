
var conn = new Mongo("localhost");
var db = conn.getDB("jsk_robot_lifelog");

var oldest = db.pr1012.find().sort({"_meta.inserted_at": 1}).limit(1).next();
var newest = db.pr1012.find().sort({"_meta.inserted_at": -1}).limit(1).next();

var min_date = oldest["_meta"]["inserted_at"];
var max_date = newest["_meta"]["inserted_at"];

var one_month = 1000 * 60 * 60 * 24 * 30;
var from_date = min_date;

while (from_date < max_date) {
  var to_date = new Date(from_date.getTime() + one_month);

  var alllen = 0;
  var allsize = 0;
  db.pr1012.find({"_meta.inserted_at": {
    "$gt": from_date,
    "$lte": to_date,
  }}).forEach(function(doc) {
    alllen += 1;
    allsize += Object.bsonsize(doc);
  });
  var ret = {"len": alllen, "size": allsize};

  printjson({
    "from": from_date,
    "to": to_date,
    "data": ret
  });

  from_date = to_date;
}
