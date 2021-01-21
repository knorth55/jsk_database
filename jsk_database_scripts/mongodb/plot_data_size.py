#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from collections import defaultdict
from bson.json_util import loads
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


with open("data_size.json") as f:
    data = loads(f.read())

df = defaultdict(list)
total = 0
for i, d in enumerate(data):
    if i < 7: continue
    df["index"].append(i)
    df["from"].append(d["from"].strftime("%Y-%m"))
    df["to"].append(d["to"])
    df["len"].append(d["data"]["len"])
    size = d["data"]["size"] * 1e-3 * 1e-3 * 1e-3  # GB
    df["size"].append(size)
    total += size
    df["total"].append(total)

df = pd.DataFrame(df)
print df.head()
ax = df.plot(x="from", y="total", grid=True)
plt.tight_layout()
plt.savefig("data_size.pdf", bbox_inches='tight', pad_inches=0.0)
plt.close("all")
