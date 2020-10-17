#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from pathlib2 import Path
import pymongo
import re
import subprocess as sp
import time
from tqdm import tqdm
from threading import Thread
import sys
try:
    from Queue import Queue, Empty
except:
    from queue import Queue, Empty


REGEX = re.compile(r"^([0-9]+/[0-9]+)$")
POSIX = 'posix' in sys.builtin_module_names


def read(fp, queue):
    def _qput(_f, _q):
        for _l in iter(_f.readline, b''):
            _q.put(_l)
    t = Thread(target=_qput, args=(fp, queue))
    t.daemon = True
    t.start()
    return t


def run(cmd):
    proc = sp.Popen(cmd,
                    stdout=sp.PIPE, stderr=sp.PIPE,
                    bufsize=1, close_fds=POSIX)
    queue = Queue()
    tout = read(proc.stdout, queue)
    terr = read(proc.stderr, queue)

    with tqdm(total=100) as bar:
        bar.desc = cmd[0]
        while proc.poll() is None:
            try:
                progress = queue.get_nowait().split()
                progress = filter(REGEX.match, progress)
                cur, total = progress[0].split('/')
                perc = int(float(cur) / float(total) * 100)
                bar.update(perc - bar.pos)
            except Empty:
                pass
            except:
                try:
                    bar.desc = progress[1]
                except:
                    pass
            finally:
                time.sleep(0.1)

    print "'%s' exited with code %d" % (' '.join(cmd), proc.poll())
    if proc.poll() != 0:
        # proc.stderr.seek(0)
        print proc.stderr.read()
    return proc.poll()


def dump(addr, port, out, db=None, col=None):
    Path(out).mkdir(parents=True, exist_ok=True)
    cmd = ['mongodump', '--verbose', '--out', out, "--host", addr, "--port", str(port)]
    if db:
        cmd += ['--db', db]
    if col:
        cmd += ['--collection', col]

    return run(cmd) == 0


def restore(dump_path, db=None, col=None):
    if not Path(dump_path).exists():
        raise OSError("dump path %s not found" % dump_path)
    cmd = ["mongorestore", "--verbose"]
    if db:
        cmd += ['--db', db]
    if col:
        cmd += ['--collection', col]

    cmd += [dump_path]

    return run(cmd) == 0


if __name__ == '__main__':
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--host", help="hostname", default="133.11.216.217")
    p.add_argument("--port", help="port number",
                   type=int, default=27017)

    p.add_argument("--db", "-d", default=None,
                   help="Database to be backed up")
    p.add_argument("--col", "-c", default=None,
                   help="Collection to be backed up")

    p.add_argument("--path", "-p", default="/tmp/replicate",
                   help="Dump path")
    p.add_argument("--rm", action="store_true")

    args = p.parse_args()

    if not dump(args.host, args.port, args.path,
                args.db, args.col):
        exit(1)
    elif not restore(args.path, args.db, args.col):
        exit(1)

    if args.rm and Path(args.path).exists():
        import shutil
        try:
            shutil.rmtree(args.path)
        except Exception as e:
            print "Failed to dump path %s: %s" % (args.path, str(e))

    exit(0)
