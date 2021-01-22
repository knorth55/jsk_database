#!/usr/bin/env python

from datetime import datetime
import os
import signal
import subprocess
import sys
import time

import rospy


class PR2GdriveRecorder(object):
    def __init__(self):
        self.is_posix = 'posix' in sys.builtin_module_names
        self.video_path = rospy.get_param('~video_path', '/tmp')
        # default: 30 min
        self.duration = rospy.get_param('~duration', 60*30)
        self.gdrive_server_name = rospy.get_param(
            '~gdrive_server_name', 'gdrive_recorder_server')
        self.timer = rospy.Timer(
            rospy.Duration(self.duration), self._timer_cb)
        self.sigint_timeout = rospy.get_param('~sigint_timeout', 20)
        self.sigterm_timeout = rospy.get_param('~sigterm_timeout', 10)
        self.process = None
        self.video_title = None
        self._start_record()

    def _timer_cb(self, event):
        self.start_time = rospy.Time.now()
        prev_process = self.process
        self.process = None
        self._start_record()
        self._stop_record(prev_process)

    def _start_record(self):
        self.start_time = rospy.Time.now()
        stamp = datetime.fromtimestamp(
            int(self.start_time.to_time()))
        stamp = stamp.strftime('%Y%m%d_%H%M%S')
        self.video_title = '{}_video.avi'.format(stamp)
        cmds = [
            'roslaunch',
            'gdrive_recorder',
            'pr2_audio_video_recorder.launch',
            'video_path:={}'.format(self.video_path),
            'video_title:={}'.format(self.video_title),
            '--screen'
        ]
        self.process = subprocess.Popen(
            args=cmds,
            close_fds=self.is_posix,
            env=os.environ.copy(),
            preexec_fn=os.setpgrp()
        )
        rospy.loginfo('start recording in {}/{}'.format(
            self.video_path, self.video_title))

    def _stop_record(self, p):
        self._kill_process(p)
        rospy.loginfo('stop recording in {}/{}'.format(
            self.video_path, self.video_title))

    def _kill_descendent_processes(self, ppid):
        try:
            output = subprocess.check_output(
                ['ps', '--ppid=' + str(ppid), '--no-headers'])
        except subprocess.CalledProcessError:
            # ppid does not exist any more
            return True

        for process_line in output.split('\n'):
            strip_process_line = process_line.strip()
            if strip_process_line:
                pid = int(strip_process_line.split(' ')[0])
                name = strip_process_line.split(' ')[-1]
                rospy.loginfo('Killing {} {}'.format(name, pid))
                os.kill(pid, signal.SIGINT)

    def _kill_child_process(self, p, sigint_timeout, sigterm_timeout):
        if p is None:
            return 0
        if p.poll() is not None:
            return p.poll()
        # If it's still running, send signals to kill.
        try:
            # 1. SIGINT
            p.send_signal(signal.SIGINT)
            timeout = time.time() + sigint_timeout
            while time.time() < timeout:
                if p.poll() is not None:
                    return p.poll()
                time.sleep(0.1)
            # 2. SIGTERM
            rospy.loginfo("Escalated to SIGTERM")
            p.send_signal(signal.SIGTERM)
            timeout = time.time() + sigterm_timeout
            while time.time() < timeout:
                if p.poll() is not None:
                    return p.poll()
                time.sleep(0.1)
            # 3. SIGKILL
            rospy.loginfo("Escalated to SIGKILL")
            p.kill()
            p.wait()
            return p.poll()
        except Exception as e:
            rospy.logerr("Failed to kill child process: {}".format(e))
        return 0

    def _kill_process(self, p):
        exit_code = self._kill_child_process(
            p, self.sigint_timeout, self.sigterm_timeout)

        if p is not None:
            try:
                self._kill_descendent_processes(p.pid)
            except Exception as e:
                rospy.logerr(
                    "Failed to kill descendent processes: {}".format(e))
        return exit_code


if __name__ == '__main__':
    rospy.init_node('pr2_gdrive_recorder')
    recorder = PR2GdriveRecorder()
    rospy.spin()
