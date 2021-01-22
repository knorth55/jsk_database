#!/usr/bin/env python

from datetime import datetime
import os
import signal
import subprocess
import sys
import time

import rospy

from gdrive_ros.srv import MultipleUpload
from gdrive_ros.srv import MultipleUploadRequest


class PR2GdriveRecorder(object):
    def __init__(self):
        self.is_posix = 'posix' in sys.builtin_module_names
        self.video_path = rospy.get_param('~video_path', '/tmp')
        # default: 30 min
        self.record_duration = rospy.get_param('~record_duration', 60*30)
        self.upload_duration = rospy.get_param('~upload_duration', 60*30)
        self.gdrive_server_name = rospy.get_param(
            '~gdrive_server_name', 'gdrive_record_server')
        self.upload_parents_path = rospy.get_param(
            '~upload_parents_path', 'pr2_recorder')
        self.record_timer = rospy.Timer(
            rospy.Duration(self.record_duration),
            self._record_timer_cb)
        self.upload_timer = rospy.Timer(
            rospy.Duration(self.upload_duration),
            self._upload_timer_cb)
        self.sigint_timeout = rospy.get_param('~sigint_timeout', 3)
        self.sigterm_timeout = rospy.get_param('~sigterm_timeout', 3)
        self.process = None
        self.video_title = None
        self._start_record()

    def _upload_timer_cb(self, event):
        file_titles = os.listdir(self.video_path)
        file_titles = [
            x for x in file_titles if x.endswith('_pr2_record_video.avi')]
        if self.video_title in self.file_titles:
            file_titles.remove(self.video_title)
        if len(file_titles) == 0:
            return
        file_paths = ['{}/{}'.format(self.video_path, x) for x in file_titles]

        req = MultipleUploadRequest()
        req.file_paths = file_paths
        req.file_titles = file_titles
        req.parents_path = self.upload_parents_path
        req.use_timestamp_folder = False
        req.use_timestamp_file_title = False
        gdrive_upload = rospy.ServiceProxy(
            self.gdrive_server_name + '/upload_multi',
            MultipleUpload
        )
        res = gdrive_upload(req)
        for success, file_path in zip(res.successes, file_paths):
            if success:
                rospy.loginfo('Upload succeeded: {}'.format(file_path))
                os.remove(file_path)
            else:
                rospy.loginfo('Upload failed: {}'.format(file_path))

    def _record_timer_cb(self, event):
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
        self.video_title = '{}_pr2_record_video.avi'.format(stamp)
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
