#!/usr/bin/env python

from datetime import datetime
import influxdb
import os
import pytz
import signal
import subprocess
import sys
import time

import rospy

from gdrive_ros.srv import MultipleUpload
from gdrive_ros.srv import MultipleUploadRequest
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class GdriveRecorder(object):
    def __init__(self):
        self.is_posix = 'posix' in sys.builtin_module_names
        self.video_path = rospy.get_param('~video_path', '/tmp')
        # default: 20 min
        self.record_duration = rospy.get_param('~record_duration', 60*20)
        self.upload_duration = rospy.get_param('~upload_duration', 60*20)
        timezone = rospy.get_param('~timezone', 'UTC')
        self.robot_type = rospy.get_param('~robot_type', 'pr2')
        self.robot_name = rospy.get_param('~robot_name', self.robot_type)
        self.gdrive_server_name = rospy.get_param(
            '~gdrive_server_name', 'gdrive_record_server')
        self.upload_parents_path = rospy.get_param(
            '~upload_parents_path', '{}_recorder'.format(self.robot_name))
        self.sigint_timeout = rospy.get_param('~sigint_timeout', 3)
        self.sigterm_timeout = rospy.get_param('~sigterm_timeout', 3)
        self.store_url = rospy.get_param('~store_url', True)
        if self.store_url:
            host = rospy.get_param('~host', 'localhost')
            port = rospy.get_param('~port', 8086)
            database = rospy.get_param('~database', 'test')
            self.client = influxdb.InfluxDBClient(
                host=host, port=port, database=database)
            self.client.create_database(database)

        self.record_timer = rospy.Timer(
            rospy.Duration(self.record_duration),
            self._record_timer_cb)
        self.upload_timer = rospy.Timer(
            rospy.Duration(self.upload_duration),
            self._upload_timer_cb)
        self.upload_service = rospy.Service(
            '~upload', Trigger, self._upload_service_cb)

        self.decompress_process = subprocess.Popen(
            args=['roslaunch', 'gdrive_recorder',
                  '{}_decompress.launch'.format(self.robot_type)],
            close_fds=self.is_posix,
            env=os.environ.copy(),
            preexec_fn=os.setpgrp()
        )
        self.process = None
        self.video_title = None
        self.tz = pytz.timezone('UTC')
        self.localtz = pytz.timezone(timezone)
        self._start_record()

    def _upload_timer_cb(self, event):
        self._upload()

    def _upload_service_cb(self, req):
        result = self._upload()
        success = True
        message = ''
        fail_count = 0
        if not result:
            success = False
            message = 'No file uploaded'
            return TriggerResponse(success=success, message=message)

        for res in result:
            for suc in res.successes:
                if not suc:
                    success = False
                    fail_count = fail_count + 1
        if fail_count > 0:
            message = 'Failed to upload {} files'.format(fail_count)
        return TriggerResponse(success=success, message=message)

    def _upload(self):
        rospy.loginfo('start uploading')
        file_titles = os.listdir(self.video_path)
        file_titles = [
            x for x in file_titles if x.endswith(
                '_{}_record_video.avi'.format(self.robot_name))]
        if self.video_title in file_titles:
            file_titles.remove(self.video_title)
        if len(file_titles) == 0:
            rospy.loginfo('No file found to upload')
            return None
        file_paths = ['{}/{}'.format(self.video_path, x) for x in file_titles]

        file_days_dict = {}
        for file_id, file_title in enumerate(file_titles):
            file_day = file_title.split('_')[0]
            if file_day in file_days_dict:
                file_days_dict[file_day].append(file_id)
            else:
                file_days_dict[file_day] = [file_id]

        result = []
        for file_day in sorted(file_days_dict.keys()):
            file_ids = file_days_dict[file_day]
            upload_file_ids = []
            for file_id in file_ids:
                file_path = file_paths[file_id]
                file_size = os.path.getsize(file_path)
                # check if file size is not empty
                if file_size == 0:
                    os.remove(file_path)
                    rospy.logwarn('Filesize is 0: {}'.format(file_path))
                else:
                    upload_file_ids.append(file_id)

            if len(upload_file_ids) == 0:
                continue

            upload_parents_path = self.upload_parents_path + '/' + file_day
            upload_file_paths = [
                x for i, x in enumerate(file_paths) if i in upload_file_ids]
            upload_file_titles = [
                x for i, x in enumerate(file_titles) if i in upload_file_ids]
            req = MultipleUploadRequest()
            req.file_paths = upload_file_paths
            req.file_titles = upload_file_titles
            req.parents_path = upload_parents_path
            req.use_timestamp_folder = False
            req.use_timestamp_file_title = False
            gdrive_upload = rospy.ServiceProxy(
                '/{}/upload_multi'.format(self.gdrive_server_name),
                MultipleUpload
            )
            res = gdrive_upload(req)
            result.append(res)

            # influxdb
            if self.store_url:
                query = []
                for success, file_url, file_path, file_title, in zip(
                        res.successes, res.file_urls,
                        upload_file_paths, upload_file_titles):
                    if success:
                        rospy.loginfo('Upload succeeded: {}'.format(file_path))
                        stamp = '_'.join(file_title.split('_')[:2])
                        stamp = self.localtz.localize(
                            datetime.strptime(stamp, '%Y%m%d_%H%M%S%Z'))
                        stamp = stamp.astimezone(self.tz).isoformat('T')
                        query.append({
                            "measurement": "gdrive_recorder",
                            "time": stamp,
                            "fields": {
                                "file_title": file_title,
                                "file_path": file_path,
                                "file_url": file_url,
                            }
                        })
                    else:
                        rospy.loginfo('Upload failed: {}'.format(file_path))
                if len(query) > 0:
                    self.client.write_points(query, time_precision='ms')

            for success, file_path in zip(res.successes,  upload_file_paths):
                if success:
                    os.remove(file_path)
        rospy.loginfo('stop uploading')
        return result

    def _record_timer_cb(self, event):
        self.start_time = rospy.Time.now()
        prev_process = self.process
        self.process = None
        self._start_record()
        self._stop_record(prev_process)

    def _start_record(self):
        self.start_time = rospy.Time.now()
        stamp = datetime.utcfromtimestamp(
            int(self.start_time.to_time()))
        stamp = self.tz.localize(stamp).astimezone(self.localtz)
        stamp = stamp.strftime('%Y%m%d_%H%M%S%Z')
        self.video_title = '{}_{}_record_video.avi'.format(
            stamp, self.robot_name)
        cmds = [
            'roslaunch',
            'gdrive_recorder',
            '{}_audio_video_recorder.launch'.format(self.robot_type),
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
    rospy.init_node('gdrive_recorder')
    recorder = GdriveRecorder()

    def hook():
        recorder._kill_process(recorder.decompress_process)
        if recorder.process:
            recorder._kill_process(recorder.process)

    rospy.on_shutdown(hook)
    rospy.spin()
