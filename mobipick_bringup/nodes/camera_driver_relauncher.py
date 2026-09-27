#!/usr/bin/env python3
"""Restart a camera's nodelet stack when its images stop arriving.

When the arm cable tangles, it can pull the USB cable of the eef Astra camera: the device
re-enumerates (dmesg "usb ... Cannot enable. Maybe the USB cable is bad?"), but the driver
nodelet keeps its old handle and publishes nothing, without dying. The camera is launched with
``respawn`` (astra.launch.xml): its nodelet manager respawns and every nodelet loader is bonded
to it and respawns, so a manager that dies (e.g. a segfault in liborbbec) brings the whole stack
back by itself. This node covers the silent case: when none of ``~topics`` delivered a message
for ``~silent_s``, it kills ``~manager`` (like ``rosnode kill``) and roslaunch restarts the stack,
then it waits ``~grace_s`` for images. Unsuccessful restarts are retried with a doubling wait up to
``~max_backoff_s``; ``~max_attempts`` > 0 limits the restarts per outage. While ``lsusb -d ~usb_id``
lists no device (the cable is out), a restart cannot help: the node waits for the device instead and
restarts as soon as it is back. Real robot only (started from gripper_astra.launch.xml).

``~status`` (latched String): "ok", "silent: ...", "restarting (attempt n)", "restarted, waiting
for images", "device missing: ...", "failed: ...". Messages are received as ``rospy.AnyMsg`` and never decoded.
"""
import subprocess
import threading

import rosnode
import rospy
from std_msgs.msg import String


class CameraDriverRelauncher:
    def __init__(self):
        self.manager = rospy.resolve_name(rospy.get_param('~manager'))
        self.topics = [rospy.resolve_name(t) for t in rospy.get_param('~topics')]
        self.silent_s = float(rospy.get_param('~silent_s', 10.0))
        self.grace_s = float(rospy.get_param('~grace_s', 30.0))
        self.max_backoff_s = float(rospy.get_param('~max_backoff_s', 120.0))
        self.max_attempts = int(rospy.get_param('~max_attempts', 0))
        self.usb_id = str(rospy.get_param('~usb_id', '2bc5:'))  # lsusb -d filter, Orbbec by default; '' disables

        self.lock = threading.Lock()
        self.last_seen = rospy.get_time()  # the startup counts as seen: the camera gets silent_s to start
        self.status = None
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        for topic in self.topics:
            rospy.Subscriber(topic, rospy.AnyMsg, self.seen, queue_size=1)
        self.set_status('starting')
        rospy.loginfo('Camera driver relauncher: restarts %s when %s stay silent for %.0f s',
                      self.manager, ', '.join(self.topics), self.silent_s)

    def seen(self, _message):
        with self.lock:
            self.last_seen = rospy.get_time()

    def silence(self):
        with self.lock:
            return rospy.get_time() - self.last_seen

    def set_status(self, status):
        if status != self.status:
            self.status = status
            self.status_pub.publish(String(data=status))

    def device_present(self):
        """False only when lsusb works and lists no ``~usb_id`` device."""
        if not self.usb_id:
            return True
        try:
            result = subprocess.run(['lsusb', '-d', self.usb_id], capture_output=True, text=True, timeout=5.0)
        except (OSError, subprocess.SubprocessError) as exc:
            rospy.logwarn_throttle(300, 'Camera driver relauncher: lsusb failed (%s), restarting blindly', exc)
            return True
        return bool(result.stdout.strip())

    def restart(self):
        """Kill the nodelet manager; roslaunch respawns it and its bonded loaders. True when it was killed."""
        try:
            killed, failed = rosnode.kill_nodes([self.manager])
        except Exception as exc:  # noqa: BLE001 - master or node unreachable: retried later
            rospy.logerr('Camera driver relauncher: killing %s failed: %s', self.manager, exc)
            return False
        if failed:
            # a manager that already died is being respawned by roslaunch: nothing to kill
            rospy.logwarn('Camera driver relauncher: could not kill %s (dead already?)', self.manager)
        return bool(killed)

    def run(self):
        rate = rospy.Rate(1.0)
        attempts, next_try = 0, 0.0
        while not rospy.is_shutdown():
            silence = self.silence()
            if silence < self.silent_s:
                if attempts:
                    rospy.logwarn('Camera driver relauncher: images back after %d restart(s)', attempts)
                attempts = 0
                self.set_status('ok')
            elif rospy.get_time() < next_try:
                pass  # a restarted stack needs time to deliver images
            elif not self.device_present():
                if not self.status.startswith('device missing'):
                    rospy.logerr('Camera driver relauncher: no USB device %s, waiting for it to be plugged in',
                                 self.usb_id)
                attempts = 0  # restart right away, with a fresh backoff, once the device is back
                self.set_status(f'device missing: no USB device {self.usb_id}, replug the camera')
            elif self.max_attempts > 0 and attempts >= self.max_attempts:
                self.set_status(f'failed: silent for {silence:.0f} s after {attempts} restart(s)')
            else:
                attempts += 1
                rospy.logerr('Camera driver relauncher: no images for %.0f s, restarting %s (attempt %d)',
                             silence, self.manager, attempts)
                self.set_status(f'restarting (attempt {attempts})')
                killed = self.restart()
                self.set_status('restarted, waiting for images' if killed else f'failed: restart attempt {attempts}')
                delay = self.grace_s if killed else self.silent_s
                next_try = rospy.get_time() + min(delay * 2 ** (attempts - 1), self.max_backoff_s)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


if __name__ == '__main__':
    rospy.init_node('camera_driver_relauncher')
    CameraDriverRelauncher().run()
