#!/usr/bin/env python3
"""Reload a camera driver nodelet when its images stop arriving.

When the arm cable tangles, it can pull the USB cable of the eef Astra camera: the device
re-enumerates (dmesg "usb ... Cannot enable. Maybe the USB cable is bad?"), but the driver
nodelet keeps its old handle and publishes nothing, without dying (so respawn does not help).
This node watches ``~topics``; when none of them delivered a message for ``~silent_s``, it
unloads the driver nodelet from its manager and loads it again with the same type and remaps
(its parameters stay on the parameter server), then waits ``~grace_s`` for images. Failed
attempts are retried with a doubling wait up to ``~max_backoff_s``; ``~max_attempts`` > 0
limits the attempts per outage. Real robot only (started from gripper_astra.launch.xml).

``~status`` (latched String): "ok", "silent: ...", "reloading (attempt n)", "reloaded, waiting
for images", "failed: ...". Messages are received as ``rospy.AnyMsg`` and never decoded.
"""
import threading

import rospy
from nodelet.srv import NodeletLoad, NodeletUnload
from std_msgs.msg import String


class CameraDriverRelauncher:
    def __init__(self):
        self.manager = rospy.resolve_name(rospy.get_param('~manager'))
        self.nodelet_name = rospy.resolve_name(rospy.get_param('~nodelet_name'))
        self.nodelet_type = rospy.get_param('~nodelet_type')
        # {source: target}, resolved like the nodelet command line does
        remaps = rospy.get_param('~remaps', {})
        self.remap_sources = [rospy.resolve_name(k) for k in remaps]
        self.remap_targets = [rospy.resolve_name(v) for v in remaps.values()]
        self.topics = [rospy.resolve_name(t) for t in rospy.get_param('~topics')]
        self.silent_s = float(rospy.get_param('~silent_s', 10.0))
        self.grace_s = float(rospy.get_param('~grace_s', 20.0))
        self.max_backoff_s = float(rospy.get_param('~max_backoff_s', 120.0))
        self.max_attempts = int(rospy.get_param('~max_attempts', 0))
        self.service_timeout_s = float(rospy.get_param('~service_timeout_s', 10.0))
        # the launch file's loader process exits when its nodelet is unloaded; give it time to go before the load
        self.unload_settle_s = float(rospy.get_param('~unload_settle_s', 3.0))

        self.lock = threading.Lock()
        self.last_seen = rospy.get_time()  # the startup counts as seen: the driver gets grace_s to start
        self.status = None
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        for topic in self.topics:
            rospy.Subscriber(topic, rospy.AnyMsg, self.seen, queue_size=1)
        self.set_status('starting')
        rospy.loginfo('Camera driver relauncher: reloads %s (%s) in %s when %s stay silent for %.0f s',
                      self.nodelet_name, self.nodelet_type, self.manager, ', '.join(self.topics), self.silent_s)

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

    def call(self, service, service_class, **request):
        rospy.wait_for_service(service, timeout=self.service_timeout_s)
        return rospy.ServiceProxy(service, service_class)(**request)

    def reload(self):
        """Unload (may fail: it may be gone already) and load the driver nodelet; True when the load succeeded."""
        try:
            unloaded = self.call(self.manager + '/unload_nodelet', NodeletUnload, name=self.nodelet_name).success
            rospy.loginfo('Camera driver relauncher: unload %s: %s', self.nodelet_name, unloaded)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn('Camera driver relauncher: unload %s failed: %s', self.nodelet_name, exc)
        rospy.sleep(self.unload_settle_s)
        try:
            # no bond: the nodelet lives as long as the manager (the original loader process exits on unload)
            loaded = self.call(self.manager + '/load_nodelet', NodeletLoad, name=self.nodelet_name,
                               type=self.nodelet_type, remap_source_args=self.remap_sources,
                               remap_target_args=self.remap_targets, my_argv=[], bond_id='').success
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr('Camera driver relauncher: load %s failed: %s', self.nodelet_name, exc)
            return False
        rospy.loginfo('Camera driver relauncher: load %s: %s', self.nodelet_name, loaded)
        return loaded

    def run(self):
        rate = rospy.Rate(1.0)
        attempts, next_try = 0, 0.0
        while not rospy.is_shutdown():
            silence = self.silence()
            if silence < self.silent_s:
                if attempts:
                    rospy.logwarn('Camera driver relauncher: images back after %d reload(s)', attempts)
                attempts = 0
                self.set_status('ok')
            elif rospy.get_time() < next_try:
                pass  # a reloaded driver needs time to deliver images; a failed load waits before the retry
            elif self.max_attempts > 0 and attempts >= self.max_attempts:
                self.set_status(f'failed: silent for {silence:.0f} s after {attempts} reload(s)')
            else:
                attempts += 1
                rospy.logerr('Camera driver relauncher: no images for %.0f s, reloading %s (attempt %d)',
                             silence, self.nodelet_name, attempts)
                self.set_status(f'reloading (attempt {attempts})')
                loaded = self.reload()
                self.set_status('reloaded, waiting for images' if loaded else f'failed: load attempt {attempts}')
                delay = self.grace_s if loaded else self.silent_s
                next_try = rospy.get_time() + min(delay * 2 ** (attempts - 1), self.max_backoff_s)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

if __name__ == '__main__':
    rospy.init_node('camera_driver_relauncher')
    CameraDriverRelauncher().run()
