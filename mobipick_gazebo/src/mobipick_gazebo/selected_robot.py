# The robot every GUI of the multi-robot sim acts on, without Qt: the topic names, the robot list and the owner's
# decision. One owner node (nodes/selected_robot.py) is the only writer of the latched /selected_robot; every switch
# (the sim robot selector, the RQT tables demo panel, a script) asks it on /selected_robot/request and changes robot
# only when /selected_robot says so. A single writer matters: ROS 1 hands a late subscriber the latched message of
# every publisher of a topic in no defined order, so two writers can leave a window on the older choice.
import re
import threading

SELECTED_ROBOT_TOPIC = "/selected_robot"
REQUEST_TOPIC = "/selected_robot/request"
DEFAULT_ROBOT = "mobipick"
_NAV_STATUS = re.compile(r"^/([^/]+)/move_base/status$")


def normalize(namespace):
    return (namespace or "").strip("/") or DEFAULT_ROBOT


def _number(namespace):
    match = re.search(r"(\d+)$", namespace)
    return int(match.group(1)) if match else 1


def robots_from_topics(topic_names, current=DEFAULT_ROBOT):
    """robot namespaces with a running move_base (from the published topic names), natural order, plus `current`"""
    robots = {normalize(current)}
    for topic in topic_names:
        match = _NAV_STATUS.match(topic)
        if match:
            robots.add(match.group(1))
    return sorted(robots, key=lambda r: (_number(r), r))


def owner_running(system_state):
    """True when a node listens on the request topic (`system_state` as rosgraph.Master.getSystemState returns it):
    then a switch asks the owner instead of publishing /selected_robot itself"""
    _publishers, subscribers, _services = system_state
    return any(topic == REQUEST_TOPIC and nodes for topic, nodes in subscribers)


class SelectedRobotOwner:
    """Keeps the selected robot. `publish(namespace)` sends the choice (latched), `topic_names()` lists the published
    topics, `warn(text)` logs why a request was refused. Requests may come in from several threads at once (rospy runs
    one thread per connected publisher): each one decides and publishes under one lock, so the last message sent, the
    one a late subscriber gets, is always the stored choice."""

    def __init__(self, publish, topic_names, warn, initial=DEFAULT_ROBOT):
        self._publish = publish
        self._topic_names = topic_names
        self._warn = warn
        self._lock = threading.Lock()
        self.robot = normalize(initial)
        self._publish(self.robot)

    def request(self, namespace):
        """switch to `namespace` when that robot runs; otherwise say why and send the current choice again, so the
        switch that asked snaps back. Returns the robot selected afterwards."""
        with self._lock:
            return self._request(namespace)

    def _request(self, namespace):
        wanted = (namespace or "").strip("/")
        if not wanted:
            self._warn(f"selected robot: empty request ignored; still {self.robot}")
        elif wanted == self.robot:
            pass
        else:
            running = robots_from_topics(self._topic_names(), self.robot)
            if wanted in running:
                self.robot = wanted
            else:
                self._warn(f"selected robot: robot {wanted} is not running (no {wanted}/move_base); still "
                           f"{self.robot}; running: {', '.join(running)}")
        self._publish(self.robot)
        return self.robot
