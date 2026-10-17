"""Simplified fetch dispatch, real WPR2 grasp and installed waypoint navigation."""
from pathlib import Path
import subprocess
import sys

import pytest
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from test_ex17_grab import Harness, Robot, make_cloud, publish_cloud


@pytest.fixture
def h(tmp_path):
    harness = Harness(tmp_path)
    # Every child shares this test's isolated DDS domain; no real hardware is launched.
    harness.prefix = ''
    yield harness
    harness.close()


class Interfaces:
    def __init__(self, h, fake_navigation=True):
        self.h = h
        self.navigation = h.collect(String, '/waterplus/navi_waypoint')
        self.commands = h.collect(String, '/wpr2/behavior')
        self.velocities = h.collect(Twist, '/cmd_vel')
        self.nav_result = h.publisher(String, '/waterplus/navi_result') if fake_navigation else None
        self.grab_result = h.publisher(String, '/wpr2/grab_result')

    def start(self):
        self.process = self.h.start('ex20_fetch')
        return self.process

    def nav(self, text):
        self.nav_result.publish(String(data=text))

    def grab(self, text):
        self.grab_result.publish(String(data=text))

    def requested(self, name):
        return any(msg.data == name for msg in self.navigation)

    def commanded(self, name):
        return any(msg.data == name for msg in self.commands)


def fetch_done(h):
    return 'fetch done' in (h.tmp_path / 'ex20_fetch.log').read_text()


def test_fetch_stage_order_duplicates_and_release(h):
    io = Interfaces(h)
    io.start()
    h.wait(lambda: io.requested('kitchen'))
    io.grab('grab done')  # A grab result during navigation cannot skip the kitchen.
    h.pump(0.05)
    assert not io.commands and not io.requested('guest')
    io.nav('navi done')
    io.nav('navi done')
    h.pump(0.05)
    h.wait(lambda: io.commanded('start grab'))
    io.nav('navi done')  # Navigation results during grasping are irrelevant.
    h.pump(0.1)
    io.grab('grab done')
    io.grab('grab done')
    h.pump(0.05)
    h.wait(lambda: io.requested('guest'))
    io.nav('navi done')
    h.wait(lambda: fetch_done(h))
    io.nav('navi done')
    io.grab('grab done')
    h.pump(0.25)
    assert [m.data for m in io.navigation] == ['kitchen', 'guest']
    assert [m.data for m in io.commands] == ['start grab']
    assert (h.tmp_path / 'ex20_fetch.log').read_text().count('fetch done') == 1
    assert not io.velocities  # Dispatcher never competes with navigation or grab cmd_vel.


def test_fetch_with_real_wpr2_detection_and_grab(h):
    robot = Robot(h)
    robot.fake_detection = False
    clouds = h.publisher(PointCloud2, '/camera/depth_registered/points', sensor=True)
    log = open(h.tmp_path / 'fetch_behaviors.log', 'w+')
    support = subprocess.Popen(
        [sys.executable, str(Path(__file__).with_name('run_fetch_behaviors.py'))],
        env=h.env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    h.processes.append((support, log))
    h.wait(lambda: clouds.get_subscription_count() > 0 and robot.goal_pub.get_subscription_count() > 0)
    navigation = h.collect(String, '/waterplus/navi_waypoint')
    nav_result = h.publisher(String, '/waterplus/navi_result')
    h.start('ex20_fetch')
    h.wait(lambda: navigation)
    assert [msg.data for msg in navigation] == ['kitchen'] and not any(robot.arm_commands.values())
    nav_result.publish(String(data='navi done'))

    def frame():
        x, y, z = robot.target()
        publish_cloud(h, clouds, make_cloud(x=x, y=y, z=z))

    h.wait(lambda: any(msg.data == 'guest' for msg in navigation), timeout=20, tick=frame)
    assert robot.done() and not robot.failed()
    assert robot.arm_commands['right_arm'][-1].position == pytest.approx(
        [-0.1593, 1.5128, -0.0382, 1.7225, 0.0637, 1.5130])
    assert any(msg.linear.x < 0 for msg in robot.velocities)
    nav_result.publish(String(data='navi done'))
    h.wait(lambda: fetch_done(h))
    assert [msg.data for msg in navigation] == ['kitchen', 'guest']
    assert support.poll() is None
    names = h.node.get_node_names()
    assert 'wpr2_grab' in names and 'wpr2_objects_3d' in names and 'ex17_grab' not in names
    h.pump(0.3)
    count = len(robot.velocities)
    h.pump(0.2)
    assert len(robot.velocities) == count


def test_fetch_with_installed_waypoint_server(h):
    # 真实航点节点 + 模拟航点查询/Nav2 action，验证两段导航与示例配合。
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer
    from wp_map_tools.srv import GetWaypointByName

    looked_up = []
    goals = []

    def lookup(request, response):
        looked_up.append(request.name)
        response.name = request.name
        response.pose.orientation.w = 1.0
        return response

    async def execute(handle):
        goals.append(handle.request)
        handle.succeed()
        return NavigateToPose.Result()

    service = h.node.create_service(GetWaypointByName, '/waterplus/get_waypoint_name', lookup)
    server = ActionServer(h.node, NavigateToPose, '/navigate_to_pose', execute)
    try:
        io = Interfaces(h, fake_navigation=False)
        binary = Path(get_package_prefix('wp_map_tools')) / 'lib/wp_map_tools/wp_navi_server'
        h.start(str(binary))
        # launch 中航点服务应先启动，等待其接口发现后再运行教学程序。
        h.wait(lambda: h.node.count_subscribers('/waterplus/navi_waypoint') >= 2)
        io.start()
        h.wait(lambda: io.commanded('start grab'))
        io.grab('grab done')
        h.wait(lambda: fetch_done(h))
        assert looked_up == ['kitchen', 'guest'] and len(goals) == 2
        assert [msg.data for msg in io.navigation] == ['kitchen', 'guest']
    finally:
        server.destroy()
        h.node.destroy_service(service)
