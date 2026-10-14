"""Exercise real DDS and UDP bridge paths using loopback only."""

import os
import socket
import struct
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_port.msg import BridgeHealth, RobotCommand, RobotFeedback, RobotOperation


def test_udp_bridge_fault_injection():
    """Verify complete packets, watchdog, clock changes and configuration failures."""
    executable = os.environ['UDP_EXECUTABLE']
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    board_command = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    board_feedback = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    intruder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    board_command.bind(('127.0.0.2', 0))
    board_feedback.bind(('127.0.0.2', 0))
    intruder.bind(('127.0.0.3', 0))
    board_feedback.setblocking(False)
    command_port = board_command.getsockname()[1]
    operation_port = board_feedback.getsockname()[1]
    assert command_port != operation_port
    arguments = [
        executable, '--ros-args', '-p', 'local_address:=127.0.0.1',
        '-p', 'peer_address:=127.0.0.2', '-p', f'command_port:={command_port}',
        '-p', f'operation_port:={operation_port}', '-p', 'test_mode:=true',
        '-p', 'enable_transmission:=true',
    ]
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('bridge_fault_injection', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    commands, feedbacks, healths, packets = [], [], [], []
    node.create_subscription(RobotCommand, '/robot/command', commands.append, 10)
    node.create_subscription(RobotFeedback, '/robot/feedback', feedbacks.append, 10)
    node.create_subscription(BridgeHealth, '/robot/communication_status', healths.append, 10)
    publisher = node.create_publisher(RobotOperation, '/robot/operation', 1)
    process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    def pump():
        """Dispatch DDS callbacks and collect operation packets without blocking."""
        executor.spin_once(timeout_sec=0.005)
        while True:
            try:
                packet, source = board_feedback.recvfrom(2048)
            except BlockingIOError:
                break
            assert source == ('127.0.0.1', operation_port)
            assert len(packet) == 32
            assert packet[0] == 0xAB and packet[1] == 0
            packets.append(packet)

    def wait_for(predicate, timeout=5.0, repeated=None):
        """Wait for an observed outcome with a bounded discovery allowance."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            assert process.poll() is None, 'bridge exited unexpectedly'
            if repeated:
                repeated()
            pump()
            if predicate():
                return
        raise AssertionError('timed out waiting for bridge outcome')

    def send_command(packet):
        """Send from the approved command endpoint."""
        board_command.sendto(packet, ('127.0.0.1', command_port))

    def send_feedback(packet):
        """Send from the approved feedback endpoint."""
        board_feedback.sendto(packet, ('127.0.0.1', operation_port))

    update_sequence = 0

    def publish_operation(status=0, generation=0, stamp_offset=0.0, reuse=None):
        """Publish a current full snapshot or a deliberate stale/duplicate one."""
        nonlocal update_sequence
        if reuse:
            publisher.publish(reuse)
            return reuse
        update_sequence += 1
        message = RobotOperation()
        message.stamp = (node.get_clock().now() + rclpy.duration.Duration(
            seconds=stamp_offset)).to_msg()
        message.producer_session = 123
        message.update_sequence = update_sequence
        message.generation = generation
        message.navigation_status = status
        if status == RobotOperation.MOVING:
            message.velocity_x_m_s = 1.0
        publisher.publish(message)
        return message

    try:
        wait_for(lambda: publisher.get_subscription_count() == 1 and len(healths) > 0)
        wait_for(lambda: packets and packets[-1][3] == RobotOperation.FAIL)
        assert struct.unpack_from('<5f', packets[-1], 4) == (0, 0, 0, 0, 0)

        valid_command = bytes([0x38, 42, 0, 1, 0, 7, 2, 0])
        valid_feedback = struct.pack('<4B10f', 0x55, 13, 1, 0, *range(10))
        wait_for(lambda: commands and feedbacks,
                 repeated=lambda: (send_command(valid_command), send_feedback(valid_feedback)))
        assert commands[-1].sequence == 42 and commands[-1].target_id == 7
        assert feedbacks[-1].sequence == 13 and feedbacks[-1].bucket_arm_height_m == 9
        wait_for(lambda: healths[-1].command_fresh and healths[-1].feedback_fresh)

        commands.clear()
        feedbacks.clear()
        # All malformed inputs contain different sequences, making partial acceptance visible.
        malformed_command = bytes([0x38, 99, 0, 1, 0, 7, 2, 0])
        malformed_feedback = struct.pack('<4B10f', 0x55, 99, 0, 0, *range(10))
        for payload in (b'', malformed_command[:-1], malformed_command + b'\0',
                        malformed_command + bytes(504), bytes([0x38, 99, 2, 0, 0, 0, 0, 0])):
            send_command(payload)
        for payload in (malformed_feedback[:-1], malformed_feedback + b'\0',
                        malformed_feedback + bytes(512), bytes([0xAB]) + bytes(43),
                        struct.pack('<4B10f', 0x55, 99, 0, 0, *([float('nan')] + [0] * 9))):
            send_feedback(payload)
        intruder.sendto(malformed_command, ('127.0.0.1', command_port))
        board_feedback.sendto(malformed_command, ('127.0.0.1', command_port))
        intruder.sendto(malformed_feedback, ('127.0.0.1', operation_port))
        wait_for(lambda: healths[-1].rejected_command_packets >= 7
                 and healths[-1].rejected_feedback_packets >= 6)
        assert not any(message.sequence == 99 for message in commands)
        assert not any(message.sequence == 99 for message in feedbacks)
        wait_for(lambda: not healths[-1].command_fresh and not healths[-1].feedback_fresh)
        wait_for(lambda: healths[-1].command_fresh and not healths[-1].feedback_fresh,
                 repeated=lambda: send_command(valid_command))

        wait_for(lambda: healths[-1].operation_fresh,
                 repeated=lambda: publish_operation())
        wait_for(lambda: packets[-1][3] == RobotOperation.MOVING,
                 repeated=lambda: publish_operation(RobotOperation.MOVING, 1))
        assert struct.unpack_from('<f', packets[-1], 4)[0] == 1.0
        wait_for(lambda: healths[-1].watchdog_active and packets[-1][3] == RobotOperation.FAIL)
        assert struct.unpack_from('<5f', packets[-1], 4) == (0, 0, 0, 0, 0)

        for status, generation, offset in ((RobotOperation.MOVING, 1, 0),
                                           (0, 1, -1.0), (0, 1, 1.0)):
            before = healths[-1].rejected_operations
            publish_operation(status, generation, offset)
            wait_for(lambda: healths[-1].rejected_operations > before)
        assert packets[-1][3] == RobotOperation.FAIL
        wait_for(lambda: healths[-1].operation_fresh,
                 repeated=lambda: publish_operation(generation=1))
        before = healths[-1].rejected_operations
        publish_operation(RobotOperation.MOVING, 1)
        wait_for(lambda: healths[-1].rejected_operations > before)
        wait_for(lambda: packets[-1][3] == RobotOperation.MOVING,
                 repeated=lambda: publish_operation(RobotOperation.MOVING, 2))
        frozen = publish_operation(RobotOperation.MOVING, 2)
        wait_for(lambda: healths[-1].watchdog_active and packets[-1][3] == RobotOperation.FAIL,
                 repeated=lambda: publish_operation(reuse=frozen))
        assert struct.unpack_from('<5f', packets[-1], 4) == (0, 0, 0, 0, 0)

        # Socket startup failure must be visible; test fixtures cannot address hardware.
        invalid = subprocess.run(arguments + ['-p', 'peer_address:=192.168.3.2'],
                                 capture_output=True, timeout=5)
        assert invalid.returncode == 1
        assert b'loopback' in invalid.stdout + invalid.stderr
        competing = subprocess.run(arguments, capture_output=True, timeout=5)
        assert competing.returncode == 1
        assert b'bind' in competing.stdout + competing.stderr

        # Health reports actual UDP sends, not board acknowledgement.
        assert healths[-1].bridge_session != 0
        assert healths[-1].test_mode and healths[-1].loopback_only
        assert healths[-1].update_sequence > healths[0].update_sequence
        assert healths[-1].last_send_success and healths[-1].send_errors == 0
        assert healths[-1].last_successful_send_age_ms >= 0
        assert healths[-1].last_sent_navigation_status == RobotOperation.FAIL
        assert healths[-1].last_sent_producer_session != 0
        previous_bridge_session = healths[-1].bridge_session
        process.terminate()
        process.communicate(timeout=5)
        process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        wait_for(lambda: healths[-1].bridge_session != previous_bridge_session)
        assert healths[-1].bridge_session != 0
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        board_command.close()
        board_feedback.close()
        intruder.close()
