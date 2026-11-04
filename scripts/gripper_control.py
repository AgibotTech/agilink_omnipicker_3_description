#!/usr/bin/env python3

import math
from pathlib import Path
import signal
import sys
import xml.etree.ElementTree as ET


def transform(xyz, rpy):
    roll, pitch, yaw = rpy
    cosine_roll, sine_roll = math.cos(roll), math.sin(roll)
    cosine_pitch, sine_pitch = math.cos(pitch), math.sin(pitch)
    cosine_yaw, sine_yaw = math.cos(yaw), math.sin(yaw)
    return (
        (cosine_yaw * cosine_pitch,
         cosine_yaw * sine_pitch * sine_roll - sine_yaw * cosine_roll,
         cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll, xyz[0]),
        (sine_yaw * cosine_pitch,
         sine_yaw * sine_pitch * sine_roll + cosine_yaw * cosine_roll,
         sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll, xyz[1]),
        (-sine_pitch, cosine_pitch * sine_roll, cosine_pitch * cosine_roll, xyz[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def multiply(first, second):
    return tuple(tuple(sum(first[row][index] * second[index][column]
                           for index in range(4))
                       for column in range(4)) for row in range(4))


def apply(frame, point=(0.0, 0.0, 0.0)):
    return tuple(sum(frame[row][column] * point[column] for column in range(3))
                 + frame[row][3] for row in range(3))


def local_point(frame, point):
    return tuple(sum(frame[row][column] * (point[row] - frame[row][3])
                     for row in range(3)) for column in range(3))


def planar(point):
    return point[0], point[2]


def difference(first, second):
    return tuple(first[index] - second[index] for index in range(len(first)))


def distance(first, second):
    return math.sqrt(sum(value * value for value in difference(first, second)))


def rotation_angle(current, reference):
    angle = math.atan2(current[1], current[0]) - math.atan2(reference[1], reference[0])
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def branch(first, second, point):
    direction = difference(second, first)
    offset = difference(point, first)
    return 1 if direction[0] * offset[1] - direction[1] * offset[0] >= 0.0 else -1


def intersection(first, first_radius, second, second_radius, branch_sign):
    direction = difference(second, first)
    separation = math.hypot(*direction)
    if (separation < 1e-12 or separation > first_radius + second_radius + 1e-10
            or separation < abs(first_radius - second_radius) - 1e-10):
        raise ValueError('Requested joint1 stroke cannot close the linkage')
    along = (first_radius ** 2 - second_radius ** 2 + separation ** 2) / (2 * separation)
    height = math.sqrt(max(0.0, first_radius ** 2 - along ** 2))
    return (
        first[0] + along * direction[0] / separation
        - branch_sign * height * direction[1] / separation,
        first[1] + along * direction[1] / separation
        + branch_sign * height * direction[0] / separation,
    )


class LinkageKinematics:
    side_joints = (
        ('joint12', 'joint13', 'joint14', 'joint16', 'joint17', 'joint18'),
        ('joint22', 'joint23', 'joint24', 'joint26', 'joint27', 'joint28'),
    )

    def __init__(self, urdf_path):
        self.model = ET.parse(urdf_path).getroot()
        self.joints = {joint.get('name'): joint for joint in self.model.findall('joint')}
        self.joint_names = tuple(name for name, joint in self.joints.items()
                                 if joint.get('type') != 'fixed')
        self.origins = {}
        for name, joint in self.joints.items():
            origin = joint.find('origin')
            xyz = tuple(float(value) for value in origin.get('xyz', '0 0 0').split())
            rpy = tuple(float(value) for value in origin.get('rpy', '0 0 0').split())
            self.origins[name] = (xyz, rpy)
        limit = self.joints['joint1'].find('limit')
        self.lower = float(limit.get('lower'))
        self.upper = float(limit.get('upper'))
        reference_origins = dict(self.origins)
        drive_xyz, drive_rpy = reference_origins['joint1']
        reference_origins['joint1'] = ((drive_xyz[0], drive_xyz[1], 0.072), drive_rpy)
        reference = self.forward({}, reference_origins)
        self.sides = []
        for names in self.side_joints:
            push, terminal, outer, tip, inner, far_terminal = names
            moving_pin = apply(reference[self.child(push)])
            closure_pin = apply(reference[self.child(terminal)])
            fixed_pin = apply(reference[self.child(outer)])
            tip_pivot = apply(reference[self.child(tip)])
            inner_pivot = apply(reference[self.child(inner)])
            far_pin = apply(reference[self.child(far_terminal)])
            signs = {}
            for name in names:
                frame = reference[self.child(name)]
                axis = tuple(float(value) for value in self.joints[name].find('axis').get('xyz').split())
                world_axis = tuple(sum(frame[row][column] * axis[column]
                                       for column in range(3)) for row in range(3))
                if math.hypot(world_axis[0], world_axis[2]) > 1e-4:
                    raise ValueError('This controller requires planar X/Z joint axes')
                signs[name] = 1 if world_axis[1] < 0.0 else -1
            self.sides.append({
                'names': names,
                'signs': signs,
                'push_vector': difference(planar(closure_pin), planar(moving_pin)),
                'outer_vector': difference(planar(closure_pin), planar(fixed_pin)),
                'inner_vector': difference(planar(far_pin), planar(inner_pivot)),
                'tip_vector': difference(planar(far_pin), planar(tip_pivot)),
                'outer_offset': local_point(reference[self.child(outer)], closure_pin),
                'tip_offset': local_point(reference[self.child(tip)], far_pin),
                'first_branch': branch(planar(moving_pin), planar(fixed_pin), planar(closure_pin)),
                'second_branch': branch(planar(inner_pivot), planar(tip_pivot), planar(far_pin)),
            })

    def child(self, name):
        return self.joints[name].find('child').get('link')

    def forward(self, positions, origins=None):
        origins = self.origins if origins is None else origins
        frames = {'base_link': transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}
        pending = dict(self.joints)
        while pending:
            progressed = False
            for name, joint in list(pending.items()):
                parent = joint.find('parent').get('link')
                if parent not in frames:
                    continue
                frame = transform(*origins[name])
                position = positions.get(name, 0.0)
                if joint.get('type') != 'fixed':
                    axis = tuple(float(value) for value in joint.find('axis').get('xyz').split())
                    if joint.get('type') == 'prismatic':
                        motion = transform(tuple(value * position for value in axis), (0.0, 0.0, 0.0))
                    else:
                        motion = transform((0.0, 0.0, 0.0), (0.0, 0.0, axis[2] * position))
                    frame = multiply(frame, motion)
                frames[self.child(name)] = multiply(frames[parent], frame)
                del pending[name]
                progressed = True
            if not progressed:
                raise ValueError('URDF joint tree is disconnected or cyclic')
        return frames

    def solve(self, stroke):
        stroke = float(stroke)
        if not math.isfinite(stroke) or not self.lower <= stroke <= self.upper:
            raise ValueError(f'joint1 must be within [{self.lower}, {self.upper}] m')
        positions = {name: 0.0 for name in self.joint_names}
        positions['joint1'] = stroke
        frames = self.forward(positions)
        for side in self.sides:
            push, terminal, outer, tip, inner, far_terminal = side['names']
            signs = side['signs']
            moving_pin = planar(apply(frames[self.child(push)]))
            fixed_pin = planar(apply(frames[self.child(outer)]))
            closure_pin = intersection(
                moving_pin, math.hypot(*side['push_vector']),
                fixed_pin, math.hypot(*side['outer_vector']), side['first_branch'],
            )
            positions[push] = rotation_angle(
                difference(closure_pin, moving_pin), side['push_vector']
            ) / signs[push]
            positions[outer] = rotation_angle(
                difference(closure_pin, fixed_pin), side['outer_vector']
            ) / signs[outer]
            positions[terminal] = (
                signs[outer] * positions[outer] - signs[push] * positions[push]
            ) / signs[terminal]
            frames = self.forward(positions)
            tip_pivot = planar(apply(frames[self.child(tip)]))
            inner_pivot = planar(apply(frames[self.child(inner)]))
            far_pin = intersection(
                inner_pivot, math.hypot(*side['inner_vector']),
                tip_pivot, math.hypot(*side['tip_vector']), side['second_branch'],
            )
            positions[inner] = rotation_angle(
                difference(far_pin, inner_pivot), side['inner_vector']
            ) / signs[inner]
            tip_rotation = rotation_angle(difference(far_pin, tip_pivot), side['tip_vector'])
            positions[tip] = (tip_rotation - signs[outer] * positions[outer]) / signs[tip]
            positions[far_terminal] = (
                signs[outer] * positions[outer] + signs[tip] * positions[tip]
                - signs[inner] * positions[inner]
            ) / signs[far_terminal]
        for name, value in positions.items():
            limit = self.joints[name].find('limit')
            lower = float(limit.get('lower')) - 1e-9
            upper = float(limit.get('upper')) + 1e-9
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f'Closed-loop solution exceeds the limit of {name}')
        if max(self.closure_errors(positions)) > 5e-6:
            raise ValueError('Closed-loop positional error exceeds 5 micrometers')
        return positions

    def closure_errors(self, positions):
        frames = self.forward(positions)
        errors = []
        for side in self.sides:
            _, terminal, outer, tip, _, far_terminal = side['names']
            errors.append(distance(apply(frames[self.child(terminal)]),
                                   apply(frames[self.child(outer)], side['outer_offset'])))
            errors.append(distance(apply(frames[self.child(far_terminal)]),
                                   apply(frames[self.child(tip)], side['tip_offset'])))
        return tuple(errors)


class GripperController:
    def __init__(self):
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        self.node = rclpy.create_node('gripper_joint_state_publisher')
        self.message_type = JointState
        urdf_path = str(self.node.declare_parameter('urdf_path', '').value)
        self.kinematics = LinkageKinematics(Path(urdf_path))
        self.gui = bool(self.node.declare_parameter('gui', True).value)
        initial = float(self.node.declare_parameter('initial_position', 0.0).value)
        self.positions = self.kinematics.solve(initial)
        self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.node.create_subscription(Float64, 'joint1_command', self.command, 10)
        self.timer = self.node.create_timer(0.02, self.publish)
        self.node.get_logger().info('Single joint1 control ready; commands on /joint1_command are in meters')

    def set_position(self, stroke):
        try:
            self.positions = self.kinematics.solve(stroke)
            return True
        except ValueError as error:
            self.node.get_logger().warning(
                'Rejected joint1 command: ' + str(error), throttle_duration_sec=2.0
            )
            return False

    def command(self, message):
        self.set_position(message.data)

    def publish(self):
        message = self.message_type()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(self.kinematics.joint_names)
        message.position = [self.positions[name] for name in message.name]
        self.publisher.publish(message)


def run_gui(controller):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.utilities import remove_ros_args
    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import QApplication, QLabel, QSlider, QVBoxLayout, QWidget

    application = QApplication(remove_ros_args(args=sys.argv))
    window = QWidget()
    window.setWindowTitle('OP3 P0 - joint1 Gripper Control')
    window.setMinimumWidth(440)
    layout = QVBoxLayout(window)
    layout.addWidget(QLabel('joint1 stroke (mm)'))
    value_label = QLabel()
    layout.addWidget(value_label)
    slider = QSlider(Qt.Horizontal)
    slider.setRange(
        round(controller.kinematics.lower * 100000),
        round(controller.kinematics.upper * 100000),
    )
    slider.setSingleStep(1)
    slider.setPageStep(100)
    layout.addWidget(slider)

    def synchronize():
        position = controller.positions['joint1']
        slider.blockSignals(True)
        slider.setValue(round(position * 100000))
        slider.blockSignals(False)
        value_label.setText(f'{position * 1000:.2f} / {controller.kinematics.upper * 1000:.2f} mm')

    def value_changed(value):
        controller.set_position(value / 100000.0)
        synchronize()

    slider.valueChanged.connect(value_changed)
    synchronize()
    window.show()
    timer = QTimer()

    def stop(*_arguments):
        timer.stop()
        application.quit()

    def spin_once():
        if not rclpy.ok():
            stop()
            return
        try:
            rclpy.spin_once(controller.node, timeout_sec=0.0)
            synchronize()
        except (KeyboardInterrupt, ExternalShutdownException):
            stop()

    timer.timeout.connect(spin_once)
    timer.start(10)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    application.exec_()
    timer.stop()


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    controller = None
    try:
        controller = GripperController()
        if controller.gui:
            run_gui(controller)
        else:
            def stop(*_arguments):
                rclpy.try_shutdown()

            signal.signal(signal.SIGINT, stop)
            signal.signal(signal.SIGTERM, stop)
            rclpy.spin(controller.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if controller is not None:
            controller.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
