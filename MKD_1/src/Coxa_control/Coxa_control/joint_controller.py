#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk

MIN_VAL = -10.0
MAX_VAL = 10.0

INITIAL_OFFSETS = {
    "Revolute_Coxa_1": 0.00,
    "Revolute_Femur_1": 0.62,
    "Revolute_Tibia_1": 2.80,

    "Revolute_Coxa_2": 0.00,
    "Revolute_Femur_2": 0.00,
    "Revolute_Tibia_2": 0.00,

    "Revolute_Coxa_3": 0.00,
    "Revolute_Femur_3": 0.83,
    "Revolute_Tibia_3": 0.45,

    "Revolute_Coxa_4": 0.00,
    "Revolute_Femur_4": 0.00,
    "Revolute_Tibia_4": 1.55,
}

class JointGUI(Node):
    def __init__(self):
        super().__init__('joint_gui')

        self.leg_controllers = {
            1: "/leg1_controller/joint_trajectory",
            2: "/leg2_controller/joint_trajectory",
            3: "/leg3_controller/joint_trajectory",
            4: "/leg4_controller/joint_trajectory",
        }

        self.joint_names = [
            'Revolute_Coxa_',
            'Revolute_Femur_',
            'Revolute_Tibia_'
        ]

        self.pubs = {
            leg: self.create_publisher(JointTrajectory, topic, 10)
            for leg, topic in self.leg_controllers.items()
        }

        self.window = tk.Tk()
        self.window.title("Hexapod Joint GUI")

        self.widgets = {}
        row = 0

        for leg in range(1, 5):
            for joint in self.joint_names:
                name = f"{joint}{leg}"

                tk.Label(self.window, text=name).grid(row=row, column=0, padx=5, pady=5)

                slider = tk.Scale(
                    self.window, from_=MIN_VAL, to=MAX_VAL, resolution=0.01,
                    orient=tk.HORIZONTAL, length=200,
                    command=lambda val, l=leg: self.send_command(l)
                )
                slider.grid(row=row, column=1)

                entry = tk.Spinbox(
                    self.window, from_=MIN_VAL, to=MAX_VAL,
                    increment=0.01, width=6,
                    command=lambda l=leg: self.entry_changed(l)
                )
                entry.grid(row=row, column=2)

                self.widgets[name] = (slider, entry)
                row += 1

        # Initialize all widgets with the offset values
        self.apply_initial_offsets()

        self.window.after(50, self.spin_once)

    def apply_initial_offsets(self):
        for name, (slider, entry) in self.widgets.items():
            offset = INITIAL_OFFSETS[name]
            slider.set(offset)
            entry.delete(0, tk.END)
            entry.insert(0, f"{offset:.2f}")

        # Send initial positions to robot
        for leg in range(1, 5):
            self.send_command(leg)

    def entry_changed(self, leg):
        for joint in self.joint_names:
            name = f"{joint}{leg}"
            slider, entry = self.widgets[name]
            try:
                slider.set(float(entry.get()))
            except ValueError:
                pass
        self.send_command(leg)

    def send_command(self, leg):
        msg = JointTrajectory()
        msg.joint_names = [f"{j}{leg}" for j in self.joint_names]
        pt = JointTrajectoryPoint()
        pt.positions = []

        for joint in self.joint_names:
            name = f"{joint}{leg}"
            slider, entry = self.widgets[name]
            val = float(slider.get())
            pt.positions.append(val)

            entry.delete(0, tk.END)
            entry.insert(0, f"{val:.2f}")

        pt.time_from_start.sec = 1
        msg.points.append(pt)
        self.pubs[leg].publish(msg)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.window.after(50, self.spin_once)

def main(args=None):
    rclpy.init(args=args)
    gui = JointGUI()
    gui.window.mainloop()

if __name__ == '__main__':
    main()
