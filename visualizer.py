# visualizer.py

import matplotlib.pyplot as plt


class LiveVisualizer:
    def __init__(self, camera):
        self.camera = camera

        self.quad_north = []
        self.quad_east = []
        self.quad_alt = []

        self.target_north = []
        self.target_east = []
        self.target_alt = []

        plt.ion()

        self.fig = plt.figure(figsize=(12, 5))

        self.ax_cam = self.fig.add_subplot(1, 2, 1)
        self.ax_traj = self.fig.add_subplot(1, 2, 2, projection="3d")

        self._setup_camera_view()
        self._setup_trajectory_view()

    def _setup_camera_view(self):
        self.ax_cam.set_title("Camera View")
        self.ax_cam.set_xlim(0, self.camera.width)
        self.ax_cam.set_ylim(self.camera.height, 0)
        self.ax_cam.set_xlabel("u [px]")
        self.ax_cam.set_ylabel("v [px]")
        self.ax_cam.grid(True)

        self.ax_cam.axvline(
            self.camera.cx,
            color="gray",
            linestyle="--",
            linewidth=1,
        )

        self.ax_cam.axhline(
            self.camera.cy,
            color="gray",
            linestyle="--",
            linewidth=1,
        )

        self.target_dot_cam, = self.ax_cam.plot(
            [],
            [],
            "ro",
            markersize=8,
            label="target",
        )

        self.ax_cam.legend(loc="upper right")

    def _setup_trajectory_view(self):
        self.ax_traj.set_title("3D Trajectory View")
        self.ax_traj.set_xlabel("East [m]")
        self.ax_traj.set_ylabel("North [m]")
        self.ax_traj.set_zlabel("Altitude [m]")

        self.quad_path_line, = self.ax_traj.plot(
            [],
            [],
            [],
            "b-",
            linewidth=2,
            label="quad path",
        )

        self.target_path_line, = self.ax_traj.plot(
            [],
            [],
            [],
            "r-",
            linewidth=2,
            label="target path",
        )

        self.quad_dot, = self.ax_traj.plot(
            [],
            [],
            [],
            "bo",
            markersize=7,
            label="quad",
        )

        self.target_dot_traj, = self.ax_traj.plot(
            [],
            [],
            [],
            "ro",
            markersize=7,
            label="target",
        )

        self.ax_traj.legend(loc="upper right")

    def update(self, quad_pos, target_state, projection):
        quad_alt = -quad_pos.z
        target_alt = -target_state.down_m

        self.quad_north.append(quad_pos.x)
        self.quad_east.append(quad_pos.y)
        self.quad_alt.append(quad_alt)

        self.target_north.append(target_state.north_m)
        self.target_east.append(target_state.east_m)
        self.target_alt.append(target_alt)

        self._update_camera_view(projection)
        self._update_trajectory_view(quad_pos, target_state)

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def _update_camera_view(self, projection):
        if projection.visible:
            self.target_dot_cam.set_data(
                [projection.u],
                [projection.v],
            )
        else:
            self.target_dot_cam.set_data([], [])

    def _update_trajectory_view(self, quad_pos, target_state):
        quad_alt = -quad_pos.z
        target_alt = -target_state.down_m

        self.quad_path_line.set_data(
            self.quad_east,
            self.quad_north,
        )
        self.quad_path_line.set_3d_properties(self.quad_alt)

        self.target_path_line.set_data(
            self.target_east,
            self.target_north,
        )
        self.target_path_line.set_3d_properties(self.target_alt)

        self.quad_dot.set_data(
            [quad_pos.y],
            [quad_pos.x],
        )
        self.quad_dot.set_3d_properties([quad_alt])

        self.target_dot_traj.set_data(
            [target_state.east_m],
            [target_state.north_m],
        )
        self.target_dot_traj.set_3d_properties([target_alt])

        self._autoscale_3d_axes()

    def _autoscale_3d_axes(self):
        all_east = self.quad_east + self.target_east
        all_north = self.quad_north + self.target_north
        all_alt = self.quad_alt + self.target_alt

        if len(all_east) < 2:
            return

        margin_xy = 10.0
        margin_z = 5.0

        min_east = min(all_east) - margin_xy
        max_east = max(all_east) + margin_xy

        min_north = min(all_north) - margin_xy
        max_north = max(all_north) + margin_xy

        min_alt = min(all_alt) - margin_z
        max_alt = max(all_alt) + margin_z

        if abs(max_east - min_east) < 1e-6:
            min_east -= margin_xy
            max_east += margin_xy

        if abs(max_north - min_north) < 1e-6:
            min_north -= margin_xy
            max_north += margin_xy

        if abs(max_alt - min_alt) < 1e-6:
            min_alt -= margin_z
            max_alt += margin_z

        self.ax_traj.set_xlim(min_east, max_east)
        self.ax_traj.set_ylim(min_north, max_north)
        self.ax_traj.set_zlim(min_alt, max_alt)

    def close(self):
        plt.ioff()
        plt.close(self.fig)
