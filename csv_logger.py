# csv_logger.py

import csv


class CSVLogger:
    def __init__(self, path):
        self.path = path
        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "time_s",

            "quad_north_m",
            "quad_east_m",
            "quad_down_m",
            "quad_alt_above_home_m",

            "target_north_m",
            "target_east_m",
            "target_down_m",
            "target_alt_above_home_m",

            "target_visible",
            "target_u_px",
            "target_v_px",
            "target_ex",
            "target_ey",
            "target_range_m",

            "cmd_roll_rad",
            "cmd_pitch_rad",
            "cmd_thrust",
            "cmd_phase",
        ])

        self.file.flush()

    def log(
        self,
        time_s,
        quad_pos,
        target_state,
        projection,
        command,
    ):
        quad_alt_above_home_m = -quad_pos.z
        target_alt_above_home_m = -target_state.down_m

        self.writer.writerow([
            f"{time_s:.3f}",

            f"{quad_pos.x:.3f}",
            f"{quad_pos.y:.3f}",
            f"{quad_pos.z:.3f}",
            f"{quad_alt_above_home_m:.3f}",

            f"{target_state.north_m:.3f}",
            f"{target_state.east_m:.3f}",
            f"{target_state.down_m:.3f}",
            f"{target_alt_above_home_m:.3f}",

            int(projection.visible),
            "" if projection.u is None else f"{projection.u:.3f}",
            "" if projection.v is None else f"{projection.v:.3f}",
            "" if projection.ex is None else f"{projection.ex:.6f}",
            "" if projection.ey is None else f"{projection.ey:.6f}",
            f"{projection.range_m:.3f}",

            f"{command['roll']:.6f}",
            f"{command['pitch']:.6f}",
            f"{command['thrust']:.6f}",
            command["phase"],
        ])

        self.file.flush()

    def close(self):
        self.file.close()
