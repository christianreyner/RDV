# logger.py

import math


class StatusLogger:
    def print_startup_info(self, camera, target_ned):
        print("Camera:")
        print(f"  resolution: {camera.width} x {camera.height}")
        print(f"  HFOV/VFOV:  {camera.hfov_deg:.1f} / {camera.vfov_deg:.1f} deg")
        print(f"  fx/fy:      {camera.fx:.2f} / {camera.fy:.2f}")

        print("Target NED:")
        print(f"  N: {target_ned[0]:.1f} m")
        print(f"  E: {target_ned[1]:.1f} m")
        print(f"  D: {target_ned[2]:.1f} m")

    def print_tracking_status(self, command, projection, altitude_above_home):
        if projection.visible:
            self._print_visible_status(
                command=command,
                projection=projection,
                altitude_above_home=altitude_above_home,
            )
        else:
            self._print_lost_status(
                command=command,
                projection=projection,
                altitude_above_home=altitude_above_home,
            )

    def _print_visible_status(self, command, projection, altitude_above_home):
        print(
            f"{command['phase']:6s} vis=1 "
            f"alt={altitude_above_home:5.2f} "
            f"u={projection.u:7.1f} "
            f"v={projection.v:7.1f} "
            f"ex={projection.ex:+.3f} "
            f"ey={projection.ey:+.3f} "
            f"roll={math.degrees(command['roll']):+.1f} "
            f"pitch={math.degrees(command['pitch']):+.1f} "
            f"thr={command['thrust']:.2f} "
            f"range={projection.range_m:.1f}"
        )

    def _print_lost_status(self, command, projection, altitude_above_home):
        bearing_cam = projection.bearing_cam

        print(
            f"{command['phase']:6s} vis=0 "
            f"alt={altitude_above_home:5.2f} "
            f"bearing_cam=["
            f"{bearing_cam[0]:+.2f}, "
            f"{bearing_cam[1]:+.2f}, "
            f"{bearing_cam[2]:+.2f}] "
            f"thr={command['thrust']:.2f} "
            f"range={projection.range_m:.1f}"
        )
