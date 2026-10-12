# main.py

import config as cfg

if cfg.USE_REAL_CAMERA:
    from live_app import LiveCameraApp as VisualLaunchTrackerApp
else:
    from app import VisualLaunchTrackerApp


def main():
    app = VisualLaunchTrackerApp()
    app.run()


if __name__ == "__main__":
    main()
