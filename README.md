# RDV
High speed rendezvous system

## Files:
RDV/  
├── main.py              # tiny entry point  
├── app.py               # main run sequence  
├── config.py            # tuning values  
├── camera.py            # camera model  
├── target_tracker.py    # target projection  
├── controller.py        # visual PD controller  
├── mavlink_client.py    # MAVLink wrapper  
├── math_utils.py        # rotation/math helpers  
└── logger.py            # console output  

## Parameter setup:
- Make sure the aircraft is properly tuned with a known hover value.
- Set the thrust value accordingly (climb and post takeoff).
- Ideally climb is enough to stabilize, then followed by higher throttle.

## Tuning values:
- camera resolution and FOV
- target position
- PD gains
- climb thrust
- max tilt
= loop rate

## SITL Setup:
1. Setup the SITL environment with port 14551 for companion  
python3 Tools/autotest/sim_vehicle.py -v ArduCopter  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551  

2. Run the program  
python3 main.py
