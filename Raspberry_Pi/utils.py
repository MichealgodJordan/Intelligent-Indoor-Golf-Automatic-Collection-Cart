import argparse

def parse_args():
    """
    Parse command-line arguments for the autonomous car controller.
    
    Returns:
        argparse.Namespace: Parsed arguments with default values.
    """
    parser = argparse.ArgumentParser(description='Autonomous ball detection and cruise control with similar triangles')
    
    # Model and serial configuration
    parser.add_argument('--model', type=str, default=r'D:\XJTLU\MEC202\runs\detect\yolov11_custom11\weights\best.pt', 
                       help='Path to YOLOv5 model')
    parser.add_argument('--serial', type=str, default='/dev/ttyacm0', 
                       help='Arduino serial port or "demo" for demo mode')
    parser.add_argument('--baud_rate', type=int, default=9600, 
                       help='Baud rate for serial communication')
    
    # Image and camera properties
    parser.add_argument('--img_width', type=int, default=640, 
                       help='Image width in pixels')
    parser.add_argument('--img_height', type=int, default=480, 
                       help='Image height in pixels')
    parser.add_argument('--EFL', type=float, default=0.0028, 
                       help='Effective focal length in meters')
    parser.add_argument('--h', type=float, default=0.13, 
                       help='Camera mounting height in meters')
    parser.add_argument('--tilt_angle', type=float, default=0.35, 
                       help='Camera downward tilt in radians')
    parser.add_argument('--camera_to_car_offset', type=float, default=0.15, 
                       help='Offset from camera to car center in meters')

    # Control and threshold parameters
    parser.add_argument('--desired_offset', type=float, default=0.10, 
                       help='Desired ball lateral offset in car coordinates in meters')
    parser.add_argument('--align_threshold', type=float, default=0.03, 
                       help='Threshold for considering rel_x small in meters')
    parser.add_argument('--car_alignment_threshold', type=float, default=0.02, 
                       help='Final alignment threshold for car in meters')
    parser.add_argument('--command_delay', type=float, default=0.1, 
                       help='Delay between commands in seconds')
    parser.add_argument('--actual_ball_size', type=float, default=0.04267, 
                       help='Actual ball size in meters')
    parser.add_argument('--left_offset', type=float, default=-0.085, 
                       help='Left offset threshold in meters')
    parser.add_argument('--right_offset', type=float, default=-0.17, 
                       help='Right offset threshold in meters')

    # YOLO detection parameters
    parser.add_argument('--conf_thres', type=float, default=0.25, 
                       help='Confidence threshold for YOLO detection')
    parser.add_argument('--iou_thres', type=float, default=0.45, 
                       help='IoU threshold for YOLO detection')

    # State variables
    parser.add_argument('--last_command_time', type=float, default=0,
                       help='Timestamp of last command sent')
    parser.add_argument('--c_cmd', type=str, default="C-R",
                       help='Default cruise command')
    parser.add_argument('--is_auto', type=bool, default=False,
                       help='Auto mode flag')
    parser.add_argument('--target_on', type=bool, default=False,
                       help='Target tracking flag')
    parser.add_argument('--first_time', type=bool, default=True,
                       help='First-time execution flag')
    parser.add_argument('--action_complete', type=bool, default=False,
                       help='Action completion flag')
    parser.add_argument('--stop_auto', type=bool, default=False,
                       help='Stop auto mode flag')
    parser.add_argument('--cruise_complete', type=bool, default=False,
                       help='Cruise action completion flag')
    parser.add_argument('--count', type=int, default=0,
                       help='Cruise action counter')
    parser.add_argument('--both_side', type=bool, default=False,
                       help='Flag for balls on both sides')
    parser.add_argument('--have_left', type=bool, default=False,
                       help='Flag for ball on left side')
    parser.add_argument('--have_right', type=bool, default=False,
                       help='Flag for ball on right side')
    
    args = parser.parse_args()
    return args