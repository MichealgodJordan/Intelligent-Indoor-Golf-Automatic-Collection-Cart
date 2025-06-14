# YOLO Car Controller

## Overview
This project implements a Raspberry Pi-based autonomous car controller that uses the YOLO model for ball detection and serial communication with an Arduino to navigate towards detected balls. It includes logic for cruising when no balls are detected.

## Hardware Requirements
- Raspberry Pi (with camera module)
- Arduino (connected via USB)
- USB camera (index 0 by default)
- Car-like robot with Arduino-controlled motors

## Software Requirements
- Python 3.8+
- OpenCV (`pip install opencv-python`)
- PyTorch (`pip install torch`)
- Ultralytics YOLO (`pip install ultralytics`)
- PySerial (`pip install pyserial`)
- NumPy (`pip install numpy`)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/yolo-car-controller.git
   cd yolo-car-controller
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Ensure the YOLO model file (e.g., `best.pt`) is available.
## Dataset structure
1. Prepare the dataset folder in this structure:
```bash
   yolov5_dataset
   ├── data.yaml
   ├── images
   │   ├── train
   │   │   ├── 1_frame_000000.jpg
   │   │                .
   │   │                .
   │   │                .
   │   └── test
   │       ├── 1_frame_000029.jpg
   │                    .
   │                    .
   │                    .
   └── labels
       ├── train
       │   ├── 1_frame_000000.txt
       │   ├── 1_frame_000087.txt
       │                .
       │                .
       │                .
       ├── test
       │   ├── 1_frame_000029.txt
       │   ├── 1_frame_000058.txt
       │                .
       │                .
       └──              .
```

## Usage
1. Connect the Arduino to the Raspberry Pi via USB.
2. Update the serial port in `run.sh` if necessary (default: `/dev/ttyUSB0`).
3. Run the script:
   ```bash
   ./run.sh
   ```
4. For demo mode (no Arduino):
   ```bash
   python raspi_new.py --model /path/to/model.pt --serial demo
   ```

## Parameters
- `--model`: Path to the YOLO model file (default: `/home/Robber/Desktop/MEC202/best.pt`)
- `--serial`: Serial port for Arduino or 'demo' for demo mode (default: `/dev/ttyUSB0`)
- Other parameters (e.g., `EFL`, `h`, `tilt_angle`) are hardcoded in `__init__` but can be modified in the script.

## Example Run
```
model loaded: /home/Robber/Desktop/MEC202/best.pt
serial connected: /dev/ttyUSB0
Connection valid
received 'start' from arduino
Enter auto mode.
[(1.20, 1.18, 0.05, 1.18, 50.00)]
have right:True, have left:False, both side:False
ball detected: d=1.20 m, d_ground=1.18 m, x_actual=0.05 m, forward_distance=1.18 m, ball_box_size=50.00 mm
False
sent command: P_R,dist=1.18
```

## Tutorial: Creating a Conda Virtual Environment and Troubleshooting

### Creating a Conda Virtual Environment
Follow these steps to set up a Conda virtual environment for your project:

1. **Install Miniconda or Anaconda**
   - Download and install Miniconda from [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or Anaconda from [Anaconda](https://www.anaconda.com/products/distribution).
   - Follow the installation instructions for your operating system (Linux for Raspberry Pi).

2. **Create a Virtual Environment**
   - Open a terminal and run:
     ```bash
     conda create -n yolo_car_env python=3.8
     ```
   - This creates an environment named `yolo_car_env` with Python 3.8.

3. **Activate the Environment**
   - Activate it with:
     ```bash
     conda activate yolo_car_env
     ```

4. **Install Dependencies**
   - Use the `requirements.txt` file:
     ```bash
     pip install -r requirements.txt
     ```
   - Or install individually:
     ```bash
     pip install opencv-python torch ultralytics pyserial numpy
     ```

5. **Verify Installation**
   - Test the setup by running a Python shell:
     ```python
     import cv2
     import torch
     import serial
     import numpy as np
     from ultralytics import YOLO
     ```
   - If no errors occur, the environment is ready.

### Dealing with Possible Problems (Arduino and Raspberry Pi)
Here are common issues and their solutions:

#### **Issue 1: Serial Port Not Found**
- **Symptom**: "Serial port not found" error.
- **Solution**:
  - List available ports on Raspberry Pi:
    ```bash
    ls /dev/tty*
    ```
  - Update the serial port in your script (e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`).

#### **Issue 2: Permission Denied for Serial Port**
- **Symptom**: "Permission denied" error.
- **Solution**:
  - Add your user to the `dialout` group:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
  - Log out and back in.

#### **Issue 3: Arduino Not Responding**
- **Symptom**: No response after sending commands.
- **Solution**:
  - Verify the Arduino code handles commands like 'start', 'stop', 'P_F'.
  - Ensure the baud rate (e.g., 9600) matches in both the script and Arduino code.

#### **Issue 4: Camera Not Detected**
- **Symptom**: "Cannot open camera" error.
- **Solution**:
  - Check camera connection and enable it on Raspberry Pi:
    ```bash
    sudo raspi-config
    ```
    - Navigate to "Interface Options" and enable the camera.
  - Test different camera indices in `cv2.VideoCapture()` (e.g., 0, 1, 2).

#### **Issue 5: YOLO Model Not Loading**
- **Symptom**: "Failed to load model" error.
- **Solution**:
  - Confirm the model file path (e.g., `/home/Robber/Desktop/MEC202/best.pt`) is correct.
  - Ensure the file exists and is not corrupted.

#### **Issue 6: Dependency Conflicts**
- **Symptom**: Version incompatibility errors.
- **Solution**:
  - Use the Conda environment to isolate dependencies.
  - Specify versions if needed (e.g., `pip install torch==1.9.0`).

## Notes
- Ensure the camera is at index 0 or adjust `cv2.VideoCapture(0)` accordingly.
- The Arduino must be programmed to respond to commands like 'start', 'stop', 'P_F', etc.
- Adjust hyperparameters in the script or via a config file for different setups.
