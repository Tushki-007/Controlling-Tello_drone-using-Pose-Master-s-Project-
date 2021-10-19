# Pose Recognition
This is repository for pose recognition module written in Python. It should have a generic class  
that act as a Python package/module so that another program can use it.  

This module shall handle all required threading and running all essential program by itself for   
parallel processing if required.

# Code Structure

```
.
├── example                             <-- Basic example using MediaPipe API
│   ├── mediaPipe_face.py
│   ├── mediaPipe_pose.py
│   └── README.md
├── example_pose_recognition_class.py   <-- Basic example to use PoseRecognition class
├── pose_recognition.py                 <-- PoseRecognition class
└── README.md                           <-- This file
```

# Installation & Requirement
In order to run example code in `example` folder and `example_pose_recognition_class.py` script,  
PIP package defined in `requirements.txt` file and their dependencies shall be installed.  

To install all PIP package required can use this command `pip3 install -r requirements.txt`
