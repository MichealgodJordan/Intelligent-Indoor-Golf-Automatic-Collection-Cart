


from ultralytics import YOLO

# load the pretrianed model
model = YOLO(r'D:\XJTLU\MEC202\models\yolov12m.pt')

# train the model
# The data.yaml file should contain the paths to the training and validation datasets
results = model.train(
    data=r'D:\XJTLU\MEC202\yolov11_dataset\data.yaml',
    epochs=1000,
    imgsz=640,
    batch=16,
    name='yolov11_custom'
)

