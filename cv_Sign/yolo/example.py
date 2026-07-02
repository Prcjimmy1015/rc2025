from main import model_predict

model_path = "best.onnx"

image_path = r"test"

for top1_id, top1_name in model_predict(image_path, model_path):
    for i in range(0,5):
        if top1_id == i:
            print(f"ID {top1_id} is {top1_name}")
