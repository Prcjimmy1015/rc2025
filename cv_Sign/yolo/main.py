from ultralytics import YOLO


def model_predict(image_path, model_path="best.onnx"):
    """
    输入：
    image_path：图片路径
    model_path：模型路径
    输出：
    top1_id：预测概率最高的类别 ID
    top1_name：预测概率最高的类别名称
    """
    # 1. 加载 ONNX 模型
    model = YOLO(model_path, task="classify")

    # 2. 进行预测（可以直接传入图片、视频路径，PIL图片或 OpenCV 读取的图片）
    results = model(image_path, stream=True)

    # 3. 解析结果
    for r in results:
        # 获取预测概率最高的类别 ID
        top1_id = r.probs.top1  
        '''
        r.probs.top1 返回的是 int （整数）类型 ，表示概率最高的类别索引（class index）。
        '''
        yield top1_id, r.names[r.probs.top1]
