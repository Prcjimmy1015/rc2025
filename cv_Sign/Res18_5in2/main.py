import cv2
import numpy as np
import onnxruntime as ort

# 1. 初始化 ONNX Runtime 会话
session = ort.InferenceSession("3in1.onnx")

# 获取模型的输入输出节点信息
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape  #  [1, 3, 720, 720]
input_height, input_width = input_shape[2], input_shape[3]

# 2. 图像预处理
img = cv2.imread(r"test\20260701_190142.jpg")
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 训练使用的是 RGB 格式
img_resized = cv2.resize(img_rgb, (input_width, input_height))  # 缩放到模型要求的尺寸

# 归一化并调整维度 (H, W, C) -> (C, H, W) -> (1, C, H, W)
img_data = img_resized.astype(np.float32) / 255.0
img_data = np.transpose(img_data, (2, 0, 1))
img_data = np.expand_dims(img_data, axis=0)

# 3. 模型推理
outputs = session.run(None, {input_name: img_data})

# 4. 后处理（解析输出）
# 模型的输出通常是未经过 Softmax 的 logits
logits = outputs[0][0]

# 手动计算 Softmax 转化为概率
exp_logits = np.exp(logits - np.max(logits))  # 减去最大值防止溢出
probs = exp_logits / np.sum(exp_logits)

# 获取最大概率的类别
class_id = np.argmax(probs)
confidence = probs[class_id]

print(f"预测类别 ID: {class_id}, 置信度: {confidence:.4f}")