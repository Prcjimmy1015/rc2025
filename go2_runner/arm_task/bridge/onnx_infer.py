#!/usr/bin/env python3
"""ONNX Runtime 推理脚本 — 供 C++ 通过 popen 调用
用法: python3 onnx_infer.py <model_path> <image_path>
输出: 分类ID (整数)，失败输出 -1
"""
import sys
import numpy as np
import cv2
import onnxruntime as ort

# ImageNet 标准归一化参数
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

def main():
    if len(sys.argv) != 3:
        print(-1)
        sys.exit(1)

    model_path = sys.argv[1]
    image_path = sys.argv[2]

    try:
        img = cv2.imread(image_path)
        if img is None:
            print(-1)
            sys.exit(1)

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (720, 720)).astype(np.float32) / 255.0
        # ImageNet 归一化: (x - mean) / std
        normalized = (resized - MEAN) / STD
        blob = normalized.transpose(2, 0, 1)  # HWC -> CHW
        blob = np.expand_dims(blob, axis=0)   # 添加 batch 维度

        session = ort.InferenceSession(model_path)
        input_name = session.get_inputs()[0].name
        outputs = session.run(None, {input_name: blob})
        class_id = int(np.argmax(outputs[0]))
        print(class_id)

    except Exception as e:
        print(-1, file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()