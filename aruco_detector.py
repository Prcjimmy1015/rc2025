import cv2
import numpy as np
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
import socket

SOCKET_HOST = '127.0.0.1'
SOCKET_PORT = 5005

def main():
    if len(sys.argv) > 1:
        print(f"Using interface: {sys.argv[1]}")
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        print("Using default interface")
        ChannelFactoryInitialize(0)

    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    # Set up TCP socket for sending detection results
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((SOCKET_HOST, SOCKET_PORT))
        print(f"Connected to socket bridge at {SOCKET_HOST}:{SOCKET_PORT}")
    except Exception as e:
        print(f"Failed to connect to socket bridge: {e}")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    marker_length = 0.10
    K = np.array([[929.7797, 0, 629.6662],
                 [0, 926.7584, 335.6207],
                 [0, 0, 1]])
    D = np.array([-0.4157, 0.1327, 0, 0])

    code, data = client.GetImageSample()
    last_detection = None  # None, int (0-5), or -1
    while code == 0:
        code, data = client.GetImageSample()
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        detected = False
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if 0 <= marker_id <= 5:
                    cv2.aruco.drawDetectedMarkers(image, [corners[i]])
                    # Send detection result over socket
                    if last_detection != marker_id:
                        try:
                            marker_size_px = np.linalg.norm(corners[i][0][0] - corners[i][0][1])
                            dist = marker_length * K[0][0] / marker_size_px
                            rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers([corners[i]], marker_length, K, D)[:2]
                            tvec = tvecs[0][0]
                            angle = np.arctan2(tvec[0], tvec[2])  # 正值=marker在右
                            print(f"Aruco detected: {marker_id} dist: {dist:.2f} angle: {angle:.2f}")
                            sock.sendall(f"{marker_id},{dist:.2f},{angle:.2f}\n".encode())
                        except Exception as e:
                            print(f"Socket send error: {e}")
                        last_detection = marker_id
                    detected = True
        if not detected and last_detection != -1:
            print("No Aruco detected")
            try:
                sock.sendall(b'-1\n')
            except Exception as e:
                print(f"Socket send error: {e}")
            last_detection = -1
        try:
            cv2.imshow('Aruco Detection', image)
            if cv2.waitKey(20) == 27:
                break
        except cv2.error:
            # Headless mode: skip imshow
            pass
    sock.close()
    if code != 0:
        print("Get image sample error. code:", code)
    try:
        cv2.destroyWindow('Aruco Detection')
    except cv2.error:
        pass

if __name__ == "__main__":
    main()
