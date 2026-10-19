from ultralytics import YOLO
import cv2
import socket
import json
import numpy as np
import struct
import threading
import queue

# 1. 加载模型
model = YOLO("best.pt") 

# ================= 核心设计：极简线程队列 =================
# 队列容量为 1，保证只拿最新的一帧，旧的全部扔掉，防止画面卡死
frame_queue = queue.Queue(maxsize=1)

def udp_receiver():
    UDP_PORT = 8888
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(('0.0.0.0', UDP_PORT))
    frame_buffer = {}
    HEADER_SIZE = 8
    
    while True:
        try:
            packet, _ = udp_sock.recvfrom(1500)
            if len(packet) < HEADER_SIZE: continue

            frame_id, total_chunks, chunk_id, payload_len = struct.unpack('>HHHH', packet[:HEADER_SIZE])
            payload = packet[HEADER_SIZE:HEADER_SIZE + payload_len]

            if frame_id not in frame_buffer:
                frame_buffer[frame_id] = {}
            frame_buffer[frame_id][chunk_id] = payload

            if len(frame_buffer[frame_id]) == total_chunks:
                full_jpeg = b''.join(frame_buffer[frame_id][i] for i in range(total_chunks))
                frame = cv2.imdecode(np.frombuffer(full_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # 如果队列满了，扔掉旧图，塞入新图
                    if frame_queue.full():
                        try: frame_queue.get_nowait()
                        except queue.Empty: pass
                    frame_queue.put(frame)
                
                # 清理旧缓存，防内存泄漏
                del frame_buffer[frame_id]
        except Exception:
            pass

# 启动接收线程
threading.Thread(target=udp_receiver, daemon=True).start()
print("UDP 接收线程已启动，等待图像...")

# ================= 主线程（YOLO + 显示 + TCP发送） =================
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 9999))
server.listen(1)
print("等待虚拟机连接...")
conn, addr = server.accept()
print(f"虚拟机已连接: {addr}")
conn.settimeout(0.1) # 防止虚拟机掉线卡死主程序

while True:
    try:
        # 拿最新的一帧
        frame = frame_queue.get(timeout=0.01)
        
        # 图像锐化 (强行让YOLO看清边缘)
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        frame = cv2.filter2D(frame, -1, kernel)
        
        # YOLO 推理
        results = model(frame, conf=0.2, verbose=False)
        boxes = results[0].boxes.xyxy.cpu().numpy()
        print(f"识别到 {len(boxes)} 个目标")  # <--- 重点看这里！
        
        if len(boxes) > 0:
            data = json.dumps({"x1": float(boxes[0][0]), "y1": float(boxes[0][1]), "x2": float(boxes[0][2]), "y2": float(boxes[0][3])})
            try:
                conn.sendall(data.encode('utf-8') + b'\n')
                print(f"坐标已发: {data}")
            except Exception as e:
                print(f"TCP 发送失败: {e}")
                # 如果断开了，尝试重连（或者你可以选择退出）
                try:
                    conn.close()
                    conn, addr = server.accept()
                except: pass

        # 显示画面
        cv2.imshow("Host Inference", results[0].plot())

    except queue.Empty:
        pass # 没拿到图，直接跳过
    
    # 🚨 必须频繁调用 waitKey，否则窗口必定假死
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

cv2.destroyAllWindows()
conn.close()
server.close()