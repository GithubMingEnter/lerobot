import cv2
import threading

# 定义一个函数来读取相机并显示视频流
def show_camera(camera_id, window_name):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"无法打开相机 {camera_id}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"无法读取相机 {camera_id} 的帧")
            break

        cv2.imshow(window_name, frame)

        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyWindow(window_name)

# 创建线程来读取不同的相机
camera_threads = []
camera_ids = [0, 2]  # 假设有两个相机，ID分别为0和1

for i, camera_id in enumerate(camera_ids):
    window_name = f"Camera {camera_id}"
    thread = threading.Thread(target=show_camera, args=(camera_id, window_name))
    camera_threads.append(thread)
    thread.start()

# 主线程处理窗口事件
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 等待所有线程完成
for thread in camera_threads:
    thread.join()

cv2.destroyAllWindows()