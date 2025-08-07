import cv2

def list_connected_cameras(max_cameras=10):
    print("Scanning for connected cameras...")
    available_cameras = []

    for index in range(max_cameras):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap is not None and cap.isOpened():
            print(f"Camera found at index {index}")
            available_cameras.append(index)
            cap.release()
        else:
            print(f"No camera at index {index}")
    
    if not available_cameras:
        print("No cameras detected.")
    else:
        print(f"\nAvailable camera indices: {available_cameras}")
    return available_cameras

if __name__ == "__main__":
    list_connected_cameras()
