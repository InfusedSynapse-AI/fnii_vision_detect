import cv2
import numpy as np

def resize_and_pad_cv2(image, target_size):
    """
    使用 OpenCV 调整图片大小以适配目标尺寸，同时保持宽高比。
    用黑色填充剩余部分，使图片尺寸与目标一致。
    """
    h, w = image.shape[:2]
    target_w, target_h = target_size
    scale = min(target_w / w, target_h / h)  # 缩放比例，保证不变形
    new_w, new_h = int(w * scale), int(h * scale)  # 计算缩放后的尺寸
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)  # 缩放图片

    # 创建黑色背景并将图片粘贴到中心
    padded_image = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    offset_x = (target_w - new_w) // 2
    offset_y = (target_h - new_h) // 2
    padded_image[offset_y:offset_y+new_h, offset_x:offset_x+new_w] = resized
    return padded_image

def stitch_n_images_cv2(image_paths, output_size=(1280, 720)):
    """
    使用 OpenCV 将 n 张图片按行列动态排列，生成指定大小的图片（默认 1280×720）。
    如果图片不足，空白区域用黑色填充。
    """
    n = len(image_paths)
    if n == 0:
        return np.zeros((output_size[1], output_size[0], 3), dtype=np.uint8)  # 返回全黑图片

    # 动态计算行数和列数
    rows = math.ceil(math.sqrt(n))  # 行数
    cols = math.ceil(n / rows)  # 列数

    # 每个子图的目标大小
    sub_width = output_size[0] // cols
    sub_height = output_size[1] // rows

    # 创建最终拼接的黑色背景图片
    stitched_image = np.zeros((output_size[1], output_size[0], 3), dtype=np.uint8)

    for idx, img_path in enumerate(image_paths):
        img = cv2.imread(img_path)  # 读取图片
        if img is None:
            continue  # 跳过无法读取的图片
        resized_img = resize_and_pad_cv2(img, (sub_width, sub_height))  # 调整大小并填充
        # 计算子图位置
        row, col = divmod(idx, cols)
        x_start, y_start = col * sub_width, row * sub_height
        stitched_image[y_start:y_start+sub_height, x_start:x_start+sub_width] = resized_img  # 放置图片

    return stitched_image

# 示例：假设有 n 张图片
image_paths = [
    "/mnt/data/image1.jpg",
    "/mnt/data/image2.jpg",
    "/mnt/data/image3.jpg",
    "/mnt/data/image4.jpg",
    "/mnt/data/image5.jpg",
]
stitched = stitch_n_images_cv2(image_paths)
output_path = "/mnt/data/stitched_n_images_cv2.jpg"
cv2.imwrite(output_path, stitched)
output_path
