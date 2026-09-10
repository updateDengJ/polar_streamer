import cv2
import sys
import numpy as np
import datetime

def decode_metadata(gray_img):
    """从灰度图像的前两行解码序号和时间戳"""
    h, w = gray_img.shape
    if h < 2 or w < 512:
        raise ValueError(f"Image too small: {w}x{h}, need at least 512x2")
    
    row0 = gray_img[0].astype(np.int32)
    row1 = gray_img[1].astype(np.int32)
    
    seq = 0
    ts_us = 0
    
    for i in range(64):
        # 序号部分
        sum_seq = np.sum(row0[i*4 : i*4+4]) + np.sum(row1[i*4 : i*4+4])
        # 时间戳部分：偏移256列
        sum_ts = np.sum(row0[256+i*4 : 256+i*4+4]) + np.sum(row1[256+i*4 : 256+i*4+4])
        
        if sum_seq > 1020:
            seq |= (1 << i)
        if sum_ts > 1020:
            ts_us |= (1 << i)
            
    return seq, ts_us

def main():
    if len(sys.argv) != 2:
        print("Usage: python parse_metadata.py <gray_bmp_file>")
        sys.exit(1)
    
    img = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: cannot read {sys.argv[1]}")
        sys.exit(1)
    
    seq, ts_us = decode_metadata(img)
    
    print(f"Sequence number: {seq}")
    print(f"Timestamp (us):  {ts_us}")
    
    # 【新增逻辑】：将微秒时间戳转换为 年-月-日 时:分:秒.微秒 格式
    if ts_us > 0:
        # 将微秒转换为秒 (包含小数部分)
        timestamp_sec = ts_us / 1_000_000.0
        # 转换为本地时间对象
        dt = datetime.datetime.fromtimestamp(timestamp_sec)
        
        # 格式化输出，%f 表示微秒
        formatted_time = dt.strftime('%Y-%m-%d %H:%M:%S.%f')
        # 如果你想要带汉字的格式，可以取消下面这行的注释：
        # formatted_time = dt.strftime('%Y年%m月%d日 %H时%M分%S秒') + f" {dt.microsecond}微秒"
        
        print(f"Readable time:   {formatted_time}")

if __name__ == "__main__":
    main()