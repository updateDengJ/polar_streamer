# ==========================================
# 极低延迟偏振相机拼接系统 Makefile
# ==========================================

# 1. 编译器设置
CC = gcc

# 2. 目标可执行文件名
TARGET = my_streamer

# 3. 源文件列表 (手动列出确保顺序，或自动获取)
# 🚀 增加了 remote_ctrl.c
SRCS = main.c ezsdk_core.c camera.c consumer.c remote_ctrl.c
OBJS = $(SRCS:.c=.o)

# 4. 编译选项 (CFLAGS)
# -O3: 开启最高级优化，对像素级计算至关重要
# -fopenmp: 开启多核并行计算加速
CFLAGS = -Wall -O3 -I. -fopenmp `pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0`

# 5. 链接选项 (LDFLAGS)
# -lm: 🚀 必须加在末尾，链接数学库以支持 sqrtf/atan2f
# -ldl: 加载动态库
# -lpthread: 多线程支持
# -fopenmp: 链接时也需要此标志以支持并行库
LDFLAGS = -ldl -lpthread -fopenmp `pkg-config --libs gstreamer-1.0 gstreamer-app-1.0` -lm

# ==========================================
# 编译规则
# ==========================================

all: $(TARGET)

# 链接规则
$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

# 编译规则
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean