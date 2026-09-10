// ==========================================
// 文件名: consumer.h
// 描  述: 消费者线程，负责图像拼接和GStreamer推流
// ==========================================
#ifndef CONSUMER_H
#define CONSUMER_H

void* StitchingThreadFunc(void* arg);

#endif // CONSUMER_H