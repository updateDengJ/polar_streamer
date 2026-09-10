// ==========================================
// 文件名: main.c
// 描  述: 主程序入口，负责生命周期管理、信号捕获与资源回收
// ==========================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <gst/gst.h>

#include "config.h"
#include "ezsdk_core.h"
#include "camera.h"
#include "consumer.h"
#include "remote_ctrl.h"

#define FINAL_W 3600
#define FINAL_H 4800

// ==========================================
// 全局变量定义 (对应 config.h 中的 extern 声明)
// ==========================================
const int TARGET_PORT = 2001;

const char* TARGET_SNS[CAM_COUNT] = {
    "FGJ26070121", // 左下 (BL)
    "FGJ26070122", // 左上 (TL)
    "FGJ26070120", // 右下 (BR)
    "FGJ26070119"  // 右上 (TR)
};

// 运行状态标志 (使用 volatile 确保多线程可见性)
volatile sig_atomic_t g_running = 1;

// 相机设备句柄数组
HDEVICE g_h_devs[CAM_COUNT] = {NULL};       
HSUBDEVICE g_h_subdevs[CAM_COUNT] = {NULL}; 

// 相机索引数组 (用于回调函数识别相机)
int g_cam_indices[CAM_COUNT] = {0, 1, 2, 3}; 

// 帧同步结构体实例化
FrameSyncBox g_sync_box = {
    .images = {NULL},
    .block_ids = {0},
    .ready_mask = 0,
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .cond = PTHREAD_COND_INITIALIZER
};

// ==========================================
// 信号处理函数
// ==========================================
void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM || sig == SIGQUIT) {
        printf("\n🛑 [Signal] 捕获到终止信号 (%d)，正在触发安全退出流程...\n", sig);
        g_running = 0;

        // 核心步骤：强制唤醒可能阻塞在 pthread_cond_wait 的消费者线程
        pthread_mutex_lock(&g_sync_box.lock);
        // 修改 ready_mask 破坏等待条件，并广播信号
        g_sync_box.ready_mask = 0xFF; 
        pthread_cond_broadcast(&g_sync_box.cond);
        pthread_mutex_unlock(&g_sync_box.lock);
    }
}

// ==========================================
// 辅助函数：提高系统网络缓冲区（防止高码率丢包）
// ==========================================
void optimize_network_buffers() {
    printf("[System] 正在优化网络缓冲区参数...\n");
    // 尝试将系统 UDP 接收/发送缓冲区提升到 8MB
    // 这需要程序有 root 权限，或者手动执行 sysctl
    int ret1 = system("sysctl -w net.core.rmem_max=8388608 > /dev/null 2>&1");
    int ret2 = system("sysctl -w net.core.wmem_max=8388608 > /dev/null 2>&1");
    int ret3 = system("sysctl -w net.core.rmem_default=8388608 > /dev/null 2>&1");
    int ret4 = system("sysctl -w net.core.wmem_default=8388608 > /dev/null 2>&1");
    
    if (ret1 == 0 && ret2 == 0) {
        printf("✅ [System] 网络缓冲区优化成功 (8MB)\n");
    } else {
        printf("⚠️ [Warning] 网络缓冲区优化可能需要 root 权限，继续使用系统默认值\n");
    }
}

// ==========================================
// 主函数
// ==========================================
int main(int argc, char *argv[]) {
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║     四路偏振相机拼接推流系统 v2.0 - 极速偏振处理版       ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

    // 0. 优化内核参数
    optimize_network_buffers();

    // 1. 初始化 GStreamer 环境
    printf("[System] 初始化 GStreamer...\n");
    gst_init(&argc, &argv);
    printf("✅ [System] GStreamer 初始化完成\n");

    // 2. 注册信号捕获
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);
    
    // 忽略 SIGPIPE，防止网络断开时程序崩溃
    signal(SIGPIPE, SIG_IGN);

    // 3. 初始化核心同步锁机制（如果使用 PTHREAD_MUTEX_INITIALIZER 则不需要重复初始化）
    // 但为了安全，检查是否需要重新初始化
    if (g_sync_box.lock.__data.__lock != 0) {
        pthread_mutex_destroy(&g_sync_box.lock);
        pthread_cond_destroy(&g_sync_box.cond);
        pthread_mutex_init(&g_sync_box.lock, NULL);
        pthread_cond_init(&g_sync_box.cond, NULL);
    }

    // 4. 加载底库 (libEzSDK_C.so)
    printf("\n[System] 正在加载 SDK 动态库...\n");
    if (InitEzSDK("./libEzSDK_C.so") != 0) {
        fprintf(stderr, "❌ [Error] 无法加载 SDK 动态库！请检查库文件是否存在。\n");
        return -1;
    }
    printf("✅ [System] SDK 动态库加载成功\n");

    // 5. 连接并启动相机流
    printf("\n📷 [System] 正在搜索并连接相机...\n");
    HMANAGER h_mgr = NULL;
    if (ConnectAndStartCameras(&h_mgr) != 0) {
        fprintf(stderr, "❌ [Error] 相机连接或启动失败！\n");
        g_running = 0;
        goto cleanup;
    }

    // 6. 等待相机流稳定 (给 SDK 一些缓冲时间)
    printf("\n⏳ [System] 等待相机流稳定 (2秒)...\n");
    sleep(2);

    // 7. 初始化远程控制服务端 (UDP 2001 端口)
    printf("\n🌐 [System] 启动远程控制服务...\n");
    if (start_remote_server(2001) != 0) {
        fprintf(stderr, "⚠️ [Warning] 远程控制服务端启动失败，继续运行...\n");
        // 非致命错误可以继续
    } else {
        printf("✅ [System] 远程控制服务已启动，监听端口: 8888\n");
    }

    // 8. 启动消费者线程（处理偏振计算与 GStreamer 推流）
    printf("\n🎬 [System] 启动拼接推流线程...\n");
    pthread_t stitch_thread_id = 0;
    if (pthread_create(&stitch_thread_id, NULL, StitchingThreadFunc, NULL) != 0) {
        fprintf(stderr, "❌ [Error] 无法创建消费者线程！\n");
        g_running = 0;
        goto cleanup;
    }
    printf("✅ [System] 拼接推流线程已启动\n");

    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("✨ [System] 系统已就绪，全速运行中！\n");
    printf("📊 目标输出: %d x %d (单通道 GRAY8)\n", FINAL_W, FINAL_H);
    printf("🎯 UDP 推流地址: 192.168.1.23:2000\n");
    printf("🎮 远程控制端口: 2001\n");
    printf("⌨️  按 Ctrl+C 可安全停止并释放硬件资源\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    // 9. 主线程循环监听运行状态
    int heartbeat_counter = 0;
    while (g_running) {
        sleep(5);
        heartbeat_counter += 5;
        
        // 心跳输出
        printf("💓 [Heartbeat] 系统运行中... (%d秒)\n", heartbeat_counter);
        
        // 可以在这里添加更多状态检查
        // 例如: 检查相机连接状态、内存使用等
    }

cleanup:
    printf("\n🛑 [System] >>> 开始执行资源回收流程 <<<\n");
    
    // A. 首先通知并回收消费者线程
    printf("[Cleanup] 正在停止推流线程...\n");
    pthread_mutex_lock(&g_sync_box.lock);
    pthread_cond_broadcast(&g_sync_box.cond); 
    pthread_mutex_unlock(&g_sync_box.lock);

    if (stitch_thread_id != 0) {
        // 等待线程最多 5 秒
        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 5;
        
        int ret = pthread_timedjoin_np(stitch_thread_id, NULL, &timeout);
        if (ret == 0) {
            printf("[Cleanup] 推流线程已正常退出。\n");
        } else {
            printf("[Cleanup] 推流线程退出超时，强制继续清理...\n");
            pthread_cancel(stitch_thread_id);
        }
    }

    // B. 停止相机采集逻辑
    printf("[Cleanup] 正在断开相机连接...\n");
    StopAndDisconnectCameras();
    printf("[Cleanup] 相机已全部断开。\n");

    // C. 销毁 SDK 管理器与卸载库
    if (h_mgr) {
        g_Manager_Destory(h_mgr);
        printf("[Cleanup] SDK 管理器已销毁。\n");
    }
    
    ReleaseEzSDK();
    printf("[Cleanup] SDK 动态库已卸载。\n");

    // D. 销毁同步原语
    pthread_mutex_destroy(&g_sync_box.lock);
    pthread_cond_destroy(&g_sync_box.cond);
    printf("[Cleanup] 同步原语已销毁。\n");

    // E. 停止远程控制服务（如果有对应的停止函数）
    // stop_remote_server(); 

    printf("\n✅ [System] 所有硬件资源已释放。程序干净退出。\n");
    printf("═══════════════════════════════════════════════════════════\n");
    return 0;
}
