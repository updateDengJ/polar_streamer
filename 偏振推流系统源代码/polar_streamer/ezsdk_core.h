// ==========================================
// 文件名: ezsdk_core.h
// 描  述: 动态加载底层 EzSDK_C.so 及其函数指针声明
// ==========================================
#ifndef EZSDK_CORE_H
#define EZSDK_CORE_H

#include "EzSDK_C.h"

// --- 函数指针类型定义 ---
typedef HMANAGER(*Manager_Create_Type)(unsigned int);
typedef void (*Manager_Destory_Type)(HMANAGER);
typedef int (*Manager_GetDevices_Type)(HMANAGER, IntfDevType);
typedef HDEVICE(*Manager_GetDeviceByIndex_Type)(HMANAGER, unsigned int, IntfDevType);
typedef int (*Device_GetSubDeviceCount_Type)(HDEVICE);
typedef HSUBDEVICE(*Device_GetSubDeviceByIndex_Type)(HDEVICE, unsigned int);
typedef void (*Device_Destory_Type)(HDEVICE);
typedef int (*SubDevice_GetSubDeviceName_Type)(HSUBDEVICE, char*, size_t);
typedef int (*SubDevice_Connect_Type)(HSUBDEVICE);
typedef int (*SubDevice_DisConnect_Type)(HSUBDEVICE);
typedef int (*SubDevice_Destory_Type)(HSUBDEVICE);
typedef int (*SubDevice_OpenStreamByIndex_Type)(HSUBDEVICE, unsigned int, unsigned int);
typedef int (*SubDevice_StartStreamByIndex_Type)(HSUBDEVICE, unsigned int, StreamCallBack, void*, unsigned int);
typedef int (*SubDevice_StopStreamByIndex_Type)(HSUBDEVICE, unsigned int);
typedef int (*SubDevice_CloseStreamByIndex_Type)(HSUBDEVICE, unsigned int);
typedef char* (*Image_GetImageBuff_Type)(HIMAGE);
typedef ImageInfo(*Image_GetImageInfo_Type)(HIMAGE);
typedef void (*Image_Recycle_Type)(HIMAGE);

// 参数节点操作指针
typedef int (*GetNodeType_Type)(HSUBDEVICE, const char*, NodeType*);
typedef int (*GetNodeValue_Double_Type)(HSUBDEVICE, const char*, double*);
typedef int (*SetNodeValue_Int_Type)(HSUBDEVICE, const char*, int64_t);
typedef int (*SetNodeValue_Double_Type)(HSUBDEVICE, const char*, double);
typedef int (*SetNodeValue_Bool_Type)(HSUBDEVICE, const char*, bool);
typedef int (*SetNodeValue_Chars_Type)(HSUBDEVICE, const char*, const char*);
typedef int (*SetNodeValue_Command_Type)(HSUBDEVICE, const char*);
typedef int (*SetNodeValue_Buffer_Type)(HSUBDEVICE, const char*, const unsigned char*, unsigned int);

// --- 全局函数指针 extern 声明 ---
extern Manager_Create_Type g_Manager_Create;
extern Manager_Destory_Type g_Manager_Destory;
extern Manager_GetDevices_Type g_Manager_GetDevices;
extern Manager_GetDeviceByIndex_Type g_Manager_GetDeviceByIndex;
extern Device_GetSubDeviceCount_Type g_Device_GetSubDeviceCount;
extern Device_GetSubDeviceByIndex_Type g_Device_GetSubDeviceByIndex;
extern Device_Destory_Type g_Device_Destory;
extern SubDevice_GetSubDeviceName_Type g_SubDevice_GetSubDeviceName;
extern SubDevice_Connect_Type g_SubDevice_Connect;
extern SubDevice_DisConnect_Type g_SubDevice_DisConnect;
extern SubDevice_Destory_Type g_SubDevice_Destory;
extern SubDevice_OpenStreamByIndex_Type g_SubDevice_OpenStreamByIndex;
extern SubDevice_StartStreamByIndex_Type g_SubDevice_StartStreamByIndex;
extern SubDevice_StopStreamByIndex_Type g_SubDevice_StopStreamByIndex;
extern SubDevice_CloseStreamByIndex_Type g_SubDevice_CloseStreamByIndex;
extern Image_GetImageBuff_Type g_Image_GetImageBuff;
extern Image_GetImageInfo_Type g_Image_GetImageInfo;
extern Image_Recycle_Type g_Image_Recycle;

extern GetNodeType_Type g_GetNodeType;
extern GetNodeValue_Double_Type g_GetNodeValue_Double;
extern SetNodeValue_Int_Type g_SetNodeValue_Int;
extern SetNodeValue_Double_Type g_SetNodeValue_Double;
extern SetNodeValue_Bool_Type g_SetNodeValue_Bool;
extern SetNodeValue_Chars_Type g_SetNodeValue_Chars;
extern SetNodeValue_Command_Type g_SetNodeValue_Command;
extern SetNodeValue_Buffer_Type g_SetNodeValue_Buffer;

// --- 模块对外接口 ---
int InitEzSDK(const char* dll_path);
void ReleaseEzSDK();

#endif // EZSDK_CORE_H
