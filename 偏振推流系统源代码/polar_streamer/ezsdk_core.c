// ==========================================
// 文件名: ezsdk_core.c
// 描  述: 真正分配函数指针并加载 so 库
// ==========================================
#include "ezsdk_core.h"
#include <dlfcn.h>
#include <stdio.h>
#include <stddef.h>

static void* hDll = NULL;

// --- 函数指针的真实定义 ---
Manager_Create_Type g_Manager_Create = NULL;
Manager_Destory_Type g_Manager_Destory = NULL;
Manager_GetDevices_Type g_Manager_GetDevices = NULL;
Manager_GetDeviceByIndex_Type g_Manager_GetDeviceByIndex = NULL;
Device_GetSubDeviceCount_Type g_Device_GetSubDeviceCount = NULL;
Device_GetSubDeviceByIndex_Type g_Device_GetSubDeviceByIndex = NULL;
Device_Destory_Type g_Device_Destory = NULL;
SubDevice_GetSubDeviceName_Type g_SubDevice_GetSubDeviceName = NULL;
SubDevice_Connect_Type g_SubDevice_Connect = NULL;
SubDevice_DisConnect_Type g_SubDevice_DisConnect = NULL;
SubDevice_Destory_Type g_SubDevice_Destory = NULL;
SubDevice_OpenStreamByIndex_Type g_SubDevice_OpenStreamByIndex = NULL;
SubDevice_StartStreamByIndex_Type g_SubDevice_StartStreamByIndex = NULL;
SubDevice_StopStreamByIndex_Type g_SubDevice_StopStreamByIndex = NULL;
SubDevice_CloseStreamByIndex_Type g_SubDevice_CloseStreamByIndex = NULL;
Image_GetImageBuff_Type g_Image_GetImageBuff = NULL;
Image_GetImageInfo_Type g_Image_GetImageInfo = NULL;
Image_Recycle_Type g_Image_Recycle = NULL;

GetNodeType_Type g_GetNodeType = NULL;
GetNodeValue_Double_Type g_GetNodeValue_Double = NULL;
SetNodeValue_Int_Type g_SetNodeValue_Int = NULL;
SetNodeValue_Double_Type g_SetNodeValue_Double = NULL;
SetNodeValue_Bool_Type g_SetNodeValue_Bool = NULL;
SetNodeValue_Chars_Type g_SetNodeValue_Chars = NULL;
SetNodeValue_Command_Type g_SetNodeValue_Command = NULL;
SetNodeValue_Buffer_Type g_SetNodeValue_Buffer = NULL;

int InitEzSDK(const char* dll_path) {
    hDll = dlopen(dll_path, RTLD_LAZY);
    if (!hDll) {
        printf("❌ [Error] 无法加载 %s: %s\n", dll_path, dlerror());
        return -1;
    }

    g_Manager_Create = (Manager_Create_Type)dlsym(hDll, "Manager_Create");
    g_Manager_Destory = (Manager_Destory_Type)dlsym(hDll, "Manager_Destory");
    g_Manager_GetDevices = (Manager_GetDevices_Type)dlsym(hDll, "Manager_GetDevices");
    g_Manager_GetDeviceByIndex = (Manager_GetDeviceByIndex_Type)dlsym(hDll, "Manager_GetDeviceByIndex");
    g_Device_GetSubDeviceCount = (Device_GetSubDeviceCount_Type)dlsym(hDll, "Device_GetSubDeviceCount");
    g_Device_GetSubDeviceByIndex = (Device_GetSubDeviceByIndex_Type)dlsym(hDll, "Device_GetSubDeviceByIndex");
    g_Device_Destory = (Device_Destory_Type)dlsym(hDll, "Device_Destory");
    g_SubDevice_GetSubDeviceName = (SubDevice_GetSubDeviceName_Type)dlsym(hDll, "SubDevice_GetSubDeviceName");
    g_SubDevice_Connect = (SubDevice_Connect_Type)dlsym(hDll, "SubDevice_Connect");
    g_SubDevice_DisConnect = (SubDevice_DisConnect_Type)dlsym(hDll, "SubDevice_DisConnect");
    g_SubDevice_Destory = (SubDevice_Destory_Type)dlsym(hDll, "SubDevice_Destory");
    g_SubDevice_OpenStreamByIndex = (SubDevice_OpenStreamByIndex_Type)dlsym(hDll, "SubDevice_OpenStreamByIndex");
    g_SubDevice_StartStreamByIndex = (SubDevice_StartStreamByIndex_Type)dlsym(hDll, "SubDevice_StartStreamByIndex");
    g_SubDevice_StopStreamByIndex = (SubDevice_StopStreamByIndex_Type)dlsym(hDll, "SubDevice_StopStreamByIndex");
    g_SubDevice_CloseStreamByIndex = (SubDevice_CloseStreamByIndex_Type)dlsym(hDll, "SubDevice_CloseStreamByIndex");
    g_Image_GetImageBuff = (Image_GetImageBuff_Type)dlsym(hDll, "Image_GetImageBuff");
    g_Image_GetImageInfo = (Image_GetImageInfo_Type)dlsym(hDll, "Image_GetImageInfo");
    g_Image_Recycle = (Image_Recycle_Type)dlsym(hDll, "Image_Recycle");

    g_GetNodeType = (GetNodeType_Type)dlsym(hDll, "GetNodeType");
    g_GetNodeValue_Double = (GetNodeValue_Double_Type)dlsym(hDll, "GetNodeValue_Double");
    g_SetNodeValue_Int = (SetNodeValue_Int_Type)dlsym(hDll, "SetNodeValue_Int");
    g_SetNodeValue_Double = (SetNodeValue_Double_Type)dlsym(hDll, "SetNodeValue_Double");
    g_SetNodeValue_Bool = (SetNodeValue_Bool_Type)dlsym(hDll, "SetNodeValue_Bool");
    g_SetNodeValue_Chars = (SetNodeValue_Chars_Type)dlsym(hDll, "SetNodeValue_Chars");
    g_SetNodeValue_Command = (SetNodeValue_Command_Type)dlsym(hDll, "SetNodeValue_Command");
    g_SetNodeValue_Buffer = (SetNodeValue_Buffer_Type)dlsym(hDll, "SetNodeValue_Buffer");

    if (!g_Manager_Create || !g_SubDevice_StartStreamByIndex || !g_Image_GetImageBuff ||
        !g_GetNodeType) {
        printf("❌ [Error] 获取核心函数指针失败！\n");
        return -1;
    }
    if (!g_GetNodeValue_Double) {
        printf("[Warning] GetNodeValue_Double not found in SDK. Master AE follow sync will be disabled.\n");
    }
    return 0;
}

void ReleaseEzSDK() {
    if (hDll) {
        dlclose(hDll);
        hDll = NULL;
    }
}
