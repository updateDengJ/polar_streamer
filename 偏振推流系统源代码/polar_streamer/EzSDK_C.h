#ifndef EZSDK_C_H
#define EZSDK_C_H

#ifdef _WIN32  // Windows 平台
#ifdef EZSDK_EXPORTS
#define EZSDK_API __declspec(dllexport)
#else
#define EZSDK_API __declspec(dllimport)
#endif // EZSDK_EXPORTS
#else  // 非 Windows 平台（例如 Linux）
#define EZSDK_API
#endif // _WIN32


#ifdef  __cplusplus
extern "C" {
#endif //  __cplusplus


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define INTERFACE_TYPE_GIGE		(1 << 1)
#define INTERFACE_TYPE_MISC		(1 << 7)

#define MAX_STREAMS 10

/* 设备类型 */
typedef enum {
	GigE = INTERFACE_TYPE_GIGE,
	Misc = INTERFACE_TYPE_MISC,
} IntfDevType;

/* 事件类型 */
typedef enum {
	Found = 1,
	Lost = 2,
	Changed = 3,
}SUB_DEVICE_EVENT;

typedef enum
{
	NI,        //!< Not implemented
	NA,        //!< Not available
	WO,        //!< Write Only
	RO,        //!< Read Only
	RW,        //!< Read and Write
	_UndefinedAccesMode,    //!< Object is not yet initialized
	_CycleDetectAccesMode   //!< used internally for AccessMode cycle detection

}EAccessMode;

typedef enum
{
	nodeTypeINTEGER,
	nodeTypeFLOAT,
	nodeTypeBOOLEAN,
	nodeTypeCOMMAND,
	nodeTypeENUMERATION,
	nodeTypeSTRING,
	nodeTypeREGISTER,
	nodeTypeUNKNOW
}NodeType;

/* 图像类型 */
typedef enum {
	ImageTypeImage = 1,
	ImageTypeRawData = 2,
	ImageTypeFile = 3,
	ImageTypeChunk = 4,
	ImageTypeJpeg = 6,
	ImageTypeJpeg2000 = 7,
	ImageTypeH264 = 8,
	ImageTypeMultiZone = 9,
	ImageTypeMultiPart = 10,
	ImageTypeDevSpecificl = 0x8001,
}ImageTypeValue;

typedef enum {
	INVALID = 0,
	UNSPECIFIED1 = 1,	//used to create format-unknown image, usually pre-allocate before format known
	UNSPECIFIED2 = 2,
	UNSPECIFIED3 = 3,
	MONO8 = 0x1080001,
	MONO10 = 0X1100003,
	MONO10_PACKED = 0X10C0004,
	MONO12 = 0X1100005,
	MONO12_PACKED = 0X10C0006,
	MONO10P = 0x010A0046,
	MONO12P = 0x010C0047,
	MONO16 = 0X01100007,
	MONO14 = 0x01100025,
	RGB8 = 0X2180014,
	BGR8 = 0X2180015,
	RGBa8 = 0x02200016,
	BGRa8 = 0x02200017,
	BAYER_GR8 = 0x1080008,
	BAYER_RG8 = 0x1080009,
	BAYER_GB8 = 0x108000A,
	BAYER_BG8 = 0x108000B,
	BAYER_GR10 = 0x0110000C,
	BAYER_RG10 = 0x0110000D,
	BAYER_BG10 = 0x0110000E,
	BAYER_GB10 = 0x0110000F,
	BAYER_GR12 = 0x01100010,
	BAYER_RG12 = 0x01100011,
	BAYER_GB12 = 0x01100012,
	BAYER_BG12 = 0x01100013,
}PixelFormatValue;

typedef void* HMANAGER;
typedef void* HDEVICE;
typedef void* HSUBDEVICE;
typedef void* HIMAGE;

typedef struct {
	SUB_DEVICE_EVENT event_type;
	HSUBDEVICE* sub_devices;
	int device_count;
}DeviceEventPair;

typedef struct CNode {
	char* nodeName;
	unsigned int nodeNameSize;
	char* nodeValue;
	unsigned int nodeValueSize;
}CNode;

typedef struct DeviceParamSetResult {
	char devName[255];
	int result;
}DeviceParamSetResult;

typedef struct Statistics {
	uint64_t imagesCount;
	uint64_t imagesLostCount;
	uint64_t segmentsResendCount;
	uint64_t segmentsLostCount;
	uint64_t imagesErrCount;
	uint64_t nextBlockId;
}Statistics;

typedef struct ImageInfo {
	ImageTypeValue imageType;
	PixelFormatValue pixelFormat;
	unsigned int width;
	unsigned int height;
	unsigned int imageDataSize;
	unsigned long long blockId;
}ImageInfo;

typedef void (*StreamCallBack)(HIMAGE h_img, HSUBDEVICE h_dev, unsigned int streamIndex, unsigned int event, void* context);

/* 获取版本信息 */
EZSDK_API int SDKVersion(char* version_name,size_t name_size);
EZSDK_API int SDKBuildVersion(char* version_name, size_t name_size);

/* Manager接口 */
EZSDK_API HMANAGER Manager_Create(unsigned int types); 
EZSDK_API void Manager_Destory(HMANAGER h_mgr);
EZSDK_API int Manager_DeviceTypeSupportedCount(HMANAGER h_mgr);
EZSDK_API int Manager_DeviceTypeSupportedByIndex(HMANAGER h_mgr, unsigned int index,IntfDevType* intfDevType);
EZSDK_API int Manager_EnumDevices(HMANAGER h_mgr);
EZSDK_API int Manager_GetDevices(HMANAGER h_mgr,IntfDevType type);
EZSDK_API HDEVICE Manager_GetDeviceByIndex(HMANAGER h_mgr, unsigned int index, IntfDevType type);
EZSDK_API HDEVICE Manager_GetDeviceByName(HMANAGER h_mgr, const char* name);
EZSDK_API int Manager_DeviceName(HMANAGER h_mgr, unsigned int index, IntfDevType type,char* name,size_t name_size);

/* Device接口 */
EZSDK_API int Device_GetDeviceName(HDEVICE h_dev, char* name,size_t name_size);
EZSDK_API void Device_Destory(HDEVICE h_dev);
EZSDK_API int Device_GetSubDeviceCount(HDEVICE h_dev);
EZSDK_API HSUBDEVICE Device_GetSubDeviceByIndex(HDEVICE h_dev, unsigned int index);

/* SubDevice接口 */
EZSDK_API int SubDevice_GetSubDeviceName(HSUBDEVICE h_subdev, char* name,size_t name_size);
EZSDK_API void SubDevice_Destory(HSUBDEVICE h_subdev);
EZSDK_API int SubDevice_Connect(HSUBDEVICE h_subdev);
EZSDK_API int SubDevice_DisConnect(HSUBDEVICE h_subdev);
EZSDK_API int SubDevice_OpenStreamByIndex(HSUBDEVICE h_subdev, unsigned int stream_index, unsigned int preAllocBuffCnt);
EZSDK_API int SubDevice_StartStreamByIndex(HSUBDEVICE h_subdev, unsigned int stream_index, StreamCallBack cb, void* context, unsigned int preAllocBuffCnt);
EZSDK_API int SubDevice_StopStreamByIndex(HSUBDEVICE h_subdev, unsigned int stream_index);
EZSDK_API int SubDevice_CloseStreamByIndex(HSUBDEVICE h_subdev, unsigned int stream_index);
/* Snap功能与历史帧获取接口 */
EZSDK_API HIMAGE SubDevice_SnapA(HSUBDEVICE h_subdev, unsigned int stream_index, unsigned int timeOutMs);
EZSDK_API const Statistics SubDevice_GetStreamStatisticsByIndex(HSUBDEVICE h_subdev, unsigned int stream_index);


/* Configurer接口 */
EZSDK_API int GetNodes(HSUBDEVICE h_subdev, CNode* nodes,unsigned int max_count,unsigned int* out_count);
EZSDK_API void FreeNodes(CNode* nodes, unsigned int nodeCount);
EZSDK_API int GetNodeValue_Int(HSUBDEVICE h_subdev, const char* nodeName,int64_t* value);
EZSDK_API int GetNodeValue_Double(HSUBDEVICE h_subdev, const char* nodeName,double* value);
EZSDK_API int GetNodeValue_Bool(HSUBDEVICE h_subdev, const char* nodeName,bool* value);
EZSDK_API int GetNodeValue_Chars(HSUBDEVICE h_subdev, const char* nodeName,char* value,size_t value_size);
EZSDK_API int GetNodeValue_Buffer(HSUBDEVICE h_subdev, const char* nodeName, unsigned char* buff, unsigned int buff_size, unsigned int* len);
EZSDK_API int SetNodeValue_Int(HSUBDEVICE h_subdev,const char* nodeName, int64_t value);
EZSDK_API int SetNodeValue_Double(HSUBDEVICE h_subdev,const char* nodeName, double value);
EZSDK_API int SetNodeValue_Bool(HSUBDEVICE h_subdev,const char* nodeName,bool value);
EZSDK_API int SetNodeValue_Chars(HSUBDEVICE h_subdev, const char* nodeName,const char* value);
EZSDK_API int SetNodeValue_Command(HSUBDEVICE h_subdev,const char* nodeName);
EZSDK_API int SetNodeValue_Buffer(HSUBDEVICE h_subdev, const char* nodeName, const unsigned char* buff, unsigned int len);

/* XML相关接口 */
EZSDK_API int SubDevice_GetXmlName(HSUBDEVICE h_subdev, char* xml_name_buff, unsigned int buff_size, unsigned int* xml_name_size);
EZSDK_API int SubDevice_GetXmlDataSize(HSUBDEVICE h_subdev, unsigned int* xml_data_size);
EZSDK_API int SubDevice_GetXmlData(HSUBDEVICE h_subdev, unsigned char* xml_buff, unsigned int buff_size);
/* 依据节点名称获取节点属性 */
EZSDK_API int GetNodeType(HSUBDEVICE h_subdev, const char* nodeName, NodeType* node_type);
EZSDK_API int GetNodeIsStreamable(HSUBDEVICE h_subdev, const char* nodeName,bool* is_streamable);
EZSDK_API int GetNodeAccessMode(HSUBDEVICE h_subdev, const char* nodeName, EAccessMode* access_mode);
EZSDK_API int GetNodeMaxValue_Int(HSUBDEVICE h_subdev, const char* nodeName, int64_t* max_val);
EZSDK_API int GetNodeMaxValue_Double(HSUBDEVICE h_subdev, const char* nodeName, double* max_val);
EZSDK_API int GetNodeMinValue_Int(HSUBDEVICE h_subdev, const char* nodeName, int64_t* min_val);
EZSDK_API int GetNodeMinValue_Double(HSUBDEVICE h_subdev, const char* nodeName, double* min_val);
EZSDK_API int GetNodeIncValue_Int(HSUBDEVICE h_subdev, const char* nodeName, int64_t* inc_val);
EZSDK_API int GetNodeIncValue_Double(HSUBDEVICE h_subdev, const char* nodeName, double* inc_val);


/* Stream层 XML获取 读写参数  参数同步*/
EZSDK_API int Stream_GetXmlName(HSUBDEVICE h_subdev, unsigned int stream_index, char* xml_name_buff, unsigned int buff_size, unsigned int* xml_name_size);
EZSDK_API int Stream_GetXmlDataSize(HSUBDEVICE h_subdev, unsigned int stream_index, unsigned int* xml_data_size);
EZSDK_API int Stream_GetXmlData(HSUBDEVICE h_subdev, unsigned int stream_index, unsigned char* xml_buff, unsigned int buff_size);
EZSDK_API int Stream_GetNodeValue_Int(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, int64_t* value);
EZSDK_API int Stream_GetNodeValue_Double(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, double* value);
EZSDK_API int Stream_GetNodeValue_Bool(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, bool* value);
EZSDK_API int Stream_GetNodeValue_Chars(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, char* value, size_t value_size);
EZSDK_API int Stream_GetNodeValue_Buffer(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, unsigned char* buff, unsigned int buff_size, unsigned int* len);
EZSDK_API int Stream_SetNodeValue_Int(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, int64_t value);
EZSDK_API int Stream_SetNodeValue_Double(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, double value);
EZSDK_API int Stream_SetNodeValue_Bool(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, bool value);
EZSDK_API int Stream_SetNodeValue_Chars(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, const char* value);
EZSDK_API int Stream_SetNodeValue_Command(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName);
EZSDK_API int Stream_SetNodeValue_Buffer(HSUBDEVICE h_subdev, unsigned int stream_index, const char* nodeName, const unsigned char* buff, unsigned int len);

/* Image接口 */
EZSDK_API ImageInfo Image_GetImageInfo(HIMAGE h_img);
EZSDK_API const unsigned char* Image_GetImageBuff(HIMAGE h_img);
EZSDK_API int Save2File(HIMAGE h_img,const char* fileName,unsigned int type);
EZSDK_API void Image_Recycle(HIMAGE h_img);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !EZSDK_C_H
