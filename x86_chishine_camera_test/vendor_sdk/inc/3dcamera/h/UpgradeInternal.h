#ifndef UPGRADE_INTERNAL_H_
#define UPGRADE_INTERNAL_H_
/* Include the libailook C header files */
#include "Upgrade.h"
#include "../hpp/Camera.hpp"

#ifdef __cplusplus
extern "C" {
#endif

//升级阶段定义
typedef enum
{
	//初始化阶段
	UPP_INITING = 0,

	//查询ADB设备
	UPP_QUERY_ADB,

	//查询相机设备
	UPP_QUERY_CAMERA,

	//连接相机设备
	UPP_CONNCT_CAMERA,

	//执行进入Recovery
	UPP_ENTERING_RECOVERY,

	//进入Recovery后重启
	UPP_RECOVERY_REBOOT,

	//进入Recovery后连接ADB
	UPP_RECOVERY_CONNCT_ADB,

	//ADB推送固件
	UPP_ADB_PUSH_FIRM,

	//ADB同步固件
	UPP_ADB_SYNC_FIRM,

	//ADB推送固件后确认在线
	UPP_ADB_PUSHED_COMF_ONLINE,

	//执行ADB升级
	UPP_ADB_UPGRADING,

	//退出Revocery
	UPP_ADB_QUIT_RECOVERY,

	//升级完成重启
	UPP_ADB_UPGRADED_REBOOT,

	//ADB升级全部完成
	UPP_ADB_UPGRADE_COMPLETE,

	//普通升级重连相机
	UPP_NORM_RECONNCT_CAMERA,

	//普通AB升级删除原固件
	UPP_NORM_AB_DEL_SRC_FIRM,

	//普通AB升级上载固件
	UPP_NORM_AB_UPL_FIRM,

	//普通AB执行升级
	UPP_NORM_AB_UPGRADING,

	//普通AB升级后重启
	UPP_NORM_AB_UPGRADED_REBOOT,

	//普通升级删除原固件
	UPP_NORM_DEL_SRC_FIRM,

	//普通升级上载升级工具
	UPP_NORM_UP_TOOL_EXE,

	//普通升级上载原固件
	UPP_NORM_UPL_SRC_FIRM,

	//普通执行升级
	UPP_NORM_UPGRADING,

	//普通升级后重启
	UPP_NORM_UPGRADED_REBOOT,

	//普通升级全部完成
	UPP_NORM_UPGRADED_COMPLETE,

	//Uknow
	UPP_UNKNOW = 0xFF,

}UPG_PROCED;

//升级结果定义
typedef enum
{
	//成功
	UPG_SUCCESS = 0,

	//已经在Recovery
	UPG_ALREADY_RECOVERY,

	//初始化失败
	UPG_INIT_ERR,

	//未找到相机
	UPG_NOT_FIND_CAMERA,

	//连接相机失败
	UPG_CONNECT_CAMERA_ERR,

	//ADB文件错误或不存在
	UPG_ADB_NOT_EXIT,

	//PIPE通道初始化失败
	UPG_PIPE_INIT_ERR,

	//未找到ADB设备
	UPG_NOT_FIND_ADB,

	//启动ADB Shell失败
	UPG_START_ADB_SHELL_ERR,

	//执行退出recovery失败
	UPG_QUIT_RECOVERY_ERR,

	//固件不存在
	UPG_FIRMWAR_NOT_EXIT,

	//升级工具不存在
	UPG_UPG_EXE_NOT_EXIT,

	//不在recovery模式
	UPG_NOT_IN_RECOVERY,

	//执行ADB方式上载固件命令失败
	UPG_ADB_UP_FIRM_CMD_ERROR,

	//ADB方式上载固件失败
	UPG_ADB_UP_FIRM_ERROR,

	//ADB方式上载固件后设备不在线
	UPG_ADB_UP_FIRM_DEV_OFFLINE,

	//ADB方式执行MD5校验命令失败
	UPG_ADB_EXECUT_MD5_CMD_ERR,

	//ADB方式MD5校验错误
	UPG_ADB_MD5_ERR,

	//ADB方式执行升级命令失败
	UPG_ADB_EXECUT_UPGRADE_CMD_ERR,

	//ADB方式升级失败
	UPG_ADB_UPGRADE_ERR,

	//重连相机失败
	UPG_RECONNECT_ERR,

	//普通升级执行删除升级日志失败
	UPG_NORM_DEL_UPG_LOG_ERR,

	//普通升级执行删除升级工具失败
	UPG_NORM_DEL_UPG_TOOL_ERR,

	//普通升级执行删除原固件失败
	UPG_NORM_DEL_SRC_FIRM_ERR,

	//普通升级上载升级工具失败
	UPG_NORM_UPL_UPG_TOOL_ERR,

	//普通升级校验升级工具MD5失败
	UPG_NORM_CHK_UPG_TOOL_MD5_ERR,

	//普通升级上载固件失败
	UPG_NORM_UPL_FIRM_ERR,

	//普通通升级校验固md5件失败
	UPG_NORM_CHK_FIRM_MD5_ERR,

	//普通通升级设置工具权限失败
	UPG_NORM_SET_TOOL_POWER_ERR,

	//普通升级执行升级命令失败
	UPG_NORM_EXECT_UPG_CMD_ERR,

	//普通升级执行失败
	UPG_NORM_EXECT_UPG_ERR,

	//普通AB升级删除原固件失败
	UPG_NORM_AB_DEL_SRC_FIRM_ERR,

	//普通AB升级上载固件失败
	UPG_NORM_AB_UPL_FIRM_ERR,

	//普通AB升级校验固件MD5失败
	UPG_NORM_AB_CHK_FIRM_MD5_ERR,

	//普通升级执行升级命令失败
	UPG_NORM_AB_EXECT_UPG_CMD_ERR,

	//普通AB升级执行失败
	UPG_NORM_AB_EXECT_UPG_ERR,

	//Uknow
	UPG_UNKNOW = 0xFF,
}UPGRADE_RET;

/**
 * @~chinese
 * @brief		获取当前升级阶段
 * @hUpgrade	升级句柄，通过upgradeCamera创建
 * @return		UPG_PROCED
 * @~english
 * @brief		get the procedure of upgrade firmware
 * @hUpgrade	the upgrade handle,create by upgradeCamera.
 * @return		UPG_PROCED
 **/
 CS_API UPG_PROCED getUpgradeCameraProcedure(HANDLE hUpgrade);

 /**
 * @~chinese
 * @brief		获取当前升级结果失败原因
 * @hUpgrade	升级句柄，通过upgradeCamera创建
 * @return		失败情况下返回失败原因
 * @~english
 * @brief		get the failed reason of upgrade firmware
 * @hUpgrade	the upgrade handle,create by upgradeCamera.
 * @return		failed reason of case for upgrade failed
 **/
 CS_API UPGRADE_RET getUpgradeFailedReason(HANDLE hUpgrade);

/**
*		进入升级模式(recovery mode)
*@pcSerial		需要升级的序列号(当前接口内部没有使用)
*/
CS_API UPGRADE_RET enterRecoveryModeInternal(const char* pcSerial,const char* upExeDir="./");

/**
*		leave recovery mode(enter normal mode)
*@cInfo	camera info.
*/
CS_API UPGRADE_RET leaveRecoveryModeInternal(CameraInfo cInfo,const char* upExeDir="./");

#ifdef __cplusplus
}
#endif


#endif
