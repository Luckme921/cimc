/*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     SDKWapper.hpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2022 / 11 / 23
*
*****************************************************************************/
#ifndef __SDK_WAPPER_HPP__
#define __SDK_WAPPER_HPP__

#include "hpp/APIExport.hpp"

namespace cs
{
	//SDK跟踪类型
	typedef enum SDKTracksType
	{
		NOT_TRACKS,				//not tracks sdk call floww
		TRACKS,					//tracks sdk call floww
		PLAY_BACK,				//play back the call flow
	}SDKTracksType;

	/**
	* @~chinese
	* @brief     初始化SDK跟踪
	* @~english
	* @brief     Set SDK tracks type
	**/
	CS_API void initSDKTracks(SDKTracksType i_emSDKTType, const char* pi_chPath);

	/**
	* @~chinese
	* @brief     释放SDK跟踪
	* @~english
	* @brief     Set SDK tracks type
	**/
	CS_API void releaseSDKTracks();
}

#endif