// RtxCConfig.hpp
// RTX 环境下参数配置文件读取
// RTX Class: Configuration
// Version 0.0 20201112 Lee <hexb66@bit.edu.cn>
#pragma once
#define RTX_CONFIG
#include <Windows.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include "RtxCIPC.hpp"

namespace bhrrtx
{
	template <typename CONFIG> 
	class RtxCConfig: public RtxCIPC<CONFIG>
	{
	private:
		WCHAR m_strFilePath[100];
	
	public:
		RtxCConfig();
		RtxCConfig(LPWSTR strName);
		~RtxCConfig();
		// 初始化，设置配置文件名称（相对路径，包括扩展名）
		void	Init(LPWSTR strFileName);
		// 读取 int 类型参数
		int		ReadIntParameter(LPCWSTR strAppName, LPCWSTR strKeyName);
		// 读取 double 类型参数
		double	ReadDoubleParameter(LPCWSTR strAppName, LPCWSTR strKeyName);
		// 读取字符串类型参数
		void	ReadStringParameter(LPCWSTR strAppName, LPCWSTR strKeyName, LPWSTR strRes, int nSize);

	private:
		void GetFilePath();
	};

	template<typename CONFIG> RtxCConfig<CONFIG>::RtxCConfig()
	{
	}
	template<typename CONFIG> RtxCConfig<CONFIG>::RtxCConfig(LPWSTR strName)
	{
		SetName(strName);
	}
	template<typename CONFIG> RtxCConfig<CONFIG>::~RtxCConfig()
	{
	}
	template<typename CONFIG> void RtxCConfig<CONFIG>::Init(LPWSTR strFileName)
	{
		GetCurrentDirectory(sizeof(m_strFilePath), m_strFilePath);
		wcscat(m_strFilePath, L"\\");
		wcscat(m_strFilePath, strFileName);
	}
	template<typename CONFIG> inline int RtxCConfig<CONFIG>::ReadIntParameter(LPCWSTR strAppName, LPCWSTR strKeyName)
	{
		return GetPrivateProfileInt(strAppName, strKeyName, 0, m_strFilePath);
	}
	template<typename CONFIG> inline double RtxCConfig<CONFIG>::ReadDoubleParameter(LPCWSTR strAppName, LPCWSTR strKeyName)
	{
		WCHAR strRes[50];
		GetPrivateProfileString(strAppName, strKeyName, L"0", strRes, sizeof(strRes), m_strFilePath);
		return _wtof(strRes);
	}
	template<typename CONFIG> inline void RtxCConfig<CONFIG>::ReadStringParameter(LPCWSTR strAppName, LPCWSTR strKeyName, LPWSTR strRes, int nSize)
	{
		GetPrivateProfileString(strAppName, strKeyName, L"0", strRes, nSize, m_strFilePath);
	}
}