// RtxCConfigJson.hpp
// RTX 环境下参数配置文件读取-基于JASON
// RTX Class: Configuration based on JSON
// Version 0.0 20201117 Lee <hexb66@bit.edu.cn>
// Version 0.1 20201226	Lee <hexb66@bit.edu.cn>
//	· 修正打开文件错误时打印的提示消息：原来使用 std::cout 打印 LPWSTR 字符串显示不正常，现使用 printf 代替
#pragma once
#define RTX_CONFIG_JSON
#include "RtxCIPC.hpp"
#include <cctype>
#include <fstream>
#include "lib\jsonxx\json.hpp"

namespace bhrrtx
{
	template <typename CONFIG> 
	class RtxCConfigJSON: public RtxCIPC<CONFIG>
	{
	private:
		WCHAR m_strFilePath[100];
		jsonxx::json m_jsonData;

	public:
		//构造与析构
		RtxCConfigJSON();
		RtxCConfigJSON(LPWSTR strName);
		~RtxCConfigJSON();

		// 读取 JSON 文件
		bool ReadJSONFile(LPWSTR strFileName);
		inline jsonxx::json * GetJSON() { return &this->m_jsonData; };
		// 读取整型参数
		int ReadIntNum(LPSTR strName);
		// 读取浮点型参数
		double ReadFloatNum(LPSTR strName);
		// 读取字符串参数
		// ...待添加
		// 读取整型数组参数
		void ReadIntArray(LPSTR strName, int nSize, int *p);
		// 读取浮点型数组参数
		void ReadFloatArray(LPSTR strName, int nSize, double *p);
	private:
		template <typename T>
		void ReadArray(LPSTR strName, int nSize, T* p);

	};

	template<typename CONFIG> RtxCConfigJSON<CONFIG>::RtxCConfigJSON()
	{
	}
	template<typename CONFIG> RtxCConfigJSON<CONFIG>::RtxCConfigJSON(LPWSTR strName)
	{
		SetName(strName);
	}
	template<typename CONFIG> RtxCConfigJSON<CONFIG>::~RtxCConfigJSON()
	{

	}

	template<typename CONFIG>
	inline bool RtxCConfigJSON<CONFIG>::ReadJSONFile(LPWSTR strFileName)
	{
		std::ifstream streamFile(strFileName);
		// 如果无法打开文件，返回错误
		if (!streamFile.is_open())
		{
			// std::cout << "Can't open JSON file: " << strFileName << std::endl;
			printf("Can't open JSON file: %ws, please check the file name and location\n", strFileName);
			return false;
		}
		streamFile >> this->m_jsonData;
		streamFile.close();
		return true;
	}

	template<typename CONFIG>
	inline int RtxCConfigJSON<CONFIG>::ReadIntNum(LPSTR strName)
	{
		return this->m_jsonData[strName].as_int();
	}

	template<typename CONFIG>
	inline double RtxCConfigJSON<CONFIG>::ReadFloatNum(LPSTR strName)
	{
		return this->m_jsonData[strName].as_float();
	}

	template<typename CONFIG>
	inline void RtxCConfigJSON<CONFIG>::ReadIntArray(LPSTR strName, int nSize, int * p)
	{
		this->ReadArray<int>(strName, nSize, p);
	}

	template<typename CONFIG>
	inline void RtxCConfigJSON<CONFIG>::ReadFloatArray(LPSTR strName, int nSize, double * p)
	{
		this->ReadArray<double>(strName, nSize, p);
	}

	template<typename CONFIG>
	template<typename T>
	inline void RtxCConfigJSON<CONFIG>::ReadArray(LPSTR strName, int nSize, T * p)
	{
		int i = 0;
		auto Array = (*GetJSON())[strName];
		for (auto iter = Array.begin(); iter != Array.end(); iter++, i++)
		{
			if (i >= nSize)
			{
				std::cout << "Read " << strName << " Array Warning: Over Desired Size" << std::endl;
				//printf("Read %s Array Warning: Over Desired Size\n", strName);
				break;
			}
			p[i] = iter.value();
		}
		if (i < nSize)
		{
			std::cout << "Read "<< strName << " Array Warnig: Less than Desired Size" << std::endl;
			//printf("Read %s Array Warning: Less than Desired Size\n", strName);
		}
		std::cout << "Read " << strName << " Over" << std::endl;
		//printf("Read %s Over\n", strName);
	}

}