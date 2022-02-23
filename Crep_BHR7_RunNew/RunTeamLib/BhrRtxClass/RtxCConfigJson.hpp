// RtxCConfigJson.hpp
// RTX �����²��������ļ���ȡ-����JASON
// RTX Class: Configuration based on JSON
// Version 0.0 20201117 Lee <hexb66@bit.edu.cn>
// Version 0.1 20201226	Lee <hexb66@bit.edu.cn>
//	�� �������ļ�����ʱ��ӡ����ʾ��Ϣ��ԭ��ʹ�� std::cout ��ӡ LPWSTR �ַ�����ʾ����������ʹ�� printf ����
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
		//����������
		RtxCConfigJSON();
		RtxCConfigJSON(LPWSTR strName);
		~RtxCConfigJSON();

		// ��ȡ JSON �ļ�
		bool ReadJSONFile(LPWSTR strFileName);
		inline jsonxx::json * GetJSON() { return &this->m_jsonData; };
		// ��ȡ���Ͳ���
		int ReadIntNum(LPSTR strName);
		// ��ȡ�����Ͳ���
		double ReadFloatNum(LPSTR strName);
		// ��ȡ�ַ�������
		// ...�����
		// ��ȡ�����������
		void ReadIntArray(LPSTR strName, int nSize, int *p);
		// ��ȡ�������������
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
		// ����޷����ļ������ش���
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