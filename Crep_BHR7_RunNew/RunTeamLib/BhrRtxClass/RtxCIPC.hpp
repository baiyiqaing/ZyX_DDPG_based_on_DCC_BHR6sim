// RtxCIPC.hpp
// RTX 环境下进程间通信 
// RTX Class: Inter Process Communication
// Version 0.0 20201111 Lee <hexb66@bit.edu.cn>
// Version 0.1 20201112 Lee <hexb66@bit.edu.cn>
//	· 添加无参数构造函数、名称设置函数
//	· 成员变量私有化
//	· 添加数据指针访问函数
#pragma once
#define RTX_IPC
#include "RtxHeader.h"
#define NO_RTX_IPC_EXIST_ERROR

namespace bhrrtx
{
	//DATA: 数据类型
	template <typename DATA>
	class RtxCIPC
	{
		typedef DATA * P_DATA; //数据类型指针

	private:
		HANDLE m_hMutex;	//互斥体
		HANDLE m_hShm;		//共享内存
		HANDLE m_hSem;		//信号量
		P_DATA m_pData;		//数据结构体指针
		WCHAR  m_strMutex[60];	//互斥体名称
		WCHAR  m_strShm[60];	//共享内存名称
		WCHAR  m_strSem[60];	//信号量名称
		// 以后再考虑
		// int m_nNameSetFlag;
		WCHAR m_strName[50];		//总体名称

	public:
		// 默认构造函数
		RtxCIPC();
		// 初始化，创建时需要指定名称
		RtxCIPC(LPWSTR strName);
		// 析构
		~RtxCIPC();
		// 名称设置函数
		virtual void SetName(LPWSTR strName);
		// 创建新的
		bool Create();
		// 打开已有的
		bool Open();
		// 锁定
		bool Lock();
		// 解锁
		void Unlock();
		// 获取数据指针
		inline P_DATA GetData(){ return this->m_pData; };

	private:
		// 打印错误
		void PrintError(LPCWSTR strError);
		// 清除
		void Clear();

	};

	template<typename DATA> inline RtxCIPC<DATA>::RtxCIPC()
	{
		m_hMutex = NULL;
		m_hShm = NULL;
		m_hSem = NULL;
		SetName(L"0");
	}

	template<typename DATA> RtxCIPC<DATA>::RtxCIPC(LPWSTR strName)
	{
		m_hMutex = NULL;
		m_hShm = NULL;
		m_hSem = NULL;
		SetName(strName);
	}

	template<typename DATA> RtxCIPC<DATA>::~RtxCIPC()
	{
		Clear();
	}

	template<typename DATA> void RtxCIPC<DATA>::SetName(LPWSTR strName)
	{
		wcscpy(m_strName, (LPCWSTR)strName);
		wcscpy(m_strMutex, (LPCWSTR)strName);	wcscat(m_strMutex, L"_IPC.Mutex");
		wcscpy(m_strShm, (LPCWSTR)strName);		wcscat(m_strShm, L"_IPC.Shm");
		wcscpy(m_strSem, (LPCWSTR)strName);		wcscat(m_strSem, L"_IPC.SemPost");
	}

	template<typename DATA> bool RtxCIPC<DATA>::Create()
	{
		// 创建互斥量
		m_hMutex = RtCreateMutex(NULL, FALSE, m_strMutex);
		if (m_hMutex == NULL)
		{
			PrintError(L"Create Mutex Failed!");
			return false;
		}
		// 创建共享内存
		m_hShm = RtCreateSharedMemory(PAGE_READWRITE, 0, sizeof(DATA), m_strShm, (LPVOID*)&m_pData);
		if (GetLastError() == ERROR_ALREADY_EXISTS)
		{
			PrintError(L"Warning: Already exist!");
		#ifndef NO_RTX_IPC_EXIST_ERROR
			return false;
		#endif
		}
		if (m_hShm == NULL)
		{
			PrintError(L"Create SharedMemory failed!");
			Clear();
			return false;
		}
		// 创建信号量
		m_hShm = RtCreateSemaphore(NULL, 0, 1, m_strSem);
		if (m_hShm == NULL)
		{
			PrintError(L"Create Semaphore failed!");
			Clear();
			return false;
		}
		return true;
	}
	template<typename DATA> bool RtxCIPC<DATA>::Open()
	{
		//打开信号量
		m_hSem = RtOpenSemaphore(SYNCHRONIZE, FALSE, m_strSem);
		if (m_hSem == NULL)
		{
			PrintError(L"Could not open Semaphore.");
			return FALSE;
		}
		//打开互斥量
		m_hMutex = RtOpenMutex(SYNCHRONIZE, FALSE, m_strMutex);
		if (m_hMutex == NULL)
		{
			PrintError(L"Could not open Mutex DATA.");
			Clear();
			return FALSE;
		}
		//打开共享内存
		m_hShm = RtOpenSharedMemory(PAGE_READWRITE, FALSE, m_strShm, (LPVOID*)&m_pData);
		if (m_hShm == NULL)
		{
			PrintError(L"Could not open Shared Memory.");
			Clear();
			return FALSE;
		}
		return true;
	}
	template<typename DATA> bool RtxCIPC<DATA>::Lock()
	{
		DWORD dw = RtWaitForSingleObject(m_hMutex, INFINITE);/*the handle of object to wait for ever*/
		if (dw == WAIT_FAILED)
		{
			PrintError(L"Mutex Lock failed!");
			return false;
		}
		else if (dw == WAIT_ABANDONED)
		{
			return false;
		}
		else if (dw == WAIT_OBJECT_0) //mutex was locked
		{
			return true;
		}
		return false;
	}
	template<typename DATA> void RtxCIPC<DATA>::Unlock()
	{
		RtReleaseMutex(m_hMutex);
	}
	//template<typename DATA> inline RtxCIPC<DATA>::P_DATA RtxCIPC<DATA>::GetData()
	//{
	//	return this->m_pData;
	//}
	template<typename DATA>	inline void RtxCIPC<DATA>::PrintError(LPCWSTR strError)
	{
	#ifndef NO_RTX_IPC_ERROR_PRINT
		printf("[IPC Error] [Name]:%ws [Content]:%ws\n", m_strName, strError);
	#endif
	}
	template<typename DATA> void RtxCIPC<DATA>::Clear()
	{
		CloseHandle(m_hMutex);
		CloseHandle(m_hShm);
		CloseHandle(m_hSem);
	}
}