// RtxCIPC.hpp
// RTX �����½��̼�ͨ�� 
// RTX Class: Inter Process Communication
// Version 0.0 20201111 Lee <hexb66@bit.edu.cn>
// Version 0.1 20201112 Lee <hexb66@bit.edu.cn>
//	�� ����޲������캯�����������ú���
//	�� ��Ա����˽�л�
//	�� �������ָ����ʺ���
#pragma once
#define RTX_IPC
#include "RtxHeader.h"
#define NO_RTX_IPC_EXIST_ERROR

namespace bhrrtx
{
	//DATA: ��������
	template <typename DATA>
	class RtxCIPC
	{
		typedef DATA * P_DATA; //��������ָ��

	private:
		HANDLE m_hMutex;	//������
		HANDLE m_hShm;		//�����ڴ�
		HANDLE m_hSem;		//�ź���
		P_DATA m_pData;		//���ݽṹ��ָ��
		WCHAR  m_strMutex[60];	//����������
		WCHAR  m_strShm[60];	//�����ڴ�����
		WCHAR  m_strSem[60];	//�ź�������
		// �Ժ��ٿ���
		// int m_nNameSetFlag;
		WCHAR m_strName[50];		//��������

	public:
		// Ĭ�Ϲ��캯��
		RtxCIPC();
		// ��ʼ��������ʱ��Ҫָ������
		RtxCIPC(LPWSTR strName);
		// ����
		~RtxCIPC();
		// �������ú���
		virtual void SetName(LPWSTR strName);
		// �����µ�
		bool Create();
		// �����е�
		bool Open();
		// ����
		bool Lock();
		// ����
		void Unlock();
		// ��ȡ����ָ��
		inline P_DATA GetData(){ return this->m_pData; };

	private:
		// ��ӡ����
		void PrintError(LPCWSTR strError);
		// ���
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
		// ����������
		m_hMutex = RtCreateMutex(NULL, FALSE, m_strMutex);
		if (m_hMutex == NULL)
		{
			PrintError(L"Create Mutex Failed!");
			return false;
		}
		// ���������ڴ�
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
		// �����ź���
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
		//���ź���
		m_hSem = RtOpenSemaphore(SYNCHRONIZE, FALSE, m_strSem);
		if (m_hSem == NULL)
		{
			PrintError(L"Could not open Semaphore.");
			return FALSE;
		}
		//�򿪻�����
		m_hMutex = RtOpenMutex(SYNCHRONIZE, FALSE, m_strMutex);
		if (m_hMutex == NULL)
		{
			PrintError(L"Could not open Mutex DATA.");
			Clear();
			return FALSE;
		}
		//�򿪹����ڴ�
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