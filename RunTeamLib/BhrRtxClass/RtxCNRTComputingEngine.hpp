// RtxCNRTComputingEngine.hpp
// RTX �����·�ʵʱ��������
// RTX Class: Non-Real Time Computing Engine
// Version 0.0 20201115 Lee <hexb66@bit.edu.cn>

#pragma once
#ifndef RTX_NRTCE
#define RTX_NRTCE

#include "RtxHeader.h"
#include "RtxCIPC.hpp"

namespace bhrrtx
{
	// ��ʵʱ�������� �̻߳ص�����
	template<typename DATA_INTERFACE, typename DATA_COMP>
	DWORD WINAPI ThreadCompEng(LPVOID p)
	{
		if (p == NULL)
		{
			std::cout << "Computing Engine NRT-Thread: Wrong configuration!" << std::endl;
			return -1;
		}
		auto pCmpEng = (RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP> *)p;
		pCmpEng->SetReady();
		pCmpEng->SetRunFlag(true);
		while (pCmpEng->GetRunFlag())
		{
			if (pCmpEng->WaitRequst())
			{
				pCmpEng->RunComputation();
				pCmpEng->SendResult();
			}
		}
		std::cout << "Computing Engine NRT-Thread: Quit!" << std::endl;
		return 0;
	}

	template<typename DATA_INTERFACE, typename DATA_COMP=void>
	class RtxNRTComputingEngine:public RtxCIPC<DATA_INTERFACE>
	{
		// ��ʵʱ���� ����ָ�� ����
		typedef void(*FPCOMP)(DATA_INTERFACE*,DATA_COMP *);

	private:
		HANDLE m_hEventReady;		// ��ʵʱ֪ͨʵʱ��������׼����ϵ��¼����
		HANDLE m_hEventPing;		// ʵʱ���������¼����
		HANDLE m_hEventPong;		// ��ʵʱ������������¼���� 
		HANDLE m_hCompEng;			// ��ʵʱ���������Ӧ�߳̾��

		WCHAR m_strEventReady[STR_NAME_LEN];	// ��ʵʱ֪ͨʵʱ��������׼����ϵ��¼�����
		WCHAR m_strEventPing[STR_NAME_LEN];	// ʵʱ���������¼�����
		WCHAR m_strEventPong[STR_NAME_LEN];	// ��ʵʱ������������¼�����

		bool	m_bRunFlag;		// ���б�־
		DWORD	m_dwWaitTimeMs;	// �¼������ȴ�ʱ��

		FPCOMP	m_fpComputaion;	// ��ʵʱ���㺯��ָ��
		DATA_COMP * m_pDataComp;	// ��ʵʱ�������ݽṹ��ָ��
		DATA_INTERFACE *m_pDataIntf;	// �ӿ����ݽṹ��ָ��


	public:
		// ��������������
		RtxNRTComputingEngine();
		RtxNRTComputingEngine(LPWSTR strName);
		~RtxNRTComputingEngine();
		// ��ȡ���б�־
		inline bool GetRunFlag() { return this->m_bRunFlag; };
		// �������б�־
		inline void SetRunFlag(bool bRunFlag) { this->m_bRunFlag = bRunFlag; };
		// ��������
		inline void nSetName(LPWSTR strName);
		// ���ô����ȴ�ʱ��
		inline void SetWaitTime(DWORD dwWaitTimeMs) { this->m_dwWaitTimeMs = dwWaitTimeMs; };
		// ��ʼ�����г�Ա����
		inline void InitAll() {
			m_hEventReady = NULL;
			m_hEventPing = NULL;
			m_hEventPong = NULL;
			m_hCompEng = NULL;
			m_bRunFlag = FALSE;
			m_dwWaitTimeMs = 1; //Ĭ�ϵȴ�ʱ�� 1ms
			m_fpComputaion = NULL;
			m_pDataComp = NULL;
			m_pDataIntf = NULL;
		};
		// ������о��
		inline void ClearAllHandles() {
			RtCloseHandle(m_hEventReady); 
			RtCloseHandle(m_hEventPing);
			RtCloseHandle(m_hEventPong);
			RtCloseHandle(m_hCompEng);
		};
		// ���ýӿ����ݽṹ��ָ��
		inline void SetInterface(DATA_INTERFACE *p) { this->m_pDataIntf = p; };
		inline DATA_INTERFACE* GetIntfPtr() { return this->m_pDataIntf; };
		inline DATA_INTERFACE* GetCompPtr() { return this->m_pDataComp; };

		/*******  ʵʱ����*********/
		// ʵʱ���ֳ�ʼ��
		bool InitRT();
		// ����ʵʱ�Ƿ�׼�����
		bool CheckNRTReady();
		// ���ռ�����
		inline bool RecvResult() { return(RecvResult(this->m_pDataIntf)); };
		bool RecvResult(DATA_INTERFACE * pData); 
		/******* ��ʵʱ����*********/
		// ���÷�ʵʱ�������ȼ�
		bool SetProcessPriority();
		bool SetProcessPriority(DWORD dwPriorityClass);
		// ���ü��㺯�������ݽṹ��
		bool SetComputaion(DATA_COMP * pDataComp, FPCOMP fpComp);
		// ��ʵʱ���ֳ�ʼ��
		bool InitNRT();
		// �������������̣߳����������ȼ�
		bool CreateCompThread();
		// �������������̣߳����������ȼ�
		bool CreateCompThread(int flag);
		// �������������߳�
		bool StartCompThread();
		// ����׼������¼�
		bool SetReady();
		// �ȴ���Ӧʵʱ��������
		bool WaitRequst();
		// ִ�м������
		void RunComputation();
		// ���������
		bool SendResult();
	};

	// ����������
	template<typename DATA_INTERFACE, typename DATA_COMP>
	RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::RtxNRTComputingEngine()
	{
		this->InitAll();
		this->nSetName(L"0");
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::RtxNRTComputingEngine(LPWSTR strName)
	{
		this->InitAll();
		this->nSetName(strName);
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::~RtxNRTComputingEngine()
	{
		this->ClearAllHandles();
	}
	// ��ʼ����Ա����
	template<typename DATA_INTERFACE, typename DATA_COMP>
	void RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::nSetName(LPWSTR strName)
	{
		((RtxCIPC<DATA_INTERFACE>*)this)->SetName(strName);		
		wcscpy(m_strEventReady,(LPCWSTR)strName);		wcscat(m_strEventReady, L"_Event.Ready");
		wcscpy(m_strEventPing, (LPCWSTR)strName);		wcscat(m_strEventPing,	L"_Event.Ping");
		wcscpy(m_strEventPong, (LPCWSTR)strName);		wcscat(m_strEventPong,	L"_Event.Pong");
	}
	// ʵʱ��Ա����
	template<typename DATA_INTERFACE, typename DATA_COMP>
	bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::InitRT()
	{
		// ����ͨ�Žӿ�
		if (!this->Create())
		{
			return false;
		}
		// �����¼�
		this->m_hEventReady = RtCreateEvent(NULL, FALSE, FALSE, this->m_strEventReady);
		this->m_hEventPing = RtCreateEvent(NULL, FALSE, FALSE, this->m_strEventPing);
		this->m_hEventPong = RtCreateEvent(NULL, FALSE, FALSE, this->m_strEventPong);
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::CheckNRTReady()
	{
		if (RtWaitForSingleObject(this->m_hEventReady, this->m_dwWaitTimeMs))
		{
			return false;
		}
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::RecvResult(DATA_INTERFACE * pData)
	{
		// �жϷ�ʵʱ�Ƿ�׼�����
		if (!this->GetRunFlag())
		{
			this->SetRunFlag(this->CheckNRTReady());
			return false;
		}
		else // ׼����ϣ���ʼ���м�������
		{
			// ��������
			if (this->Lock()) (*(this->GetData())) = *pData; this->Unlock();
			// ��������
			RtResetEvent(this->m_hEventPong); // 20201116 ���ػ����Է��֣��������Ӧ�¼��Ļ����޷���֤�ϸ�ͬ��
			RtSetEvent(this->m_hEventPing);
			// �ȴ����ؽ��
			if (RtWaitForSingleObject(this->m_hEventPong, this->m_dwWaitTimeMs))
			{
				std::cout << "Wait for NRTCE over time (>" << this->m_dwWaitTimeMs << " ms)!!" << std::endl;
				return false;
			}
			// ȡ�����
			if (this->Lock())  *pData = (*(this->GetData())); this->Unlock();
		}
		return true;
	}

	// ��ʵʱ��Ա����
	template<typename DATA_INTERFACE, typename DATA_COMP>
	bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::SetProcessPriority()
	{
		return(SetProcessPriority(REALTIME_PRIORITY_CLASS));
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::SetProcessPriority(DWORD dwPriorityClass)
	{
		HANDLE hMain = GetCurrentProcess();
		SetPriorityClass(hMain, dwPriorityClass);
		SleepEx(10, FALSE);
		SetProcessPriorityBoost(hMain, true);
		SleepEx(10, FALSE);
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::SetComputaion(DATA_COMP * pData, FPCOMP fpComp)
	{
		this->m_pDataComp = pData;
		this->m_fpComputaion = fpComp;
		return false;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::InitNRT()
	{
		// ��ʼ��ͨ��
		if (!this->Open())
		{
			return false;
		}
		// ���¼�
		this->m_hEventReady = RtOpenEvent(EVENT_ALL_ACCESS, FALSE, this->m_strEventReady);
		this->m_hEventPing = RtOpenEvent(EVENT_ALL_ACCESS, FALSE, this->m_strEventPing);
		this->m_hEventPong = RtOpenEvent(EVENT_ALL_ACCESS, FALSE, this->m_strEventPong);
		SleepEx(100, FALSE);
		// �������������߳�
		this->CreateCompThread(0);
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::CreateCompThread()
	{
		this->CreateCompThread(1);
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::CreateCompThread(int flag)
	{
		this->m_hCompEng = RtCreateThread(
			NULL,
			0,
			ThreadCompEng<DATA_INTERFACE, DATA_COMP>,
			LPVOID(this),
			CREATE_SUSPENDED,
			NULL
		);
		SleepEx(10, FALSE);
		//if (!RtSetProxyThreadPriority(htCompEng, THREAD_PRIORITY_TIME_CRITICAL/*RT_PRIORITY_MIN*/))
		if (!SetThreadPriority(this->m_hCompEng, THREAD_PRIORITY_TIME_CRITICAL))
		{
			std::cout << "Failed to set priority" << std::endl;
		}
		SetThreadPriorityBoost(this->m_hCompEng, true);
		if (flag == 1) this->StartCompThread();
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::StartCompThread()
	{
		RtResumeThread(this->m_hCompEng);
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::SetReady()
	{
		RtSetEvent(this->m_hEventReady);
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::WaitRequst()
	{
		if (RtWaitForSingleObject(this->m_hEventPing, this->m_dwWaitTimeMs))
		{
			std::cout << "Wait for RT Request over time (>" << this->m_dwWaitTimeMs << "ms)" << std::endl;
			return false;
		}
		return true;
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline void RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::RunComputation()
	{
		//ȡ���ӿ��е�����
		if (this->Lock())
		{
			*(this->m_pDataIntf) = *(this->GetData());
		}
		this->Unlock();
		//����
		this->m_fpComputaion(this->m_pDataIntf, this->m_pDataComp);
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::SendResult()
	{
		//��������
		if (this->Lock())
		{
			*(this->GetData()) = *(this->m_pDataIntf);
		}
		this->Unlock();
		RtSetEvent(this->m_hEventPong);
		return true;
	}
}

#endif