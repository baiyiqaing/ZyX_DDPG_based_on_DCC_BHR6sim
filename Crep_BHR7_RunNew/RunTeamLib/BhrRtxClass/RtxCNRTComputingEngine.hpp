// RtxCNRTComputingEngine.hpp
// RTX 环境下非实时计算引擎
// RTX Class: Non-Real Time Computing Engine
// Version 0.0 20201115 Lee <hexb66@bit.edu.cn>

#pragma once
#ifndef RTX_NRTCE
#define RTX_NRTCE

#include "RtxHeader.h"
#include "RtxCIPC.hpp"

namespace bhrrtx
{
	// 非实时计算引擎 线程回掉函数
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
		// 非实时计算 函数指针 类型
		typedef void(*FPCOMP)(DATA_INTERFACE*,DATA_COMP *);

	private:
		HANDLE m_hEventReady;		// 非实时通知实时计算引擎准备完毕的事件句柄
		HANDLE m_hEventPing;		// 实时触发计算事件句柄
		HANDLE m_hEventPong;		// 非实时触发计算完成事件句柄 
		HANDLE m_hCompEng;			// 非实时计算引擎对应线程句柄

		WCHAR m_strEventReady[STR_NAME_LEN];	// 非实时通知实时计算引擎准备完毕的事件名称
		WCHAR m_strEventPing[STR_NAME_LEN];	// 实时触发计算事件名称
		WCHAR m_strEventPong[STR_NAME_LEN];	// 非实时触发计算完成事件名称

		bool	m_bRunFlag;		// 运行标志
		DWORD	m_dwWaitTimeMs;	// 事件触发等待时间

		FPCOMP	m_fpComputaion;	// 非实时计算函数指针
		DATA_COMP * m_pDataComp;	// 非实时计算数据结构体指针
		DATA_INTERFACE *m_pDataIntf;	// 接口数据结构体指针


	public:
		// 构造与析构函数
		RtxNRTComputingEngine();
		RtxNRTComputingEngine(LPWSTR strName);
		~RtxNRTComputingEngine();
		// 获取运行标志
		inline bool GetRunFlag() { return this->m_bRunFlag; };
		// 设置运行标志
		inline void SetRunFlag(bool bRunFlag) { this->m_bRunFlag = bRunFlag; };
		// 设置名称
		inline void nSetName(LPWSTR strName);
		// 设置触发等待时间
		inline void SetWaitTime(DWORD dwWaitTimeMs) { this->m_dwWaitTimeMs = dwWaitTimeMs; };
		// 初始化所有成员变量
		inline void InitAll() {
			m_hEventReady = NULL;
			m_hEventPing = NULL;
			m_hEventPong = NULL;
			m_hCompEng = NULL;
			m_bRunFlag = FALSE;
			m_dwWaitTimeMs = 1; //默认等待时间 1ms
			m_fpComputaion = NULL;
			m_pDataComp = NULL;
			m_pDataIntf = NULL;
		};
		// 清除所有句柄
		inline void ClearAllHandles() {
			RtCloseHandle(m_hEventReady); 
			RtCloseHandle(m_hEventPing);
			RtCloseHandle(m_hEventPong);
			RtCloseHandle(m_hCompEng);
		};
		// 设置接口数据结构体指针
		inline void SetInterface(DATA_INTERFACE *p) { this->m_pDataIntf = p; };
		inline DATA_INTERFACE* GetIntfPtr() { return this->m_pDataIntf; };
		inline DATA_INTERFACE* GetCompPtr() { return this->m_pDataComp; };

		/*******  实时部分*********/
		// 实时部分初始化
		bool InitRT();
		// 监测非实时是否准备完毕
		bool CheckNRTReady();
		// 接收计算结果
		inline bool RecvResult() { return(RecvResult(this->m_pDataIntf)); };
		bool RecvResult(DATA_INTERFACE * pData); 
		/******* 非实时部分*********/
		// 设置非实时进程优先级
		bool SetProcessPriority();
		bool SetProcessPriority(DWORD dwPriorityClass);
		// 设置计算函数及数据结构体
		bool SetComputaion(DATA_COMP * pDataComp, FPCOMP fpComp);
		// 非实时部分初始化
		bool InitNRT();
		// 创建计算引擎线程，并设置优先级
		bool CreateCompThread();
		// 创建计算引擎线程，并设置优先级
		bool CreateCompThread(int flag);
		// 启动计算引擎线程
		bool StartCompThread();
		// 触发准备完毕事件
		bool SetReady();
		// 等待响应实时计算请求
		bool WaitRequst();
		// 执行计算过程
		void RunComputation();
		// 传输计算结果
		bool SendResult();
	};

	// 构造与析构
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
	// 初始化成员函数
	template<typename DATA_INTERFACE, typename DATA_COMP>
	void RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::nSetName(LPWSTR strName)
	{
		((RtxCIPC<DATA_INTERFACE>*)this)->SetName(strName);		
		wcscpy(m_strEventReady,(LPCWSTR)strName);		wcscat(m_strEventReady, L"_Event.Ready");
		wcscpy(m_strEventPing, (LPCWSTR)strName);		wcscat(m_strEventPing,	L"_Event.Ping");
		wcscpy(m_strEventPong, (LPCWSTR)strName);		wcscat(m_strEventPong,	L"_Event.Pong");
	}
	// 实时成员函数
	template<typename DATA_INTERFACE, typename DATA_COMP>
	bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::InitRT()
	{
		// 创建通信接口
		if (!this->Create())
		{
			return false;
		}
		// 创建事件
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
		// 判断非实时是否准备完毕
		if (!this->GetRunFlag())
		{
			this->SetRunFlag(this->CheckNRTReady());
			return false;
		}
		else // 准备完毕，则开始进行计算请求
		{
			// 填入数据
			if (this->Lock()) (*(this->GetData())) = *pData; this->Unlock();
			// 触发计算
			RtResetEvent(this->m_hEventPong); // 20201116 工控机测试发现，不清除响应事件的话，无法保证严格同步
			RtSetEvent(this->m_hEventPing);
			// 等待返回结果
			if (RtWaitForSingleObject(this->m_hEventPong, this->m_dwWaitTimeMs))
			{
				std::cout << "Wait for NRTCE over time (>" << this->m_dwWaitTimeMs << " ms)!!" << std::endl;
				return false;
			}
			// 取出结果
			if (this->Lock())  *pData = (*(this->GetData())); this->Unlock();
		}
		return true;
	}

	// 非实时成员函数
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
		// 初始化通道
		if (!this->Open())
		{
			return false;
		}
		// 打开事件
		this->m_hEventReady = RtOpenEvent(EVENT_ALL_ACCESS, FALSE, this->m_strEventReady);
		this->m_hEventPing = RtOpenEvent(EVENT_ALL_ACCESS, FALSE, this->m_strEventPing);
		this->m_hEventPong = RtOpenEvent(EVENT_ALL_ACCESS, FALSE, this->m_strEventPong);
		SleepEx(100, FALSE);
		// 创建计算引擎线程
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
		//取出接口中的数据
		if (this->Lock())
		{
			*(this->m_pDataIntf) = *(this->GetData());
		}
		this->Unlock();
		//计算
		this->m_fpComputaion(this->m_pDataIntf, this->m_pDataComp);
	}
	template<typename DATA_INTERFACE, typename DATA_COMP>
	inline bool RtxNRTComputingEngine<DATA_INTERFACE, DATA_COMP>::SendResult()
	{
		//发出数据
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