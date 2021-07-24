// RtxCEvent.hpp
#pragma once
#include "RtxHeader.h"

namespace bhrrtx 
{
	template<DWORD WaitTimeMs>
	class RtxCEvent
	{
	public:
		RtxCEvent(LPWSTR strName);
		~RtxCEvent();
		bool Create();
		bool Open();
		inline void Set() { RtSetEvent(this->m_hEvent); };
		void Reset() { RtResetEvent(this->m_hEvent); };
		bool Wait();
		void SetWaitTime(DWORD dwWaitTime);
	private:
		HANDLE	m_hEvent;
		WCHAR	m_strEvent[STR_NAME_LEN];
		DWORD	m_dwWaitTimeMs;
	};

	template<DWORD WaitTimeMs>
	inline RtxCEvent<WaitTimeMs>::RtxCEvent(LPWSTR strName)
	{
		wcscpy(m_strEvent, strName);
		m_hEvent = NULL;
		SetWaitTime(WaitTimeMs);
	}
	template<DWORD WaitTimeMs>
	inline RtxCEvent<WaitTimeMs>::~RtxCEvent()
	{
		RtCloseHandle(m_hEvent);
	}
	template<DWORD WaitTimeMs>
	inline bool RtxCEvent<WaitTimeMs>::Create()
	{
		this->m_hEvent = RtCreateEvent(NULL, FALSE, FALSE, this->m_strEvent);
		if (this->m_hEvent == NULL)
		{
			return false;
		}
		return true;
	}
	template<DWORD WaitTimeMs>
	inline bool RtxCEvent<WaitTimeMs>::Open()
	{
		this->m_hEvent = RtOpenEvent(EVENT_ALL_ACCESS, FALSE, this->m_strEvent);
		if (this->m_hEvent == NULL)
		{
			return false;
		}
		return true;
	}
	template<DWORD WaitTimeMs>
	inline bool RtxCEvent<WaitTimeMs>::Wait()
	{
		if (RtWaitForSingleObject(this->m_hEvent, this->m_dwWaitTimeMs))
		{
			std::cout<<m_strEvent<< ": Wait over time (>" << this->m_dwWaitTimeMs << "ms)" << std::endl;
			return false;
		}
		return true;
	}
	template<DWORD WaitTimeMs>
	inline void RtxCEvent<WaitTimeMs>::SetWaitTime(DWORD dwWaitTime)
	{
		m_dwWaitTimeMs = dwWaitTime;
	}
}