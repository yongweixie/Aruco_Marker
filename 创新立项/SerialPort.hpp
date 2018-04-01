#ifndef SERIALPORT_HPP_    
#define SERIALPORT_HPP_    

#include <Windows.h>
#include <process.h>
#include <iostream>

/** 当串口无数据时,sleep至下次查询间隔的时间,单位:ms */
#define SLEEP_TIME_INTERVAL 5

class SerialPort
{
public:
	SerialPort(UINT  portNo, UINT  baud = CBR_115200, char  parity = 'N', UINT  databits = 8, UINT  stopsbits = 1) :
		m_hListenThread(INVALID_HANDLE_VALUE),
		hComm(INVALID_HANDLE_VALUE) {
		InitializeCriticalSection(&m_csCommunicationSync);
		InitPort(portNo, baud, parity, databits, stopsbits);
	}
	SerialPort() :
		m_hListenThread(INVALID_HANDLE_VALUE),
		hComm(INVALID_HANDLE_VALUE) {
		InitializeCriticalSection(&m_csCommunicationSync);
	}
	~SerialPort(){
		StopListen();
		close_port();
		DeleteCriticalSection(&m_csCommunicationSync);
	}

public:
	bool isOpened() {
		return hComm != INVALID_HANDLE_VALUE;
	}
	typedef void(*ReceiveCallback)(unsigned char byte, void* userdata);
	void setReceiveCallback(ReceiveCallback charHandler, void* userdata = NULL) {
		charHandlerFunc = charHandler;
		ruserdata = userdata;
	}
	bool StartListen(){
		if (m_hListenThread != INVALID_HANDLE_VALUE)	return false;
		s_bExit = false;
		UINT threadId;
		m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
		if (!m_hListenThread)	return false;
		if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))	return false;
		return true;
	}
	bool StopListen(){
		if (m_hListenThread != INVALID_HANDLE_VALUE){
			s_bExit = true;
			Sleep(10);
			CloseHandle(m_hListenThread);
			m_hListenThread = INVALID_HANDLE_VALUE;
		}
		return true;
	}
	bool WriteData(unsigned char* pData, unsigned int length){
		BOOL   bResult = TRUE;
		DWORD  BytesToSend = 0;
		if (hComm == INVALID_HANDLE_VALUE)	return false;
		EnterCriticalSection(&m_csCommunicationSync);
		bResult = WriteFile(hComm, pData, length, &BytesToSend, NULL);
		if (!bResult){
			DWORD dwError = GetLastError();
			PurgeComm(hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		}
		LeaveCriticalSection(&m_csCommunicationSync);
		return bResult;
	}

private:
	bool InitPort(UINT  portNo, UINT  baud = CBR_9600, char  parity = 'N', UINT  databits = 8, UINT  stopsbits = 1/*, DWORD dwCommEvents = EV_RXCHAR*/) {
		if (!open_port(portNo))	return false;

		EnterCriticalSection(&m_csCommunicationSync);
		COMMTIMEOUTS  CommTimeouts;
		CommTimeouts.ReadIntervalTimeout = 0;
		CommTimeouts.ReadTotalTimeoutMultiplier = 0;
		CommTimeouts.ReadTotalTimeoutConstant = 0;
		CommTimeouts.WriteTotalTimeoutMultiplier = 0;
		CommTimeouts.WriteTotalTimeoutConstant = 0;
		if (!SetCommTimeouts(hComm, &CommTimeouts))	return false;

		DCB  dcb;
		char szDCBparam[50];
		sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);
		DWORD len = MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t *pwText = new wchar_t[len];
		MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, pwText, len);
		GetCommState(hComm, &dcb) && BuildCommDCB(pwText, &dcb);
		delete[] pwText;
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
		if (!SetCommState(hComm, &dcb))	return false;
		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return true;
	}
	bool InitPort(UINT portNo, const LPDCB& plDCB) {
		if (!open_port(portNo))	return false;

		EnterCriticalSection(&m_csCommunicationSync);
		if (!SetCommState(hComm, plDCB))	return false;
		PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return true;
	}

	bool open_port(UINT portNo){
		EnterCriticalSection(&m_csCommunicationSync);
		char PortName[50];
		sprintf_s(PortName, "COM%d", portNo);
		hComm = CreateFileA(PortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, 0);
		LeaveCriticalSection(&m_csCommunicationSync);
		return hComm != INVALID_HANDLE_VALUE;
	}
	void close_port(){
		if (hComm != INVALID_HANDLE_VALUE){
			CloseHandle(hComm);
			hComm = INVALID_HANDLE_VALUE;
		}
	}

	bool ReadChar(char &cRecved) {
		BOOL  bResult = TRUE;
		DWORD BytesRead = 0;
		if (hComm == INVALID_HANDLE_VALUE)	return false;
		EnterCriticalSection(&m_csCommunicationSync);
		bResult = ReadFile(hComm, &cRecved, 1, &BytesRead, NULL);
		if ((!bResult)) {
			DWORD dwError = GetLastError();
			PurgeComm(hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		}
		LeaveCriticalSection(&m_csCommunicationSync);
		return bResult;
	}
	UINT QueueSize() {
		DWORD dwError = 0;
		COMSTAT  comstat;
		memset(&comstat, 0, sizeof(COMSTAT));
		ClearCommError(hComm, &dwError, &comstat);
		return comstat.cbInQue;
	}

	static UINT WINAPI ListenThread(void* pParam){
		/** 得到本类的指针 */
		SerialPort *pSerialPort = reinterpret_cast<SerialPort*>(pParam);

		// 线程循环,轮询方式读取串口数据    
		while (!pSerialPort->s_bExit)
		{
			UINT BytesInQue = pSerialPort->QueueSize();
			if (BytesInQue == 0){
				Sleep(SLEEP_TIME_INTERVAL);
				continue;
			}
			char cRecved;
			do
			{
				if (pSerialPort->ReadChar(cRecved))
				{
					if (charHandlerFunc)charHandlerFunc(cRecved, ruserdata);
				}
			} while (--BytesInQue);
		}
		return 0;
	}

private:
	HANDLE  hComm;
	static bool s_bExit;
	volatile HANDLE    m_hListenThread;
	CRITICAL_SECTION   m_csCommunicationSync;
	static ReceiveCallback charHandlerFunc;
	static void* ruserdata;
};
bool SerialPort::s_bExit = false;
SerialPort::ReceiveCallback SerialPort::charHandlerFunc = NULL;
void* SerialPort::ruserdata = NULL;

#endif //SERIALPORT_HPP_   