from libc.stdint cimport uint8_t, uint32_t
from libcpp cimport bool

cdef extern from "cerialport/c_serial_port.h":
    ctypedef enum CerialPortByteSize:
        CerialPortByteSize_FiveBits
        CerialPortByteSize_SixBits
        CerialPortByteSize_SevenBits
        CerialPortByteSize_EightBits
    
    ctypedef enum CerialPortParity:
        CerialPortParity_None
        CerialPortParity_Odd
        CerialPortParity_Even
        CerialPortParity_Mark
        CerialPortParity_Space
    
    ctypedef enum CerialPortStopBits:
        CerialPortStopBits_One
        CerialPortStopBits_OnePointFive
        CerialPortStopBits_Two
    
    ctypedef enum CerialPortError:
        CerialPortError_None
        CerialPortError_Fail
        CerialPortError_TimedOut
        CerialPortError_InvalidParam
        CerialPortError_AccessDenied
        CerialPortError_NotExist
        CerialPortError_OpenFailed
        CerialPortError_NotOpen
        CerialPortError_Unknown

    ctypedef struct CerialPortConfig:
        uint32_t baud_rate
        CerialPortByteSize byte_size
        CerialPortParity parity
        CerialPortStopBits stop_bits
        bool xon_xoff
        bool rts_cts
        bool dsr_dtr
        uint32_t timeout
        uint32_t inter_byte_timeout
        uint32_t write_timeout
        bool exclusive
    
    ctypedef struct CerialPort:
        pass
    
    CerialPort* CerialPort_create(const char* port, CerialPortConfig* config)

    void CerialPort_destroy(CerialPort* cport)

    const char* CerialPort_port(CerialPort* cport)
    void CerialPort_setPort(CerialPort* cport, const char* port)

    uint32_t CerialPort_baud_rate(CerialPort* cport)
    void CerialPort_setBaudRate(CerialPort* cport, uint32_t baud_rate)

    CerialPortByteSize CerialPort_byte_size(CerialPort* cport)
    void CerialPort_setByteSize(CerialPort* cport, CerialPortByteSize byte_size)

    CerialPortParity CerialPort_parity(CerialPort* cport)
    void CerialPort_setParity(CerialPort* cport, CerialPortParity parity)

    CerialPortStopBits CerialPort_stop_bits(CerialPort* cport)
    void CerialPort_setStopBits(CerialPort* cport, CerialPortStopBits stop_bits)

    bool CerialPort_xon_xoff(CerialPort* cport)
    void CerialPort_setXonXoff(CerialPort* cport, bool xon_xoff)

    bool CerialPort_rts_cts(CerialPort* cport)
    void CerialPort_setRtsCts(CerialPort* cport, bool rts_cts)

    bool CerialPort_dsr_dtr(CerialPort* cport)
    void CerialPort_setDsrDtr(CerialPort* cport, bool dsr_dtr)

    uint32_t CerialPort_timeout(CerialPort* cport)
    void CerialPort_setTimeout(CerialPort* cport, uint32_t timeout)

    uint32_t CerialPort_inter_byte_timeout(CerialPort* cport)
    void CerialPort_setInterByteTimeout(CerialPort* cport, uint32_t inter_byte_timeout)

    uint32_t CerialPort_write_timeout(CerialPort* cport)
    void CerialPort_setWriteTimeout(CerialPort* cport, uint32_t write_timeout)

    bool CerialPort_exclusive(CerialPort* cport)
    void CerialPort_setExclusive(CerialPort* cport, bool exclusive)

    bool CerialPort_break_condition(CerialPort* cport)

    void CerialPort_setBreakCondition(CerialPort* cport, bool cond)

    bool CerialPort_rts(CerialPort* cport)

    void CerialPort_setRts(CerialPort* cport, bool rts)

    bool CerialPort_dtr(CerialPort* cport)

    void CerialPort_setDtr(CerialPort* cport, bool dtr)

    bool CerialPort_cts(CerialPort* cport)

    bool CerialPort_dsr(CerialPort* cport)

    bool CerialPort_ri(CerialPort* cport)

    bool CerialPort_cd(CerialPort* cport)

    int CerialPort_open(CerialPort* cport)

    void CerialPort_close(CerialPort* cport)

    bool CerialPort_isOpen(CerialPort* cport)

    int CerialPort_read(CerialPort* cport, uint8_t* data, uint32_t size)

    int CerialPort_write(CerialPort* cport, const uint8_t* data, uint32_t size)

    void CerialPort_flush(CerialPort* cport)

    uint32_t CerialPort_rx_waiting(CerialPort* cport)

    uint32_t CerialPort_tx_waiting(CerialPort* cport)

    void CerialPort_resetRxBuffer(CerialPort* cport)

    void CerialPort_resetTxBuffer(CerialPort* cport)

    void CerialPort_sendBreak(CerialPort* cport, uint32_t duration)

    void CerialPort_cancelRead(CerialPort* cport)

    void CerialPort_cancelWrite(CerialPort* cport)

    #----- platform specific -----#

    void CerialPort_setRxFlowControl(CerialPort* cport, bool enable)

    void CerialPort_setTxFlowControl(CerialPort* cport, bool enable)

    void CerialPort_setBufferSize(CerialPort* cport, uint32_t rx_size, uint32_t tx_size)
