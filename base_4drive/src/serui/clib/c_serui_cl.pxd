from .c_serial_port cimport CerialPortConfig, CerialPort
from libc.stdint cimport uint8_t, uint16_t, uint32_t
from libcpp cimport bool


cdef extern from "usrif/serui.h":

    ctypedef enum SerUiProtocol:
        SerUiProtocol_ModbusRtu
        SerUiProtocol_ModbusExtended


cdef extern from "usrif/c_serui_cl.h":

    ctypedef void (*SerUiCl_OnRecvBytes)(const uint8_t*, uint16_t, void*)
    ctypedef void (*SerUiCl_OnErrorResponse)(uint16_t, uint8_t, uint8_t, uint8_t, void*)
    ctypedef void (*SerUiCl_OnReadHoldingReg)(uint16_t, uint8_t, uint16_t, const uint16_t*, void*)
    ctypedef void (*SerUiCl_OnReadInputReg)(uint16_t, uint8_t, uint16_t, const uint16_t*, void*)
    ctypedef void (*SerUiCl_OnWriteSingleReg)(uint16_t, uint8_t, uint16_t, uint16_t, void*)
    ctypedef void (*SerUiCl_OnWriteMultiReg)(uint16_t, uint8_t, uint16_t, uint16_t, void*)
    ctypedef void (*SerUiCl_OnVersion)(uint16_t, uint8_t, uint32_t, uint8_t, uint16_t, void*)
    ctypedef void (*SerUiCl_OnInvokeMethod)(uint16_t, uint8_t, const uint8_t*, uint16_t, void*)
    ctypedef void (*SerUiCl_OnContinuousReport)(uint16_t, uint8_t, const uint8_t*, uint16_t, void*)
    ctypedef void (*SerUiCl_OnLogString)(uint16_t, uint8_t, const char*, void*)
    ctypedef void (*SerUiCl_OnSwitchResponse)(uint16_t, uint8_t, void*)

    ctypedef struct SerUiCl:
        CerialPort* serial_port
        
        bool no_response_mode

        pass

    bool SerUiCl_create(SerUiCl* cl,
        const char* port, CerialPortConfig* config,
        uint8_t* rx_buf, uint16_t rx_buf_size,
        SerUiProtocol protocol, uint16_t pdu_len_hint)
    
    void SerUiCl_destroy(SerUiCl* cl)

    void SerUiCl_pollOnce(SerUiCl* cl) noexcept nogil

    void SerUiCl_poll(SerUiCl* cl, bool* stop) noexcept nogil

    #----- request interface -----#

    bool SerUiCl_readHoldingReg(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id,
        uint16_t reg_addr_base, uint16_t nreg)

    bool SerUiCl_readInputReg(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id,
        uint16_t reg_addr_base, uint16_t nreg)

    bool SerUiCl_writeSingleReg(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id,
        uint16_t reg_addr, uint16_t reg_val)

    bool SerUiCl_writeMultiReg(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id,
        uint16_t reg_addr_base, uint16_t nreg, const uint16_t* reg_vals)

    bool SerUiCl_version(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id)

    bool SerUiCl_invokeMethod(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id,
        uint16_t method, const uint8_t* args, uint16_t nbyte)

    bool SerUiCl_setContinuousReport(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id,
        uint16_t nreg, const uint16_t* reg_addrs)

    bool SerUiCl_switchResponse(
        SerUiCl* cl,
        uint16_t trans_id, uint8_t periph_id,
        bool on)

    #----- set response callback -----#

    void SerUiCl_setCallbackUserData(SerUiCl* cl, void* user_data)

    void SerUiCl_setRecvBytesCallback(SerUiCl* cl, SerUiCl_OnRecvBytes on_recv_bytes)

    void SerUiCl_setErrorResponseCallback(SerUiCl* cl, SerUiCl_OnErrorResponse on_error_response)

    void SerUiCl_setReadHoldingRegCallback(SerUiCl* cl, SerUiCl_OnReadHoldingReg on_read_holding_reg)

    void SerUiCl_setReadInputRegCallback(SerUiCl* cl, SerUiCl_OnReadInputReg on_read_input_reg)

    void SerUiCl_setWriteSingleRegCallback(SerUiCl* cl, SerUiCl_OnWriteSingleReg on_write_single_reg)

    void SerUiCl_setWriteMultiRegCallback(SerUiCl* cl, SerUiCl_OnWriteMultiReg on_write_multi_reg)

    void SerUiCl_setVersionCallback(SerUiCl* cl, SerUiCl_OnVersion on_version)

    void SerUiCl_setInvokeMethodCallback(SerUiCl* cl, SerUiCl_OnInvokeMethod on_invoke_method)

    void SerUiCl_setContinuousReportCallback(SerUiCl* cl, SerUiCl_OnContinuousReport on_continuous_report)

    void SerUiCl_setLogStringCallback(SerUiCl* cl, SerUiCl_OnLogString on_log_string)

    void SerUiCl_setSwitchResponseCallback(SerUiCl* cl, SerUiCl_OnSwitchResponse on_switch_response)
