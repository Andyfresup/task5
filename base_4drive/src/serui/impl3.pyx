from serui.clib cimport c_serui_cl as _cl
from serui.clib cimport c_serial_port as _sp
from libc.stdlib cimport malloc, free
from libc.string cimport strcpy, strlen
from libc.stdint cimport uint8_t, uint16_t, uint32_t
from libcpp cimport bool
from cpython cimport buffer, array
import array
import typing as T
import enum


@enum.unique
class ByteSize(enum.IntEnum):
    FiveBits    = _sp.CerialPortByteSize_FiveBits
    SixBits     = _sp.CerialPortByteSize_SixBits
    SevenBits   = _sp.CerialPortByteSize_SevenBits
    EightBits   = _sp.CerialPortByteSize_EightBits


@enum.unique
class Parity(enum.IntEnum):
    No      = _sp.CerialPortParity_None
    Odd     = _sp.CerialPortParity_Odd
    Even    = _sp.CerialPortParity_Even
    Mark    = _sp.CerialPortParity_Mark
    Space   = _sp.CerialPortParity_Space


@enum.unique
class StopBits(enum.IntEnum):
    One             = _sp.CerialPortStopBits_One
    OnePointFive    = _sp.CerialPortStopBits_OnePointFive
    Two             = _sp.CerialPortStopBits_Two


@enum.unique
class Error(enum.IntEnum):
    Ok              = _sp.CerialPortError_None
    Fail            = _sp.CerialPortError_Fail
    TimedOut        = _sp.CerialPortError_TimedOut
    InvalidParam    = _sp.CerialPortError_InvalidParam
    AccessDenied    = _sp.CerialPortError_AccessDenied
    NotExist        = _sp.CerialPortError_NotExist
    OpenFailed      = _sp.CerialPortError_OpenFailed
    NotOpen         = _sp.CerialPortError_NotOpen
    Unknown         = _sp.CerialPortError_Unknown


@enum.unique
class SerUiProtocol(enum.IntEnum):
    ModbusRtu       = _cl.SerUiProtocol.SerUiProtocol_ModbusRtu
    ModbusExtended  = _cl.SerUiProtocol.SerUiProtocol_ModbusExtended


cdef extern from "Python.h":
    cdef memoryview PyMemoryView_FromMemory(char* mem, Py_ssize_t size, int flags)


cdef class SerUiCl:

    cdef _cl.SerUiCl* _cimpl
    cdef bool _stop
    cdef public object __on_recv_bytes
    cdef public object __on_error_response
    cdef public object __on_read_holding_reg
    cdef public object __on_read_input_reg
    cdef public object __on_write_single_reg
    cdef public object __on_write_multi_reg
    cdef public object __on_version
    cdef public object __on_invoke_method
    cdef public object __on_continuous_report
    cdef public object __on_log_string
    cdef public object __on_switch_response

    def __cinit__(
        self,
        port: T.Optional[str] = None,
        *,
        baud_rate: int = 9600,
        byte_size: ByteSize = ByteSize.EightBits,
        parity: Parity = Parity.No,
        stop_bits: StopBits = StopBits.One,
        xon_xoff: bool = False,
        rts_cts: bool = False,
        dsr_dtr: bool = False,
        timeout: T.Optional[int] = None,
        inter_byte_timeout: T.Optional[int] = None,
        write_timeout: T.Optional[int] = None,
        exclusive: bool = False,
        rx_buf_size: int = 64,
        protocol: SerUiProtocol = SerUiProtocol.ModbusExtended,
        pdu_len_hint: int = 128,
        no_response_mode: bool = False,
        **kwargs,
    ):
        self._cimpl = <_cl.SerUiCl*>malloc(sizeof(_cl.SerUiCl))
        if self._cimpl is NULL:
            raise MemoryError()

        if timeout is None:
            timeout = 0xFFFFFFFF
        elif timeout < 0 or timeout > 0xFFFFFFFF:
            raise ValueError()
        
        if inter_byte_timeout is None:
            inter_byte_timeout = 0
        elif inter_byte_timeout < 0 or inter_byte_timeout > 0xFFFFFFFF:
            raise ValueError()
        
        if write_timeout is None:
            write_timeout = 0xFFFFFFFF
        elif write_timeout < 0 or write_timeout > 0xFFFFFFFF:
            raise ValueError()
    
        cdef _sp.CerialPortConfig cfg
        cfg.baud_rate = baud_rate
        cfg.byte_size = byte_size
        cfg.parity = parity
        cfg.stop_bits = stop_bits
        cfg.xon_xoff = xon_xoff
        cfg.rts_cts = rts_cts
        cfg.dsr_dtr = dsr_dtr
        cfg.timeout = timeout
        cfg.inter_byte_timeout = inter_byte_timeout
        cfg.write_timeout = write_timeout
        cfg.exclusive = exclusive

        port_py_bytes = (port or '').encode('utf-8')
        cdef char* port_c_str = port_py_bytes

        if not _cl.SerUiCl_create(self._cimpl,
            port=port_c_str,
            config=&cfg,
            rx_buf=NULL,
            rx_buf_size=rx_buf_size,
            protocol=protocol,
            pdu_len_hint=pdu_len_hint):

            free(self._cimpl)
            self._cimpl = NULL
            raise IOError()
        
        self._cimpl.no_response_mode = no_response_mode
        self._stop = False
        _cl.SerUiCl_setCallbackUserData(self._cimpl, <void*>self)
        self.__on_recv_bytes = None
        self.__on_error_response = None
        self.__on_read_holding_reg = None
        self.__on_read_input_reg = None
        self.__on_write_single_reg = None
        self.__on_write_multi_reg = None
        self.__on_version = None
        self.__on_invoke_method = None
        self.__on_continuous_report = None
        self.__on_log_string = None
        self.__on_switch_response = None

    def __dealloc__(self):
        if self._cimpl is not NULL:
            _cl.SerUiCl_destroy(self._cimpl)
            self._cimpl = NULL

    @staticmethod
    def _cvt2pystr(const char* cstr) -> str:
        cdef Py_ssize_t cstr_len = strlen(cstr)
        cdef char* cstr_cpy = <char*>malloc((cstr_len + 1) * sizeof(char))
        if not cstr_cpy:
            raise MemoryError()
        strcpy(cstr_cpy, cstr)

        try:
            pystr = cstr_cpy[:cstr_len]
        finally:
            free(cstr_cpy)
        return pystr.decode('utf-8')

    @property
    def stop(self) -> bool:
        return self._stop
    
    @stop.setter
    def stop(self, val: bool):
        self._stop = val

    @property
    def no_response_mode(self):
        return self._cimpl.no_response_mode

    def pollOnce(self):
        with nogil:
            _cl.SerUiCl_pollOnce(self._cimpl)

    def poll(self):
        with nogil:
            _cl.SerUiCl_poll(self._cimpl, &self._stop)

    #----- serial port interface -----#

    @property
    def port(self) -> str:
        return self._cvt2pystr(_sp.CerialPort_port(self._cimpl.serial_port))
    
    @port.setter
    def port(self, val: str):
        port_py_bytes = val.encode('utf-8')
        cdef char* port_c_str = port_py_bytes
        _sp.CerialPort_setPort(self._cimpl.serial_port, port_c_str)
    
    @property
    def baud_rate(self) -> int:
        return _sp.CerialPort_baud_rate(self._cimpl.serial_port)
    
    @baud_rate.setter
    def baud_rate(self, val: int):
        if 0 <= val <= 0xFFFFFFFF:
            _sp.CerialPort_setBaudRate(self._cimpl.serial_port, val)

    @property
    def byte_size(self) -> ByteSize:
        return ByteSize(_sp.CerialPort_byte_size(self._cimpl.serial_port))
    
    @byte_size.setter
    def byte_size(self, val: ByteSize):
        _sp.CerialPort_setByteSize(self._cimpl.serial_port, val)
    
    @property
    def parity(self) -> Parity:
        return Parity(_sp.CerialPort_parity(self._cimpl.serial_port))
    
    @parity.setter
    def parity(self, val: Parity):
        _sp.CerialPort_setParity(self._cimpl.serial_port, val)
    
    @property
    def stop_bits(self) -> StopBits:
        return StopBits(_sp.CerialPort_stop_bits(self._cimpl.serial_port))
    
    @stop_bits.setter
    def stop_bits(self, val: StopBits):
        _sp.CerialPort_setStopBits(self._cimpl.serial_port, val)

    @property
    def xon_xoff(self) -> bool:
        return _sp.CerialPort_xon_xoff(self._cimpl.serial_port)
    
    @xon_xoff.setter
    def xon_xoff(self, val: bool):
        _sp.CerialPort_setXonXoff(self._cimpl.serial_port, val)

    @property
    def rts_cts(self) -> bool:
        return _sp.CerialPort_rts_cts(self._cimpl.serial_port)
    
    @rts_cts.setter
    def rts_cts(self, val: bool):
        _sp.CerialPort_setRtsCts(self._cimpl.serial_port, val)

    @property
    def dsr_dtr(self) -> bool:
        return _sp.CerialPort_dsr_dtr(self._cimpl.serial_port)
    
    @dsr_dtr.setter
    def dsr_dtr(self, val: bool):
        _sp.CerialPort_setDsrDtr(self._cimpl.serial_port, val)

    @property
    def timeout(self) -> T.Optional[int]:
        cdef int to = _sp.CerialPort_timeout(self._cimpl.serial_port)
        if to == 0xFFFFFFFF:
            return None
        return to
    
    @timeout.setter
    def timeout(self, val: T.Optional[int]):
        if val is None:
            val = 0xFFFFFFFF
        if 0 <= val <= 0xFFFFFFFF:
            _sp.CerialPort_setTimeout(self._cimpl.serial_port, val)

    @property
    def inter_byte_timeout(self) -> T.Optional[int]:
        return _sp.CerialPort_inter_byte_timeout(self._cimpl.serial_port)
    
    @inter_byte_timeout.setter
    def inter_byte_timeout(self, val: T.Optional[int]):
        if val is None:
            val = 0
        if 0 <= val <= 0xFFFFFFFF:
            _sp.CerialPort_setInterByteTimeout(self._cimpl.serial_port, val)

    @property
    def write_timeout(self) -> T.Optional[int]:
        cdef int to = _sp.CerialPort_write_timeout(self._cimpl.serial_port)
        if to == 0xFFFFFFFF:
            return None
        return to
    
    @write_timeout.setter
    def write_timeout(self, val: T.Optional[int]):
        if val is None:
            val = 0xFFFFFFFF
        if 0 <= val <= 0xFFFFFFFF:
            _sp.CerialPort_setWriteTimeout(self._cimpl.serial_port, val)

    @property
    def exclusive(self) -> bool:
        return StopBits(_sp.CerialPort_exclusive(self._cimpl.serial_port))
    
    @exclusive.setter
    def exclusive(self, val: bool):
        _sp.CerialPort_setExclusive(self._cimpl.serial_port, val)

    @property
    def break_condition(self) -> bool:
        return _sp.CerialPort_break_condition(self._cimpl.serial_port)
    
    @break_condition.setter
    def break_condition(self, val: bool):
        _sp.CerialPort_setBreakCondition(self._cimpl.serial_port, val)

    @property
    def rts(self) -> bool:
        return _sp.CerialPort_rts(self._cimpl.serial_port)
    
    @rts.setter
    def rts(self, val: bool):
        _sp.CerialPort_setRts(self._cimpl.serial_port, val)

    @property
    def dtr(self) -> bool:
        return _sp.CerialPort_dtr(self._cimpl.serial_port)
    
    @dtr.setter
    def dtr(self, val: bool):
        _sp.CerialPort_setDtr(self._cimpl.serial_port, val)

    @property
    def cts(self) -> bool:
        return _sp.CerialPort_cts(self._cimpl.serial_port)

    @property
    def dsr(self) -> bool:
        return _sp.CerialPort_dsr(self._cimpl.serial_port)

    @property
    def ri(self) -> bool:
        return _sp.CerialPort_ri(self._cimpl.serial_port)

    @property
    def cd(self) -> bool:
        return _sp.CerialPort_cd(self._cimpl.serial_port)
    
    def open(self) -> int:
        return _sp.CerialPort_open(self._cimpl.serial_port)
    
    def close(self):
        _sp.CerialPort_close(self._cimpl.serial_port)
    
    def isOpen(self) -> bool:
        return _sp.CerialPort_isOpen(self._cimpl.serial_port)
    
    def read(self, size: int) -> bytes:
        buf = bytearray(size)
        nread = self.read_to(buf, size)
        if nread > 0:
            return bytes(buf[:nread])
        return None
    
    def read_to(self, buf: bytearray, size: int) -> int:
        cdef char* cbuf = buf
        return _sp.CerialPort_read(self._cimpl.serial_port, <uint8_t*>cbuf, size)

    def write(self, data: bytes) -> int:
        cdef char* cdata = data
        return _sp.CerialPort_write(self._cimpl.serial_port, <uint8_t*>cdata, len(data))

    def flush(self):
        _sp.CerialPort_flush(self._cimpl.serial_port)

    @property
    def rx_waiting(self) -> int:
        return _sp.CerialPort_rx_waiting(self._cimpl.serial_port)

    @property
    def tx_waiting(self) -> int:
        return _sp.CerialPort_tx_waiting(self._cimpl.serial_port)

    def reset_rx_buffer(self):
        _sp.CerialPort_resetRxBuffer(self._cimpl.serial_port)

    def reset_tx_buffer(self):
        _sp.CerialPort_resetTxBuffer(self._cimpl.serial_port)

    def send_break(self, duration: int):
        if duration < 0:
            duration = 0
        elif duration > 0xFFFFFFFF:
            duration = 0xFFFFFFFF
        _sp.CerialPort_sendBreak(self._cimpl.serial_port, duration)

    def cancel_read(self):
        _sp.CerialPort_cancelRead(self._cimpl.serial_port)

    def cancel_write(self):
        _sp.CerialPort_cancelWrite(self._cimpl.serial_port)

    #----- serial port interface: platform specific -----#

    def set_rx_flow_control(self, enable: bool):
        _sp.CerialPort_setRxFlowControl(self._cimpl.serial_port, enable)

    def set_tx_flow_control(self, enable: bool):
        _sp.CerialPort_setTxFlowControl(self._cimpl.serial_port, enable)

    def set_buffer_size(self, rx_size: int, tx_size: int):
        if rx_size < 0:
            rx_size = 0
        elif rx_size > 0xFFFFFFFF:
            rx_size = 0xFFFFFFFF
        
        if tx_size < 0:
            tx_size = 0
        elif tx_size > 0xFFFFFFFF:
            tx_size = 0xFFFFFFFF

        _sp.CerialPort_setBufferSize(self._cimpl.serial_port, rx_size, tx_size)

    #----- request interface -----#

    def read_holding_reg(self,
                         trans_id: int, periph_id: int,
                         reg_addr_base: int, nreg: int) -> bool:
        return _cl.SerUiCl_readHoldingReg(self._cimpl, trans_id, periph_id, reg_addr_base, nreg)

    def read_input_reg(self,
                       trans_id: int, periph_id: int,
                       reg_addr_base: int, nreg: int) -> bool:
        return _cl.SerUiCl_readInputReg(self._cimpl, trans_id, periph_id, reg_addr_base, nreg)

    def write_single_reg(self,
                         trans_id: int, periph_id: int,
                         reg_addr: int, reg_val: int) -> bool:
        return _cl.SerUiCl_writeSingleReg(self._cimpl, trans_id, periph_id, reg_addr, reg_val)

    def write_multi_reg(self,
                        trans_id: int, periph_id: int,
                        reg_addr_base: int, reg_vals: T.Sequencep[int]) -> bool:
        cdef array.array ar = array.array('H', reg_vals)
        return _cl.SerUiCl_writeMultiReg(self._cimpl, trans_id, periph_id, reg_addr_base, <uint16_t>len(reg_vals), ar.data.as_ushorts)

    def version(self, 
                trans_id: int, periph_id: int) -> bool:
        return _cl.SerUiCl_version(self._cimpl, trans_id, periph_id)
    
    def invoke_method(self,
                      trans_id: int, periph_id: int,
                      method: int, args: bytes) -> bool:
        cdef char* cdata = args
        return _cl.SerUiCl_invokeMethod(self._cimpl, trans_id, periph_id, method, <const uint8_t*>cdata, <uint16_t>len(args))

    def set_continuous_report(self,
                          trans_id: int, periph_id: int,
                          reg_vals: T.Sequence[int]) -> bool:
        cdef array.array ar = array.array('H', reg_vals)
        return _cl.SerUiCl_setContinuousReport(self._cimpl, trans_id, periph_id, <uint16_t>len(reg_vals), ar.data.as_ushorts)

    def switch_response(self,
                        trans_id: int, periph_id: int,
                        bool on) -> bool:
        return _cl.SerUiCl_switchResponse(self._cimpl, trans_id, periph_id, on)

    #----- response callback -----#

    def register_callback_on_recv_bytes(self, cb: T.Callable[[bytes], None]):
        '''callback: (data: bytes) -> None'''
        assert cb
        if self.__on_recv_bytes:
            if cb != self.__on_recv_bytes:
                self.__on_recv_bytes = cb
        else:
            self.__on_recv_bytes = cb
            _cl.SerUiCl_setRecvBytesCallback(self._cimpl, self._cb_on_recv_bytes)

    cdef unregister_callback_on_recv_bytes(self):
        if self.__on_recv_bytes:
            _cl.SerUiCl_setRecvBytesCallback(self._cimpl, NULL)
            self.__on_recv_bytes = None

    def register_callback_on_error_response(self, cb: T.Callable[[int, int, int, int], None]):
        '''callback: (trans_id: int, periph_id: int, fcode: int, err_code: int) -> None'''
        assert cb
        if self.__on_error_response:
            if cb != self.__on_error_response:
                self.__on_error_response = cb
        else:
            self.__on_error_response = cb
            _cl.SerUiCl_setErrorResponseCallback(self._cimpl, self._cb_on_error_response)

    cdef unregister_callback_on_error_response(self):
        if self.__on_error_response:
            _cl.SerUiCl_setErrorResponseCallback(self._cimpl, NULL)
            self.__on_error_response = None

    def register_callback_on_read_holding_reg(self, cb: T.Callable[[int, int, T.Sequence[int]], None]):
        '''callback: (trans_id: int, periph_id: int, reg_vals: T.Sequence[int]) -> None'''
        assert cb
        if self.__on_read_holding_reg:
            if cb != self.__on_read_holding_reg:
                self.__on_read_holding_reg = cb
        else:
            self.__on_read_holding_reg = cb
            _cl.SerUiCl_setReadHoldingRegCallback(self._cimpl, self._cb_on_read_holding_reg)

    cdef unregister_callback_on_read_holding_reg(self):
        if self.__on_read_holding_reg:
            _cl.SerUiCl_setReadHoldingRegCallback(self._cimpl, NULL)
            self.__on_read_holding_reg = None

    def register_callback_on_read_input_reg(self, cb: T.Callable[[int, int, T.Sequence[int]], None]):
        '''callback: (trans_id: int, periph_id: int, reg_vals: T.Sequence[int]) -> None'''
        assert cb
        if self.__on_read_input_reg:
            if cb != self.__on_read_input_reg:
                self.__on_read_input_reg = cb
        else:
            self.__on_read_input_reg = cb
            _cl.SerUiCl_setReadInputRegCallback(self._cimpl, self._cb_on_read_input_reg)

    cdef unregister_callback_on_read_input_reg(self):
        if self.__on_read_input_reg:
            _cl.SerUiCl_setReadInputRegCallback(self._cimpl, NULL)
            self.__on_read_input_reg = None

    def register_callback_on_write_single_reg(self, cb: T.Callable[[int, int, int, int], None]):
        '''callback: (trans_id: int, periph_id: int, reg_addr: int, reg_val: int) -> None'''
        assert cb
        if self.__on_write_single_reg:
            if cb != self.__on_write_single_reg:
                self.__on_write_single_reg = cb
        else:
            self.__on_write_single_reg = cb
            _cl.SerUiCl_setWriteSingleRegCallback(self._cimpl, self._cb_on_write_single_reg)

    cdef unregister_callback_on_write_single_reg(self):
        if self.__on_write_single_reg:
            _cl.SerUiCl_setWriteSingleRegCallback(self._cimpl, NULL)
            self.__on_write_single_reg = None

    def register_callback_on_write_multi_reg(self, cb: T.Callable[[int, int, int, int], None]):
        '''callback: (trans_id: int, periph_id: int, reg_addr_base: int, nreg: int) -> None'''
        assert cb
        if self.__on_write_multi_reg:
            if cb != self.__on_write_multi_reg:
                self.__on_write_multi_reg = cb
        else:
            self.__on_write_multi_reg = cb
            _cl.SerUiCl_setWriteMultiRegCallback(self._cimpl, self._cb_on_write_multi_reg)

    cdef unregister_callback_on_write_multi_reg(self):
        if self.__on_write_multi_reg:
            _cl.SerUiCl_setWriteMultiRegCallback(self._cimpl, NULL)
            self.__on_write_multi_reg = None

    def register_callback_on_version(self, cb: T.Callable[[int, int, int, int, int], None]):
        '''callback: (trans_id: int, periph_id: int, version: int, protocol: int, protocol_version: int) -> None'''
        assert cb
        if self.__on_version:
            if cb != self.__on_version:
                self.__on_version = cb
        else:
            self.__on_version = cb
            _cl.SerUiCl_setVersionCallback(self._cimpl, self._cb_on_version)

    cdef unregister_callback_on_version(self):
        if self.__on_version:
            _cl.SerUiCl_setVersionCallback(self._cimpl, NULL)
            self.__on_version = None

    def register_callback_on_invoke_method(self, cb: T.Callable[[int, int, bytes], None]):
        '''callback: (trans_id: int, periph_id: int, ret_bytes: bytes) -> None'''
        assert cb
        if self.__on_invoke_method:
            if cb != self.__on_invoke_method:
                self.__on_invoke_method = cb
        else:
            self.__on_invoke_method = cb
            _cl.SerUiCl_setInvokeMethodCallback(self._cimpl, self._cb_on_invoke_method)

    cdef unregister_callback_on_invoke_method(self):
        if self.__on_invoke_method:
            _cl.SerUiCl_setInvokeMethodCallback(self._cimpl, NULL)
            self.__on_invoke_method = None

    def register_callback_on_continuous_report(self, cb: T.Callable[[int, int, bytes], None]):
        '''callback: (trans_id: int, periph_id: int, ret_bytes: bytes) -> None'''
        assert cb
        if self.__on_continuous_report:
            if cb != self.__on_continuous_report:
                self.__on_continuous_report = cb
        else:
            self.__on_continuous_report = cb
            _cl.SerUiCl_setContinuousReportCallback(self._cimpl, self._cb_on_continuous_report)

    cdef unregister_callback_on_continuous_report(self):
        if self.__on_continuous_report:
            _cl.SerUiCl_setContinuousReportCallback(self._cimpl, NULL)
            self.__on_continuous_report = None

    def register_callback_on_log_string(self, cb: T.Callable[[int, int, str], None]):
        '''callback: (trans_id: int, periph_id: int, log_str: str) -> None'''
        assert cb
        if self.__on_log_string:
            if cb != self.__on_log_string:
                self.__on_log_string = cb
        else:
            self.__on_log_string = cb
            _cl.SerUiCl_setLogStringCallback(self._cimpl, self._cb_on_log_string)

    cdef unregister_callback_on_log_string(self):
        if self.__on_log_string:
            _cl.SerUiCl_setLogStringCallback(self._cimpl, NULL)
            self.__on_log_string = None

    def register_callback_on_switch_response(self, cb: T.Callable[[int, int], None]):
        '''callback: (trans_id: int, periph_id: int) -> None'''
        assert cb
        if self.__on_switch_response:
            if cb != self.__on_switch_response:
                self.__on_switch_response = cb
        else:
            self.__on_switch_response = cb
            _cl.SerUiCl_setSwitchResponseCallback(self._cimpl, self._cb_on_switch_response)

    cdef unregister_callback_on_switch_response(self):
        if self.__on_switch_response:
            _cl.SerUiCl_setSwitchResponseCallback(self._cimpl, NULL)
            self.__on_switch_response = None

    @staticmethod
    cdef void _cb_on_recv_bytes(const uint8_t* data, uint16_t nbyte,
                                void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_recv_bytes = obj.__on_recv_bytes
        if __on_recv_bytes:
            mv = PyMemoryView_FromMemory(<char*>data, nbyte, buffer.PyBUF_READ)
            __on_recv_bytes(bytes(mv))

    @staticmethod
    cdef void _cb_on_error_response(uint16_t trans_id, uint8_t periph_id, 
                                    uint8_t fcode, uint8_t err_code,
                                    void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_error_response = obj.__on_error_response
        if __on_error_response:
            __on_error_response(trans_id, periph_id, fcode, err_code)

    @staticmethod
    cdef void _cb_on_read_holding_reg(uint16_t trans_id, uint8_t periph_id, 
                                      uint16_t nreg, const uint16_t* reg_vals,
                                      void* user_data) noexcept with gil:
        cdef array.array ar = array.array('H', [])
        obj = <object>user_data
        __on_read_holding_reg = obj.__on_read_holding_reg
        if __on_read_holding_reg:
            array.extend_buffer(ar, <char*>reg_vals, nreg)
            __on_read_holding_reg(trans_id, periph_id, ar.tolist())

    @staticmethod
    cdef void _cb_on_read_input_reg(uint16_t trans_id, uint8_t periph_id, 
                                    uint16_t nreg, const uint16_t* reg_vals,
                                    void* user_data) noexcept with gil:
        cdef array.array ar = array.array('H', [])
        obj = <object>user_data
        __on_read_input_reg = obj.__on_read_input_reg
        if __on_read_input_reg:
            array.extend_buffer(ar, <char*>reg_vals, nreg)
            __on_read_input_reg(trans_id, periph_id, ar.tolist())

    @staticmethod
    cdef void _cb_on_write_single_reg(uint16_t trans_id, uint8_t periph_id, 
                                      uint16_t reg_addr, uint16_t reg_val,
                                      void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_write_single_reg = obj.__on_write_single_reg
        if __on_write_single_reg:
            __on_write_single_reg(trans_id, periph_id, reg_addr, reg_val)

    @staticmethod
    cdef void _cb_on_write_multi_reg(uint16_t trans_id, uint8_t periph_id, 
                                     uint16_t reg_addr_base, uint16_t nreg,
                                     void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_write_multi_reg = obj.__on_write_multi_reg
        if __on_write_multi_reg:
            __on_write_multi_reg(trans_id, periph_id, reg_addr_base, nreg)

    @staticmethod
    cdef void _cb_on_version(uint16_t trans_id, uint8_t periph_id, 
                             uint32_t version, uint8_t protocol, uint16_t protocol_version,
                             void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_version = obj.__on_version
        if __on_version:
            __on_version(trans_id, periph_id, version, protocol, protocol_version)
    
    @staticmethod
    cdef void _cb_on_invoke_method(uint16_t trans_id, uint8_t periph_id, 
                                   const uint8_t* ret_bytes, uint16_t nbyte,
                                   void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_invoke_method = obj.__on_invoke_method
        if __on_invoke_method:
            mv = PyMemoryView_FromMemory(<char*>ret_bytes, nbyte, buffer.PyBUF_READ)
            __on_invoke_method(trans_id, periph_id, bytes(mv))
    
    @staticmethod
    cdef void _cb_on_continuous_report(uint16_t trans_id, uint8_t periph_id, 
                                       const uint8_t* ret_bytes, uint16_t nbyte,
                                       void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_continuous_report = obj.__on_continuous_report
        if __on_continuous_report:
            mv = PyMemoryView_FromMemory(<char*>ret_bytes, nbyte, buffer.PyBUF_READ)
            __on_continuous_report(trans_id, periph_id, bytes(mv))

    @staticmethod
    cdef void _cb_on_log_string(uint16_t trans_id, uint8_t periph_id, 
                                const char* log_str,
                                void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_log_string = obj.__on_log_string
        if __on_log_string:
            __on_log_string(trans_id, periph_id, SerUiCl._cvt2pystr(log_str))

    @staticmethod
    cdef void _cb_on_switch_response(uint16_t trans_id, uint8_t periph_id,
                                     void* user_data) noexcept with gil:
        obj = <object>user_data
        __on_switch_response = obj.__on_switch_response
        if __on_switch_response:
            __on_switch_response(trans_id, periph_id)
