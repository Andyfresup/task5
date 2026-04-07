import sys
if sys.version_info.major < 3:
    from .impl2 import ByteSize, Parity, StopBits, Error, SerUiProtocol, SerUiCl
else:
    from .impl3 import ByteSize, Parity, StopBits, Error, SerUiProtocol, SerUiCl


__all__ = [
    'ByteSize',
    'Parity',
    'StopBits', 
    'Error',
    'SerUiProtocol',
    'SerUiCl'
]
