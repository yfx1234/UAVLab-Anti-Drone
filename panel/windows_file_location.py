"""Reveal a Unicode filename through the Shell API, without command-line parsing."""
import ctypes


def reveal_file(path):
    ole = ctypes.WinDLL('ole32')
    shell = ctypes.WinDLL('shell32')
    ole.CoInitializeEx.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    ole.CoInitializeEx.restype = ctypes.c_long
    ole.CoUninitialize.argtypes = []
    ole.CoUninitialize.restype = None
    ole.CoTaskMemFree.argtypes = [ctypes.c_void_p]
    ole.CoTaskMemFree.restype = None
    shell.SHParseDisplayName.argtypes = [ctypes.c_wchar_p, ctypes.c_void_p,
                                       ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint32, ctypes.c_void_p]
    shell.SHParseDisplayName.restype = ctypes.c_long
    shell.SHOpenFolderAndSelectItems.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    shell.SHOpenFolderAndSelectItems.restype = ctypes.c_long
    initialized = ole.CoInitializeEx(None, 2)
    # A worker may already have initialized COM using a different apartment.
    if initialized < 0 and (initialized & 0xffffffff) != 0x80010106:
        raise OSError(f'Windows 文件位置初始化失败：0x{initialized & 0xffffffff:08X}')
    item = ctypes.c_void_p()
    try:
        result = shell.SHParseDisplayName(str(path), None, ctypes.byref(item), 0, None)
        if result < 0 or not item.value:
            raise OSError(f'Windows 无法解析视频位置：0x{result & 0xffffffff:08X}；文件：{path}')
        # With zero child items, the fully qualified file PIDL opens its parent
        # folder and selects that file (Microsoft's documented API contract).
        result = shell.SHOpenFolderAndSelectItems(item, 0, None, 0)
        if result < 0:
            raise OSError(f'Windows 无法打开并选中视频：0x{result & 0xffffffff:08X}；文件：{path}')
    finally:
        if item.value:
            ole.CoTaskMemFree(item)
        if initialized >= 0:
            ole.CoUninitialize()
