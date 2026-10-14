"""Shell selection consumes the full Unicode file identity and reports failures."""
import ctypes
import pathlib
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'panel'))
from windows_file_location import reveal_file


class WindowsFileLocationTests(unittest.TestCase):
    def libraries(self, initialized=0, parsed=0, selected=0):
        ole = SimpleNamespace(CoInitializeEx=Mock(return_value=initialized),
                              CoUninitialize=Mock(), CoTaskMemFree=Mock())
        def parse(path, context, output, flags, attributes):
            if parsed >= 0:
                output._obj.value = 12345
            return parsed
        shell = SimpleNamespace(SHParseDisplayName=Mock(side_effect=parse),
                                SHOpenFolderAndSelectItems=Mock(return_value=selected))
        return ole, shell

    def test_selects_the_exact_unicode_video_and_releases_resources(self):
        ole, shell = self.libraries()
        path = pathlib.Path('C:/资料/北京时间 场次/整场 rgb.view.mp4')
        with patch.object(ctypes, 'WinDLL', side_effect=[ole, shell], create=True):
            reveal_file(path)
        self.assertEqual(shell.SHParseDisplayName.call_args.args[0], str(path))
        item, count, children, flags = shell.SHOpenFolderAndSelectItems.call_args.args
        self.assertEqual(item.value, 12345)
        self.assertEqual((count, children, flags), (0, None, 0))
        ole.CoTaskMemFree.assert_called_once_with(item)
        ole.CoUninitialize.assert_called_once()

    def test_shell_error_reaches_the_job_and_still_frees_resources(self):
        ole, shell = self.libraries(selected=-2147024891)
        with patch.object(ctypes, 'WinDLL', side_effect=[ole, shell], create=True):
            with self.assertRaisesRegex(OSError, '80070005'):
                reveal_file('C:/video.mp4')
        ole.CoTaskMemFree.assert_called_once()
        ole.CoUninitialize.assert_called_once()

    def test_missing_file_does_not_open_an_unrelated_folder(self):
        ole, shell = self.libraries(parsed=-2147024894)
        with patch.object(ctypes, 'WinDLL', side_effect=[ole, shell], create=True):
            with self.assertRaisesRegex(OSError, '80070002'):
                reveal_file('C:/missing.mp4')
        shell.SHOpenFolderAndSelectItems.assert_not_called()
        ole.CoTaskMemFree.assert_not_called()
        ole.CoUninitialize.assert_called_once()

    def test_does_not_uninitialize_a_preexisting_com_apartment(self):
        ole, shell = self.libraries(initialized=-2147417850)
        with patch.object(ctypes, 'WinDLL', side_effect=[ole, shell], create=True):
            reveal_file('C:/video.mp4')
        shell.SHOpenFolderAndSelectItems.assert_called_once()
        ole.CoUninitialize.assert_not_called()


if __name__ == '__main__':
    unittest.main()
