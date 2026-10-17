import pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
class NxBootTests(unittest.TestCase):
    def test_candidate_preserves_original_boot_entry_and_append(self):
        from prepare_nx_boot import render_extlinux
        original='TIMEOUT 30\nDEFAULT ZED_DAMIAO\nLABEL ZED_DAMIAO\n LINUX /boot/Image\n INITRD /boot/initrd\n FDT /boot/old.dtb\n OVERLAYS /boot/mono.dtbo\n APPEND ${cbootargs} root=PARTUUID=abc rw\n'
        output=render_extlinux(original,'/boot/dtb/uavlab-nx16.dtb','/boot/mono.dtbo')
        self.assertIn('DEFAULT UAVLAB_NX16\n',output)
        self.assertIn(original.split('LABEL ZED_DAMIAO')[1],output)
        self.assertIn('LABEL UAVLAB_NX16\n',output)
        self.assertEqual(output.count('APPEND ${cbootargs} root=PARTUUID=abc rw'),2)
        self.assertEqual(render_extlinux(output,'/boot/dtb/uavlab-nx16.dtb','/boot/mono.dtbo'),output)
    def test_missing_source_entry_does_not_emit_a_candidate(self):
        from prepare_nx_boot import render_extlinux
        with self.assertRaises(ValueError):render_extlinux('DEFAULT absent\nLABEL other\n','/boot/new.dtb','/boot/mono.dtbo')
    def test_rejects_field_injection_and_existing_label_collision(self):
        from prepare_nx_boot import render_extlinux
        old='DEFAULT old\nLABEL old\n LINUX /boot/Image\n INITRD /boot/initrd\n APPEND root=a\n'
        with self.assertRaises(ValueError):render_extlinux(old,'/boot/new.dtb\nAPPEND bad','/boot/mono.dtbo')
        with self.assertRaises(ValueError):render_extlinux(old+'LABEL UAVLAB_NX16\n FDT /boot/unrelated.dtb\n','/boot/new.dtb','/boot/mono.dtbo')
