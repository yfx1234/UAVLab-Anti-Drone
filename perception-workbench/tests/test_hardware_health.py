import json,pathlib,sys,tempfile,unittest
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
import hardware_health

class HardwareHealthTests(unittest.TestCase):
 def fixture(self,root):
  def write(name,value):
   p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(value)
  write('sys/class/thermal/thermal_zone0/type','cpu-thermal');write('sys/class/thermal/thermal_zone0/temp','60000')
  write('sys/class/thermal/thermal_zone1/type','gpu-thermal');write('sys/class/thermal/thermal_zone1/temp','55000')
  write('sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon2/in1_label','VDD_IN')
  write('sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon2/in1_input','5000')
  write('sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon2/curr1_input','2000')
  write('proc/meminfo','MemAvailable: 8388608 kB\n')
  write('proc/net/route','Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\neth0 0032A8C0 00000000 0001 0 0 0 00FFFFFF 0 0 0\n')
  write('sys/class/net/eth0/carrier','1');write('sys/class/net/eth0/speed','100')
 def test_sensor_units_and_unknown_radio_are_explicit(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);self.fixture(root);h=hardware_health.read_hardware(root,root)
   self.assertEqual(h['temperatures_c'],{'cpu':60.0,'gpu':55.0,'max':60.0})
   self.assertEqual(h['module_input_voltage_v'],5.0);self.assertEqual(h['module_power_w'],10.0)
   self.assertEqual(h['network']['interface'],'eth0');self.assertTrue(h['network']['carrier'])
   self.assertEqual(h['network']['speed_mbps'],100);self.assertIsNone(h['radio_signal_dbm'])
 def test_absent_sensors_stay_unknown_instead_of_zero(self):
  with tempfile.TemporaryDirectory() as tmp:
   h=hardware_health.read_hardware(pathlib.Path(tmp),pathlib.Path(tmp))
   self.assertIsNone(h['module_input_voltage_v']);self.assertIsNone(h['temperatures_c']['max'])
   self.assertIsNone(h['network']['carrier']);self.assertIsNone(h['radio_signal_dbm'])
 def test_cache_avoids_resampling_and_rejects_old_boot_and_future_samples(self):
  with tempfile.TemporaryDirectory() as tmp:
   cache=pathlib.Path(tmp)/'health.json';sample={'temperatures_c':{'max':55}}
   with patch('hardware_health.read_hardware',return_value=sample) as read:
    first=hardware_health.hardware_snapshot('boot1',cache,now=10);second=hardware_health.hardware_snapshot('boot1',cache,now=11)
    self.assertEqual(read.call_count,1);self.assertEqual(second['sample_age_s'],1)
    hardware_health.hardware_snapshot('boot2',cache,now=11);self.assertEqual(read.call_count,2)
    hardware_health.hardware_snapshot('boot2',cache,now=9);self.assertEqual(read.call_count,3)
 def test_corrupt_cache_does_not_break_status(self):
  with tempfile.TemporaryDirectory() as tmp:
   cache=pathlib.Path(tmp)/'health.json';cache.write_text('{bad')
   with patch('hardware_health.read_hardware',return_value={}):
    self.assertEqual(hardware_health.hardware_snapshot('boot',cache,now=1)['sample_age_s'],0)

if __name__=='__main__':unittest.main()
