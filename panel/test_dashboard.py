import unittest
from dashboard import valid_request,Panel

class DashboardTests(unittest.TestCase):
    def test_remote_origin_rejected(self):
        self.assertFalse(valid_request('127.0.0.1:8767','https://example.com','token','token'))
        self.assertFalse(valid_request('127.0.0.1:8767','http://127.0.0.1:8767','bad','token'))
        self.assertTrue(valid_request('127.0.0.1:8767','http://127.0.0.1:8767','token','token'))
    def test_flight_action_rejected_before_any_network_job(self):
        panel=Panel()
        try:
            for action in ('chain_start_live','chain_start','chain_stop','watcher_start'):
                with self.assertRaises(ValueError):panel.submit({'action':action})
            self.assertFalse(panel.snapshot()['connected'])
        finally:panel.close()
if __name__=='__main__':unittest.main()
