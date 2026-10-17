"""Bounded asynchronous jobs. No slow operation holds the status cache lock."""
from concurrent.futures import ThreadPoolExecutor
import threading
import time
import uuid

class JobManager:
    def __init__(self):
        self.lock=threading.Lock();self.jobs={};self.ids={}
        self.pool=ThreadPoolExecutor(max_workers=3,thread_name_prefix='panel-job')
    def submit(self,request_id,work):
        with self.lock:
            if request_id in self.ids:return self.ids[request_id]
            if sum(j['state'] in ('queued','running') for j in self.jobs.values())>=8:raise ValueError('已有太多后台任务')
            # Retain bounded completed history; never evict an active job.
            for key in list(self.jobs):
                if len(self.jobs)<128:break
                if self.jobs[key]['state'] not in ('queued','running'):
                    rid=self.jobs.pop(key)['request_id'];self.ids.pop(rid,None)
            jid=uuid.uuid4().hex;self.ids[request_id]=jid
            self.jobs[jid]={'operation_id':jid,'request_id':request_id,'state':'queued','created_at':time.time()}
        def run():
            with self.lock:self.jobs[jid]['state']='running'
            try:
                result=work()
                with self.lock:self.jobs[jid].update(state='complete',result=result)
            except Exception as e:
                with self.lock:self.jobs[jid].update(state='error',error=str(e))
        self.pool.submit(run);return jid
    def get(self,jid):
        with self.lock:
            if jid not in self.jobs:raise KeyError(jid)
            return self.jobs[jid].copy()
