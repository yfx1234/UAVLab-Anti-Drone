"""Transfer only an isolated annotation job, never recording directories."""
import re

JOBS_ROOT='/home/uavlab/Jetson_ROS2_V0/perception_annotation_jobs'


class AnnotationTransport:
    def __init__(self,board):self.board=board

    @staticmethod
    def folder(job_id):
        if not isinstance(job_id,str) or not re.fullmatch('[0-9a-f]{32}',job_id):
            raise ValueError('检测任务 ID 无效')
        return JOBS_ROOT+'/'+job_id

    def upload(self,job_id,stream,size,progress):
        folder=self.folder(job_id)
        with self.board.connection() as client:
            sftp=client.open_sftp()
            try:
                try:sftp.mkdir(JOBS_ROOT)
                except OSError:sftp.stat(JOBS_ROOT)
                sftp.mkdir(folder)
                sftp.get_channel().settimeout(1800)
                sftp.putfo(stream,folder+'/source.mp4',file_size=size,callback=progress,confirm=True)
            finally:sftp.close()

    def cleanup(self,job_id):
        folder=self.folder(job_id)
        with self.board.connection() as client:
            sftp=client.open_sftp()
            try:
                for name in ('source.mp4','output.mp4'):
                    try:sftp.remove(folder+'/'+name)
                    except FileNotFoundError:pass
                try:sftp.rmdir(folder)
                except FileNotFoundError:pass
            finally:sftp.close()
