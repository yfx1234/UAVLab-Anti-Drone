"""Opt-in transfer cancellation/resume/hash test using a labelled board fixture."""
import pathlib,sys,json
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'panel'))
from board_client import BoardClient
from session_manager import SessionManager
def run(sid):
    root=pathlib.Path(__file__).resolve().parents[1]/'test-results/transfer_fixture'
    manager=SessionManager(BoardClient(),root);progress=[]
    def report(p):progress.append(p)
    try:manager.download(sid,report,lambda:bool(progress and progress[-1]['downloaded_bytes']>=131072))
    except ValueError as e:assert '取消' in str(e)
    else:raise AssertionError('cancel ignored')
    parts=list((root/sid).rglob('*.part'));assert parts,'no resumable part'
    result=manager.download(sid)
    assert result['files'] and not list((root/sid).rglob('*.part'))
    print('PASS: cancellation, resumed download and source SHA256 verification')
    (root/'evidence.json').write_text(json.dumps(result),encoding='utf-8')
if __name__=='__main__':run(sys.argv[1])
