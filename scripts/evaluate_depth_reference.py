"""Assess independently recomputed stereo references; never alter live calibration."""
import argparse
import collections
import json
import math
import pathlib
import statistics


def percentile(values, fraction):
    ordered=sorted(values)
    at=(len(ordered)-1)*fraction;low=int(at);high=min(low+1,len(ordered)-1)
    return ordered[low]*(high-at)+ordered[high]*(at-low) if low!=high else ordered[low]


def reference(row):
    """No nominal-size or previously accepted online-mode gate is used."""
    if row.get('prior_size_gate_used') is not False or not row.get('quality_accepted') or row.get('clipped') or not row.get('background_separated'):return None
    try:
        z=float(row['depth_z_m']);fx=float(row['fx']);fy=float(row['fy'])
        x1,y1,x2,y2=map(float,row['bbox_xyxy']);bw=x2-x1;bh=y2-y1
        numbers=[z,fx,fy,bw,bh,row['depth_iqr_m'],row['depth_valid_ratio'],row['confidence']]
        if not all(math.isfinite(v) for v in numbers):return None
        if not .5<=z<=10 or min(fx,fy,bh)<=0 or max(bw,bh)<8 or bw<=0:return None
        if row['confidence']<.7 or row['depth_valid_ratio']<.2 or row['depth_valid_pixels']<30 or row['depth_iqr_m']/z>.05:return None
        return dict(row,normalized_extent=max(bw/fx,bh/fy),effective_size_m=z*max(bw/fx,bh/fy))
    except (KeyError,ValueError,TypeError):return None


def errors(rows, size):
    relative=[abs(size/r['effective_size_m']-1) for r in rows]
    absolute=[r['depth_z_m']*e for r,e in zip(rows,relative)]
    return {'seconds':len(rows),'median_absolute_error_m':statistics.median(absolute),
            'median_absolute_relative_error':statistics.median(relative),'p90_absolute_relative_error':percentile(relative,.9)}


def is_excluded(row, exclusions):
    for item in exclusions:
        if row.get('session_id')!=item['session_id']:continue
        if 'segment_id' not in item:return True
        if row.get('segment_id')==item['segment_id'] and item.get('start_svo_position',0)<=row.get('svo_position',-1)<=item.get('end_svo_position',math.inf):return True
    return False


def evaluate(rows, reviews, exclusions=None):
    exclusions=exclusions or []
    counts=collections.Counter();seconds={};all_bindings=set()
    for row in rows:
        if is_excluded(row,exclusions):counts['user_exclusion']+=1;continue
        qualified=reference(row)
        if qualified is None:counts['failed_depth_or_detection_quality']+=1;continue
        key=row['session_id']+'/'+row.get('image_id','')
        if not row.get('image_id') or reviews.get(key)!='target':counts['not_visually_confirmed_target']+=1;continue
        binding=(row['model_id'],row['camera_serial'],row['image_width'],row['image_height'])
        all_bindings.add(binding)
        block=(row['session_id'],row['stamp_ns']//1000000000)
        if block not in seconds or qualified['confidence']>seconds[block]['confidence']:seconds[block]=qualified
    selected=list(seconds.values());groups=collections.defaultdict(list)
    for row in selected:groups[row['session_id']].append(row)
    limitations=['stereo_reference_is_not_external_ground_truth','not_validated_for_other_airframes_models_or_ranges']
    candidate=None;holdouts=[]
    if len(selected)<20:limitations.append('fewer_than_20_reviewed_seconds')
    if len(groups)<2:limitations.append('fewer_than_two_independent_sessions')
    if len(all_bindings)>1:limitations.append('mixed_model_or_camera_binding')
    zvalues=[r['depth_z_m'] for r in selected]
    if zvalues and max(zvalues)/min(zvalues)<1.5:limitations.append('narrow_depth_coverage')
    if len(selected)>=20 and len(all_bindings)==1:
        # Equal scene weight prevents the longest recording from defining the scale.
        candidate=statistics.median(statistics.median(r['effective_size_m'] for r in group) for group in groups.values())
        if len(groups)>=2:
            for sid,held in groups.items():
                train=[statistics.median(r['effective_size_m'] for r in group) for key,group in groups.items() if key!=sid]
                fitted=statistics.median(train)
                holdouts.append(dict(session_id=sid,train_effective_size_m=fitted,**errors(held,fitted)))
            if any(h['median_absolute_relative_error']>.15 or h['p90_absolute_relative_error']>.30 for h in holdouts):
                limitations.append('cross_session_errors_exceed_candidate_gate')
    return {'calibrated':False,'applied_to_live_system':False,'reference':'independent_recomputed_stereo',
            'candidate_effective_size_m':candidate,'selected_seconds':len(selected),
            'model_ids':sorted({b[0] for b in all_bindings}),'binding':list(next(iter(all_bindings))) if len(all_bindings)==1 else None,
            'depth_z_range_m':[min(zvalues),max(zvalues)] if zvalues else None,
            'rejected_counts':dict(counts),'exclusions':exclusions,'limitations':limitations,'leave_one_session_out':holdouts,
            'by_session':{sid:{'reviewed_seconds':len(group),'median_effective_size_m':statistics.median(r['effective_size_m'] for r in group),
                               'depth_z_range_m':[min(r['depth_z_m'] for r in group),max(r['depth_z_m'] for r in group)],
                               'baseline_0_30m':errors(group,.30),'candidate':errors(group,candidate) if candidate else None}
                          for sid,group in groups.items()}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('audit_root',type=pathlib.Path);parser.add_argument('--reviews',type=pathlib.Path,required=True)
    parser.add_argument('--exclusions',type=pathlib.Path)
    parser.add_argument('--output',type=pathlib.Path,required=True);args=parser.parse_args()
    rows=[json.loads(line) for file in args.audit_root.glob('session_*/samples.jsonl') for line in file.read_text(encoding='utf-8').splitlines()]
    reviews=json.loads(args.reviews.read_text(encoding='utf-8'))
    exclusions=json.loads(args.exclusions.read_text(encoding='utf-8')) if args.exclusions else []
    report=evaluate(rows,reviews,exclusions)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':main()
