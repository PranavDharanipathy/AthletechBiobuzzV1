#!/usr/bin/env python3
"""Summarize observed outcomes, not modeled success probabilities, from experiment CSV."""
import argparse
import csv
import json
from collections import Counter

VALID={'STAY','EXIT','MISS','RIM','INVALID'}

def summarize(stream):
    attempts={}
    for row in csv.DictReader(stream):
        batch=int(row['batch']); attempt=int(row['attempt'])
        if attempt<=0:
            continue
        key=(batch,attempt)
        data=attempts.setdefault(key,dict(outcome='UNLABELED',rpm=row['target_rpm'],
                                        hood=row['hood_position'],angle=row['launch_deg'],distance=row['distance_ft']))
        if row['outcome'] in VALID:
            data['outcome']=row['outcome']
    batches=[]
    for batch in sorted({k[0] for k in attempts}):
        rows=[v for k,v in attempts.items() if k[0]==batch]
        counts=Counter(r['outcome'] for r in rows)
        clean=counts['STAY']+counts['EXIT']
        labeled=clean+counts['MISS']+counts['RIM']
        batches.append(dict(batch=batch,rpm=rows[0]['rpm'],hood_position=rows[0]['hood'],
                            launch_deg=rows[0]['angle'],distance_ft=rows[0]['distance'],
                            observed_attempts=len(rows),outcomes=dict(counts),
                            clean_entry_fraction=clean/labeled if labeled else None,
                            retention_after_clean_entry=counts['STAY']/clean if clean else None,
                            clean_entry_and_retained_fraction=counts['STAY']/labeled if labeled else None))
    return batches

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file')
    args=parser.parse_args()
    with open(args.csv_file,newline='') as stream:
        print(json.dumps(summarize(stream),indent=2))

if __name__=='__main__':
    main()
