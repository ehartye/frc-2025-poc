"""Resolve two actual station identities and the same wall clock for each layout."""
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'experiments/six-wheel'
sys.path.insert(0, str(BASE / 'scripts'))
from receiver_contact import departure, flight_center


def main():
    source = json.loads((BASE / 'sourced-match/source-545.json').read_bytes())['feed']
    with (BASE / 'spatial-contact/events.csv').open() as stream:
        events = list(csv.DictReader(stream))
    result = {'clocks': {}}
    for case, scenario in [('nominal', 'nominal'), ('adverse', 'lateB')]:
        clock = {}
        for event in events:
            if event['approach'] == 'nominal' and event['scenario'] == scenario:
                clock.setdefault(event['piece'], {})[event['event']] = float(event['wall_ms']) / 1000
        result['clocks'][case] = clock
        adverse = case == 'adverse'
        speed = source['lip_mps'] * (.85 if adverse else 1.)
        lip = departure(speed)
        x, z = flight_center(lip, 0)
        result[case] = dict(x=x-(.444525+(.020 if adverse else 0.)), z=z,
                            vx=lip['velocity_x_mps'], vz=lip['velocity_z_mps'],
                            omega=speed/.05715, feedToLipS=source['free_drop_s']+source['ramp_time_s'])
    dest = ROOT / 'artifacts/split-drop-native/launch-inputs.json'
    dest.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
