import math
import csv
import re
from datetime import datetime

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0)**2
    return 2.0 * R * math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

print("=" * 70)
print("   RTK ROVER FIELD TEST ANALYSIS REPORT")
print("=" * 70)

# 1. Analyze Surveyed Pins vs Waypoints
print("\n--- [1. SURVEY LOGGED PINS VS DESIGN WAYPOINTS] ---")
wp_dict = {}
with open('RTK_Rover/test/waypoints.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        lbl = row.get('label') or row.get('name')
        lat = float(row.get('latitude') or row.get('lat', 0.0))
        lon = float(row.get('longitude') or row.get('lon', 0.0))
        elev = float(row.get('elevation') or row.get('elev', 0.0))
        if lbl:
            wp_dict[lbl] = (lat, lon, elev)

pins = []
with open('RTK_Rover/test/survey_logged_pins.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        lbl = row.get('label') or ''
        lat = float(row.get('latitude') or row.get('lat', 0.0))
        lon = float(row.get('longitude') or row.get('lon', 0.0))
        elev = float(row.get('elevation') or row.get('elevation_m', 0.0))
        fix = row.get('pos_fix') or row.get('fix_type') or 'UNKNOWN'
        hacc = float(row.get('h_acc_m') or 0.0)
        samples = int(row.get('samples') or 0)
        ts = row.get('timestamp') or ''
        notes = row.get('notes') or ''
        pins.append({
            'label': lbl, 'lat': lat, 'lon': lon, 'alt': elev,
            'fix': fix, 'hacc': hacc, 'samples': samples,
            'time': ts, 'notes': notes,
            'base_id': row.get('base_id', '--'),
            'hdg_fix': row.get('hdg_fix', '--'),
            'fw': row.get('firmware_ver', 'v0.2.9')
        })

for pin in pins:
    print(f"Pin: {pin['label']} (FW: {pin['fw']}, Base: {pin['base_id']})")
    print(f"  Fix Quality:     {pin['fix']} / Hdg: {pin['hdg_fix']} (Accuracy: +/-{pin['hacc']*100:.1f} cm)")
    print(f"  Samples Averaged:{pin['samples']} epochs")
    print(f"  Position:        {pin['lat']:.8f}, {pin['lon']:.8f} (Elevation: {pin['alt']:.3f} m)")
    print(f"  Notes:           {pin['notes']}")

    matched_key = None
    for k in wp_dict:
        if k in pin['label'] or pin['label'] in k or (('TREE' in k and 'TREE' in pin['label']) or ('DRIVEWAY' in k and 'DRIVEWAY' in pin['label'])):
            matched_key = k
            break
    if matched_key:
        ref_lat, ref_lon, ref_alt = wp_dict[matched_key]
        d_err = haversine_m(ref_lat, ref_lon, pin['lat'], pin['lon'])
        d_brg = bearing_deg(ref_lat, ref_lon, pin['lat'], pin['lon'])
        d_north = (pin['lat'] - ref_lat) * 111132.954
        d_east  = (pin['lon'] - ref_lon) * 111132.954 * math.cos(math.radians(ref_lat))
        print(f"  Offset from Phone/KML initial estimate ({ref_lat:.7f}, {ref_lon:.7f}):")
        print(f"    Total 2D Error: {d_err:.2f} m ({d_err*3.28084:.2f} ft) @ {d_brg:.1f} deg")
        print(f"    Delta:          dN = {d_north:+.2f} m, dE = {d_east:+.2f} m")

if len(pins) >= 2:
    p1 = pins[0]
    p2 = pins[1]
    dist_pins = haversine_m(p1['lat'], p1['lon'], p2['lat'], p2['lon'])
    brg_pins  = bearing_deg(p1['lat'], p1['lon'], p2['lat'], p2['lon'])
    d_alt     = p2['alt'] - p1['alt']
    print(f"\nGround Truth Baseline between the Two Found Pins:")
    print(f"  Distance: {dist_pins:.3f} m ({dist_pins*3.28084:.2f} ft)")
    print(f"  Bearing:  {brg_pins:.2f} deg ({p1['label']} -> {p2['label']})")
    print(f"  Elev Diff:{d_alt:+.3f} m ({d_alt*3.28084:+.2f} ft)")

# 2. Analyze Route CSV
print("\n--- [2. ROUTE TRACK RECORDING ANALYSIS] ---")
route_file = 'RTK_Rover/test/route_2026-10-26_15-47-32-240.csv'
points = []
with open(route_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        points.append(row)

print(f"Total Logged Track Points: {len(points)}")
if points:
    print(f"Start Time: {points[0]['timestamp']}")
    print(f"End Time:   {points[-1]['timestamp']}")

fix_counts = {}
sats_list = []
hacc_list = []
baseline_lens = []
baseline_pitch = []
coords = []

for pt in points:
    fix = pt.get('pos_fix') or pt.get('fix_type') or 'UNKNOWN'
    fix_counts[fix] = fix_counts.get(fix, 0) + 1
    sats_list.append(int(pt['sats']))
    hacc_list.append(float(pt['hacc_m']))
    baseline_lens.append(float(pt['baseline_len_m']))
    baseline_pitch.append(float(pt['baseline_pitch_deg']))
    coords.append((float(pt['tip_lat']), float(pt['tip_lon'])))

print("Fix Type Distribution:")
for fix, count in fix_counts.items():
    pct = (count / len(points)) * 100.0
    print(f"  {fix:12}: {count:4d} points ({pct:.1f}%)")

print(f"Satellites Tracked: Min={min(sats_list)}, Max={max(sats_list)}, Mean={sum(sats_list)/len(sats_list):.1f}")
hacc_sorted = sorted(hacc_list)
cep50 = hacc_sorted[int(len(hacc_sorted)*0.5)]
r95 = hacc_sorted[int(len(hacc_sorted)*0.95)]
print(f"Horizontal Accuracy: Min={min(hacc_list)*100:.1f} cm, Max={max(hacc_list)*100:.1f} cm, Mean={sum(hacc_list)/len(hacc_list)*100:.1f} cm")
print(f"  50th Percentile (CEP): {cep50*100:.1f} cm")
print(f"  95th Percentile (R95): {r95*100:.1f} cm")

valid_bl = [b for b in baseline_lens if b > 0.1]
if valid_bl:
    mean_bl = sum(valid_bl) / len(valid_bl)
    var_bl  = sum((b - mean_bl)**2 for b in valid_bl) / len(valid_bl)
    std_bl  = math.sqrt(var_bl)
    print(f"Dual-Antenna Baseline Length: Mean={mean_bl*100:.1f} cm (Design=55.5 cm, StdDev={std_bl*100:.2f} cm)")
print(f"Baseline Pitch Angle: Min={min(baseline_pitch):+.1f} deg, Max={max(baseline_pitch):+.1f} deg, Mean={sum(baseline_pitch)/len(baseline_pitch):+.1f} deg")

tot_dist = 0.0
for i in range(1, len(coords)):
    tot_dist += haversine_m(coords[i-1][0], coords[i-1][1], coords[i][0], coords[i][1])
print(f"Total Trajectory Length Walked: {tot_dist:.1f} m ({tot_dist*3.28084:.1f} ft)")

# 3. Analyze NMEA Stream
print("\n--- [3. RAW NMEA STREAM ANALYSIS] ---")
nmea_file = 'RTK_Rover/test/nmea_2026-10-26_15-47-30-240.log'
counts = {}
diff_ages = []
hdop_master = []
hdop_slave = []
sats_slave = []
corrupted = 0
total_lines = 0

with open(nmea_file, 'r', errors='ignore') as f:
    for line in f:
        total_lines += 1
        line_s = line.strip()
        if not line_s:
            continue
        if line_s.startswith('$') or line_s.startswith('#'):
            hdr = line_s.split(',')[0].split(';')[0]
            counts[hdr] = counts.get(hdr, 0) + 1

            if hdr == '$GNGGA':
                fields = line_s.split(',')
                if len(fields) >= 14 and fields[8] != '':
                    try:
                        hdop_master.append(float(fields[8]))
                    except ValueError:
                        pass
                if len(fields) >= 14 and fields[13] != '':
                    try:
                        diff_ages.append(float(fields[13]))
                    except ValueError:
                        pass
            elif hdr == '$GNGGAH':
                fields = line_s.split(',')
                if len(fields) >= 9:
                    try:
                        sats_slave.append(int(fields[7]))
                        hdop_slave.append(float(fields[8]))
                    except (ValueError, IndexError):
                        pass
        else:
            corrupted += 1

print(f"Total Stream Sentences Logged: {sum(counts.values()):,}")
print("Sentence Breakdown:")
for k, v in sorted(counts.items(), key=lambda x: -x[1]):
    print(f"  {k:16}: {v:6,d} sentences")

if diff_ages:
    print(f"RTCM Differential Age: Min={min(diff_ages):.1f}s, Max={max(diff_ages):.1f}s, Mean={sum(diff_ages)/len(diff_ages):.1f}s")
if hdop_master:
    print(f"Master Antenna HDOP:   Min={min(hdop_master):.2f}, Max={max(hdop_master):.2f}, Mean={sum(hdop_master)/len(hdop_master):.2f}")
if hdop_slave:
    print(f"Slave Antenna HDOP:    Min={min(hdop_slave):.2f}, Max={max(hdop_slave):.2f}, Mean={sum(hdop_slave)/len(hdop_slave):.2f}")
if sats_slave:
    print(f"Slave Antenna Sats:    Min={min(sats_slave)}, Max={max(sats_slave)}, Mean={sum(sats_slave)/len(sats_slave):.1f}")
print(f"Non-NMEA / Corrupted Fragments: {corrupted} ({(corrupted/max(1, total_lines))*100:.3f}%)")
print("=" * 70)
