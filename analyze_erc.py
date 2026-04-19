import json

d = json.load(open(r'f:\OneDrive\Code\mobile_spl\hardware\kicad\erc.json'))
viols = [v for s in d['sheets'] for v in s.get('violations', [])]

print("=== ERROR violations ===")
for v in viols:
    if v.get('severity') == 'error':
        print(f"\nType: {v['type']}")
        print(f"Desc: {v.get('description', '')}")
        for item in v.get('items', []):
            pos = item.get('pos', {})
            x = round(pos.get('x', 0) * 100, 2)
            y = round(pos.get('y', 0) * 100, 2)
            print(f"  item: {item.get('description', '')} @ ({x}, {y})")
