import sys


def extract_symbol(lib_path, sym_name):
    content = open(lib_path, encoding='utf-8').read()
    start = content.find(f'(symbol "{sym_name}"')
    if start == -1:
        return None
    depth = 0
    i = start
    while i < len(content):
        if content[i] == '(':
            depth += 1
        elif content[i] == ')':
            depth -= 1
            if depth == 0:
                return content[start:i+1]
        i += 1
    return None


device_lib = r'E:\Apps\KiCad\10.0\share\kicad\symbols\Device.kicad_sym'
conn_lib = r'E:\Apps\KiCad\10.0\share\kicad\symbols\Connector_Generic.kicad_sym'
power_lib = r'E:\Apps\KiCad\10.0\share\kicad\symbols\power.kicad_sym'
my_lib = r'f:\OneDrive\Code\mobile_spl\hardware\kicad\mobile_spl_lib.kicad_sym'

syms = [
    (device_lib, 'R'),
    (device_lib, 'C'),
    (device_lib, 'SW_Push'),
    (power_lib, 'GND'),
    (power_lib, '+3V3'),
    (conn_lib, 'Conn_01x06'),
    (conn_lib, 'Conn_01x08'),
    (conn_lib, 'Conn_01x10'),
    (my_lib, 'OPA2376'),
    (my_lib, 'SPH8878LR5H-1'),
]

out = []
for lib, name in syms:
    s = extract_symbol(lib, name)
    if s:
        print(f'FOUND: {name} ({len(s)} chars)', file=sys.stderr)
        out.append(s)
    else:
        print(f'NOT FOUND: {name}', file=sys.stderr)

print('(lib_symbols')
for s in out:
    # indent 2 spaces
    for line in s.splitlines():
        print('  ' + line)
print(')')
