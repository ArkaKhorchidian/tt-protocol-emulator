import re,sys
t=open(sys.argv[1]).read()
t=t[t.rfind('Printing statistics'):]
for m in re.finditer(r"=== (\S+) ===.*?Chip area for module '([^']+)': ([\d.]+)",t,re.S):
    print(f"{m.group(1)[-40:]:42s} {float(m.group(3)):10.0f}")
m=re.search(r"design hierarchy ===.*?Chip area for top module '[^']+': ([\d.]+)",t,re.S)
if m: print("TOTAL", m.group(1))
