"""Build the white paper .docx from Induced_Vibration_White_Paper.md.
Display equations ($$...$$ {#label}) are numbered in order at the right margin;
(@label) in the text is replaced with the equation number."""
import re, subprocess, zipfile, sys
src = 'Induced_Vibration_White_Paper.md'
out = sys.argv[1] if len(sys.argv) > 1 else 'Induced_Vibration_White_Paper.docx'
t = open(src).read()

# number equations
labels, n = {}, 0
def num(m):
    global n
    n += 1
    if m.group(2): labels[m.group(2)] = n
    return '$$' + m.group(1) + '$$'
t = re.sub(r'\$\$(.+?)\$\$(?:\s*\{#(\w+)\})?', num, t, flags=re.S)
missing = set(re.findall(r'\(@(\w+)\)', t)) - set(labels)
if missing: sys.exit('unknown equation labels: %s' % missing)
t = re.sub(r'\(@(\w+)\)', lambda m: '(%d)' % labels[m.group(1)], t)

# upright multi-letter subscripts in math
words = 'ref|struct|rotor|cm|max|ripple|cog|gyro|total|rms|up|down|lat|ax|brg'
def math_fix(m):
    s = re.sub(r'_\{([^{}]*)\}', lambda k: '_{' + re.sub(r'\b(' + words + r')\b', r'\\mathrm{\1}', k.group(1)) + '}', m.group(1))
    return '$$' + s + '$$'
t = re.sub(r'\$\$(.+?)\$\$', math_fix, t, flags=re.S)

# subscripts in prose (X_ab -> X~ab~), outside math and image paths
parts = re.split(r'(\$\$.+?\$\$|!\[.*?\]\(.*?\)\{.*?\})', t, flags=re.S)
for i in range(0, len(parts), 2):
    parts[i] = re.sub(r'(?<![\w~])([A-Za-zΩθαωΣ]+|Ω̇)_([A-Za-z0-9⊥]+(?:,[A-Za-z]+)?)', r'\1~\2~', parts[i])
t = ''.join(parts)
open('_build.md', 'w').write(t)

subprocess.run(['pandoc', '_build.md', '-o', '_raw.docx'], check=True)
z = zipfile.ZipFile('_raw.docx'); files = {k: z.read(k) for k in z.namelist()}; z.close()
x = files['word/document.xml'].decode()
width = 9360
count = [0]
def rep(m):
    om = re.search(r'(<m:oMath>.*</m:oMath>)', m.group(0), re.S).group(1)
    count[0] += 1
    return ('<w:p><w:pPr><w:tabs><w:tab w:val="center" w:pos="%d"/><w:tab w:val="right" w:pos="%d"/></w:tabs>'
            '<w:spacing w:before="120" w:after="120"/></w:pPr><w:r><w:tab/></w:r>%s'
            '<w:r><w:tab/><w:t>(%d)</w:t></w:r></w:p>') % (width // 2, width, om, count[0])
x = re.sub(r'<w:p>(?:(?!<w:p>).)*?<m:oMathPara>.*?</m:oMathPara>.*?</w:p>', rep, x, flags=re.S)
if count[0] != n: sys.exit('equation count mismatch %d vs %d' % (count[0], n))
files['word/document.xml'] = x.encode()
with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as o:
    for k, v in files.items(): o.writestr(k, v)
print('built', out, 'with', n, 'numbered equations')
