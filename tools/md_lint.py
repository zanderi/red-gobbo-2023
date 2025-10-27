import re
import sys

path = 'README.md'
with open(path, 'r', encoding='utf-8') as f:
    txt = f.read()

issues = []
# check for unclosed code fences
opens = re.findall(r"```", txt)
if len(opens) % 2 != 0:
    issues.append('Unbalanced code fences (```).')
# check for trailing spaces
for i, line in enumerate(txt.splitlines(), start=1):
    if line.endswith(' '):
        issues.append(f'Trailing space on line {i}')
# check for lines longer than 120
for i, line in enumerate(txt.splitlines(), start=1):
    if len(line) > 120:
        issues.append(f'Long line (>120 chars) at {i} ({len(line)} chars)')
# check header starts with single blank line before
lines = txt.splitlines()
for i, line in enumerate(lines):
    if line.startswith('#'):
        if i>0 and lines[i-1].strip() != '':
            issues.append(f'Header without preceding blank line at {i+1}: "{line}"')

print('MD Lint report for', path)
if not issues:
    print('No issues found.')
    sys.exit(0)
else:
    for it in issues:
        print('-', it)
    print('\nOffending lines (number: length):')
    for i,l in enumerate(open(path, encoding='utf-8').read().splitlines()):
        if l.endswith(' ') or len(l) > 120:
            print(f"{i+1}: {len(l)}: {l}")
    sys.exit(2)
