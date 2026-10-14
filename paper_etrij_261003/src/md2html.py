"""Local Markdown → single-file HTML (images embedded, tables scrollable, dark mode)."""
import base64, re, sys
from pathlib import Path
import markdown
CSS = Path(__file__).with_name('md2html.css').read_text(encoding='utf-8')
src = Path(sys.argv[1]); out = src.with_suffix('.html')
L = src.read_text(encoding='utf-8').split('\n'); fix = []
for l in L:   # Markdown needs a blank line before a list that follows a paragraph
    if re.match(r'\s*(- |\d+\. )', l) and fix and fix[-1].strip() and not re.match(r'\s*(- |\d+\. )', fix[-1]) and not fix[-1].startswith('|'):
        fix.append('')
    fix.append(l)
body = markdown.markdown('\n'.join(fix), extensions=['tables', 'fenced_code'])
body = re.sub(r'src="(images/[^"]+\.(png|jpg))"', lambda m: f'src="data:image/{"jpeg" if m.group(2)=="jpg" else "png"};base64,'
              + base64.b64encode((src.parent / m.group(1)).read_bytes()).decode() + '"', body)
body = re.sub(r'(?<!["=])(https?://[^\s<)]+)', r'<a href="\1">\1</a>', body)  # bare URLs → links
body = re.sub(r'(?<![\w/])ax:(\d{4}\.\d{4,5})', r'<a href="https://arxiv.org/abs/\1">arXiv:\1</a>', body)
body = body.replace('<table>', '<div class="tw"><table>').replace('</table>', '</table></div>')
title = re.search(r'^# (.+)$', src.read_text(encoding='utf-8'), re.M)
out.write_text(f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
               f'<title>{title.group(1) if title else src.stem}</title><style>{CSS}</style></head><body><main>{body}</main></body></html>', encoding='utf-8')
print(out)
