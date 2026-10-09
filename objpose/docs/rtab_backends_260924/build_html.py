#!/usr/bin/env python3
"""rtab_backends_260924.md -> rtab_backends_260924.html (one file, images embedded).

  python3 build_html.py
"""
import base64, re
from pathlib import Path
import markdown

HERE = Path(__file__).resolve().parent
SRC = HERE / "rtab_backends_260924.md"
OUT = HERE / "rtab_backends_260924.html"

CSS = """
:root { --bg:#ffffff; --fg:#1d2127; --muted:#5b6470; --line:#dfe3e8; --head:#f4f6f8; --accent:#1f6feb;
        --code:#f1f3f5; --hi:#fff5d6; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#12161c; --fg:#e4e8ee; --muted:#9aa4b1; --line:#2c333d; --head:#1a2029; --accent:#58a6ff;
          --code:#1f2630; --hi:#3a3212; } }
* { box-sizing: border-box; }
body { margin:0; background:var(--bg); color:var(--fg);
       font: 16px/1.7 "Pretendard","Noto Sans KR","Malgun Gothic","Apple SD Gothic Neo",system-ui,sans-serif; }
main { max-width: 980px; margin: 0 auto; padding: 40px 20px 80px; }
h1 { font-size: 1.9rem; line-height: 1.3; margin: 0 0 6px; }
h1 + p { color: var(--muted); margin-top: 0; }
h2 { font-size: 1.35rem; margin: 48px 0 12px; padding-top: 16px; border-top: 1px solid var(--line); }
h3 { font-size: 1.08rem; margin: 28px 0 8px; }
a { color: var(--accent); }
code { font: 0.88em/1.4 "D2Coding","JetBrains Mono",Consolas,monospace; background: var(--code);
       padding: 1px 5px; border-radius: 4px; }
.table-wrap { overflow-x: auto; margin: 14px 0 18px; border: 1px solid var(--line); border-radius: 8px; }
table { border-collapse: collapse; width: 100%; font-size: 0.92rem; }
th, td { padding: 8px 12px; border-bottom: 1px solid var(--line); text-align: left; vertical-align: top; }
th { background: var(--head); font-weight: 600; white-space: nowrap; }
tr:last-child td { border-bottom: 0; }
td:first-child { white-space: nowrap; }
td strong { background: var(--hi); padding: 0 3px; border-radius: 3px; }
figure { margin: 18px 0 8px; }
figure img { width: 100%; height: auto; border: 1px solid var(--line); border-radius: 8px; display: block; }
figcaption { color: var(--muted); font-size: 0.88rem; margin-top: 6px; }
ul.tasks { list-style: none; padding-left: 4px; }
ul.tasks li { display: flex; gap: 10px; align-items: baseline; }
ul.tasks input { margin: 0; transform: translateY(2px); }
li { margin: 4px 0; }
nav.toc { border: 1px solid var(--line); border-radius: 8px; padding: 12px 18px; margin: 20px 0 8px;
          background: var(--head); font-size: 0.93rem; }
nav.toc ol { margin: 6px 0 0; padding-left: 20px; }
@media print { body { font-size: 11pt; } h2 { break-after: avoid; } figure, .table-wrap { break-inside: avoid; } }
"""


def embed(m):
    alt, path = m.group(1), m.group(2)
    p = HERE / path
    data = base64.b64encode(p.read_bytes()).decode()
    return (f'<figure><img alt="{alt}" src="data:image/png;base64,{data}">'
            f"<figcaption>{alt}</figcaption></figure>")


def main():
    md = SRC.read_text()
    title = md.split("\n", 1)[0].lstrip("# ").strip()
    html = markdown.markdown(md, extensions=["tables", "fenced_code", "toc", "sane_lists"])
    # images on their own line come out as <p><img ...></p>; turn them into captioned figures
    html = re.sub(r'<p><img alt="([^"]*)" src="([^"]+)" ?/?></p>', embed, html)
    html = html.replace("<table>", '<div class="table-wrap"><table>').replace("</table>", "</table></div>")
    # GitHub task lists
    html = re.sub(r"<ul>\n(<li>\[[ x]\].*?)</ul>", lambda m: '<ul class="tasks">\n' + m.group(1) + "</ul>",
                  html, flags=re.S)
    html = re.sub(r"<li>\[ \] ", '<li><input type="checkbox" disabled> <span>', html)
    html = re.sub(r'(<li><input type="checkbox" disabled> <span>.*?)</li>', r"\1</span></li>", html, flags=re.S)
    # table of contents from the h2 headings
    heads = re.findall(r'<h2 id="([^"]+)">(.*?)</h2>', html)
    toc = '<nav class="toc"><strong>목차</strong><ol>' + "".join(
        f'<li><a href="#{i}">{t}</a></li>' for i, t in heads) + "</ol></nav>"
    html = re.sub(r"(<h1[^>]*>.*?</h1>\s*<p>.*?</p>)", r"\1" + toc, html, count=1, flags=re.S)
    OUT.write_text(f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title><style>{CSS}</style></head>
<body><main>
{html}
</main></body></html>
""")
    print(OUT, f"{OUT.stat().st_size/1e6:.1f} MB")


if __name__ == "__main__":
    main()
