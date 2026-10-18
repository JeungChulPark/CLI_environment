#!/usr/bin/env python3
"""md2html.py — turn a report written in Markdown into one self-contained HTML page (images inlined as data URIs,
table of contents from the ## headings, light/dark palette of the other html/ reports).

    python3 html/md2html.py SRC.md OUT.html [--title "..."]
Needs the `markdown` package (e.g. orbslam_ws/output/openloris/venv/bin/python).
"""
from __future__ import annotations
import argparse, base64, html, mimetypes, re
from pathlib import Path
import markdown

CSS = """
:root{--paper:#EDF1F6;--surface:#FFFFFF;--surface-2:#F6F9FC;--line:#C6D2DE;--line-soft:#DDE5EC;--ink:#11202F;--ink-2:#425568;
--muted:#5A6D80;--accent:#2a78d6;--accent-fill:#E3EEFA;--code:#EEF2F6;
--sans:"Pretendard Variable",Pretendard,"Apple SD Gothic Neo","Noto Sans KR","Malgun Gothic",-apple-system,"Helvetica Neue",Arial,sans-serif;
--mono:ui-monospace,SFMono-Regular,Menlo,"Cascadia Mono",Consolas,"D2Coding",monospace}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--paper:#0D1620;--surface:#15202C;--surface-2:#1A2734;--line:#28394A;
--line-soft:#223142;--ink:#E3EBF3;--ink-2:#B3C3D1;--muted:#90A4B6;--accent:#5A9FEA;--accent-fill:#16283B;--code:#1E2C3A}}
:root[data-theme="dark"]{--paper:#0D1620;--surface:#15202C;--surface-2:#1A2734;--line:#28394A;--line-soft:#223142;--ink:#E3EBF3;
--ink-2:#B3C3D1;--muted:#90A4B6;--accent:#5A9FEA;--accent-fill:#16283B;--code:#1E2C3A}
*{box-sizing:border-box}html{-webkit-text-size-adjust:100%}
body{margin:0;background:var(--paper);color:var(--ink);font:15px/1.7 var(--sans)}
.wrap{max-width:1180px;margin:0 auto;padding:28px 16px 64px;display:grid;grid-template-columns:230px minmax(0,1fr);gap:28px}
nav{position:sticky;top:16px;align-self:start;font-size:13px;max-height:calc(100vh - 32px);overflow:auto}
nav b{display:block;color:var(--muted);font-weight:600;margin-bottom:6px}
nav a{display:block;color:var(--ink-2);text-decoration:none;padding:3px 8px;border-left:2px solid var(--line-soft)}
nav a:hover{color:var(--accent);border-left-color:var(--accent)}
nav .back{margin-bottom:12px;border:0;padding-left:0;color:var(--accent)}
main{background:var(--surface);border:1px solid var(--line-soft);border-radius:10px;padding:28px 36px;min-width:0}
h1{font-size:26px;line-height:1.35;margin:0 0 18px;padding-bottom:14px;border-bottom:2px solid var(--line)}
h2{font-size:20px;margin:40px 0 12px;padding-top:6px;border-top:1px solid var(--line-soft)}
h3{font-size:16.5px;margin:26px 0 8px;color:var(--ink-2)}
p,li{color:var(--ink)}strong{color:var(--ink)}a{color:var(--accent)}
code{font:13px/1.5 var(--mono);background:var(--code);padding:1px 5px;border-radius:4px;word-break:break-word}
pre{background:var(--code);padding:12px 14px;border-radius:8px;overflow:auto}pre code{background:none;padding:0}
.tbl{overflow-x:auto;margin:12px 0 18px;border:1px solid var(--line-soft);border-radius:8px}
table{border-collapse:collapse;width:100%;font-size:13.5px;font-variant-numeric:tabular-nums}
th{background:var(--surface-2);color:var(--ink-2);font-weight:600;text-align:left;white-space:nowrap}
th,td{padding:7px 10px;border-bottom:1px solid var(--line-soft);vertical-align:top}
tr:last-child td{border-bottom:0}td strong{color:var(--accent)}
img{max-width:100%;height:auto;border:1px solid var(--line-soft);border-radius:8px;background:#fff;margin:8px 0}
blockquote{margin:12px 0;padding:8px 14px;border-left:3px solid var(--accent);background:var(--accent-fill);border-radius:0 6px 6px 0}
.meta{color:var(--muted);font-size:13px;margin:-8px 0 18px}
@media (max-width:860px){.wrap{grid-template-columns:1fr}nav{position:static;max-height:none}main{padding:20px 16px}}
"""


def slug(s, used):
    b = re.sub(r"[^\w가-힣]+", "-", s).strip("-").lower() or "s"
    k, n = b, 2
    while k in used:
        k = f"{b}-{n}"; n += 1
    used.add(k)
    return k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("out"); ap.add_argument("--title", default="")
    a = ap.parse_args()
    src = Path(a.src); text = src.read_text()

    def inline(m):
        alt, path = m.group(1), m.group(2)
        f = (src.parent / path)
        if not f.exists():
            return m.group(0)
        mt = mimetypes.guess_type(f.name)[0] or "image/png"
        return f"![{alt}](data:{mt};base64,{base64.b64encode(f.read_bytes()).decode()})"
    text = re.sub(r"!\[([^\]]*)\]\(([^)\s]+)\)", inline, text)
    body = markdown.markdown(text, extensions=["tables", "fenced_code", "sane_lists"])
    body = body.replace("<table>", '<div class="tbl"><table>').replace("</table>", "</table></div>")
    used, toc = set(), []

    def head(m):
        lvl, inner = m.group(1), m.group(2)
        k = slug(re.sub("<[^>]+>", "", inner), used)
        if lvl == "2":
            toc.append((k, re.sub("<[^>]+>", "", inner)))
        return f'<h{lvl} id="{k}">{inner}</h{lvl}>'
    body = re.sub(r"<h([23])>(.*?)</h\1>", head, body)
    h1 = re.search(r"<h1>(.*?)</h1>", body)
    title = a.title or (re.sub("<[^>]+>", "", h1.group(1)) if h1 else src.stem)
    nav = "".join(f'<a href="#{k}">{html.escape(html.unescape(t))}</a>' for k, t in toc)
    page = f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(html.unescape(title))}</title><style>{CSS}</style></head>
<body><div class="wrap"><nav><a class="back" href="261018_index.html">← 문서 목록</a><b>목차</b>{nav}</nav>
<main>{body}<p class="meta">원본: <code>{html.escape(str(src))}</code></p></main></div></body></html>
"""
    Path(a.out).write_text(page)
    print(f"{a.out}: {len(page) // 1024} KB, {len(toc)} sections")


if __name__ == "__main__":
    main()
