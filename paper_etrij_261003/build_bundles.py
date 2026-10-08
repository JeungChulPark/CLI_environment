"""paper_etrij_261003 문서 묶기: 개별 문서(개별문서/NN_*.md)를 주제별 묶음(A~E) 하나씩의 .md/.html 로 이어 붙이고 index.html 을 만든다.
내용은 바꾸지 않고 제목 단계만 한 칸 내린다(# → ##). 개별 문서를 고친 뒤 `python3 build_bundles.py` 로 다시 생성."""
import re, pathlib, markdown, datetime
ROOT = pathlib.Path(__file__).resolve().parent; SRC = ROOT / "개별문서"
CSS = (ROOT / "style.css").read_text()
BUNDLES = [
 ("A_논문_기획_투고처_비교대상_목차", "논문 기획 — 투고처 검토, 추가 실험 계획, 비교 대상 논문, 목차, 재시작 메모, 논문 비교표 설계(17)",
  ["00_투고처_검토", "01_추가실험_계획", "02_비교대상_논문", "03_목차_v0.2", "08_재시작_메모", "17_논문_비교표_설계_개선안3_261008", "18_연휴실험_261008"]),
 ("B_한장평가_YCB-V_900장", "한 장 평가(YCB-V 900장·36장) — 기준 시스템 비교, 단계별 분해(L0→L4), 4090 확인 실행, 인식 방법 4가지, ViT-S 대 ViT-L, 개선안 3·군집 먼저 900장과 같은 물체끼리 ADD-S(16)",
  ["04_비교실험_결과", "06_단계별_분해실험", "09_4090_확인실행_261006", "11_인식방법_비교표_261007", "12_ViT-S_ViT-L_비교표_261007", "16_한장평가_개선안3_군집먼저_900장_261007"]),
 ("C_실시간_객체지도", "실시간 객체 지도 — 실시간 비교 실험(05), 실시간 비교표(08), 4090 재측정(10), 원본 0.5 대 개선안 전 장면(13), 4090 여섯 구성·검증 수정·군집 먼저(15)",
  ["05_실시간_지도_비교실험", "08_실시간_비교표", "10_실시간_비교표_261006", "13_실시간_원본_개선안_비교표_261007", "15_실시간_기술조합_4090_261007"]),
 ("D_전체_비교표_07", "전체 비교표(07) — 2026-10-05~07 노트북 실험 모음(한 장 평가 + 실시간 + 마스크·판정·검증 연구). 다른 묶음의 수치 출처",
  ["07_전체_비교표"]),
 ("E_기술_설명_원본_대_개선안", "방법 설명 — 원본 SAM-6D 와 우리 기술/개선안의 단계별 차이, 관문·독점 배정·배정 대안·PEM·자세 검증, 개선안 목록, 논문 논리",
  ["14_기술_상세_원본_대_개선안_261007"]),
]
def shift(md):
    out = []; fence = False
    for line in md.splitlines():
        if line.startswith("```"): fence = not fence
        if not fence and re.match(r"^#{1,5} ", line): line = "#" + line
        out.append(line)
    return "\n".join(out)
def html(md, title):
    body = markdown.markdown(md, extensions=["tables", "fenced_code", "toc"]).replace("<table>", '<div class="tw"><table>').replace("</table>", "</table></div>")
    return f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title><style>{CSS}</style></head><body><main>{body}</main></body></html>'
rows = []
for key, desc, parts in BUNDLES:
    chapters, toc = [], []
    for p in parts:
        f = SRC / f"{p}.md"
        if not f.exists(): print("missing", f); continue
        md = f.read_text(); title = md.splitlines()[0].lstrip("# ").strip()
        toc.append(f"- **{p}** — {title}")
        chapters.append(f"\n\n---\n\n<a id=\"{p}\"></a>\n\n" + shift(md))
    head = f"# {key.split('_',1)[1].replace('_',' ')}\n\n> {desc}. 개별 문서 {len(parts)}개를 그대로 이어 붙인 묶음입니다(제목 단계만 한 칸 내림). 원문은 `개별문서/` 폴더에 있습니다. 생성 {datetime.date.today()}.\n\n**이 묶음의 장**\n\n" + "\n".join(toc) + "\n\n[TOC]\n"
    md = head + "".join(chapters)
    (ROOT / f"{key}.md").write_text(md); (ROOT / f"{key}.html").write_text(html(md, key.replace("_", " ")))
    rows.append((key, desc, parts)); print("built", key, len(md.splitlines()), "lines")
li = "".join(f'<li><a href="{k}.html"><b>{k}</b></a> — {d}<br><small>{", ".join(p)}</small></li>' for k, d, p in rows)
others = sorted(x.name for x in SRC.glob("*.html"))
idx = f'''<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>paper_etrij_261003 문서 색인</title><style>{CSS}</style></head><body><main>
<h1>ETRI Journal 논문 문서 색인 (paper_etrij_261003)</h1>
<p>비슷한 실험을 묶은 다섯 문서입니다. 개별 문서 원문은 <a href="개별문서/">개별문서/</a> 에 있습니다. 갱신 {datetime.date.today()}.</p>
<h2>묶음</h2><ul>{li}</ul>
<h2>개별 문서 (원문)</h2><ul>{"".join(f'<li><a href="개별문서/{n}">{n}</a></li>' for n in others)}</ul>
<p>원고: <code>etrij/</code>, <code>ko/</code>, <code>manuscript_en.md</code> · 코드·결과: <code>src/</code></p></main></body></html>'''
(ROOT / "index.html").write_text(idx); print("index.html written")
