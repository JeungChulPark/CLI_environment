#!/usr/bin/env python3
"""analyze_clip_prompts.py — PLY 렌더 기반 CLIP 프롬프트 판별력 분석 (READ-ONLY, 학습 0).

YOLO-World 가 실제로 쓰는 텍스트 인코더(clip:ViT-B/32)를 그대로 사용해,
onboarding 산출물(PLY 렌더 42뷰)만으로 다음을 계산한다. **사람 GT 미사용.**

  1) 현행 10개 프롬프트의 self/other 유사도와 margin
     margin(t) = sim(t, 자기 렌더) - max_j≠self sim(t, 다른 객체 렌더)
     → margin 이 작거나 음수면 그 프롬프트는 혼동을 일으킨다(가설 검증)
  2) 현행 프롬프트가 예측하는 '혼동 상대'가 사람이 확인한 혼동과 일치하는가
  3) 후보 문구 사전(사전 등록)에 대해 margin 최대 문구를 자동 선택 → v3 후보

산출: outputs/phase27_clip_prompt/{csv,metrics}/  (웹 뷰어가 이걸 읽는다)
"""
import argparse, csv, glob, json, os, sys
import numpy as np
import torch
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n
OUT = os.path.join(REPO, "outputs", "phase27_clip_prompt")
VOCAB = os.path.join(HERE, "candidate_vocab_preregistered.json")


def embed(model, objs, device, n_view=42):
    """객체별 렌더 이미지 CLIP 임베딩 (평균 + 전체)."""
    emb = {}
    for o in objs:
        ps = sorted(glob.glob(os.path.join(o["template_dir"], "rgb_*.png")))[:n_view]
        ims = [model.image_preprocess(Image.open(p).convert("RGB")) for p in ps]
        with torch.no_grad():
            f = model.model.encode_image(torch.stack(ims).to(device)).float()
        f = torch.nn.functional.normalize(f, dim=-1)
        emb[o["name"]] = f.cpu()
    return emb


def text_emb(model, texts, device):
    with torch.no_grad():
        f = model.encode_text(model.tokenize(list(texts))).float()
    return torch.nn.functional.normalize(f, dim=-1).cpu()


def margins(temb, iemb, names):
    """각 문구 × 각 객체 유사도(렌더 평균) → self/other/margin."""
    M = np.stack([iemb[n].mean(0).numpy() for n in names])       # [obj, D]
    M = M / np.linalg.norm(M, axis=1, keepdims=True)
    S = temb.numpy() @ M.T                                        # [text, obj]
    return S


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--n-view", type=int, default=42); a = ap.parse_args()
    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    from ultralytics.nn.text_model import build_text_model
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = build_text_model("clip:ViT-B/32", device)

    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    names = [o["name"] for o in objs]
    cur = {o["name"]: o["yolo_prompt"] for o in objs}
    print(f"CLIP ViT-B/32 · 객체 {len(names)} · 렌더 {a.n_view}뷰")
    iemb = embed(model, objs, device, a.n_view)

    # ---------- 1) 현행 프롬프트 margin ----------
    curtxt = [cur[n] for n in names]
    S = margins(text_emb(model, curtxt, device), iemb, names)     # [10 text, 10 obj]
    rows = []
    for i, n in enumerate(names):
        self_s = S[i, i]
        others = [(names[j], S[i, j]) for j in range(len(names)) if j != i]
        others.sort(key=lambda x: -x[1])
        rows.append({"object": n, "prompt": cur[n], "sim_self": round(float(self_s), 4),
                     "top_confusable": others[0][0], "sim_other": round(float(others[0][1]), 4),
                     "margin": round(float(self_s - others[0][1]), 4),
                     "rank_of_self": int(1 + sum(1 for j in range(len(names)) if S[i, j] > self_s)),
                     "second_confusable": others[1][0], "sim_second": round(float(others[1][1]), 4)})
    rows.sort(key=lambda r: r["margin"])
    with open(os.path.join(OUT, "csv", "current_prompt_margins.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    # 전체 유사도 행렬
    with open(os.path.join(OUT, "csv", "similarity_matrix.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["prompt_of"] + names)
        for i, n in enumerate(names):
            w.writerow([n] + [round(float(x), 4) for x in S[i]])

    print("\n=== 현행 프롬프트 판별력 (margin 낮은 순 = 위험) ===")
    for r in rows:
        flag = "  ⚠자기가 1위 아님" if r["rank_of_self"] > 1 else ""
        print(f"  {r['object']:22s} '{r['prompt']:26s}' self {r['sim_self']:.3f} "
              f"vs {r['top_confusable']:20s} {r['sim_other']:.3f}  margin {r['margin']:+.4f}{flag}")

    # ---------- 2) 사람 확인 혼동과 대조 ----------
    actual = {}
    p = os.path.join(REPO, "outputs/phase24_localization/csv/localization_confusion_pairs.csv")
    if os.path.isfile(p):
        for r in csv.DictReader(open(p)):
            t, g, c = r["target_class"], r["actually_detected"], int(r["count"])
            if g in names and (t not in actual or c > actual[t][1]):
                actual[t] = (g, c)
    hit = 0; comp = []
    for r in rows:
        act = actual.get(r["object"])
        ok = bool(act) and act[0] == r["top_confusable"]
        hit += ok
        comp.append({"object": r["object"], "clip_predicts": r["top_confusable"],
                     "human_observed": act[0] if act else "", "count": act[1] if act else 0,
                     "match": int(ok), "margin": r["margin"]})
    with open(os.path.join(OUT, "csv", "confusion_prediction.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(comp[0].keys())); w.writeheader(); w.writerows(comp)
    n_have = sum(1 for c in comp if c["human_observed"])
    print(f"\n=== CLIP 예측 혼동상대 vs 사람 확인 : {hit}/{n_have} 일치 ===")
    for c in comp:
        if c["human_observed"]:
            print(f"  {c['object']:22s} CLIP→{c['clip_predicts']:20s} 사람→{c['human_observed']:20s} "
                  f"{'MATCH' if c['match'] else ''}")

    # ---------- 3) 후보 문구 사전 → margin 최대 자동 선택 ----------
    if os.path.isfile(VOCAB):
        vocab = json.load(open(VOCAB))
        best = []
        for n in names:
            cands = vocab.get(n, [])
            if not cands:
                continue
            T = text_emb(model, cands, device)
            Sv = margins(T, iemb, names)
            si = names.index(n)
            recs = []
            for k, t in enumerate(cands):
                self_s = Sv[k, si]
                om = max(Sv[k, j] for j in range(len(names)) if j != si)
                oj = max(((names[j], Sv[k, j]) for j in range(len(names)) if j != si), key=lambda x: x[1])[0]
                recs.append((t, float(self_s), float(om), float(self_s - om), oj))
            recs.sort(key=lambda x: -x[3])
            for rk, (t, s, om, mg, oj) in enumerate(recs, 1):
                best.append({"object": n, "rank": rk, "candidate": t, "sim_self": round(s, 4),
                             "sim_other_max": round(om, 4), "margin": round(mg, 4),
                             "closest_other": oj, "is_current": int(t == cur[n])})
        with open(os.path.join(OUT, "csv", "candidate_ranking.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(best[0].keys())); w.writeheader(); w.writerows(best)
        chosen = {}
        for n in names:
            rs = [b for b in best if b["object"] == n]
            if rs:
                chosen[n] = rs[0]
        with open(os.path.join(OUT, "csv", "prompts_v3_selected.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(["object", "current", "v3_selected", "margin_current", "margin_v3", "gain"])
            for n in names:
                cm = next((b["margin"] for b in best if b["object"] == n and b["is_current"]), None)
                c3 = chosen.get(n)
                if c3:
                    w.writerow([n, cur[n], c3["candidate"],
                                cm if cm is not None else "", c3["margin"],
                                round(c3["margin"] - cm, 4) if cm is not None else ""])
        print("\n=== margin 최대 문구 자동 선택 (v3 후보) ===")
        for n in names:
            c3 = chosen.get(n); cm = next((b["margin"] for b in best if b["object"] == n and b["is_current"]), None)
            if c3:
                g = f"{c3['margin']-cm:+.4f}" if cm is not None else "n/a"
                print(f"  {n:22s} '{c3['candidate'][:40]:40s}' margin {c3['margin']:+.4f} ({g})")
    else:
        print(f"\n[skip] 후보 사전 없음: {VOCAB}")

    json.dump({"encoder": "clip:ViT-B/32 (YOLO-World 동일)", "n_view": a.n_view,
               "confusion_pred_match": f"{hit}/{n_have}",
               "note": "GT 미사용 · onboarding 렌더만 사용 (INV-01 준수)"},
              open(os.path.join(OUT, "metrics", "clip_analysis.json"), "w"), indent=2, ensure_ascii=False)
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()
