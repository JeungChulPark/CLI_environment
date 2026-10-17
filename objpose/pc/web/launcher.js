// launcher.js — mode bar injected by objpose/pc/launcher.py above the hub's page.
// Row 0: one button per RECORDING DATE (datasets grouped by their "date"); Row 1: one TAB per dataset of that date.
// Row 2: which dataset+backend(+feature count) is running, in the backend colour, and the backend buttons of
// the active tab + 멈춤. ORB-SLAM3 backends carry a feature-count row (f2000 / f3000 / f4000): each count is
// its own start button, so runs on .3 and on the Mac are distinguishable by host AND by feature count.
// Only one hub runs at a time, so starting anything stops the running one first.
(() => {
  const bar = document.getElementById("launcher-bar");
  if (!bar) return;
  const css = document.createElement("style");
  css.textContent = `
    #launcher-bar { display: flex; flex-wrap: wrap; align-items: center; gap: 10px 14px; margin: 0 0 12px; padding: 10px 14px;
      background: var(--panel, #131920); border: 1px solid var(--rule, #25303a); border-left: 6px solid var(--lb, #5f6b77); border-radius: 8px; }
    #launcher-bar .now { font-size: 15px; font-weight: 600; }
    #launcher-bar .now small { font-weight: 400; color: var(--muted, #95a1ad); font-size: 12px; margin-left: 8px; font-family: var(--mono, monospace); }
    #launcher-bar .tabs { display: flex; align-items: center; gap: 8px; flex-basis: 100%; flex-wrap: wrap;
      border-bottom: 1px solid var(--rule, #25303a); padding-bottom: 8px; }
    #launcher-bar .tabs .lbl { font-size: 12px; color: var(--muted, #95a1ad); }
    /* row 0: recording dates; row 1: the datasets of the chosen date */
    #launcher-bar .dates { display: flex; align-items: center; gap: 8px; flex-basis: 100%; flex-wrap: wrap; }
    #launcher-bar .dates .lbl { font-size: 12px; color: var(--muted, #95a1ad); }
    #launcher-bar .dates button { flex-direction: row; align-items: baseline; gap: 8px; padding: 5px 12px; border-radius: 999px; }
    #launcher-bar .dates button.date-on { background: var(--panel, #131920); border-color: var(--ink, #e6ebef); }
    #launcher-bar .dates button.date-on b { color: var(--ink, #e6ebef); }
    #launcher-bar .dates button small { font-size: 11px; }
    #launcher-bar .tabs { border-top: 1px dashed var(--rule, #25303a); padding-top: 8px; }
    #launcher-bar .tabs button { flex-direction: row; align-items: baseline; gap: 8px; padding: 6px 12px; }
    #launcher-bar .tabs button.tab-on { background: var(--panel, #131920); border-color: var(--ink, #e6ebef); }
    #launcher-bar .tabs button.tab-on b { color: var(--ink, #e6ebef); }
    #launcher-bar .tabs button .run-dot { color: var(--rc, #5f6b77); font-size: 11px; }
    #launcher-bar .modes { display: flex; gap: 8px; margin-left: auto; flex-wrap: wrap; align-items: stretch; }
    #launcher-bar button { font: inherit; font-size: 13px; padding: 7px 14px; border-radius: 6px; cursor: pointer; color: var(--ink, #e6ebef);
      background: var(--panel-2, #19212a); border: 1px solid var(--rule, #25303a); display: flex; flex-direction: column; align-items: flex-start; gap: 2px; }
    #launcher-bar button small { font-size: 11px; color: var(--muted, #95a1ad); font-family: var(--mono, monospace); }
    #launcher-bar button.on { border-color: var(--bc); box-shadow: inset 0 0 0 1px var(--bc); background: color-mix(in srgb, var(--bc) 14%, var(--panel-2, #19212a)); }
    #launcher-bar button.on b { color: var(--bc); }
    #launcher-bar button:disabled { opacity: .55; cursor: wait; }
    /* a backend with selectable feature counts: a card = title + one small button per count */
    #launcher-bar .grp { display: flex; flex-direction: column; gap: 5px; padding: 6px 10px 7px; border-radius: 6px;
      border: 1px solid var(--rule, #25303a); background: var(--panel-2, #19212a); }
    #launcher-bar .grp.on { border-color: var(--bc); box-shadow: inset 0 0 0 1px var(--bc); background: color-mix(in srgb, var(--bc) 10%, var(--panel-2, #19212a)); }
    #launcher-bar .grp .ttl { font-size: 13px; font-weight: 600; }
    #launcher-bar .grp.on .ttl { color: var(--bc); }
    #launcher-bar .grp .sub { font-size: 11px; color: var(--muted, #95a1ad); font-family: var(--mono, monospace); }
    #launcher-bar .grp .fs { display: flex; gap: 4px; }
    #launcher-bar .grp .fs button { padding: 3px 9px; font-size: 12px; font-family: var(--mono, monospace); flex-direction: row; gap: 4px; border-radius: 4px; }
    #launcher-bar .grp .fs button.def { border-style: dashed; }
    #launcher-bar .grp .fs button.on { border-style: solid; }
    #launcher-bar .note { font-size: 12px; color: var(--muted, #95a1ad); flex-basis: 100%; font-family: var(--mono, monospace); }
  `;
  document.head.appendChild(css);

  let loadedRun = null;
  let tab = null;                       // the dataset tab the user is looking at
  let dateTab = null;                   // the recording date the user is looking at
  const dateOf = (k, d) => (d && d.date) || (/^\d{6}_/.test(k) ? `20${k.slice(0, 2)}-${k.slice(2, 4)}-${k.slice(4, 6)}` : "기타");
  const KO_BUSY = { starting: "시작하는 중", stopping: "멈추는 중" };
  const esc = s => String(s).replace(/[<>&"]/g, c => ({ "<": "&lt;", ">": "&gt;", "&": "&amp;", '"': "&quot;" }[c]));
  const withF = (label, f, td) => (f ? `${label} f${f}` : label) + (td && td !== 40 ? ` ThDepth ${td}` : "");
  const tdSel = {};                     // per ORB card: the ThDepth the user picked (default = card default / running)

  async function refresh() {
    let s;
    try { s = await (await fetch("/api/state", { cache: "no-store" })).json(); } catch (e) { return; }
    if (loadedRun === null) loadedRun = s.run_id;
    else if (s.run_id !== loadedRun) { location.reload(); return; }

    const D = s.datasets, runDs = s.dataset || s.last_dataset || s.default_dataset;
    if (tab === null || !D[tab]) tab = runDs;                  // follow the running dataset until the user picks a tab
    const B = D[tab].backends, cur = s.backend && D[runDs] && D[runDs].backends[s.backend];
    bar.style.setProperty("--lb", cur ? cur.color : "#5f6b77");

    // row 0 — recording dates (datasets grouped by their "date"); the chosen date follows the active dataset tab
    const byDate = {};
    Object.entries(D).forEach(([k, d]) => { (byDate[dateOf(k, d)] = byDate[dateOf(k, d)] || []).push(k); });
    const dates = Object.keys(byDate).sort();
    if (dateTab === null || !byDate[dateTab] || !byDate[dateTab].includes(tab)) dateTab = dateOf(tab, D[tab]);
    const dateRow = `<span class="lbl">녹화 날짜</span>` + dates.map(dt => {
      const running = s.dataset && byDate[dt].includes(s.dataset), isOn = dateTab === dt;
      const dot = running && cur ? `<span class="run-dot" style="--rc:${cur.color};color:${cur.color}">●</span>` : "";
      return `<button data-date="${esc(dt)}" class="${isOn ? "date-on" : ""}"><b>${esc(dt)}</b><small>${byDate[dt].length}개</small>${dot}</button>`;
    }).join("");

    // row 1 — the datasets of the chosen date
    const tabs = `<span class="lbl">입력 데이터 (${esc(dateTab)})</span>` + byDate[dateTab].map(k => {
      const d = D[k], running = s.dataset === k, isOn = tab === k;
      const dot = running && cur ? `<span class="run-dot" style="--rc:${cur.color};color:${cur.color}">● 실행 중</span>` : "";
      return `<button data-tab="${esc(k)}" class="${isOn ? "tab-on" : ""}" title="${esc(d.detail || "")}"><b>${esc(d.label)}</b>${dot}</button>`;
    }).join("");

    // row 2 — what is running + the backend buttons of the active tab
    const now = cur
      ? `<span class="now" style="color:${cur.color}">실행 중: ${esc(D[runDs].label)} · ${esc(withF(cur.label, s.features, s.thdepth))}<small>${esc(cur.host)} · ${esc(cur.detail)}</small></span>`
      : `<span class="now" style="color:var(--muted,#95a1ad)">실행 중인 SLAM 없음${
          s.last_backend && D[runDs] && D[runDs].backends[s.last_backend]
            ? `<small>마지막: ${esc(D[runDs].label)} · ${esc(withF(D[runDs].backends[s.last_backend].label, s.last_features, s.last_thdepth))}${s.last_exit !== null ? " · 종료 코드 " + s.last_exit : ""}</small>` : ""}</span>`;
    // backends that share a "group" (LiDAR per host) render as ONE card with a button per algorithm
    const groups = {};
    Object.entries(B).forEach(([k, b]) => { if (b.group) (groups[b.group] = groups[b.group] || []).push([k, b]); });
    const seenGroup = new Set();
    const modes = Object.entries(B).map(([k, b]) => {
      const onB = s.dataset === tab && s.backend === k;
      if (b.group) {
        if (seenGroup.has(b.group)) return "";
        seenGroup.add(b.group);
        const members = groups[b.group];
        const onG = members.some(([mk]) => s.dataset === tab && s.backend === mk);
        const col = members[0][1].color;
        const algos = members.map(([mk, mb]) => {
          const on = s.dataset === tab && s.backend === mk;
          return `<button data-b="${esc(mk)}" class="${on ? "on" : ""}" style="--bc:${mb.color}" ${s.busy ? "disabled" : ""} title="${esc(mb.detail)}">${esc(mb.algo)}${on ? " ●" : ""}</button>`;
        }).join("");
        return `<span class="grp ${onG ? "on" : ""}" style="--bc:${col}" title="${esc(members[0][1].host)}"><span class="ttl">${esc(b.group)}</span><span class="sub">${esc(members[0][1].host)} · 알고리즘 선택</span><span class="fs">${algos}</span></span>`;
      }
      if (b.features) {
        // ORB-SLAM3: one small button per feature count (starts the run); below it a ThDepth toggle (40 = 3.0 m
        // yaml default, 70 = 5.2 m) that only selects — the feature button uses the selected value
        const key = `${tab}/${k}`;
        if (tdSel[key] === undefined) tdSel[key] = onB && s.thdepth ? s.thdepth : (b.default_thdepth || 40);
        const tdNow = tdSel[key];
        const fs = b.features.map(f => {
          const on = onB && s.features === f, def = f === b.default_features;
          return `<button data-b="${esc(k)}" data-f="${f}" class="${on ? "on" : ""}${def ? " def" : ""}" style="--bc:${b.color}" ${s.busy ? "disabled" : ""}
            title="${esc(b.label)} · 특징 ${f}${def ? " (기본)" : ""} · ThDepth ${tdNow}${on ? " · 실행 중 (누르면 다시 시작)" : ""}">f${f}${on ? " ●" : ""}</button>`;
        }).join("");
        const tds = (b.thdepth || []).map(td => {
          const sel = tdNow === td, running = onB && s.thdepth === td;
          return `<button data-td="${td}" data-key="${esc(key)}" class="${sel ? "on" : ""}${td === (b.default_thdepth || 40) ? " def" : ""}" style="--bc:${b.color}"
            title="Stereo.ThDepth ${td} (깊이 ${(td * 0.0745).toFixed(1)} m 안의 특징점을 3D 로 직접 사용)${running ? " · 실행 중" : ""}">ThDepth ${td}${running ? " ●" : ""}</button>`;
        }).join("");
        return `<span class="grp ${onB ? "on" : ""}" style="--bc:${b.color}" title="${esc(b.detail)}"><span class="ttl">${esc(b.label)}</span><span class="sub">${esc(b.host)} · 특징 수 = 시작 버튼 · ThDepth = 선택</span><span class="fs">${fs}</span><span class="fs">${tds}</span></span>`;
      }
      return `<button data-b="${esc(k)}" class="${onB ? "on" : ""}" style="--bc:${b.color}" ${s.busy ? "disabled" : ""} title="${esc(b.detail)}">
        <b>${esc(b.label)}</b><small>${esc(b.host)} · ${onB ? "실행 중 · 다시 시작" : "이것으로 바꾸기"}</small></button>`;
    }).join("");
    const stop = `<button data-b="" ${s.busy || !s.hub_alive ? "disabled" : ""}><b>멈춤</b><small>허브 종료 · summary 저장</small></button>`;
    const note = s.busy ? `<span class="note">${KO_BUSY[s.busy] || s.busy} — 한 번에 하나의 SLAM 만 실행합니다. 바꾸면 현재 실행을 먼저 멈춥니다.</span>`
                        : `<span class="note">날짜 → 입력 데이터셋 순으로 고릅니다 · ORB-SLAM3 는 ThDepth(40/70) 를 고른 뒤 f2000/f3000/f4000 버튼으로 시작하고, LiDAR 는 KISS-ICP / hdl_graph_slam 버튼으로 알고리즘을 골라 시작합니다(점선 = 기본값). 한 번에 하나의 SLAM 만 실행하며, 바꾸면 현재 실행을 먼저 멈추고(결과는 output 에 남음) 새로 시작합니다.${s.out ? " · " + esc(s.out.replace(/.*\/objpose\//, "objpose/")) : ""}</span>`;
    bar.innerHTML = `<span class="dates">${dateRow}</span><span class="tabs">${tabs}</span>` + now + `<span class="modes">${modes}${stop}</span>` + note;

    bar.querySelectorAll("button[data-date]").forEach(btn => btn.addEventListener("click", () => {
      dateTab = btn.dataset.date;        // pick a date: show its datasets, select the first (or the running one)
      const ks = byDate[dateTab] || [];
      tab = ks.includes(s.dataset) ? s.dataset : (ks[0] || tab);
      refresh();
    }));
    bar.querySelectorAll("button[data-tab]").forEach(btn => btn.addEventListener("click", () => {
      tab = btn.dataset.tab;             // switching the tab only changes which buttons are shown
      refresh();
    }));
    bar.querySelectorAll("span.modes button[data-td]").forEach(btn => btn.addEventListener("click", () => {
      tdSel[btn.dataset.key] = Number(btn.dataset.td);      // selection only; the next feature button uses it
      refresh();
    }));
    bar.querySelectorAll("span.modes button:not([data-td])").forEach(btn => btn.addEventListener("click", async () => {
      const b = btn.dataset.b, f = btn.dataset.f ? Number(btn.dataset.f) : null;
      const td = f !== null ? (tdSel[`${tab}/${b}`] || 40) : null;
      const here = `${D[tab].label} · ${b && B[b] ? withF(B[b].label, f, td) : ""}`;
      const same = b && s.dataset === tab && s.backend === b && (f === null || (s.features === f && (s.thdepth || 40) === td));
      if (same && !confirm(`${here} 을(를) 처음부터 다시 실행할까요?`)) return;
      if (b && s.backend && !same
          && !confirm(`${D[runDs].label} · ${cur ? withF(cur.label, s.features, s.thdepth) : s.backend} 실행을 멈추고 ${here} 을(를) 시작할까요?`)) return;
      btn.disabled = true;
      const q = b ? `/api/run?dataset=${encodeURIComponent(tab)}&backend=${encodeURIComponent(b)}${f ? "&features=" + f : ""}${td ? "&thdepth=" + td : ""}` : "/api/stop";
      await fetch(q, { method: "POST" });
      refresh();
    }));
  }
  refresh();
  setInterval(refresh, 1000);
})();
