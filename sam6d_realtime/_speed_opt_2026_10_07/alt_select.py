"""drop-in sequential_candidate_select: same decisions and the same rows, but every tensor crosses to the CPU once
instead of one .item() per value (300 candidates x ~12 values x objects = thousands of GPU syncs per frame)."""
import math, os, time, collections
import numpy as np, torch
STATS = collections.Counter()


def make(mu):
    _rounded_finite, _sym_group, _pairwise_angle_deg = mu._rounded_finite, mu._sym_group, mu._pairwise_angle_deg

    def fast(pred_rs, pred_ts_m, geo_scores, texture_scores, shape, names, ver, proposal_ids=None):
        B, P = geo_scores.shape
        mask_min = float(ver.get('mask_iou_min', ver.get('iou_min', 0.420998)))
        texture_min = float(ver.get('texture_min_score', 0.449562))
        rot_limit = float(ver.get('cluster_rotation_deg', 20.0))
        trans_limit_m = float(ver.get('cluster_translation_mm', 25.0)) / 1000.0
        occupancy_min = float(ver.get('cluster_min_occupancy', 0.5))
        sym_tab = ver.get('symmetry_axes') or {}
        sym_step = int(ver.get('sym_step_deg', 10))
        selected_out, rows = [], []
        for b in range(B):
            geo_order = torch.argsort(geo_scores[b], descending=True, stable=True)
            rank_geo = torch.empty(P, dtype=torch.long, device=geo_scores.device)
            rank_geo[geo_order] = torch.arange(P, device=geo_scores.device)
            R = pred_rs[b]; t = pred_ts_m[b]
            pose_valid = (torch.isfinite(R).flatten(1).all(1) & torch.isfinite(t).flatten(1).all(1) & torch.isfinite(geo_scores[b]))
            pose_valid &= (torch.linalg.norm(R.transpose(-1, -2) @ R - torch.eye(3, device=R.device, dtype=R.dtype),
                                             dim=(-2, -1)) <= 1e-2) & (torch.linalg.det(R.float()) > 0.99)
            if shape is None:
                projection_valid = torch.zeros(P, dtype=torch.bool, device=R.device)
                mask_iou = torch.full((P,), float('nan'), device=R.device)
            else:
                projection_valid = shape['projection_valid'][b].bool(); mask_iou = shape['mask_iou'][b]
            mask_pass = pose_valid & projection_valid & torch.isfinite(mask_iou) & (mask_iou >= mask_min)
            texture_pass = (mask_pass & torch.isfinite(texture_scores[b]) & (texture_scores[b] >= texture_min))
            survivors = torch.where(texture_pass)[0]
            # one transfer for everything the decision and the rows need
            geo_order_l = geo_order.tolist(); rank_l = rank_geo.tolist()
            pv_l, prv_l, mp_l, tp_l = pose_valid.tolist(), projection_valid.tolist(), mask_pass.tolist(), texture_pass.tolist()
            geo_l, miou_l, tex_l = geo_scores[b].tolist(), mask_iou.tolist(), texture_scores[b].tolist()
            surv_l = survivors.tolist()
            accepted, reason, occupancy, cluster_l, center = False, None, 0.0, [], None
            if not any(a and c for a, c in zip(pv_l, prv_l)):
                reason = 'candidate_projection_invalid'
            elif not any(mp_l):
                reason = 'mask_filter_empty'
            elif not any(tp_l):
                reason = 'texture_filter_empty'
            elif len(surv_l) == 1:
                cluster_l, center, occupancy, accepted = surv_l, surv_l[0], 1.0, True
            else:
                sym = _sym_group(sym_tab.get(names[b]), sym_step, R.device, R.dtype)
                angles = _pairwise_angle_deg(R, sym)
                translations = torch.cdist(t, t)
                near = ((angles[survivors][:, survivors] <= rot_limit)
                        & (translations[survivors][:, survivors] <= trans_limit_m)).cpu().numpy()
                counts = near.sum(1)
                # largest neighbourhood first; ties -> the better geometry-ranked centre (ranks are unique)
                best = max(range(len(surv_l)), key=lambda i: (int(counts[i]), -rank_l[surv_l[i]]))
                center = surv_l[best]
                cluster_l = [s for s, m in zip(surv_l, near[best]) if m]
                occupancy = float(len(cluster_l)) / float(len(surv_l))
                accepted = occupancy >= occupancy_min
                if not accepted:
                    reason = 'convergence_insufficient'
            chosen = geo_order_l[0]
            if accepted:
                chosen = min(cluster_l, key=lambda c: rank_l[c])
            selected_out.append(chosen)
            prop_l = list(range(P)) if proposal_ids is None else proposal_ids[b].tolist()
            have_cov = shape is not None and 'coverage' in shape
            if have_cov:
                cov_l, sr_l, area_l = shape['coverage'][b].tolist(), shape['size_ratio'][b].tolist(), shape['rendered_mask_area_px'][b].tolist()
            member = set(cluster_l)
            candidates = []
            for original in geo_order_l:
                row = {'rank_geo': rank_l[original], 'index300': original, 'proposal6000_index': int(prop_l[original]),
                       'geometry_score': _rounded_finite(geo_l[original], 8),
                       'mask_iou': _rounded_finite(miou_l[original], 6) if math.isfinite(miou_l[original]) else None,
                       'texture_score': _rounded_finite(tex_l[original], 8) if math.isfinite(tex_l[original]) else None,
                       'pose_valid': pv_l[original], 'projection_valid': prv_l[original], 'mask_pass': mp_l[original],
                       'texture_pass': tp_l[original], 'cluster_member': original in member}
                if have_cov:
                    row.update({'coverage': _rounded_finite(cov_l[original], 6), 'size_ratio': _rounded_finite(sr_l[original], 6),
                                'rendered_mask_area_px': int(area_l[original])})
                candidates.append(row)
            sc = candidates[rank_l[chosen]]
            rows.append({'selection_method': 'geometry_mask_texture_convergence', 'accepted': accepted, 'rejection_reason': reason,
                         'rank_geo': rank_l[chosen], 'selected_index300': chosen if accepted else None,
                         'selected_proposal6000_index': (int(prop_l[chosen]) if accepted else None),
                         'mask_iou_min': mask_min, 'texture_score_min': texture_min, 'mask_survivors': sum(mp_l),
                         'texture_survivors': sum(tp_l), 'cluster_size': len(cluster_l), 'cluster_occupancy': round(occupancy, 6),
                         'cluster_center_rank_geo': (None if center is None else rank_l[center]),
                         'mask_iou': sc.get('mask_iou'), 'texture_score': sc.get('texture_score'), 'coverage': sc.get('coverage'),
                         'size_ratio': sc.get('size_ratio'), 'candidates': candidates})
        return torch.as_tensor(selected_out, dtype=torch.long, device=geo_scores.device), rows
    return fast


def install(mu, orig, timed):
    fast = make(mu); ref = orig['sequential_candidate_select']
    if os.environ.get('ALT_COMPARE') == '1':
        def both(*a, **k):
            s0, r0 = ref(*a, **k); s1, r1 = fast(*a, **k)
            STATS['calls'] += 1; STATS['same_selected'] += int(torch.equal(s0, s1)); STATS['same_rows'] += int(r0 == r1)
            STATS['objects'] += len(r0); STATS['accepted'] += sum(1 for r in r0 if r['accepted'])
            if r0 != r1 and STATS['printed'] < 3:
                STATS['printed'] += 1
                for x, y in zip(r0, r1):
                    d = {k: (x[k], y[k]) for k in x if k != 'candidates' and x[k] != y[k]}
                    dc = [(i, {k: (c0[k], c1[k]) for k in c0 if c0[k] != c1[k]}) for i, (c0, c1) in enumerate(zip(x['candidates'], y['candidates'])) if c0 != c1][:3]
                    print('DIFF', d, dc)
            return s1, r1
        mu.sequential_candidate_select = both
        import atexit; atexit.register(lambda: print('compare:', dict(STATS)))
    else:
        mu.sequential_candidate_select = fast
    timed('sequential_candidate_select')
