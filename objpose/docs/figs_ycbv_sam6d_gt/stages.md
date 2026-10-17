| 객체 | YOLO 프롬프트 | 보이는 GT 인스턴스 | ① YOLO 박스 (라벨 무관) | ② YOLO 박스 (자기 프롬프트) | ③ ISM 통과 | ④ 최종 포즈 맞음 | ①은 되고 ②가 안 될 때 붙은 프롬프트 | ②는 되고 ③이 안 될 때 ISM 판정 |
|---|---|---|---|---|---|---|---|---|
| pudding_box | brown pudding box | 172 | 0.0% | 0.0% | 0.0% | 0.0% |  |  |
| gelatin_box | red gelatin box | 172 | 0.0% | 0.0% | 0.0% | 0.0% |  |  |
| potted_meat_can | potted meat can | 512 | 42.8% | 0.2% | 0.2% | 0.2% | yellow sugar box 107, tomato soup can 103 |  |
| large_clamp | black clamp | 418 | 15.8% | 0.2% | 0.2% | 0.2% | power drill 37, large black clamp 24 |  |
| foam_brick | red foam brick | 188 | 89.4% | 0.5% | 0.0% | 0.0% | marker pen 167 | (below-sim) 1 |
| scissors | scissors | 201 | 28.9% | 28.9% | 25.4% | 33.3% |  | (below-appe) 4, detected 3 |
| power_drill | power drill | 695 | 57.8% | 44.7% | 41.4% | 46.6% | red cracker box 40, marker pen 31 | detected 19, (below-appe) 4 |
| extra_large_clamp | large black clamp | 413 | 60.8% | 59.1% | 50.6% | 34.1% | marker pen 7 | (below-sim) 33, detected 2 |
| sugar_box | yellow sugar box | 990 | 64.5% | 63.1% | 55.4% | 55.3% | wooden block 10, scissors 2 | (below-appe) 64, (below-sim) 13 |
| cracker_box | red cracker box | 572 | 81.8% | 79.9% | 42.0% | 48.8% | yellow mustard bottle 8, tomato soup can 2 | (below-appe) 141, (below-sim) 45 |
| tomato_soup_can | tomato soup can | 1034 | 96.0% | 80.0% | 76.2% | 71.7% | white cleanser bottle 163, yellow mustard bottle 1 | (below-appe) 39 |
| wood_block | wooden block | 183 | 92.9% | 90.7% | 7.1% | 6.0% | yellow sugar box 4 | (below-sim) 149, (below-appe) 4 |
| tuna_fish_can | small tuna can | 743 | 95.4% | 95.2% | 93.5% | 89.6% | red bowl 2 | (below-appe) 10, (cross-object-nms) 2 |
| bleach_cleanser | white cleanser bottle | 765 | 95.7% | 95.7% | 83.7% | 78.7% |  | (below-sim) 58, (below-appe) 21 |
| master_chef_can | blue coffee can | 732 | 100.0% | 100.0% | 80.3% | 75.4% |  | (cross-object-nms) 108, (below-appe) 21 |
| mug | red mug | 408 | 100.0% | 100.0% | 99.8% | 99.3% |  | (below-sim) 1 |
| bowl | red bowl | 325 | 100.0% | 100.0% | 16.9% | 0.3% |  | (below-sim) 270 |
| mustard_bottle | yellow mustard bottle | 282 | 100.0% | 100.0% | 100.0% | 92.2% |  |  |
| banana | banana | 315 | 100.0% | 100.0% | 99.7% | 78.4% |  | (below-appe) 1 |
| pitcher_base | blue plastic pitcher | 385 | 100.0% | 100.0% | 100.0% | 94.8% |  |  |
| large_marker | marker pen | 375 | 100.0% | 100.0% | 93.6% | 8.5% |  | (below-appe) 20, detected 4 |
| **전체** | | **9880** | **77.9%** | **70.4%** | **59.3%** | **53.1%** | | |
