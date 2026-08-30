# 뷰어 포트 대장

이 연구에서 띄운 URL 뷰어의 **단일 출처**는 `registry.csv` 한 파일이다.
새 뷰어를 만들면 거기에 한 줄 추가하면 `viewers.sh` 가 그대로 인식한다.

## 사용법
```bash
_ism_research_2026_07/viewers/viewers.sh list      # 등록된 뷰어 목록
_ism_research_2026_07/viewers/viewers.sh status    # 지금 살아있는 것 확인
_ism_research_2026_07/viewers/viewers.sh start     # 전부 기동 (이미 뜬 건 건너뜀)
_ism_research_2026_07/viewers/viewers.sh start 8779
_ism_research_2026_07/viewers/viewers.sh stop 8774 8775
```
서버는 `setsid` 로 떼어 띄우므로 터미널을 닫아도 살아 있다. 로그는
`outputs/viewer_logs/<name>.log`.

## 포트 배정
| 포트 | 이름 | 용도 | 사람 입력 저장처 |
|---|---|---|---|
| 8766 | location_review | TP 박스 위치 검증(283건) | phase23 location_verification_task.csv |
| 8767 | fpfn_review | FP/FN 검증(399건) | phase23 fp_fn_verification_task.csv |
| 8768 | triptych | 원본+인식+포즈 3면 뷰(723장) | — |
| 8769 | candidate_choice | YOLO top-3 후보 정답 선택(275건) | phase25 candidate_choice_task.csv |
| 8770 | clip_report | CLIP 프롬프트 분석 리포트 | — |
| 8771 | clip_boxviz | CLIP 프롬프트별 실제 bbox | — |
| 8772 | yoloe_compare | YOLO-World vs YOLOE 비교 | — |
| 8773 | gate_reject | 정답 후보인데 게이트 탈락(189건) | — |
| 8774 | appe_ab_old | appearance A/B 구버전(겹침 렌더 버그) | — |
| 8775 | appe_ab | appearance A/B 오프라인 4패널 | — |
| 8776 | fp_audit | FP 166건 재판정 **(완료: T24/F142)** | phase32 fp_audit_task.csv |
| 8777 | pipeline_ab | 실제 파이프라인 Phase3 OFF/ON A/B | — |
| 8778 | change_review | 변화 117건 위치 판정 **(완료: 좋음82/나쁨35)** | phase34 change_review_task.csv |
| 8779 | deploy_bag | 승격된 배포 설정 ros2 bag 결과(263프레임) | — |

## 주의
- 사람 입력을 받는 뷰어(8766/8767/8769/8776/8778)는 저장 CSV 옆에 `.bak` 원본을
  남긴다. 그 CSV 를 지우면 판정 결과가 사라진다.
- 프록시(`http_proxy=127.0.0.1:18080`) 때문에 `curl` 로 확인할 땐 `--noproxy '*'` 필요.
- `pkill -f` 는 이 환경에서 자기 셸까지 종료시킨 적이 있다. `viewers.sh stop` 은
  `/proc` 를 훑어 해당 GT_PORT 프로세스만 골라 죽인다.
