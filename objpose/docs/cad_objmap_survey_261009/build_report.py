#!/usr/bin/env python3
"""build_report.py — 'CAD 모델 기반 객체 지도 알고리즘 · 공개 코드와 데이터 · 제안 방법과의 비교 근거' 조사 보고서를 HTML 세 가지로 만든다.
  cad_objmap_survey.html / cad_objmap_survey_local.html / cad_objmap_survey_notion.html  (양식은 ycb_combo_261008 보고서와 같음)
내용은 2026-10-09 에 논문 원문과 GitHub 저장소를 직접 열어 확인한 조사 결과다. 칸의 '(추정)'은 직접 확인하지 않은 것, '확인 못 함'은 찾지 못한 것.
    python3 objpose/docs/cad_objmap_survey_261009/build_report.py"""
import re, sys
from pathlib import Path
HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/docs/objmap_eval_261009"))
from build_report import Doc          # noqa: E402  (표 · 목록을 꾸밈 있는 판과 글만 있는 판으로 함께 만드는 도우미)
REF = REPO / "objpose/docs/ycb_combo_261008/ycb_combo_report.html"
S = lambda t: f"<strong>{t}</strong>"
A = lambda url, t=None: f'<a href="{url}">{t or url.replace("https://", "")}</a>'
GH = lambda r: A(f"https://github.com/{r}", r)


def main():
    d = Doc()
    d.h2("1. 요약")
    d.table(["질문", "결과"], [
        [S("CAD 모델로 객체 지도를 만드는 알고리즘이 있는가"), "있습니다. 물체의 6D 자세를 여러 시점에 걸쳐 세계 좌표에 유지하는 방법으로 MoreFusion, SUO-SLAM, CosyPose 다시점, PrimA6D++ 객체 SLAM, Lu 2022, Object-RPE, CT-Map, SLAM++ 등이 있습니다. 확인한 것은 모두 물체별 학습(YCB 21종 등)이 필요합니다."],
        [S("그중 공식 코드로 지금 쓸 수 있는 것은"), "공식 저장소와 YCB-Video 수치가 함께 있는 것은 다섯 개입니다: CosyPose 다시점(HappyPose), Lu 2022(slam-super-6d), SUO-SLAM, PrimA6D++, MoreFusion. 최신 GPU에서 바로 돌릴 수 있을 것으로 보이는 것은 HappyPose와 slam-super-6d입니다."],
        [S("제안 방법과 구조가 가장 가까운 것은"), "Lu 2022입니다. ORB-SLAM3(RGB-D)의 오도메트리와 프레임별 물체 자세를 자세 그래프로 묶습니다. 다만 목적이 추정기의 자기학습용 라벨 생성이고, 물체별로 학습한 추정기(DOPE)를 씁니다. 개념상 직접 선행 연구는 MoreFusion(외부 ORB-SLAM2 + CAD 객체 지도)입니다."],
        [S("제안 방법만의 차이로 남는 것은"), "학습이 필요 없다는 점입니다. 학습 없는 CAD 기반 추정기(FoundationPose, SAM-6D 등)를 SLAM과 묶어 지속되는 객체 지도를 만든 2024~2026년 공개 연구는 여러 번 검색했지만 찾지 못했습니다. 없다는 증명은 아닙니다."],
        [S("비교에 쓸 수 있는 데이터는"), "YCB-Video가 공통입니다. 위 다섯 방법이 모두 YCB-Video에서 ADD-S / ADD(-S) AUC를 보고합니다. 물체가 방문 사이에 옮겨지는 상황은 ObChange가 맞고, CAD와 6D 정답까지 갖춘 실제 RGB-D 데이터는 확인하지 못했습니다."],
        [S("추천하는 비교 순서"), "1) CosyPose 다시점(HappyPose) 2) Lu 2022의 자세 그래프 백엔드 3) SUO-SLAM. PrimA6D++와 MoreFusion은 실행 대신 논문 수치를 인용하는 편이 현실적입니다."]], widths=["24%", None])

    d.h2("2. 조사 방법과 표기")
    d.dl([("조사 기준일", "2026-10-09. GitHub 정보는 이날 조회한 값"), ("확인한 방법", "논문은 arXiv 원문을 내려받아 본문과 표를 확인. 저장소는 직접 조회해 마지막 푸시, 보관 여부, 라이선스, README의 요구 환경을 확인"),
          ("표기", "아무 표시가 없는 칸은 직접 확인한 것. <strong>(추정)</strong>은 근거는 있으나 직접 확인하지 않은 것. <strong>확인 못 함</strong>은 찾지 못한 것"),
          ("주의", "'공식 저장소를 찾지 못함'은 '없음'과 다릅니다. 최신 GPU에서의 실행 난이도는 README의 요구 사항을 근거로 한 판단이고, 실제로 돌려 본 것은 없습니다")])

    d.h2("3. CAD 모델 기반 객체 지도 알고리즘")
    d.p("여러 프레임에 걸쳐 물체의 6D 자세를 세계(지도) 좌표계에 유지하는 방법을 계열별로 정리했습니다. 제안 방법은 '외부 SLAM의 카메라 자세 + 학습 없는 CAD 기반 6D 추정 → 물체별 이름과 6D 자세를 저장하는 지도'입니다.")
    d.h3("3.1 객체 수준 SLAM · 지도 + 알려진 물체 모델")
    d.table(["알고리즘", "발표", "입력과 사전 정보", "지도에 저장하는 것", "카메라 자세", "물체별 학습", "평가 데이터와 지표"], [
        [S("SLAM++"), "CVPR 2013", "RGB-D, 물체 3D 모델 (추정)", "물체 6D 자세 그래프", "직접 추정 (추정)", "확인 못 함", "확인 못 함"],
        [S("CT-Map"), "IROS 2018", "RGB-D, 물체 3D 메시, 주어진 로봇 자세", "물체 클래스와 6D 자세의 믿음 분포 (입자 필터)", "외부에서 받음 (ORB-SLAM을 예로 듦)", "검출기 미세조정 필요", "자체 20개 시퀀스. 검출 mAP, 위치·회전 기준 통과율"],
        [S("Object-RPE"), "ECMR 2019", "RGB-D, 물체 모델", "서펠 지도 + 물체 6D 자세 (칼만 융합)", "직접 추정 (ElasticFusion)", "필요 (Mask R-CNN, DenseFusion)", "YCB-Video, 자체 창고 물체. ADD-S AUC"],
        [S("MoreFusion"), "CVPR 2020", "RGB-D, CAD", "물체별 점유 격자. 여러 시점에서 합의되면 CAD를 세계 좌표 6D 자세로 배치", "외부에서 받음 (ORB-SLAM2 또는 로봇 기구학)", "필요 (분할 망, 자세 예측 망)", "YCB-Video, 합성 Cluttered YCB. ADD(-S) / ADD-S AUC"],
        [S("MHPE") + " (Fu 2021)", "IROS 2021", "RGB (+ICP용 깊이), CAD", "카메라와 물체 자세 (다중 가설, iSAM2)", "오도메트리를 받아 물체 자세와 함께 최적화. 출처는 확인 못 함", "필요", "YCB-Video, 합성 YCB 영상. ADD / ADD-S AUC"],
        [S("Lu 2022"), "IROS 2022", "프레임별 물체 자세 (DOPE), 카메라 오도메트리", "자세 그래프의 물체 6D 자세 + 카메라 궤적", "외부에서 받음 (YCB-V 실험은 ORB-SLAM3 RGB-D, 루프 폐쇄 없음)", "필요 (DOPE를 자기학습으로 미세조정)", "YCB-Video, 실제 로봇. ADD AUC, ATE, 세계 기준 물체 자세 오차"],
        [S("SUO-SLAM"), "CVPR 2022", "단안 RGB, CAD, 수작업 키포인트 라벨", "물체 6D 자세 + 3D 키포인트 + 카메라 자세", "직접 추정 (물체 키포인트 PnP)", "필요 (키포인트 망)", "YCB-Video (ADD-S / ADD(-S) AUC), T-LESS"],
        [S("PrimA6D++ 객체 SLAM"), "RA-L 2022", "RGB (+깊이 ICP), CAD", "인자 그래프로 최적화한 여러 물체의 6D 자세", "외부에서 받음 (로봇 순기구학)", "필요 (물체별 모델)", "YCB-Video (ADD(-S) / ADD-S), T-LESS"],
        [S("PoseRBPF"), "RSS 2019 (추정)", "RGB 또는 RGB-D, CAD", "지도 없음. 카메라 기준 추적 (추정)", "해당 없음", "필요 (추정)", "YCB-Video, T-LESS"]], widths=["12%", "8%", None, None, None, None, None])
    d.p("Fusion++(3DV 2018)는 CAD 없이 물체별 TSDF를 만드는 방법이라 이 표에서 뺐습니다. Deep-SLAM++는 범주별 형상 사전을 쓰므로 3.6에 해당합니다.", "note")
    d.h3("3.2 다시점 6D 자세 정합")
    d.table(["알고리즘", "발표", "입력", "출력", "카메라 자세", "물체별 학습", "평가"], [
        [S("CosyPose"), "ECCV 2020", "RGB 여러 장, CAD. 카메라 자세 불필요", "물체 수·종류·6D 자세와 카메라 자세를 담은 장면 모델", "직접 추정 (물체 후보 RANSAC + 객체 수준 번들 조정)", "필요", "YCB-Video (ADD-S AUC), T-LESS"],
        [S("MV6D"), "IROS 2022", "다시점 RGB-D, 카메라 자세", "물체 6D 자세", "외부에서 받음", "필요", "YCB-Video, 합성 MV-YCB"],
        [S("SyMFM6D"), "RA-L 2023", "다시점 RGB-D, CAD, 카메라 자세", "물체 6D 자세", "외부에서 받음", "필요", "YCB-Video, MV-YCB"]], widths=["12%", "8%", None, None, None, "8%", None])
    d.h3("3.3 CAD 검색 · 정렬로 장면을 CAD 지도로 만드는 계열")
    d.table(["알고리즘", "발표", "입력", "출력", "카메라 자세"], [
        [S("Scan2CAD"), "CVPR 2019", "완성된 RGB-D 스캔, ShapeNet", "CAD별 9자유도 정렬", "스캔이 이미 주어짐"], [S("FroDO"), "CVPR 2020", "자세가 주어진 RGB 영상", "물체별 형상 코드와 자세 (범주 수준)", "외부에서 받음"],
        [S("ODAM"), "ICCV 2021", "자세가 주어진 RGB 영상", "초이차곡면. CAD는 쓰지 않음", "외부에서 받음"], [S("Vid2CAD"), "T-PAMI 2022", "RGB 영상, CAD 데이터베이스", "물체별 CAD와 9자유도 자세", "외부 SfM"],
        [S("ROCA"), "CVPR 2022", "단일 RGB", "CAD 검색과 9자유도 자세", "해당 없음"]], widths=["12%", "10%", None, None, None])
    d.p("이 계열은 모두 ShapeNet에서 비슷한 모양을 찾아 맞추는 범주 수준 방법입니다. 그 물체의 정확한 CAD를 전제하지 않고, 가구급 물체를 다룹니다. 제안 방법의 비교 대상으로는 맞지 않습니다.", "note")
    d.h3("3.4 장면 그래프에서 CAD를 맞추는 것")
    d.ul(["<strong>3D Dynamic Scene Graphs (Kimera).</strong> CAD가 주어진 물체 클래스는 CAD와 재구성 메시에서 키포인트를 뽑아 TEASER++로 정합합니다. CAD가 없으면 중심점과 경계 상자만 저장합니다. 물체 자세의 정량 평가 데이터는 확인하지 못했습니다.",
          "<strong>Hydra.</strong> README에 CAD 정합에 대한 언급이 없습니다. 물체를 CAD 자세로 저장하지 않는 것으로 보입니다 (추정)."])
    d.h3("3.5 학습 없는 6D 추정기를 SLAM · 지도와 결합한 연구 (2024~2026)")
    d.table(["연구", "확인한 내용", "제안 방법과의 관계"], [
        [S("UnPose") + " (CoRL 2025)", "CAD 없이 동작. 카메라 자세는 기성 SLAM(ORB-SLAM3 인용)에서 받음. FoundationPose의 정제 망 사용. 물체별 3DGS + 자세 그래프. YCB-Video와 LM-O에서 ADD / ADD-S AUC를 그림으로 제시. 단일 물체 중심. 코드 링크 없음", "외부 SLAM + 학습 없음이 같음. CAD를 쓰지 않고 한 물체를 다룸"],
        [S("Asset-Centric Metric-Semantic Maps") + " (arXiv 2510.10778)", "RGB-D와 로봇 오도메트리를 받아, 메시·CAD 데이터베이스에서 검색한 물체의 메시·범주·자세를 저장. 비교 대상은 Clio, SAM3D. 코드 링크 없음", "객체를 메시와 자세로 저장하는 점이 같음. 가구급 물체, 검색 기반"],
        [S("6DOPE-GS") + " (arXiv 2412.01543)", "CAD 없이 3DGS로 단일 물체의 자세를 추정·추적", "객체 지도가 아님"],
        [S("CAD 기반 파운데이션 추정기 + SLAM → 지속되는 객체 지도"), "네 차례 검색에서 해당 논문을 찾지 못함", "제안 방법이 해당하는 자리"]], widths=["24%", None, "26%"])
    d.h3("3.6 형상 사전(범주 수준) 계열")
    d.p("NodeSLAM, DSP-SLAM(DeepSDF 형상 사전), CubeSLAM(직육면체), QuadricSLAM(이차곡면), OA-SLAM(타원체), VOOM은 그 물체의 CAD가 아니라 범주의 일반적인 모양이나 기하 근사를 씁니다. 물체별 정확한 6D 자세(ADD 계열)로 평가하지 않습니다 (추정). 관련 연구에서 구분해 서술하면 되고 비교 대상은 아닙니다.")

    d.h2("4. 공식 코드가 있어 지금 쓸 수 있는 알고리즘")
    d.p("3장의 목록 가운데 공식 저장소가 실제로 존재하는 것만 추렸습니다. 왼쪽에 색 띠가 있는 줄이 YCB-Video 수치와 코드가 함께 있는 다섯 개입니다.", "note")
    rows = [
        [S("CosyPose 다시점") + "<br>(HappyPose)", GH("agimus-project/happypose"), "2026-10", "아니오", "BSD-2", "제공", "pip / uv / conda. CUDA 12.4용 설치 항목 있음", "BOP 형식. 다시점 코드 경로 있음", "쉬움 (추정). 다시점 실행이 지금 되는지는 확인 못 함"],
        [S("Lu 2022") + "<br>(slam-super-6d)", GH("520xyxyzq/slam-super-6d"), "2024-07", "아니오", "MIT", "DOPE 가중치 링크", "README에 OS 표기 확인 못 함", "TUM 형식 텍스트(물체별 검출 자세, 오도메트리). YCB-V용 자료 동봉", "쉬움 (추정). 자세 그래프 최적화는 GPU 불필요"],
        [S("SUO-SLAM"), GH("rpng/suo_slam"), "2022-05", "아니오", "MIT", "제공", "Ubuntu 18.04에서 시험, torch 1.8.1, Ceres, g2opy 빌드", "BOP 형식 YCB-V·T-LESS. 재현 스크립트 있음", "중간 (추정). g2opy 빌드가 변수"],
        [S("PrimA6D++"), GH("MyungHwanJeon/PrimA6D"), "2023-11", "아니오", "MIT", "제공", "Docker 이미지", "BOP YCB-V 스크립트", "중간~어려움 (추정). 객체 SLAM 부분의 공개 범위 확인 못 함"],
        [S("MoreFusion"), GH("wkentaro/morefusion"), "2023-03", "예", "기타", "제공", "Ubuntu 16.04, ROS Kinetic, CUDA 10.1", "YCB-Video 예제, ROS RGB-D 라이브", "매우 어려움 (추정). 요구 환경이 오래됨"],
        ["CosyPose (원본)", GH("yannlabb/cosypose"), "2022-09", "아니오", "MIT", "제공", "python 3.7.6, pytorch 1.3.1, CUDA 10.1. 유지 보수 중단 표기", "BOP YCB-V·T-LESS", "어려움 (추정). HappyPose 권장"],
        ["Object-RPE", GH("hoangcuongbk80/Object-RPE"), "2023-03", "아니오", "없음", "데이터 폴더 링크", "Ubuntu 16.04, ROS Kinetic", "자체 창고 데이터", "매우 어려움 (추정). 라이선스 없음"],
        ["PoseRBPF", GH("NVlabs/PoseRBPF"), "2021-04", "아니오", "기타", "제공", "ROS Kinetic, Python 2.7 노드", "YCB-Video·T-LESS 데모", "어려움 (추정). 지도가 아니라 추적"],
        ["SyMFM6D", GH("boschresearch/SyMFM6D"), "2023-06", "예", "AGPL-3.0", "확인 못 함", "CUDA 10.1, python 3.6, pytorch 1.7.1", "YCB-Video, MV-YCB", "어려움 (추정)"],
        ["FoundationPose", GH("NVlabs/FoundationPose"), "2026-04", "아니오", "기타", "제공", "Docker 권장. 최신 GPU용 CUDA 12.1 이미지 안내", "LINEMOD, YCB-Video 스크립트", "쉬움~중간 (추정). 프레임 단위 추정·추적기"],
        ["MegaPose", GH("megapose6d/megapose6d"), "2024-12", "아니오", "기타", "확인 못 함", "확인 못 함", "HappyPose에 통합", "HappyPose로 사용"],
        ["SAM-6D", GH("JiehongLin/SAM-6D"), "2024-07", "아니오", "없음", "제공", "", "", "이미 사용 중"]]
    d.table(["알고리즘", "저장소", "마지막 푸시", "보관", "라이선스", "가중치", "README의 요구 환경", "입력과 바로 돌릴 수 있는 데이터", "Ubuntu 22.04 + RTX 5090"], rows, group=lambda i, r: "g2" if i < 5 else "", widths=["11%", "13%", "6%", "5%", "6%", "7%", None, None, None])
    d.h3("공식 저장소를 찾지 못한 것")
    d.p("SLAM++, Fusion++, CT-Map, MHPE, MV6D, Vid2CAD, FroDO, UnPose, Asset-Centric Metric-Semantic Maps, 6DOPE-GS. 찾지 못했다는 뜻이고 없다고 단정할 수는 없습니다. 3.3의 Scan2CAD, ROCA, ODAM은 저장소가 있지만 ScanNet의 가구급 물체를 다뤄 비교 대상에서 뺐습니다.")

    d.h2("5. 공식 배포되는 데이터셋")
    d.p("CAD가 있는 물체, 연속 영상, 카메라 자세, 물체 6D 정답을 갖췄는지를 기준으로 봤습니다. 왼쪽에 색 띠가 있는 줄이 지금 이 연구에서 쓰고 있는 데이터입니다.", "note")
    rows = [
        [S("YCB-Video") + " (BOP)", A("https://bop.felk.cvut.cz/datasets/", "BOP"), "예", "있음 (이 PC의 변환본에서 확인)", "있음", "21종", "영상 92개. 평가용 12장면", "정지", "MIT"],
        [S("YCB-M"), A("https://zenodo.org/records/2579173", "Zenodo"), "예 (카메라 7종)", "파일에 따로 들어 있는지 확인 못 함. 이 연구에서는 정지한 물체의 자세에서 복원해 씀", "있음", "있음 (YCB 21종)", "로봇 팔이 연속 이동한 32장면", "정지", "CC BY 4.0, 15.8 GB"],
        [S("ObChange"), A("https://researchdata.tuwien.ac.at/records/y3ggy-hxp10", "TU Wien"), "예", "녹화 파일의 로봇 위치 추정값", "점 단위 라벨. 물체별 자세 행렬도 있으나 이 연구의 CAD와 좌표축이 맞지 않아 쓰지 못함 (이 PC에서 확인)", "따로 제공 안 함 (추정). YCB 물체", "연속 녹화", "방 5곳을 5회 이상 방문, 방문 사이에 물체 재배치", "CC BY 4.0, 109 GiB"],
        ["HOPE-Video", GH("swtyree/hope-dataset"), "예", "있음. 단위 주의(외부 파라미터 m, 물체 자세 cm)", "있음 (카메라 기준)", "28종", "10개 시퀀스, 2,038프레임", "정지 탁상, 물체 5~20개", "CC BY-NC-SA 4.0, 2.9 GB"],
        ["T-LESS (BOP)", A("https://bop.felk.cvut.cz/datasets/", "BOP"), "예", "확인 못 함", "있음", "30종", "20장면. 연속 여부 확인 못 함", "정지 (추정)", "CC BY 4.0"],
        ["TUD-L (BOP)", A("https://bop.felk.cvut.cz/datasets/", "BOP"), "확인 못 함", "확인 못 함", "있음", "3종", "이미지 시퀀스", "물체가 움직임", "CC BY-SA 4.0"],
        ["GraspNet-1Billion", A("https://graspnet.net/datasets.html", "graspnet.net"), "예", "있음 (장면당 256개)", "있음", "88종", "190장면 × 256프레임", "정지 (추정)", "CC BY-NC-SA, 125 GB"],
        ["Aria Digital Twin", A("https://www.projectaria.com/datasets/adt/", "Project Aria"), "RGB + 깊이 지도", "있음", "있음 (시퀀스별 물체 궤적)", "대부분 물체에 3D 모델", "200개 시퀀스, 약 400분", "물체 398개 중 74개가 움직임. 재방문은 확인 못 함", "별도 계약, 연구자 한정"],
        ["HOT3D (BOP)", GH("facebookresearch/hot3d"), "아니오 (어안 RGB·흑백)", "있음", "있음", "33종", "150프레임 클립", "손으로 조작 (추정)", "별도 계약"],
        ["MR6D", "arXiv 2508.13775", "확인 못 함", "확인 못 함", "있음 (추정)", "확인 못 함", "92장면, 물체 16종", "움직이는 장면 포함", "확인 못 함"],
        ["Scan2CAD (ScanNet)", GH("skanti/Scan2CAD"), "예", "있음 (추정)", "CAD 9자유도 정렬 주석", "ShapeNet (범주 수준)", "예", "정지", "신청 필요"]]
    d.table(["데이터", "배포처", "RGB-D", "프레임별 카메라 자세", "물체 6D 정답", "CAD", "영상 구성", "물체 이동 · 재방문", "라이선스 · 크기"], rows, group=lambda i, r: "g2" if i < 3 else "", widths=["11%", "9%", "8%", None, None, "9%", None, None, "10%"])
    d.h3("상황별로 쓸 수 있는 데이터")
    d.ul(["<strong>정지한 장면에서의 등록 정확도와 6D 자세.</strong> YCB-Video가 공통 기준입니다. 4장의 다섯 방법이 모두 여기서 수치를 보고합니다. YCB-M과 HOPE-Video는 같은 조건의 두 번째 데이터로 쓸 수 있습니다. HOPE는 이 PC에 내려받아 둔 것이 있습니다.",
          "<strong>방문 사이에 물체가 옮겨지는 상황.</strong> ObChange가 맞습니다. 다만 물체 방향의 정답을 쓸 수 없어 6D 자세 평가는 안 됩니다. CAD와 6D 정답까지 갖춘 실제 RGB-D 데이터는 확인하지 못했습니다. Aria Digital Twin이 가장 가깝지만 안경형 기기 영상이고 별도 계약이 필요합니다.",
          "<strong>움직이지 않은 물체가 시야를 벗어났다 다시 보이는 상황.</strong> 이를 명시적으로 평가하는 CAD 물체 벤치마크는 찾지 못했습니다. 이 PC의 ObChange 자료로 세어 보면 SmallRoom에서 물체 25개가 모두 한 방문 안에서 시야를 벗어났다 다시 보입니다(벗어난 시간 중앙값 26초). 연속 궤적과 카메라 자세가 있는 YCB-Video, YCB-M, HOPE-Video에서 구간을 잘라 구성하는 방법도 있습니다 (추정)."])

    d.h2("6. 제안 방법과 비교할 수 있는 근거")
    d.h3("6.1 같은 점과 다른 점")
    d.table(["알고리즘", "카메라 자세", "CAD", "학습", "지도 표현", "실시간", "제안 방법과 같은 점", "다른 점"], [
        [S("제안 방법"), "외부 (ORB-SLAM3 RGB-D), SLAM은 고치지 않음", "사용", "불필요", "물체별 이름 + 세계 좌표 6D 자세", "실시간 처리", "", ""],
        [S("MoreFusion"), "외부 (ORB-SLAM2 또는 기구학)", "사용", "필요", "점유 격자 + CAD 6D 자세", "실시간이라고 밝힘", "외부 SLAM, CAD를 세계 좌표에 배치, 여러 시점에서 합의된 뒤 등록", "학습 필요, 체적 융합과 충돌 기반 정제"],
        [S("Lu 2022"), "외부 (ORB-SLAM3 RGB-D)", "사용 (DOPE용)", "필요 + 자기학습", "자세 그래프의 물체 6D 자세", "확인 못 함", "ORB-SLAM3 오도메트리 + 프레임별 물체 자세를 지도에서 묶음", "목적이 의사 라벨 생성과 재학습. 오도메트리도 함께 최적화"],
        [S("SUO-SLAM"), "직접 추정", "사용 + 키포인트 라벨", "필요", "물체 6D + 카메라 그래프", "앞단 11 Hz, 뒷단 2 Hz (GTX 1080Ti)", "물체의 6D 자세를 세계 좌표에 유지", "RGB만 사용, 물체로 카메라를 추정, 키포인트를 손으로 지정"],
        [S("CosyPose 다시점"), "직접 추정 (오프라인)", "사용", "필요", "장면 모델 (물체 + 카메라)", "오프라인", "여러 시점의 일관성으로 잘못된 검출을 제거", "순서 없는 소수 시점, 영상 전체를 모아 품"],
        [S("PrimA6D++"), "외부 (로봇 기구학)", "사용", "필요 (물체별)", "인자 그래프의 여러 물체 6D", "실시간 데모라고 밝힘", "외부 카메라 자세 + 물체 자세를 그래프로 묶음", "고정된 작업 공간, 불확실성을 예측하는 망"],
        [S("UnPose"), "외부 (ORB-SLAM3)", "미사용", "불필요", "물체별 3DGS + 자세 그래프", "실시간 어려움", "외부 SLAM, 학습 없음", "CAD 없음, 단일 물체, 코드 없음"]],
        group=lambda i, r: "g0" if i == 0 else "", widths=["10%", None, "7%", "8%", None, None, None, None])
    d.h3("6.2 각 논문이 YCB-Video에서 보고한 수치")
    d.p("원문의 표에서 직접 확인한 값입니다. 모두 ADD-S 또는 ADD(-S)의 AUC(0~10 cm) 계열입니다.", "note")
    d.table(["출처", "표", "조건", "ADD(-S) AUC", "ADD-S AUC"], [
        ["MoreFusion", "Table 2", "정답 마스크, 단일 시점", "91.0", "95.7"], ["CosyPose", "Table 1a", "단일 시점", "84.5", "89.8"], ["CosyPose", "Table 2a", "다시점 5장", "–", "93.4"],
        ["SUO-SLAM", "Table 1", "객체 SLAM", "84.7", "90.3"], ["SUO-SLAM", "Table 1", "같은 방법의 단일 시점", "56.9", "65.7"], ["MHPE (Fu 2021)", "Table I", "최선 가설 + ICP", "72.6 (ADD)", "85.2"],
        ["Object-RPE", "Table I", "전체 시스템", "–", "95.9"], ["PrimA6D++", "Table II", "그래프 최적화, RGB", "91.5", "94.8"], ["PrimA6D++", "Table II", "그래프 최적화, RGB-D", "94.4", "96.9"]])
    d.table(["출처", "표", "지표", "값"], [["Lu 2022", "Table III", "YCB-V 시퀀스 0049: 카메라 궤적 오차(ATE)", "0.199 → 0.081 m"], ["Lu 2022", "Table III", "같은 시퀀스의 설탕 상자: 세계 기준 위치 오차", "0.054 → 0.039 m"],
                                     ["Lu 2022", "Table III", "시퀀스 0059: ATE", "0.618 → 0.105 m"], ["Lu 2022", "Table III", "같은 시퀀스의 크래커 상자: 세계 기준 위치 오차", "0.234 → 0.057 m"]])
    d.ul(["<strong>검출 조건이 논문마다 다릅니다.</strong> PoseCNN 검출, 정답 마스크, 자체 검출이 섞여 있어 수치를 그대로 나란히 놓으면 안 됩니다.",
          "<strong>MoreFusion의 91.0 / 95.7은 정답 마스크를 준 단일 시점 값입니다.</strong> MoreFusion 원문은 YCB-Video에서 다시점 전체 시스템의 수치를 보고하지 않습니다. 다른 논문의 인용표에서 이 값이 단일 시점 칸에 들어가 있습니다.",
          "<strong>인용표끼리 조건 표기가 어긋난 곳이 있습니다.</strong> PrimA6D++의 표는 CosyPose 93.4를 8장 조건으로 적었는데 CosyPose 원문은 5장입니다.",
          "<strong>제안 방법의 수치와 바로 비교할 수 없습니다.</strong> 지금 가진 값(YCB-Video에서 ADD(-S) 중앙값 4.7 mm, 지름 10 % 통과율 100 %)은 최종 지도의 맞게 올라간 객체에만 잰 것입니다. 위 표와 견주려면 프레임마다 모든 정답 물체에 대해 AUC를 계산해야 합니다. 아직 계산하지 않았습니다."])
    d.h3("6.3 비교 대상 추천 순위")
    d.table(["순위", "대상", "이유", "비교할 지표", "직접 실행", "난이도"], [
        ["1", S("CosyPose 다시점") + " (HappyPose)", "저장소가 지금도 유지되고 최신 CUDA를 지원합니다. YCB-V 학습 가중치가 있고 다시점 수치(93.4)가 논문에 있습니다. 'CAD 기반이면서 학습이 필요한 방법'의 대표가 됩니다", "ADD-S AUC, ADD(-S) AUC, BOP AR, 객체 단위 Precision·Recall", "YCB-V 가능. YCB-M은 YCB-V 가중치로 추론 가능 (추정)", "낮음~중간"],
        ["2", S("Lu 2022") + "의 자세 그래프", "제안 방법과 구조가 가장 가깝습니다. 입력이 텍스트 형식이라 제안 방법의 프레임별 자세를 그대로 넣을 수 있습니다. 세계 기준 물체 오차를 원문이 보고합니다", "세계 기준 물체 위치·회전 오차, 카메라 궤적 오차", "YCB-V, YCB-M 모두 가능 (추정)", "낮음"],
        ["3", S("SUO-SLAM"), "'객체 SLAM'이라는 이름의 직접 경쟁자입니다. YCB-V 재현 스크립트와 가중치가 있습니다. RGB만 쓰고 카메라를 직접 추정하므로 외부 SLAM을 쓰는 설계와 대조됩니다", "ADD-S / ADD(-S) AUC (원문 90.3 / 84.7)", "YCB-V 가능. YCB-M은 키포인트 망이 맞지 않을 수 있음 (추정)", "중간"],
        ["4", S("PrimA6D++"), "외부 카메라 자세 + 물체 자세 그래프 구조이고 YCB-V에서 가장 높은 수치(94.4 / 96.9)를 보고합니다", "ADD(-S) / ADD-S", "Docker로 시도 가능. 최신 GPU 호환 확인 못 함", "중간~높음. 수치 인용 권장"],
        ["5", S("MoreFusion"), "개념상 가장 직접적인 선행 연구입니다. 환경이 오래됐고 저장소가 보관 상태입니다", "ADD(-S) / ADD-S AUC", "사실상 어려움 (추정)", "매우 높음. 수치 인용과 관련 연구 서술 권장"],
        ["참고", "FoundationPose · MegaPose를 같은 지도에 연결", "지도는 같게 두고 학습 없는 추정기만 바꾸는 실험입니다. 진행 중인 추정기 비교와 겹칩니다", "같음", "가능", "낮음"]], widths=["6%", "15%", None, None, None, "11%"])
    d.h3("6.4 지표는 두 겹으로")
    d.ul(["<strong>프레임 단위.</strong> 지도에 저장된 자세를 카메라 좌표로 되돌려 ADD-S / ADD(-S) AUC와 BOP AR을 계산합니다. 6.2의 논문 수치와 대응됩니다.",
          "<strong>지도 단위.</strong> 세계 기준 물체 위치·회전 오차와 객체 단위 Precision·Recall입니다. Lu 2022의 표, 그리고 FM-Fusion·DualMap과의 비교에 대응됩니다."])

    d.h2("7. 이 조사가 논문에 주는 의미")
    d.ul(["<strong>관련 연구의 서술.</strong> CAD로 6D 객체 지도를 만든 연구는 여럿 있지만 확인한 것은 모두 물체별 학습을 전제합니다. 제안 방법은 학습 없는 추정기로 같은 종류의 지도를 만든다는 점에서 구분됩니다.",
          "<strong>지금 비교의 빈자리.</strong> FM-Fusion과 DualMap은 CAD를 쓰지 않는 방법입니다. CAD를 쓰는 방법과의 비교가 없으면 '이름 구분에서 앞서는 것은 CAD 덕분'이라는 지적에 답하기 어렵습니다. CosyPose 다시점이나 Lu 2022가 그 자리를 채울 수 있습니다.",
          "<strong>예상되는 결과.</strong> 위 방법들은 YCB-Video로 학습한 모델을 쓰므로 YCB-Video에서는 제안 방법보다 정확할 가능성이 있습니다. 그 경우 주장은 '더 정확하다'가 아니라 '학습 없이 비슷한 수준의 지도를 만든다'가 됩니다. 학습에 쓰이지 않은 YCB-M이나 HOPE에서의 비교가 이 주장을 받칩니다.",
          "<strong>먼저 할 일.</strong> 제안 방법의 프레임 단위 ADD-S AUC를 YCB-Video에서 계산해 6.2의 표와 나란히 놓는 것입니다. 실행 없이 저장된 결과로 할 수 있습니다."])

    d.h2("8. 확인하지 못한 것")
    d.ul(["SLAM++, Fusion++, CT-Map, MHPE, MV6D, Vid2CAD, FroDO, 6DOPE-GS, NodeSLAM의 공식 코드 유무. OA-SLAM의 공식 저장소 위치.", "SLAM++, Fusion++, NodeSLAM의 원문. 이번에 열어 보지 않아 해당 칸은 추정입니다.",
          "MHPE가 쓰는 오도메트리의 출처. PrimA6D 저장소에서 객체 SLAM 부분이 실제로 공개돼 있는지와 Docker 이미지의 CUDA 버전.", "HappyPose에서 CosyPose 다시점 파이프라인이 현재 버전으로 실제 동작하는지. 코드 경로가 있다는 것만 확인했습니다.",
          "YCB-M 압축 파일에 프레임별 카메라 자세가 따로 들어 있는지. T-LESS, LM-O, TUD-L 등 BOP 데이터의 프레임별 카메라 자세와 크기.", "Aria Digital Twin에서 같은 장소의 재방문과 방문 사이 물체 이동이 실제로 있는지.",
          "시야를 벗어났다 다시 보이는 상황을 명시적으로 평가하는 CAD 물체 벤치마크의 존재.", "UnPose의 YCB-Video 수치. 그림으로만 제시돼 있어 옮기지 못했습니다.", "어느 방법도 이 PC에서 실제로 설치해 돌려 보지 않았습니다. 실행 난이도는 README를 근거로 한 판단입니다."])

    d.h2("9. 참고한 자료"); d.h3("논문")
    d.ul([A("https://arxiv.org/abs/2004.04336", "MoreFusion") + ", " + A("https://arxiv.org/abs/2205.01823", "SUO-SLAM") + ", " + A("https://arxiv.org/abs/2008.08465", "CosyPose") + ", " + A("https://arxiv.org/abs/2108.01225", "MHPE") + ", " + A("https://arxiv.org/abs/2203.04424", "Lu 2022") + ", "
          + A("https://arxiv.org/abs/2211.00960", "PrimA6D++ 객체 SLAM") + ", " + A("https://arxiv.org/abs/1810.11525", "CT-Map") + ", " + A("https://arxiv.org/abs/1908.08601", "Object-RPE"),
          A("https://arxiv.org/abs/2208.01172", "MV6D") + ", " + A("https://arxiv.org/abs/2307.00306", "SyMFM6D") + ", " + A("https://arxiv.org/abs/2012.04641", "Vid2CAD") + ", " + A("https://arxiv.org/abs/2005.05125", "FroDO") + ", " + A("https://arxiv.org/abs/2108.10165", "ODAM") + ", "
          + A("https://arxiv.org/abs/2112.01988", "ROCA") + ", " + A("https://arxiv.org/abs/2002.06289", "3D Dynamic Scene Graphs"),
          A("https://arxiv.org/abs/2508.15972", "UnPose") + ", " + A("https://arxiv.org/abs/2510.10778", "Asset-Centric Metric-Semantic Maps") + ", " + A("https://arxiv.org/abs/2004.11657", "YCB-M")])
    d.h3("저장소와 데이터")
    d.ul([", ".join(GH(r) for r in ("agimus-project/happypose", "520xyxyzq/slam-super-6d", "rpng/suo_slam", "MyungHwanJeon/PrimA6D", "wkentaro/morefusion", "yannlabb/cosypose", "hoangcuongbk80/Object-RPE", "NVlabs/PoseRBPF", "boschresearch/SyMFM6D")),
          ", ".join(GH(r) for r in ("NVlabs/FoundationPose", "megapose6d/megapose6d", "nv-nguyen/gigapose", "JiehongLin/SAM-6D", "MIT-SPARK/TEASER-plusplus", "MIT-SPARK/Hydra", "skanti/Scan2CAD", "cangumeli/ROCA", "likojack/ODAM")),
          ", ".join([A("https://bop.felk.cvut.cz/datasets/", "BOP 데이터셋"), A("https://zenodo.org/records/2579173", "YCB-M"), A("https://researchdata.tuwien.ac.at/records/y3ggy-hxp10", "ObChange"), GH("swtyree/hope-dataset"),
                     A("https://www.projectaria.com/datasets/adt/", "Aria Digital Twin"), A("https://graspnet.net/datasets.html", "GraspNet-1Billion")]),
          "이 보고서: <code>objpose/docs/cad_objmap_survey_261009/build_report.py</code>"])

    ref = REF.read_text(); head = ref[: ref.index('<div class="wrap">')].replace("<title>YCB 조합 실험 보고</title>", "<title>CAD 기반 객체 지도 조사</title>").replace("</style>", "a{color:var(--accent)}table.sum td{vertical-align:top}</style>")
    hdr = ('<header><div class="kicker">조사 보고 · CAD 기반 객체 지도 · 2026-10-09</div><h1>CAD 모델 기반 객체 지도: 알고리즘, 공개 코드와 데이터, 제안 방법과의 비교 근거</h1>'
           '<p class="lead">제안 방법은 SLAM의 카메라 자세와 학습 없는 CAD 기반 6D 자세 추정을 결합해 객체 지도를 만듭니다. 지금까지 비교한 FM-Fusion과 DualMap은 CAD를 쓰지 않아, CAD를 쓰는 방법끼리의 비교가 빠져 있습니다. '
           '그 자리를 채울 알고리즘과 데이터를 찾으려고 논문 원문과 GitHub 저장소를 직접 열어 확인했습니다. 실제로 설치해 돌려 본 것은 없습니다.</p></header>')
    body = hdr + "\n" + "\n".join(a for a, _ in d.parts)
    (HERE / "cad_objmap_survey.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "cad_objmap_survey_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "CAD 기반 객체 지도 조사")
    nb = "<p>조사 보고 · CAD 기반 객체 지도 · 2026-10-09</p>" + re.sub(r"<header>.*?</div>", "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "") + "\n" + "\n".join(b for _, b in d.parts)
    (HERE / "cad_objmap_survey_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    for f in ("cad_objmap_survey.html", "cad_objmap_survey_local.html", "cad_objmap_survey_notion.html"): print(f, (HERE / f).stat().st_size)


if __name__ == "__main__":
    main()
