# slam_net — 맥(.113)이 ORB-SLAM3 를 돌리고 결과를 이 PC(.100)로 보낸다

```
   192.168.219.113  (macOS)                  192.168.219.100  (Windows + WSL2)
   ────────────────────────                  ──────────────────────────────────
   ORB-SLAM3                                 slam_receiver.py  (127.0.0.1:5765)
     └─ CameraTrajectory_live.txt                    ▲
          │                                          │
   slam_sender.py ──▶ localhost:5765 ─────역터널─────┘
   slamctl.sh    ──▶ localhost:2222 ─────역터널─────▶ sshd (WSL2)

                    ▲                                 │
                    └──────── ssh -N -R ──────────────┘
                              (WSL2 가 밖으로 건다)
```

**명령은 전부 맥에서**, **결과는 전부 .100 에** 남는다.

---

## 왜 WSL2 가 연결을 거는가

WSL2 는 NAT 뒤에 있어 **들어오는** 연결을 받지 못한다. 실측했다:

```
WSL2 안:      0.0.0.0:8899 LISTEN
맥에서 보면:  192.168.219.100:8899  →  닫힘
```

그러나 **나가는** 연결은 자유롭다. 이것도 실측했다:

```
WSL2 → 192.168.219.113:5900  연결 성공
WSL2 → 192.168.219.113:88    연결 성공
```

그래서 **WSL2 가 맥으로 SSH 를 걸고, `-R` 역터널로 맥이 되돌아 들어오게** 한다.
연결을 건 쪽과 명령을 내리는 쪽이 반대여도 된다 — 그게 역터널의 요점이다.

이 구조의 이득:

| | 역터널 (이 방식) | portproxy / mirrored |
|---|---|---|
| Windows 관리자 권한 | **불필요** | 필요 |
| `.wslconfig` 수정 | **불필요** | mirrored 는 필요 |
| `wsl --shutdown` | **불필요** | mirrored 는 필요 |
| WSL2 IP 가 바뀌면 | **상관없음** | portproxy 규칙이 깨짐 |
| 맥 쪽 준비 | 원격 로그인 ON | 원격 로그인 ON |

---

## 선행 조건은 하나뿐

**맥에서 원격 로그인을 켠다.** 현재 꺼져 있다(실측: `192.168.219.113:22` 닫힘).

```bash
# 맥에서
sudo systemsetup -setremotelogin on
# 또는  시스템 설정 > 일반 > 공유 > 원격 로그인
```

그리고 WSL2 의 공개키를 맥에 등록한다(터널이 비밀번호 없이 재접속하려면 필요).

```bash
# WSL2 에서
ssh-keygen -t ed25519 -C slam_net      # 없으면
ssh-copy-id <맥사용자>@192.168.219.113
```

---

## IP 고정

주소는 **이미 원하는 값**이다 — `.100` = 이 PC, `.113` = 맥. 바꿀 필요 없이 고정만 하면 된다.

**공유기(192.168.219.1) DHCP 예약을 권한다.** OS 쪽 고정 IP 보다 안전하다 —
장비를 다른 망에 가져갔을 때 통신이 끊기지 않는다.

OS 쪽으로 하겠다면:
* Windows: 관리자 PowerShell 에서 `New-NetIPAddress` (설정 > 네트워크에서 GUI 로도 된다)
* 맥: 시스템 설정 > 네트워크 > 세부사항 > TCP/IP > 수동

> WSL2 에서는 Windows 의 IP 를 바꿀 수 없다. 이 세션은 WSL interop 이 꺼져 있어
> (`ipconfig.exe` → `Exec format error`) Windows 명령 자체를 실행하지 못한다.
> 그리고 LAN 주소는 Windows 의 것이지 WSL2 의 것이 아니다(WSL2 는 172.23.73.105).

---

## 설치

```bash
# 1) 맥: 원격 로그인 + 키 등록
bash setup/mac_prepare.sh          # 맥에서 실행

# 2) WSL2: sshd (맥에서 WSL2 를 제어하려면. 데이터 전송만이면 생략 가능)
bash setup/install_sshd.sh         # sudo 비밀번호를 물어본다

# 3) 맥으로 파일 복사
scp -r mac setup <맥사용자>@192.168.219.113:~/slam_net/
```

## 사용

```bash
# WSL2(.100) 에서 한 번 — 터널 + 수신기
./receiver/link.sh up --daemon
./receiver/link.sh status

# 맥(.113) 에서 — 이후 모든 명령
./slamctl.sh check                       연결 점검
./slamctl.sh demo 300                    SLAM 없이 전송 경로만 확인
./slamctl.sh start ~/orbslam/CameraTrajectory_live.txt
./slamctl.sh remote 'ls ~/slam_results'  WSL2 에서 명령 실행
./slamctl.sh fetch ./out                 결과 회수
```

---

## 전송 규약

`\n` 으로 끝나는 JSON 한 줄이 한 메시지.

```json
{"type":"hello","source":"mac-orbslam3","session":"20260913-101500","fps":30}
{"type":"pose","frame":12,"stamp":1789.412,"state":2,
 "t_mm":[12.3,-4.5,987.6],"q":[0.0,0.0,0.0,1.0],"track_ms":16.4}
{"type":"stat","frame":300,"fps":29.8,"sent":301,"dropped":0}
{"type":"bye","frames":7144}
```

* `q` 는 `[x,y,z,w]`, `t_mm` 은 밀리미터, `state` 는 ORB-SLAM3 `eTrackingState`.
* 수신기는 받은 줄을 그대로 `raw.jsonl` 에 남기고, 동시에 TUM 형식
  `CameraTrajectory.txt`(미터)로도 쓴다. **받은 것을 먼저 남기므로** 형식이
  바뀌거나 파싱에 실패해도 데이터를 잃지 않는다.
* 끊기면 송신기가 자동 재접속하고, 수신기는 같은 세션 폴더에 이어 쓴다.

## 검증 상태

| 항목 | 상태 |
|---|---|
| 수신기 ↔ 송신기 왕복 | ✅ 로컬 실측 (120 프레임 무손실, TUM 생성 확인) |
| WSL2 → 맥 아웃바운드 | ✅ 실측 (5900/88 연결 성공) |
| 역터널 | ⏳ 맥의 원격 로그인이 켜지면 확인 가능 |
| 맥의 ORB-SLAM3 | ❌ **미구축** — `setup/macos_orbslam3.md` 참고 |

맥에 접근 권한이 없어 맥 쪽은 검증하지 못했다. 검증하지 않은 것을 "된다" 고
쓰지 않는다.
