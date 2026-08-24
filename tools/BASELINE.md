# QP 솔버 업그레이드 — 기준선·벤치마크·게이트 기록

날짜: 2026-08-23~24. 로컬 전용 작업(원격 반영 금지). 기준 모델: rby1m v1.3 (시뮬).

## 환경
- 공식 시뮬: `rainbowroboticsofficial/rby1-sim:0.10.6-m_v1.3` (gRPC localhost:50051)
- 수정 빌드 시뮬: `rby1-sim:dev`(M v1.3) / `dev-m1.2` / `dev-a` (docker/sim.dockerfile)
- 클라이언트: rby1-sdk Python 0.10.0 (PyPI), sibling 체크아웃 examples
- **주의**: 시뮬 컨테이너는 `RBY1_SKIP_CONTROL_TIMEOUT=1`로 실행됨 — 1.8ms 워치독
  (= `dt_ * 0.9`, core control_manager.cpp)이 시뮬에서는 비활성. 타이밍 판정은
  qp_benchmark / GetLastSolveTime 측정값으로 해야 함.

## 모션 스위트 정의
1. `examples/python/24_demo_motion.py --address localhost:50051 --model m`
2. `tools/motion_suite_limits.py --address localhost:50051 --model m`
   - limit-graze: torso 조인트를 URDF 리밋 95%까지, vel scaling 0.8
   - target-switch: 전신 cartesian 타깃 A/B 교대 6회(웜스타트 최악 조건)
   - deepfold(`--scenario deepfold`): 깊은 토르소 폴드 명령 — braking/damper 거동 시험

## 계측 기준선 (Stage 1)
- QP 덤프 훅(`RBY1_QP_DUMP_DIR`, 인스턴스당 256MB 캡)으로 28,237개 실문제 캡처.
- 덤프→재생 재현성: osqp/warm **dx_vs_dump = 0 (28,237/28,237)**.
- 현행 OSQP(eps 1e-3) 기준선: 전신 60×100 p50 57µs / p99 ~100µs, **제약 위반 최대 2.8e-3 rad/s**
  (기본 eps의 실증 — 속도 리밋이 해에서 실제로 초과됨), 전 구간 iter_p50=25(check_termination
  간격 조기 종료).
- 부수 발견: 시뮬은 런 간 비트 비결정적 → 동등성 검증은 동일 입력 재생 A/B
  (`qp_benchmark --ab-backends`)로 해야 함.

## QP 하이지니 (Stage 2) — 게이트 통과
- 비용 행렬 사전할당(row-cursor), `QPSolverSettings`(time limit 2ms→1ms), `GetAConst()` 버그 수정.
- **이전-해 웜스타트(primal+dual, `RBY1_QP_WARMSTART_PREV=1`)**: 비용 0으로 위반 10~100× 개선
  (60×100: 1.4e-3→1.3e-4). eps 1e-5 단독 강화는 팔 21×35 악조건에서 2,475 iter/1.67ms 폭주로 기각.
- 롤백: `RBY1_QP_LEGACY_SETTINGS=1`.

## 백엔드 추상화 (Stage 3) — 게이트 통과
- 내부 `QPBackend` 인터페이스(qp_backend.h), OSQP 로직을 qp_backend_osqp.cpp로 이동
  (nnz 패턴락 웜업데이트 보존). env `RBY1_QP_BACKEND`.
- `--ab-backends`: 전 22파일/28,237레코드 **비트 단위 동일 해** 확인.

## ProxQP/DAQP 백엔드 (Stage 4) — 게이트 통과
- 서브모듈: proxsuite(BSD-2, VECTORIZE off), daqp(MIT). NOTICE 갱신.
- 악조건의 뿌리(스펙트럼 분석): 슬랙 더미 1e-6²=1e-12 고유값 14개 + 여유자유도 nullspace reg
  1e-4²=1e-8 → **cond(H) ~1e12가 구조적**(전 레코드). 폭주 트리거는 "관절이 위치 리밋 5° 마진 내"
  (폭주 188건 전부 margin-in; 운동학적 특이점과는 무상관 — 실측).

## sqrt braking envelope (Stage 5, `RBY1_QP_SQRT_BRAKING=1`) — 게이트 통과
- 1-스텝 정지 법칙 `2(q_lim−q)/dt − q̇`(리밋 고속 접근 시 infeasible→모션 중단)을 이산 시간
  viability 엔벨로프 `v ≤ a(−dt/2 + √(dt²/4 + 2·d_eff/a))`, `d_eff = d − q̇·dt/2`로 교체.
  a_brake = min(URDF qddot, 호출자 가속) — Cartesian 경로의 1e6 가속 무력화 문제도 해결.
- 유닛(tools/test_sqrt_braking): 구법칙 중단 재현 / 신법칙 중단 0·오버슈트 0·감속 ≤ a_brake PASS.

## 슬랙 제거 (Stage 9, `RBY1_QP_NO_SLACK=1`) — 게이트 통과
- 소프트 경계 슬랙의 기능(리밋 앞 완충)은 braking 엔벨로프가 대체. 전신 QP 60×100→**20×20**,
  cond(H) 1e12 해소.
- 발견/수정: 슬랙 제거로 정적 리밋 데드 스톨 노출(demo cartesian2, arm_2=±180° 프레스) →
  `GetPositionBoundPressed()` + cartesian_control의 정체-종료(0.5s 무진행 시 정상 finish)로 해소.
  정체 감지는 해당 컨트롤러의 관절만 본다(next_v는 미선택 관절의 측정 속도를 포함).

## 백엔드 재선정 (Stage 10, no-slack 덤프 37k+) — 게이트 통과

| 백엔드 (eps 1e-6) | 전신 20×20 p50/p99/max | 팔 7×7 max | viol_max | 판정 |
|---|---|---|---|---|
| **ProxQP (박스 모드)** | **16 / 40 / 190µs** | 45µs | **≤9e-7** | **채택** |
| DAQP | 11 / 27 / 43µs | 19µs | ~1e-16 | 폴백(최속) |
| OSQP | 49 / 84 / 159µs | **1,140µs (3,700 iter)** | 2.6e-6 | 리밋-프레스 폭주 — 탈락 |

- ProxQP: no-slack 문제는 A=I 순수 박스 → proxsuite 박스 제약 전용 경로 자동 감지, eps 1e-6이
  1e-3과 동일 비용(백엔드에 min(eps,1e-6) 클램프). 시뮬 내 실측 전신 p50 26µs / p99 ~100µs /
  max 404µs, 실패 0.
- 선정 근거: Pinocchio 생태계 정렬·유지보수성(속도 자체는 DAQP 최속).

## 팔 거리 damper (`RBY1_ARM_DAMPER=1`) — 게이트 통과
- 매 틱 최근접 팔-vs-구조체 쌍에 velocity damper 행 (witness-point space Jacobian,
  d_infl=0.20m, d_safe=0.03m, ξ=0.5m/s, 최대 8행). **소프트 제약 정식화**:
  `a·q̇ + s ≥ −ξ(d−d_s)/(d_i−d_s)`, `s ≥ 0`, 비용 `(50·s)²` — 슬랙이 속도/braking 박스와의
  충돌을 흡수해 **구조적으로 항상 feasible**하고, 위반은 자동으로 최소(=최대 노력 제동).
  위치-슬랙과 달리 강한 가중이라 조건수 무해. 위반 임박 쌍만 행 추가(상시 근접 쌍 필터).
- 하드-행 시도의 교훈(폐기 근거): 진입 속도 초과·다중 행 결합에서 infeasible이 반복되고,
  실현 가능 램프/예산 분배/드롭-재시도는 감속 요구를 무력화하거나 보호 공백을 만들었음 —
  깊은 폴드 시나리오에서 관통(−20~−37mm) 실측. 소프트 전환 후 **+30mm 바닥 정확 유지**.
- 베이스 데크 레일 캡슐을 컨트롤러 dyn 로봇에 런타임 보강(출하 URDF에 베이스 지오메트리 없음 —
  손 근사, **실기 검증 필수**).
- dense 백엔드 전용(OSQP는 매 틱 희소 패턴 변화로 재분해). v1.2 팔 기하(전완 53.5mm 차,
  손목 축 플립)에서 damper-블록 정체 발견 → 정체-종료 게이트에 `GetDamperActive()` 포함으로 해소.
- 유닛(tools/test_arm_damper, 링크 타깃 기반 깊은 폴드): OFF −60mm 관통(기준 결함 실증) /
  ON **+30.2mm** 바닥 유지·abort 0 PASS.

## 주의: SDK MakeState 조인트 이름 라벨 ≠ set_q 순서 (URDF 로드 로봇)
- `LoadRobotFromURDF`로 만든 `dyn::Robot`에서 `MakeState(links, joint_names)`의 조인트 이름
  인자는 **라벨일 뿐 q 순서를 재배열하지 않음** — 이름→인덱스 매핑으로 set_q를 쓰면 엉뚱한
  관절이 움직임(FK 수치 실험으로 확인). 링크 인덱스·자코비안 열·set_q는 내부 순서로 상호 일관.
  core는 제어계가 정식 순서 q를 공급하므로 무관. **네이티브 도구/테스트는 이름 매핑 금지**
  (test_arm_damper는 링크 타깃 기반으로 재작성됨). Rainbow에 SDK 이슈로 보고 권장.
- space Jacobian 관례는 spatial twist(ṗ = ω×p + v)가 맞음을 수치 검증(오차 2e-7).

## 최종 검증 매트릭스
- 기본 설정(env 전부 off): 24_demo 완주 — **기본 동작 무변경** (M v1.2/v1.3, A 전부)
- 권장 조합: 24_demo 완주 + limits 스위트 기준선 동일 + deepfold 정상 종료
- 모델: rby1m v1.3 / v1.2 / rby1a 시뮬 e2e 완료 (도구·런타임 모두 이름 기반 모델 범용)

## 권장 env 세트 (전 기능 opt-in, 기본 무변경, 항목별 한 줄 롤백)
```
RBY1_QP_BACKEND=proxqp  RBY1_QP_NO_SLACK=1  RBY1_QP_SQRT_BRAKING=1  RBY1_ARM_DAMPER=1
(+선택: RBY1_QP_WARMSTART_PREV=1 — OSQP 백엔드용)
```

## 백로그
- 실로봇 검증·배포: env OFF 회귀 → 타이밍 실측(1.8ms 워치독) → 저속 단계 활성화.
  베이스 레일 캡슐 근사의 실측 확인이 damper 신뢰의 관문.
- DAQP workspace 재사용 + active-set 웜스타트(폴백 강화).
- OSQP 완전 은퇴 시 패턴락 더미·슬랙 경로 코드 정리.
- Pinocchio 도입(경계면 확보됨: QP는 행렬 인터페이스, damper 거리 질의는 교체 가능).
- torso_3–torso_4 기구 간섭(충돌 캡슐 미표현 — 실측/스펙 확보 시 제약 편입 검토).
