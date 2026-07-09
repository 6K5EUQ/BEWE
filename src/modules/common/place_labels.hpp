#pragma once
// ── 지도 오버레이용 한국 주요 항구 지명 (위경도 + 영문 이름) ─────────────────
// modview_map(map_view.cpp)이 배/기지 마커와 별도로 항구 위치를 앵커(⚓)+이름으로
// 그린다. 정적 데이터 — 네트워크/파일 없음. 좌표는 각 항만 접안구역 근사 중심.
//   - map_view.cpp 에서만 include (GUI 빌드 전용, 1회 컴파일).
#include <cstddef>

namespace modview_map {

struct PlaceLabel {
    double      lat, lon;
    const char* name;
};

// 한국 주요 무역/어항 (KR bbox 32~43N, 123~132E 안). 굵직한 항만 위주 13곳.
// 이름은 영문 표기 — 현재 ImGui 기본 폰트(ASCII 전용)에 한글 글리프가 없어서.
static const PlaceLabel KR_PORTS[] = {
    { 35.1010, 129.0400, "Busan" },
    { 37.4520, 126.5980, "Incheon" },
    { 35.5000, 129.3870, "Ulsan" },
    { 34.7460, 127.7500, "Yeosu" },
    { 34.9410, 127.6960, "Gwangyang" },
    { 36.9660, 126.8200, "Pyeongtaek" },
    { 34.7860, 126.3820, "Mokpo" },
    { 36.0180, 129.3650, "Pohang" },
    { 35.9750, 126.6360, "Gunsan" },
    { 34.8180, 128.4190, "Tongyeong" },
    { 33.5170, 126.5270, "Jeju" },
    { 37.7530, 128.9110, "Donghae" },
    { 38.2070, 128.5920, "Sokcho" },
};
static const size_t KR_PORTS_COUNT = sizeof(KR_PORTS)/sizeof(KR_PORTS[0]);

} // namespace modview_map
