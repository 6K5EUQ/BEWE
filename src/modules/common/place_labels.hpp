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

// 한국 주요 무역/어항 (KR bbox 32~43N, 123~132E 안). 굵직한 항만 위주 16곳.
// 이름은 영문 표기 — 현재 ImGui 기본 폰트(ASCII 전용)에 한글 글리프가 없어서.
// 좌표 = 각 항만 대표 접안부두/터미널 위치, 소수 4자리(~11m). 웹조사(위키 geo-tag/
// OSM Nominatim/포트DB) 기반 — 조사 출처·신뢰도는 커밋 및 조사 세션 참조.
static const PlaceLabel KR_PORTS[] = {
    { 35.1039, 129.0789, "Busan" },        // 부산항 북항 기준점(위키)
    { 37.4650, 126.6160, "Incheon" },      // 인천항 내항 갑문 일대(±0.5km)
    { 35.5170, 129.3670, "Ulsan" },        // 울산항 본항 기준점(위키)
    { 34.7444, 127.7466, "Yeosu" },        // 여수항 연안여객터미널(주소, ±0.3km)
    { 34.9167, 127.6833, "Gwangyang" },    // 광양항 컨테이너터미널(포트DB)
    { 36.9931, 126.8030, "Pyeongtaek" },   // 평택항 기준점(포트DB)
    { 34.7853, 126.3789, "Mokpo" },        // 목포항 연안여객터미널(주소)
    { 36.0350, 129.3650, "Pohang" },       // 포항 구항/동빈내항(±0.5km)
    { 35.9719, 126.5598, "Gunsan" },       // 군산항 현 상업항 주부두(OSM)
    { 34.8398, 128.4203, "Tongyeong" },    // 통영항 여객선터미널(OSM)
    { 33.5194, 126.5352, "Jeju" },         // 제주항 연안여객터미널(OSM)
    { 37.4890, 129.1246, "Donghae" },      // 동해항 본항 입구(OSM, 묵호 아님)
    { 38.2112, 128.5978, "Sokcho" },       // 속초항 국제여객터미널(OSM)
    { 35.1985, 128.5735, "Masan" },        // 마산항 구항 제1부두 일대(OSM 역지오코드)
    { 35.1681, 128.5808, "Gapo" },         // 가포신항(마산 서항, OSM)
    { 35.1325, 128.6949, "Jinhae" },       // 진해항 진해내항 제1부두(OSM)
};
static const size_t KR_PORTS_COUNT = sizeof(KR_PORTS)/sizeof(KR_PORTS[0]);

} // namespace modview_map
