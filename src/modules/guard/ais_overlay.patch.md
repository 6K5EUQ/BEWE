# GUARD → ais_view.cpp 지도 오버레이 통합 제안 (diff)

GUARD 활성 경보 선박을 AIS 지도 위에 **유형색 펄스 링**(2~3겹 확장 링, 시간기반 알파)으로
표시한다. 전부 `#ifdef BEWE_MODULE_GUARD` 가드 — guard 폴더 제거 시 흔적 없이 빠진다
(CMakeLists.txt 26-30행에서 `guard_module.cpp` 존재 시 define 주입, 이미 반영됨).

사용 API: `guard_mod::vessel_alert(uint32_t mmsi, uint8_t& typ, uint8_t& sev)`
(guard_meta.hpp — 그 MMSI 관련 활성 경보 중 최고 sev 1건 반환, REPORT 제외).

수정 지점은 4곳. 기준 리비전: ais-app 브랜치 현재 (draw_map 호출 = 618행).

---

## 1) include 추가 — 파일 상단 include 블록 끝 (20행 `#include <algorithm>` 직후)

```diff
 #include <algorithm>
+#ifdef BEWE_MODULE_GUARD
+#include "../guard/guard_meta.hpp"   // guard_mod::vessel_alert (경보 선박 펄스 링)
+#endif
```

## 2) 링 캐시 선언 — 공유 캐시 블록 (305행 `static std::vector<std::string> tip1, tip2;` 직후)

```diff
     static std::vector<modview_map::MapPoint> pts;
     static std::vector<std::vector<float>>    trailbuf;
     static std::vector<std::string>           tip1, tip2;
+#ifdef BEWE_MODULE_GUARD
+    struct GuardRing { double lat, lon; uint8_t typ, sev; };   // 경보 선박 위치+유형 (펄스 링용)
+    static std::vector<GuardRing> grings;
+#endif
```

## 3) 링 수집 + 시각적 우선순위 — "지도 오버레이 빌드" 블록 (363~399행)

3-a. 블록 첫 줄 clear 에 grings 포함 (364행):

```diff
     {
-        pts.clear(); trailbuf.clear(); tip1.clear(); tip2.clear();
+        pts.clear(); trailbuf.clear(); tip1.clear(); tip2.clear();
+#ifdef BEWE_MODULE_GUARD
+        grings.clear();
+#endif
```

3-b. 루프 내부, `pts.push_back(mpt);` (391행) 직전 — 표시되는 배(=지도 마커가 실제로
생기는 배)만 질의하므로 표⟺지도 동기 원칙 유지:

```diff
             tip1.emplace_back(b1); tip2.emplace_back(b2);
+#ifdef BEWE_MODULE_GUARD
+            { uint8_t gt, gs;                                   // 활성 경보 선박 → 펄스 링 대상
+              if(guard_mod::vessel_alert(G.mmsi, gt, gs)) grings.push_back({m.lat, m.lon, gt, gs}); }
+#endif
             pts.push_back(mpt);
```

3-c. 포인터 채움 루프 (393~398행) **직후**, 블록 닫기 전 — 경보 선박 마커를 벡터 뒤로
보내 지도에서 맨 위에 그려지게 (draw_map 은 pts 순서대로 그림 = 시각적 우선순위).
포인터는 요소 내부에 저장돼 있어 구조체 재배치에도 안전 (trailbuf/tip 을 가리킴):

```diff
         for(size_t i=0;i<pts.size();i++){
             pts[i].trail   = trailbuf[i].empty()? nullptr : trailbuf[i].data();
             pts[i].trail_n = (int)trailbuf[i].size()/2;
             pts[i].tip_l1  = tip1[i].c_str();
             pts[i].tip_l2  = tip2[i].empty()? nullptr : tip2[i].c_str();
         }
+#ifdef BEWE_MODULE_GUARD
+        // 경보 선박 마커를 뒤로 정렬 → 밀집 해역에서도 경보 배가 위에 그려짐
+        std::stable_partition(pts.begin(), pts.end(), [](const modview_map::MapPoint& p){
+            uint8_t gt, gs; return !guard_mod::vessel_alert((uint32_t)p.id, gt, gs); });
+#endif
     }
```

## 4) 펄스 링 렌더 — draw_map 호출 (618행) 직전/직후

draw_map 은 진입 시 `GetCursorScreenPos()` 를 캔버스 좌상단(p0)으로 쓰고, 등거리원통
투영 `x = p0.x + (lon-lon0)/(lon1-lon0)*W` 로 그린다 (map_view.cpp 152-155행).
호출 직전 커서를 캡처하면 동일 투영을 밖에서 재현 가능 (줌/팬은 draw_map 내부에서
렌더 **전에** 반영되므로 호출 후의 `mv` = 이번 프레임 렌더 카메라와 일치):

```diff
+#ifdef BEWE_MODULE_GUARD
+    ImVec2 map_p0 = ImGui::GetCursorScreenPos();   // draw_map 캔버스 좌상단 (투영 기준점)
+#endif
     auto mres = modview_map::draw_map("##ais_map", mv, pts, ImVec2(mapw, map_h), just_opened, &stns, &links);
+#ifdef BEWE_MODULE_GUARD
+    // ── GUARD 경보 펄스 링: 유형색 3겹 확장 링 (위상 1/3 어긋남 + 알파 페이드) ──
+    if(!grings.empty()){
+        ImDrawList* gdl = ImGui::GetWindowDrawList();
+        gdl->PushClipRect(map_p0, ImVec2(map_p0.x+mapw, map_p0.y+map_h), true);
+        auto ll2px=[&](double lat,double lon){                 // map_view.cpp LL2PX 와 동일 투영
+            return ImVec2((float)(map_p0.x+(lon-mv.lon0)/(mv.lon1-mv.lon0)*mapw),
+                          (float)(map_p0.y+(mv.lat1-lat)/(mv.lat1-mv.lat0)*map_h)); };
+        float tnow=(float)ImGui::GetTime();
+        for(const auto& g : grings){
+            ImU32 base;                                        // guard_view 유형색과 동일 (알파 0)
+            switch(g.typ){ case 1: base=IM_COL32(255, 97, 82,0); break;   // 충돌 빨강
+                           case 2: base=IM_COL32(255,158, 64,0); break;   // 좌초 주황
+                           case 3: base=IM_COL32(242,217, 89,0); break;   // 이상항적 노랑
+                           case 4: base=IM_COL32(204,140,255,0); break;   // 위협 보라
+                           default:base=IM_COL32(115,179,255,0); break; } // 기타 파랑
+            ImVec2 c = ll2px(g.lat, g.lon);
+            float spd = g.sev>=3? 1.6f : 1.0f;                 // 긴급(sev3)은 빠른 펄스
+            for(int k=0;k<3;k++){
+                float ph = fmodf(tnow*spd + k/3.f, 1.f);       // 0→1 확장 위상
+                gdl->AddCircle(c, 7.f+ph*16.f, base | ((ImU32)((1.f-ph)*170.f)<<24), 0, 2.f);
+            }
+            gdl->AddCircle(c, 6.f, base | (200u<<24), 0, 1.5f);   // 고정 내륜 (위치 앵커)
+        }
+        gdl->PopClipRect();
+    }
+#endif
     if(mres.clicked_station>=0 && mres.clicked_station<(int)stn_names.size()){
```

---

## 비고

- `<cmath>`(fmodf), `<algorithm>`(stable_partition) 은 ais_view.cpp 가 이미 include.
- 링은 draw_map 뒤에 같은 window drawlist 로 그려 항상 마커/툴팁 위 레이어.
  클립은 지도 캔버스 사각형으로 제한 — 표 영역 침범 없음.
- grings 는 지도 오버레이 빌드 캐시와 같은 수명(프레임마다 재구성, 표시 선박만) —
  타임라인/필터/Hist 모드와 자동 동기. vessel_alert 는 grps 수(수백) × 프레임 호출이나
  guard 활성 경보는 소수라 비용 무시 가능. 필요 시 3-b 결과를 3-c 에서 재사용하는
  aid set 캐시로 줄일 수 있음 (현재는 단순성 우선).
- mv.big(지도 전체화면)에서도 mapw/map_h 가 그대로 전달되므로 추가 처리 불필요.
