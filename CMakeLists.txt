cmake_minimum_required(VERSION 3.16)
project(BEWE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CLI "Build CLI-only binary (no OpenGL/GLFW/ImGui)" OFF)

# ── 선택 설치형 모듈 (src/modules/<id>/) — 존재하는 모듈만 컴파일 ─────────────
# 폴더가 없으면 빈 목록 → 코어만 빌드 (기능 흔적 없음).
# *_view.cpp 는 ImGui 의존 → GUI 빌드 전용.
file(GLOB BEWE_MODULE_SRCS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/modules/*/*.cpp)
file(GLOB BEWE_MODULE_VIEW_SRCS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/modules/*/*_view.cpp)
set(BEWE_MODULE_SRCS_CLI ${BEWE_MODULE_SRCS})
if(BEWE_MODULE_VIEW_SRCS)
    list(REMOVE_ITEM BEWE_MODULE_SRCS_CLI ${BEWE_MODULE_VIEW_SRCS})
endif()

# ── BEAE AIS AI (Match_AI) — 독립 판매/제거 가능 모듈 ─────────────────────────
# ais_ai.cpp 있을 때만 컴파일 define 주입. 파일 삭제 = Match_AI 열·기능·링크 전부
# 자동 제외(host_ai/ai_enabled 호출부가 #ifdef 로 빠짐) → 코어 운용 무영향.
if(EXISTS ${CMAKE_SOURCE_DIR}/src/modules/ais/ais_ai.cpp)
    add_compile_definitions(BEWE_MODULE_AIS_AI)
endif()

# ── GUARD (차세대 해양관제 플랫폼: 경보/의사결정지원) — 모듈 존재 시만 define ──
# ais_view 오버레이·ais_module 데몬 연결부가 #ifdef 로 빠짐 → 폴더 제거 = 무영향.
if(EXISTS ${CMAKE_SOURCE_DIR}/src/modules/guard/guard_module.cpp)
    add_compile_definitions(BEWE_MODULE_GUARD)
endif()

# ── mbelib (DMR AMBE+2 음성 보코더, ISC) — DMR 모듈 있을 때만 컴파일 ──────────
# 모듈 부재 = 흔적 없음 원칙 유지. libs/mbelib 는 C 소스.
set(MBELIB_SRCS "")
if(EXISTS ${CMAKE_SOURCE_DIR}/src/modules/dmr)
    file(GLOB MBELIB_SRCS ${CMAKE_SOURCE_DIR}/libs/mbelib/*.c)
endif()

find_package(PkgConfig REQUIRED)
find_package(ZLIB REQUIRED)              # 모듈 히스토리 스트림 압축
pkg_check_modules(FFTW    REQUIRED fftw3f)
pkg_check_modules(ALSA    REQUIRED alsa)
pkg_check_modules(MPG123  REQUIRED libmpg123)
pkg_check_modules(LIBPNG  REQUIRED libpng)
pkg_check_modules(OPUS   REQUIRED opus)      # 오디오 Opus 코덱 (HOST 인코드/JOIN 디코드)
pkg_check_modules(ZSTD   REQUIRED libzstd)   # FFT uint8 무손실 추가압축

# ── SDR 드라이버 — HOST(CLI) 빌드 전용 ────────────────────────────────────
# JOIN(GUI)은 하드웨어를 열지 않으므로 이 -dev 패키지들이 필요 없다. REQUIRED 를
# 무조건 걸어두면 JOIN 만 쓰는 PC 에서도 설치를 강요하게 된다.
if(CLI)
    pkg_check_modules(BLADERF REQUIRED libbladeRF)
    pkg_check_modules(RTLSDR  REQUIRED librtlsdr)
    pkg_check_modules(VOLK    REQUIRED volk)
    pkg_check_modules(IIO     REQUIRED libiio)
    pkg_check_modules(AD9361  REQUIRED libad9361)
endif()

if(CLI)
    # ── CLI build (no OpenGL/GLFW/ImGui) ──────────────────────────────────
    add_executable(BEWE
        src/main.cpp
        src/cli_host.cpp
        src/host_state.cpp
        src/fft_viewer.cpp
        src/bladerf_io.cpp
        src/demod.cpp
        src/module_registry.cpp
        ${BEWE_MODULE_SRCS_CLI}
        ${MBELIB_SRCS}
        src/iq_record.cpp
        src/sched_record.cpp
        src/mission.cpp
        src/mission_push.cpp
        src/hist_check.cpp
        src/audio.cpp
        src/timemachine.cpp
        src/region_save.cpp
        src/rtlsdr_io.cpp
        src/hw_detect.cpp
        src/login.cpp
        src/net_server.cpp
        src/net_client.cpp
        src/net_stream.cpp
        src/central_client.cpp
        src/host_band_plan.cpp
        src/host_band_categories.cpp
        src/long_waterfall.cpp
        src/pluto_io.cpp
        src/kraken_io.cpp
        src/df/heimdall_client.cpp
        src/df/df_linalg.cpp
        src/df/df_manifold.cpp
        src/df/df_xspec.cpp
        src/df/df_estimator.cpp
        src/df/df_engine.cpp
        # 합성 배열 시뮬레이터 회귀 하네스. `BEWE --df-selftest` 로만 도달한다
        # (main.cpp). DF DSP 를 고칠 때 유일한 검증 수단이라 HOST 빌드에 넣는다.
        src/df/df_selftest.cpp
    )

    target_compile_definitions(BEWE PRIVATE BEWE_HEADLESS=1)

    target_include_directories(BEWE PRIVATE
        src
        ${CMAKE_SOURCE_DIR}/libs/mbelib
        ${BLADERF_INCLUDE_DIRS}
        ${FFTW_INCLUDE_DIRS}
        ${ALSA_INCLUDE_DIRS}
        ${MPG123_INCLUDE_DIRS}
        ${RTLSDR_INCLUDE_DIRS}
        ${LIBPNG_INCLUDE_DIRS}
        ${VOLK_INCLUDE_DIRS}
        ${IIO_INCLUDE_DIRS}
        ${AD9361_INCLUDE_DIRS}
        ${OPUS_INCLUDE_DIRS}
        ${ZSTD_INCLUDE_DIRS}
    )

    target_link_libraries(BEWE PRIVATE
        ${BLADERF_LIBRARIES}
        ${FFTW_LIBRARIES}
        ${ALSA_LIBRARIES}
        ${MPG123_LIBRARIES}
        ${RTLSDR_LIBRARIES}
        ${LIBPNG_LIBRARIES}
        ${VOLK_LIBRARIES}
        ${IIO_LIBRARIES}
        ${AD9361_LIBRARIES}
        ${OPUS_LIBRARIES}
        ${ZSTD_LIBRARIES}
        ZLIB::ZLIB
        pthread
        m
    )

    target_compile_options(BEWE PRIVATE -O3 -march=native -DNDEBUG)

else()
    # ── GUI build (original, unchanged) ───────────────────────────────────
    find_package(OpenGL REQUIRED)
    find_package(GLEW   REQUIRED)
    find_package(glfw3  REQUIRED)

    set(IMGUI_PATH          ${CMAKE_SOURCE_DIR}/libs/imgui)
    set(IMGUI_BACKENDS_PATH ${IMGUI_PATH}/backends)
    set(IMGUI_SOURCES
        ${IMGUI_PATH}/imgui.cpp
        ${IMGUI_PATH}/imgui_draw.cpp
        ${IMGUI_PATH}/imgui_tables.cpp
        ${IMGUI_PATH}/imgui_widgets.cpp
        ${IMGUI_BACKENDS_PATH}/imgui_impl_glfw.cpp
        ${IMGUI_BACKENDS_PATH}/imgui_impl_opengl3.cpp
    )

    # GUI = JOIN 전용. HOST 쪽 소스(SDR 백엔드·NetServer·복조·녹음 엔진·DF 엔진)는
    # 여기 없다 — 링크 단계에서 아예 불가능하게 만드는 것이 이 분리의 핵심이다.
    # (HOST 는 build-cli 의 cli_host.cpp 하나뿐이다.)
    add_executable(BEWE
        src/main.cpp
        src/fft_viewer.cpp
        src/module_registry.cpp
        src/demod_panel.cpp
        ${BEWE_MODULE_SRCS}
        ${MBELIB_SRCS}
        src/iq_record.cpp
        src/mission.cpp
        src/audio.cpp
        src/audio_playback.cpp
        src/ui.cpp
        src/timemachine.cpp
        src/sa_compute.cpp
        src/eid_compute.cpp
        src/login.cpp
        src/net_client.cpp
        src/globe.cpp
        src/sat_tle.cpp
        src/SGP4.cpp
        src/MathTimeLib.cpp
        src/sat_view.cpp
        src/central_client.cpp
        src/host_band_categories.cpp
        src/long_waterfall.cpp
        src/long_waterfall_view.cpp
        src/sig_lib_view.cpp
        src/mission_view.cpp
        src/df_view.cpp
        src/df_join.cpp
        src/session_spawn.cpp
        ${IMGUI_SOURCES}
    )

    target_include_directories(BEWE PRIVATE
        src
        ${CMAKE_SOURCE_DIR}/libs/mbelib
        ${FFTW_INCLUDE_DIRS}
        ${ALSA_INCLUDE_DIRS}
        ${MPG123_INCLUDE_DIRS}
        ${LIBPNG_INCLUDE_DIRS}
        ${OPUS_INCLUDE_DIRS}
        ${ZSTD_INCLUDE_DIRS}
        ${IMGUI_PATH}
        ${IMGUI_BACKENDS_PATH}
    )

    target_link_libraries(BEWE PRIVATE
        ${FFTW_LIBRARIES}
        ${ALSA_LIBRARIES}
        ${MPG123_LIBRARIES}
        ${LIBPNG_LIBRARIES}
        OpenGL::GL
        GLEW::GLEW
        glfw
        ${OPUS_LIBRARIES}
        ${ZSTD_LIBRARIES}
        ZLIB::ZLIB
        pthread
        m
    )

    target_compile_options(BEWE PRIVATE -O3 -march=native -DNDEBUG)

endif()
