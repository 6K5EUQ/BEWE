#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "session_args.hpp"
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>
#include <atomic>
#include <signal.h>

// SIGINT/SIGTERM 수신 시 main loop가 정상 종료 경로(미션 자동 end + cleanup) 거치도록
// 플래그만 set. ui.cpp / cli_host.cpp 가 매 iteration에서 폴링.
std::atomic<bool> g_signal_shutdown{false};
static void bewe_sig_handler(int){ g_signal_shutdown.store(true); }
static void install_signal_handlers(){
    struct sigaction sa{};
    sa.sa_handler = bewe_sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // SA_RESTART 없음 — sleep/read를 깨워야 main loop가 깨어남
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

#ifndef BEWE_HEADLESS
#include "sat_view.hpp"
#endif

#ifdef BEWE_HOST_BUILD
#include "df/df_selftest.hpp"   // --df-selftest
#endif

SessionArgs g_session_args;

static bool starts_with(const char* s, const char* prefix){
    size_t n = std::strlen(prefix);
    return std::strncmp(s, prefix, n) == 0;
}

static void parse_args(int argc, char** argv){
    for(int i = 1; i < argc; i++){
        const char* a = argv[i];
#ifdef BEWE_HOST_BUILD
        // SDR 은 HOST 빌드만 연다. GUI(JOIN)는 하드웨어를 아예 링크하지 않으므로
        // 이 플래그를 받아도 할 수 있는 게 없다.
        // DF DSP 회귀 하네스. 합성 배열만 쓰므로 SDR 도 heimdall 도 필요 없다.
        // 즉시 실행하고 실패 개수를 종료코드로 낸다 — CI/스크립트에서 바로 쓴다.
        if(starts_with(a, "--df-selftest")){
            int verb = 1;
            const char* eq = std::strchr(a, '=');
            if(eq) verb = std::atoi(eq + 1);
            std::exit(df::run_selftest(verb));
        }
        if(std::strcmp(a, "--sdr") == 0 && i+1 < argc){
            std::string v = argv[i+1];
            if(v == "bladerf" || v == "rtlsdr" || v == "pluto" || v == "kraken"){
                g_sdr_force = v;
                fprintf(stderr, "[BEWE] forcing SDR = %s\n", v.c_str());
            } else {
                fprintf(stderr, "[BEWE] unknown --sdr '%s' (use bladerf|rtlsdr|pluto|kraken)\n", v.c_str());
            }
            i++;
        } else if(starts_with(a, "--session-mode=")){
#else
        if(starts_with(a, "--session-mode=")){
#endif
            std::string v = a + std::strlen("--session-mode=");
#ifdef BEWE_HOST_BUILD
            if(v == "host" || v == "join"){
#else
            // GUI 는 JOIN 전용이다. host 세션은 cli_host 가 맡는다.
            if(v == "join"){
#endif
                g_session_args.mode_set = true;
                g_session_args.mode     = v;
            } else {
#ifdef BEWE_HOST_BUILD
                fprintf(stderr, "[BEWE] unknown --session-mode '%s' (use host|join)\n", v.c_str());
#else
                fprintf(stderr, "[BEWE] unknown --session-mode '%s' (use join)\n", v.c_str());
#endif
            }
        } else if(starts_with(a, "--station-id=")){
            g_session_args.station_id = a + std::strlen("--station-id=");
        } else if(starts_with(a, "--station-name=")){
            g_session_args.station_name = a + std::strlen("--station-name=");
        } else if(starts_with(a, "--station-lat=")){
            g_session_args.station_lat = (float)std::atof(a + std::strlen("--station-lat="));
        } else if(starts_with(a, "--station-lon=")){
            g_session_args.station_lon = (float)std::atof(a + std::strlen("--station-lon="));
        } else if(std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0){
            fprintf(stderr,
                "BEWE options:\n"
#ifdef BEWE_HOST_BUILD
                "  --sdr bladerf|rtlsdr|pluto|kraken  force a specific SDR backend\n"
                "  --df-selftest[=verbosity]    run the DF DSP regression suite and exit\n"
                "  --session-mode=host|join     internal: child-process boot mode\n"
#else
                "  --session-mode=join          internal: child-process boot mode\n"
#endif
                "  --station-id=<id>            internal: Central room id (JOIN)\n"
                "  --station-name=<utf8>        internal: station display name\n"
                "  --station-lat=<deg>          internal: station latitude (HOST)\n"
                "  --station-lon=<deg>          internal: station longitude (HOST)\n"
                "  -h, --help                   this message\n");
        }
    }
}

#ifdef BEWE_HEADLESS
int main(int argc, char** argv){
    parse_args(argc, argv);
    BEWEPaths::ensure_dirs();
    install_signal_handlers();
    run_cli_host();
    return 0;
}
#else
int main(int argc, char** argv){
    parse_args(argc, argv);
    BEWEPaths::ensure_dirs();
    setenv("GTK_IM_MODULE","none",1);
    setenv("QT_IM_MODULE","none",1);
    setenv("XMODIFIERS","@im=none",1);
    setenv("GLFW_IM_MODULE","none",1);
    install_signal_handlers();
    // TLE fetch is now lazy: only when the user picks ALL in the sat tracker.
    run_streaming_viewer();
    return 0;
}
#endif
