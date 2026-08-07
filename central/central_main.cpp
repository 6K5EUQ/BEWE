#include "central_server.hpp"
#include <cstdio>
#include <cstring>
#include <csignal>
#include <atomic>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <ctime>
#include <string>
#include <sys/stat.h>

// async-signal-safe 정책: sig handler는 atomic flag만 set. mutex/IO/join 금지.
// 두번째 Ctrl+C는 강제 종료 (정상 stop()이 어딘가 stuck인 경우의 비상 탈출).
static std::atomic<int>  g_sigint_count{0};
static std::atomic<bool> g_should_stop{false};
static void sig_handler(int){
    int n = g_sigint_count.fetch_add(1) + 1;
    if(n >= 2){
        // 두번째 신호: 즉시 _exit (stdio flush 안 함 — async-signal-safe)
        _exit(130);
    }
    g_should_stop.store(true);
}

// ── 궤도원소 자동 수집 ───────────────────────────────────────────────────────
// Central 이 켜져 있으면 원소는 알아서 쌓여야 한다. cron 에 맡겼더니 스크립트가
// 배포되기 전에 등록돼 한 번도 안 돌았고(로그 파일조차 없었다), 그 사이 녹화분은
// 12일 낡은 원소로 매칭돼 순위가 조용히 틀렸다. 서버가 자기 데이터를 자기가 챙긴다.
//
// 스크립트를 부르는 이유: 수집은 Celestrak/space-track HTTP + TLE 필터링인데
// 이미 central-tle-sync.sh 에 다 있다. C++ 로 옮기면 curl/awk 의존만 늘고 같은
// 로직이 두 벌 된다.
static void tle_sync_thread(const std::atomic<bool>& stop){
    const char* home = getenv("HOME");
    if(!home) return;
    const std::string script = std::string(home) + "/BEWE/scripts/central-tle-sync.sh";
    const std::string tledir = std::string(home) + "/BEWE/DataBase/tle";
    struct stat st{};
    if(stat(script.c_str(), &st) != 0){
        printf("[Central] TLE auto-sync disabled (no %s)\n", script.c_str());
        return;
    }
    // 부팅 직후 한 번 재우는 이유: 기동 로그와 수집 로그가 엉키지 않게, 그리고
    // 재시작을 연달아 해도 Celestrak 을 두들기지 않게.
    int wait_s = 60;
    while(!stop.load()){
        for(int i = 0; i < wait_s && !stop.load(); i++)
            std::this_thread::sleep_for(std::chrono::seconds(1));
        if(stop.load()) break;

        // 오늘 원소가 이미 있으면 건너뛴다. 파일명은 받아 온 원소의 **중앙 에폭**
        // 으로 지어지므로(스크립트) 보통 어제 날짜가 된다 — 어제것이 있으면 오늘
        // 실행해도 같은 파일을 덮어쓸 뿐이라, 오늘/어제 둘 다 없을 때만 돈다.
        const time_t now = time(nullptr);
        bool have = false;
        for(int back = 0; back <= 1 && !have; back++){
            const time_t t = now - (time_t)back*86400;
            struct tm g{}; gmtime_r(&t, &g);
            char p[256];
            snprintf(p, sizeof p, "%s/leo_%04d%02d%02d.txt", tledir.c_str(),
                     g.tm_year+1900, g.tm_mon+1, g.tm_mday);
            struct stat s2{};
            if(stat(p, &s2) == 0 && s2.st_size > 0) have = true;
        }
        if(!have){
            printf("[Central] TLE sync: fetching current elements...\n");
            const std::string cmd = "\"" + script + "\" >> \"" + std::string(home)
                                  + "/bewe-tle.log\" 2>&1";
            const int rc = system(cmd.c_str());
            printf("[Central] TLE sync: rc=%d\n", rc);
        }
        wait_s = 6*3600;   // 6시간마다 확인 — 놓친 날을 하루 안에 메운다
    }
}

int main(int argc, char** argv){
    setbuf(stdout, nullptr);  // stdout 라인 버퍼링 해제 → 즉시 출력
    setbuf(stderr, nullptr);
    int port = CENTRAL_PORT; // 기본 7700 (단일 포트) 나중에 보안검토 다시 할때 수정하기 ...
    for(int i=1; i<argc; i++){
        if(!strcmp(argv[i],"--port") && i+1<argc) port = atoi(argv[++i]);
    }

    printf("=== BEWE Central Server ===\n");
    printf("  Port: %d (HOST/JOIN/LIST 통합)\n", port);
    printf("Press Ctrl+C to stop. (Twice for force quit.)\n\n");

    CentralServer central_srv;
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    if(!central_srv.start(port)){
        fprintf(stderr,"[Central] start failed\n");
        return 1;
    }
    std::thread tle_th(tle_sync_thread, std::cref(g_should_stop));

    // sig handler는 flag만 set. 메인 루프가 polling으로 stop() 호출 — 재진입/락 데드락 방지.
    while(!g_should_stop.load()){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    printf("[Central] received shutdown signal, stopping...\n");
    if(tle_th.joinable()) tle_th.join();
    central_srv.stop();
    printf("[Central] shutdown complete\n");
    return 0;
}
