#include <signal.h>
#include <iostream>
#include <pthread.h>
#include <time.h>
#include <cmath>

void loop(int sig, siginfo_t* si, void* v) {
    std::cout << "loop\n";
}

int main() {
    pthread_t tid;
    
    pthread_create(&tid, nullptr, [](void*) -> void* {
        for (int i = 0; i < 999; i++) {
            std::cout << "loop = " << i;
            timer_t timer_id;
            struct sigevent sev;
            sev.sigev_notify = SIGEV_SIGNAL;
            sev.sigev_signo = SIGUSR1;
            sev.sigev_value.sival_ptr = &timer_id;

            struct itimerspec its;
            its.it_interval.tv_sec = 0;
            its.it_interval.tv_nsec = 1e6;
            its.it_value.tv_sec = 0;
            its.it_value.tv_nsec = 1e6;

            struct sigaction sa;
            sa.sa_flags = SA_SIGINFO;
            sa.sa_sigaction = loop;
            sigemptyset(&sa.sa_mask);
            if (sigaction(SIGUSR1, &sa, nullptr) == -1) {
                perror("sigaction failed");
                return nullptr;
            }

            timer_create(CLOCK_MONOTONIC, &sev, &timer_id);

            timer_settime(timer_id, 0, &its, nullptr);

            // struct timespec next;
            // clock_gettime(CLOCK_MONOTONIC, &next);

            // long last_t_usec = 0;
            // int avg_elapsed = 0;

            // for (int i = 0; i < 1000; ++i) {
            //     struct timespec start, end;
            //     clock_gettime(CLOCK_MONOTONIC, &start);

            //     // computation
            //     double x = 0;
            //     for (int j = 0; j < 10000; ++j) {
                //         x += std::sin(j) * std::cos(j);
                //     }

                //     clock_gettime(CLOCK_MONOTONIC, &end);

            //     // calculate time taken for computation
            //     long elapsed_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            //     long sleep_ns = 1e6 - elapsed_ns;

            //     if (sleep_ns > 0) {
                //         next.tv_nsec += sleep_ns;
                //         // nsec may not overflow so we bump the nsec to the sec
            //         if (next.tv_nsec >= 1e9) {
                //             next.tv_sec += 1;
            //             next.tv_nsec -= 1e9;
            //         }
            //     }

            //     // sleep until next period
            //     clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

            //     // print time
            //     struct timespec now;
            //     clock_gettime(CLOCK_MONOTONIC, &now);
            //     long t_usec = now.tv_sec * 1e6 + now.tv_nsec / 1e3;
            //     std::cout << "Loop at " << t_usec << " us";
            //     if (last_t_usec != 0) {
                //         std::cout << " (elapsed: " << t_usec - last_t_usec << " us) sleep: " << sleep_ns / 1e3 << " compute: " << elapsed_ns / 1e3;
                //         avg_elapsed += t_usec - last_t_usec;
                //     }
                //     std::cout << "\n";
                //     last_t_usec = t_usec;
                // }

                // std::cout << "Avg elapsed: " << avg_elapsed / 999 << " us\n";

        }
        return nullptr;
        }, nullptr);

    pthread_join(tid, nullptr);
    return 0;
}