#include <iostream>
#include <pthread.h>
#include <time.h>
#include <cmath>

int main() {
    pthread_t tid;

    pthread_create(&tid, nullptr, [](void*) -> void* {
        struct timespec next;
        clock_gettime(CLOCK_MONOTONIC, &next);

        long last_t_usec = 0;
        int avg_elapsed = 0;

        for (int i = 0; i < 1000; ++i) {
            struct timespec start, end;
            clock_gettime(CLOCK_MONOTONIC, &start);

            // computation
            double x = 0;
            for (int j = 0; j < 10000; ++j) {
                x += std::sin(j) * std::cos(j);
            }

            clock_gettime(CLOCK_MONOTONIC, &end);

            // calculate time taken for computation
            long elapsed_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            long sleep_ns = 1e6 - elapsed_ns;

            if (sleep_ns > 0) {
                next.tv_nsec += sleep_ns;
                // nsec may not overflow so we bump the nsec to the sec
                if (next.tv_nsec >= 1e9) {
                    next.tv_sec += 1;
                    next.tv_nsec -= 1e9;
                }
            }

            // sleep until next period
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

            // print time
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long t_usec = now.tv_sec * 1e6 + now.tv_nsec / 1e3;
            std::cout << "Loop at " << t_usec << " us";
            if (last_t_usec != 0) {
                std::cout << " (elapsed: " << t_usec - last_t_usec << " us) sleep: " << sleep_ns/1e3 << " compute: " << elapsed_ns/1e3;
                avg_elapsed += t_usec - last_t_usec;
            }
            std::cout << "\n";
            last_t_usec = t_usec;
        }

        std::cout << "Avg elapsed: " << avg_elapsed / 999 << " us\n";

        return nullptr;
        }, nullptr);

    pthread_join(tid, nullptr);
    return 0;
}