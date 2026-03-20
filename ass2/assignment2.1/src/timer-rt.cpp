//=============================================================================
// Authors : Jonathan Matarazzi, Lieuwe van den Berg
// Group : 13
// License : LGPL open source license
//
// Brief : A program that runs a computation on a 1kHz POSIX timer interval and measures the iteration and computation times.
//
//=============================================================================

#include "timer-rt.hpp"

// void loop(int sig, siginfo_t* si, void* v) {
void loop() {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    // computation
    double x = 0;
    for (int j = 0; j < COMPUTATIONS; j++) {
        x += std::sin(j) * std::cos(j);
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    // calculate time taken for computation
    long elapsed_ns = (end.tv_sec - start.tv_sec) * 1e6 + (end.tv_nsec - start.tv_nsec) / 1e3;

    // print time
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long t_usec = now.tv_sec * 1e6 + now.tv_nsec / 1e3;
    if (last_t_usec != 0) {
        times.push_back(t_usec);
        elapsed.push_back(t_usec - last_t_usec);
        compute.push_back(elapsed_ns);
        avg_elapsed += t_usec - last_t_usec;
    }
    last_t_usec = t_usec;
}

int main() {
    pthread_t tid;

    // create a signal mask so the SIGUSR1 is queued for the sigwait
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &sigset, nullptr);

    pthread_create(&tid, nullptr, [](void*) -> void* {
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

        timer_create(CLOCK_MONOTONIC, &sev, &timer_id);

        timer_settime(timer_id, 0, &its, nullptr);

        // create a sigset for sigwait to wait for SIGUSR1
        sigset_t sigset;
        sigemptyset(&sigset);
        sigaddset(&sigset, SIGUSR1);

        while (times.size() < ITERATIONS) {
            int sig;
            sigwait(&sigset, &sig);
            loop();
        }

        return nullptr;
        }, nullptr);

    pthread_join(tid, nullptr);
    std::cout << "Average elapsed: " << avg_elapsed / ITERATIONS << " us\n";

    std::ofstream output("output.csv");
    if (!output.is_open()) {
        std::cout << "Failed to open output file\n";
    } else {
        output << "time,elapsed_time,compute_time\n";
        auto t = times.begin();
        auto e = elapsed.begin();
        auto c = compute.begin();
        for (; t != times.end() && e != elapsed.end() && c != compute.end(); ++t, ++e, ++c) {
            output << *t << "," << *e << "," << *c << "\n";
        }
    }

    return 0;
}