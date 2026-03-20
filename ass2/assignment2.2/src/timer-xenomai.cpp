//=============================================================================
// Authors : Jonathan Matarazzi, Lieuwe van den Berg
// Group : 13
// License : LGPL open source license
//
// Brief : A program that runs a computation on a 1kHz Xenomai timer interval and measures the iteration and computation times.
//
//=============================================================================

#include "timer-xenomai.hpp"

// void loop(int sig, siginfo_t* si, void* v) {
void loop() {
    struct timespec start, end;
    evl_read_clock(EVL_CLOCK_MONOTONIC, &start);

    // computation
    double x = 0;
    for (int j = 0; j < COMPUTATIONS; j++) {
        x += std::sin(j) * std::cos(j);
    }

    evl_read_clock(EVL_CLOCK_MONOTONIC, &end);

    // calculate time taken for computation
    long elapsed_ns = (end.tv_sec - start.tv_sec) * 1e6 + (end.tv_nsec - start.tv_nsec) / 1e3;

    // print time
    struct timespec now;
    // evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
    long t_usec = start.tv_sec * 1e6 + start.tv_nsec / 1e3;
    if (last_t_usec != 0) {
        times.push_back(t_usec);
        elapsed.push_back(t_usec - last_t_usec);
        compute.push_back(elapsed_ns);
        avg_elapsed += t_usec - last_t_usec;
    }
    last_t_usec = t_usec;
}

void timespec_add_ns(struct timespec *__restrict r,
	     		     const struct timespec *__restrict t,
			     long ns)
{
	long s, rem;

	s = ns / 1000000000;
	rem = ns - s * 1000000000;
	r->tv_sec = t->tv_sec + s;
	r->tv_nsec = t->tv_nsec + rem;
	if (r->tv_nsec >= 1000000000) {
			r->tv_sec++;
			r->tv_nsec -= 1000000000;
	}
}

int main() {
    pthread_t tid;


    pthread_create(&tid, nullptr, [](void*) -> void* {
		struct itimerspec value, ovalue;
		int tfd, tmfd, ret, n = 0;
		struct timespec now;
		__u64 ticks;

		/* Attach to the core. */
		tfd = evl_attach_self("periodic-timer");

		/* Create a timer on the built-in monotonic clock. */
		tmfd = evl_new_timer(EVL_CLOCK_MONOTONIC);

		/* Set up a 1 kHz periodic timer. */
		ret = evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
		/* EVL always uses absolute timeouts, add 1s to the current date */
		timespec_add_ns(&value.it_value, &now, 3000000000ULL);
		value.it_interval.tv_sec = 0;
		value.it_interval.tv_nsec = 1e6;
		ret = evl_set_timer(tmfd, &value, &ovalue);

        while (times.size() < ITERATIONS) {
			/* Wait for the next tick to be notified. */
		    ret = oob_read(tmfd, &ticks, sizeof(ticks));
            if (ticks > 0) {
            	loop();
			}
        }

		/* Disable the timer (not required if closing). */
		value.it_interval.tv_sec = 0;
		value.it_interval.tv_nsec = 0;
		ret = evl_set_timer(tmfd, &value, NULL);

		return 0;


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
