	#include <evl/thread.h>
	#include <evl/timer.h>
	#include <evl/clock.h>
	#include <evl/proxy.h>

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

	int main(int argc, char *argv[])
	{
		struct itimerspec value, ovalue;
		int tfd, tmfd, ret, n = 0;
		struct timespec now;
		__u64 ticks;

		/* Attach to the core. */
		tfd = evl_attach_self("periodic-timer:%d", getpid());
		check_this_fd(tfd);

		/* Create a timer on the built-in monotonic clock. */
		tmfd = evl_new_timer(EVL_CLOCK_MONOTONIC);
		check_this_fd(tmfd);

		/* Set up a 1 Hz periodic timer. */
		ret = evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
		check_this_status(ret);
		/* EVL always uses absolute timeouts, add 1s to the current date */
		timespec_add_ns(&value.it_value, &now, 1000000000ULL);
		value.it_interval.tv_sec = 0;
		value.it_interval.tv_nsec = 1e6;
		ret = evl_set_timer(tmfd, &value, &ovalue);
		check_this_status(ret);

		for (;;) {
		    /* Wait for the next tick to be notified. */
		    ret = oob_read(tmfd, &ticks, sizeof(ticks));
		    check_this_status(ret);
		    if (ticks > 1) {
		       	      fprintf(stder, "timer overrun! %lld ticks late\n",
			      	      ticks - 1);
			      break;
		    }
		    evl_printf("TICKED, loops=%d\n", n++);
		}

		/* Disable the timer (not required if closing). */
		value.it_interval.tv_sec = 0;
		value.it_interval.tv_nsec = 0;
		ret = evl_set_timer(tmfd, &value, NULL);
		check_this_status(ret);

		return 0;
}