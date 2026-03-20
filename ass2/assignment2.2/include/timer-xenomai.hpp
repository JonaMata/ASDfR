//=============================================================================
// Authors : Jonathan Matarazzi, Lieuwe van den Berg
// Group : 13
// License : LGPL open source license
//
// Brief : A program that runs a computation on a 1kHz Xenomai timer interval and measures the iteration and computation times.
//
//=============================================================================

#include <signal.h>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <time.h>
#include <cmath>
#include <list>

#include <evl/thread.h>
#include <evl/timer.h>
#include <evl/clock.h>
#include <evl/proxy.h>

const int ITERATIONS = 1000;
const int COMPUTATIONS = 1000;

std::list<long> times = {};
std::list<long> elapsed = {};
std::list<long> compute = {};
long last_t_usec = 0;
long avg_elapsed = 0;

void loop();

void timespec_add_ns(struct timespec *__restrict r,
	     		     const struct timespec *__restrict t,
			     long ns);

int main();
