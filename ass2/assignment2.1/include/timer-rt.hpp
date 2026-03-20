//=============================================================================
// Authors : Jonathan Matarazzi, Lieuwe van den Berg
// Group : 13
// License : LGPL open source license
//
// Brief : A program that runs a computation on a 1kHz POSIX timer interval and measures the iteration and computation times.
//
//=============================================================================

#include <signal.h>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <time.h>
#include <cmath>
#include <list>

const int ITERATIONS = 1000;
const int COMPUTATIONS = 1000;

std::list<long> times = {};
std::list<long> elapsed = {};
std::list<long> compute = {};
long last_t_usec = 0;
long avg_elapsed = 0;

void loop();

int main();