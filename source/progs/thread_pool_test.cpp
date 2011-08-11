#include <boost/thread/mutex.hpp>

#include "entrypoint_types.h"
#include "thread_pool.h"

boost::mutex m;

void Process(int nn) {
	int steps;
	int n = nn;
	for (steps = 0; n != 1; steps++) {
		if (n % 2 == 0) n /= 2;
		else n = n*3+1;
	}
	boost::mutex::scoped_lock lock(m);
	cout << nn << ": " << steps << " steps" << endl;
}

int main(int argc, char **argv) {
	thread_pool pool;
	for (int i = 1000000; i < 200000000; i+=1000000) {
		pool.add(bind(&Process, i));
	}
	pool.join();

	return 0;
}
