#include "entrypoint_types.h"
#include "timer.h"

#include "table.tpp"

double arr[250][250][5][5];

int main(int argc, char **argv) {

	Table<4,double> t(250, 250, 5, 5);

	int misses = 0;
	TIMED("Ten million table lookups")
		for (int i = 0; i < 1e+7; i++) {
			double x = t(rand()%250, rand()%250, rand()%5, rand()%5);
			if (x < 0) misses++;
		}

	// = new double[250][250][5][5];

	int misses2 = 0;
	TIMED("Ten million array lookups")
		for (int i = 0; i < 1e+7; i++) {
			double x = arr[rand()%250][rand()%250][rand()%5][rand()%5];
			if (x < 0) misses++;
		}

	DREPORT(misses);
	return 0;
}
