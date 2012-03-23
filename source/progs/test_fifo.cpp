#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>

int main(int argc, char **argv) {

	char* path = tempnam(NULL, NULL);
	mkfifo(path);
	FILE* fd = fopen(path, "w");
	fputs("test string");
	fclose(fd);

	FILE* gd = fopen(path, "r");
	int n = fread

	printf(path);
	//unlink(path);
	return 0;
}
