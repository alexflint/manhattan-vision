#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "common_types.h"
#include "map.pb.h"

#define PROTO_CLASS indoor_context::proto::TruthedMap

using namespace std;

int main(int argc, char **argv) {
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" MAP.pro";
		return -1;
	}

	PROTO_CLASS inmsg, outmsg;

	const char* path;
	if (argv[1][0] == '+') {
		path = argv[1]+1;
		inmsg = PROTO_CLASS::default_instance();
	} else {
		path = argv[1];
		struct stat statdata;
		if (stat(path, &statdata)) {
			perror("Error reading input file");
			printf("Use %s +%s to create a new file\n", argv[0], argv[1]);
			return -1;
		}
		std::ifstream in1(argv[1], std::ios::binary);
		inmsg.ParseFromIstream(&in1);
	}

	char tmppath[] = "/tmp/proto-XXXXXX";
	int tmpfd = mkstemp(tmppath);

	std::string s = inmsg.DebugString();
	size_t n = write(tmpfd, s.c_str(), s.length());
	close(tmpfd);

	std::string cmd = std::string("emacs -nw ")+tmppath;
	system(cmd.c_str());

	int tmpfd2 = fileno(fopen(tmppath, "r"));
	google::protobuf::io::FileInputStream in2(tmpfd2);

	google::protobuf::TextFormat::Parse(&in2, &outmsg);
	outmsg.PrintDebugString();
	bool repeat = true;
	while (repeat) {
		cout << "Save changes to " << path << "? (y or n) ";
		string ans;
		cin >> ans;
		if (ans == "y" || ans == "Y") {
			std::ofstream out2(path, std::ios::binary);
			outmsg.SerializeToOstream(&out2);
			repeat = false;
		} else if (ans == "n" || ans == "N") {
			repeat = false;
		}
	}

	return 0;
}

