#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>

#include <google/protobuf/text_format.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "common_types.h"

//using namespace std;
//using namespace indoor_context;

using namespace google::protobuf;

int main(int argc, char **argv) {
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" DESCRIPTOR.proto DATA.pro";
		return -1;
	}

	// Read the descriptor
	io::ErrorCollector errors;
	io::FileInputStream input(open(argv[1]));
	io::Tokenizer tok(&input, &errors);
	io::Parser parser;
	io::FileDescriptorProto file_proto;
	parser.Parse(&tok, &file_proto)

	google::protobuf::DynamicMessage inmsg, outmsg;

	const char* path;
	if (argv[1][0] == '+') {
		path = argv[1]+1;
		inmsg = indoor_context::proto::TruthedMap::default_instance();
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
	write(tmpfd, s.c_str(), s.length());
	close(tmpfd);

	std::string cmd = std::string("emacs -nw ")+tmppath;
	system(cmd.c_str());

	int tmpfd2 = fileno(fopen(tmppath, "r"));
	google::protobuf::io::FileInputStream in2(tmpfd2);
	std::ofstream out2(path, std::ios::binary);

	google::protobuf::TextFormat::Parse(&in2, &outmsg);
	outmsg.PrintDebugString();
	outmsg.SerializeToOstream(&out2);

	return 0;
}

