#include "common_types.h"
#include "textons.h"

// Represents a token "empty list" type
struct Empty {
};

// Represents the union of two entities
template <typename Head, typename Tail=Empty>
struct Cons {
	typedef Head head;
	typedef Tail tail;
};

// Represnets a token "null"
struct Nothing {
};

// Unites two elements unless the first one is "Nothing", in which case it returns null
template <typename Head, typename Tail=Empty>
struct ConsIf {
	typedef Cons<Head, typename Tail::result> result;
};

// Special case for a Nothing element
template <typename Tail>
struct ConsIf<Nothing, Tail> {
	typedef Empty result;
};

// Create a list of items
template <typename T1,
					typename T2=Nothing,
					typename T3=Nothing,
					typename T4=Nothing,
					typename T5=Nothing,
					typename T6=Nothing>
struct MakeList {
#define C ConsIf
	typedef typename C<T1, C<T2, C<T3, C<T4, C<T5, C<T5, C<T6> > > > > > >::result result;
#undef C
};

// Represents a list of arguments
template <typename TypeList, unsigned I=0>
struct ArgList {
	typedef typename TypeList::head argtype;
	ArgList<typename TypeList::tail, I+1> next;
	ArgList(int argc, char **argv) : next(argc-1, argv+1) {
		if (argc == 0) {
			DLOG << "Too few parameters";
			exit(-1);
		} else {
			argtype val = lexical_cast<argtype>(argv[0]);
			DLOG << "Arg is " << argv[0] << " -> " << val << endl;
		}
	}
};

// Represents an empty list of arguments
template <unsigned I>
struct ArgList<Empty, I> {
	ArgList(int argc, char **argv) {
		if (argc > 0) {
			DLOG << "Too many parameters";
			exit(-1);
		}
	}
};

// Check arguments
template<typename T1,
				 typename T2=Nothing,
				 typename T3=Nothing,
				 typename T4=Nothing,
				 typename T5=Nothing,
				 typename T6=Nothing>
struct ArgCheck {
	typedef typename MakeList<T1,T2,T3,T4,T5,T6>::result argtypes;
	ArgList<argtypes> args;
	ArgCheck(int argc, char **argv) : args(argc-1, argv+1) { 
	}
};
