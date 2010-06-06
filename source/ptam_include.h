// This file is a hack to put PTAM into a namespace. We include all
// the files included from PTAM outside the namespace, then include
// PTAM itself inside the namespace

// Command to generate the includes is: grep -h -o '#include <[^ ]*' *.h | sort | uniq

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <cvd/byte.h>
#include <cvd/gl_helpers.h>
#include <cvd/image.h>
#include <cvd/image_ref.h>
#include <cvd/rgba.h>
#include <cvd/rgb.h>
#include <cvd/synchronized.h>
#include <cvd/thread.h>
#include <cvd/timer.h>
#include <cvd/utility.h>
#include <cvd/vector_image_ref.h>

#include <TooN/helpers.h>
#include <TooN/lapack.h>
#include <TooN/se2.h>
#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/TooN.h>

#include <GLWindow.h>
#include <GUICommandQueuer.h>

#include <gvars3/gvars3.h>
#include <gvars3/instances.h>

#include "ATANCamera.h"

#undef Bool // this gets defined somewhere in CVD, TooN, PTAM, or GKTools
